#!/usr/bin/env python3
"""
Header:
- Purpose: Interactive keyboard TUI to jog SO100/SO101 motors.
- What it does: Connects to a fixed 6-motor map, shows goal/present/min/max values,
  and lets you move the selected motor left/right by a configurable tick step.
- Safety notes: Writes Goal_Position while running; on exit, torque is kept enabled
  unless --disable-torque-on-exit is passed.

Usage:
  ./scripts/lerobot_motor_tui.py --port /dev/tty.usbmodemXXXX

Controls:
  ↑/↓  select motor
  ←/→  move selected motor by --step raw ticks
  q    quit
"""

import argparse
import curses
import sys
import time
from pathlib import Path

# Allow local source import without editable install.
REPO_ROOT = Path(__file__).resolve().parents[1]
LOCAL_SRC = REPO_ROOT / "lerobot" / "src"
try:
    if LOCAL_SRC.exists() and str(LOCAL_SRC) not in sys.path:
        sys.path.insert(0, str(LOCAL_SRC))
except OSError:
    pass

from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus, OperatingMode

MOTOR_NAMES = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "gripper",
    "wrist_roll",
]


def make_bus(port: str) -> FeetechMotorsBus:
    return FeetechMotorsBus(
        port=port,
        motors={
            "shoulder_pan": Motor(1, "sts3215", MotorNormMode.DEGREES),
            "shoulder_lift": Motor(2, "sts3215", MotorNormMode.DEGREES),
            "elbow_flex": Motor(3, "sts3215", MotorNormMode.DEGREES),
            "wrist_flex": Motor(4, "sts3215", MotorNormMode.DEGREES),
            "wrist_roll": Motor(5, "sts3215", MotorNormMode.DEGREES),
            "gripper": Motor(6, "sts3215", MotorNormMode.RANGE_0_100),
        },
    )


def clamp(v: int, lo: int, hi: int) -> int:
    return max(lo, min(hi, v))


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Simple motor jog TUI")
    p.add_argument("--port", required=True, help="Serial port, e.g. /dev/tty.usbmodemXXXX")
    p.add_argument("--step", type=int, default=10, help="Ticks to move per left/right keypress")
    p.add_argument(
        "--disable-torque-on-exit",
        action="store_true",
        help="Disable torque when exiting (default: keep torque enabled)",
    )
    return p.parse_args()


def add_line(stdscr, y: int, x: int, text: str, attr: int = 0) -> None:
    h, w = stdscr.getmaxyx()
    if y < 0 or y >= h or x >= w:
        return
    max_len = max(0, w - x - 1)
    if max_len <= 0:
        return
    stdscr.addnstr(y, x, text, max_len, attr)


def draw_ui(
    stdscr,
    selected_index: int,
    targets: dict[str, int],
    present: dict[str, int],
    mins: dict[str, int],
    maxes: dict[str, int],
    step: int,
    status: str,
) -> None:
    stdscr.erase()
    h, w = stdscr.getmaxyx()

    if h < 12 or w < 72:
        add_line(stdscr, 0, 0, f"Terminal too small ({w}x{h}). Resize to at least 72x12.")
        add_line(stdscr, 2, 0, "Press q to quit.")
        stdscr.noutrefresh()
        return

    try:
        stdscr.box()
    except curses.error:
        pass

    header_attr = curses.A_BOLD
    selected_attr = curses.A_BOLD | curses.A_REVERSE

    add_line(stdscr, 1, 2, "LeRobot Motor Jog", header_attr)
    add_line(stdscr, 2, 2, f"↑/↓ choose motor   ←/→ move by {step} ticks   q quit")

    table_y = 4
    add_line(stdscr, table_y, 2, "Sel Motor          Goal  Present   Min   Max", curses.A_BOLD)
    add_line(stdscr, table_y + 1, 2, "--- ------------- ------ ------- ----- -----")

    y = table_y + 2
    for i, name in enumerate(MOTOR_NAMES):
        marker = ">" if i == selected_index else " "
        line = f" {marker}  {name:<13} {targets[name]:>6} {present[name]:>7} {mins[name]:>5} {maxes[name]:>5}"
        add_line(stdscr, y + i, 2, line, selected_attr if i == selected_index else 0)

    status_y = y + len(MOTOR_NAMES) + 1
    add_line(stdscr, status_y, 2, "Status:", curses.A_BOLD)
    add_line(stdscr, status_y + 1, 2, status or "ready")

    stdscr.noutrefresh()


def run_tui(stdscr, bus: FeetechMotorsBus, step: int) -> None:
    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.keypad(True)
    curses.noecho()
    curses.cbreak()

    selected = 0
    status = "ready"

    mins = {m: int(bus.read("Min_Position_Limit", m, normalize=False)) for m in MOTOR_NAMES}
    maxes = {m: int(bus.read("Max_Position_Limit", m, normalize=False)) for m in MOTOR_NAMES}
    present = {m: int(bus.read("Present_Position", m, normalize=False)) for m in MOTOR_NAMES}
    targets = present.copy()

    last_read_at = 0.0
    read_interval = 0.08

    while True:
        now = time.monotonic()
        if now - last_read_at >= read_interval:
            try:
                present = {m: int(bus.read("Present_Position", m, normalize=False)) for m in MOTOR_NAMES}
            except Exception as exc:
                status = f"Read failed: {exc}"
            last_read_at = now

        draw_ui(stdscr, selected, targets, present, mins, maxes, step, status)
        curses.doupdate()

        key = stdscr.getch()
        if key == -1:
            time.sleep(0.01)
            continue

        if key in (ord("q"), ord("Q"), 3):
            break
        if key == curses.KEY_UP:
            selected = (selected - 1) % len(MOTOR_NAMES)
            continue
        if key == curses.KEY_DOWN:
            selected = (selected + 1) % len(MOTOR_NAMES)
            continue

        motor = MOTOR_NAMES[selected]
        if key in (curses.KEY_RIGHT, curses.KEY_LEFT):
            delta = step if key == curses.KEY_RIGHT else -step
            new_goal = clamp(targets[motor] + delta, mins[motor], maxes[motor])
            try:
                bus.write("Goal_Position", motor, new_goal, normalize=False)
                targets[motor] = new_goal
                status = f"Moved {motor} to {new_goal}"
            except Exception as exc:
                status = f"Write failed on {motor}: {exc}"


def main() -> None:
    args = parse_args()

    if not sys.stdin.isatty() or not sys.stdout.isatty():
        raise SystemExit("This script requires an interactive TTY.")

    bus = make_bus(args.port)
    bus._connect(handshake=False)  # noqa: SLF001

    try:
        with bus.torque_disabled():
            for motor in MOTOR_NAMES:
                bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)

        bus.enable_torque()
        curses.wrapper(lambda stdscr: run_tui(stdscr, bus, args.step))
    finally:
        if bus.is_connected:
            bus.disconnect(disable_torque=args.disable_torque_on_exit)


if __name__ == "__main__":
    main()
