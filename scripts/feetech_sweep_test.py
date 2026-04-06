#!/usr/bin/env python3
"""
Controlled sweep test for SO101/Feetech motors.

Purpose:
- Move selected motors across most of their configured range in small steps.
- Log command vs. measured position to identify where motion stops tracking.

Use when:
- A joint appears to stop early (e.g., elbow/gripper not reaching expected travel).

Safety:
- This script WRITES `Goal_Position`.
- It stays inside motor register limits with a configurable edge margin.
- Keep clear of the robot while running.

Example:
  .venv/bin/python scripts/feetech_sweep_test.py \
    --port /dev/tty.usbmodem59700737971 \
    --motors elbow_flex gripper
"""

from __future__ import annotations

import argparse
import csv
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

DEFAULT_PORT = "/dev/tty.usbmodem59700737971"
MOTOR_MAP = {
    "shoulder_pan": Motor(1, "sts3215", MotorNormMode.DEGREES),
    "shoulder_lift": Motor(2, "sts3215", MotorNormMode.DEGREES),
    "elbow_flex": Motor(3, "sts3215", MotorNormMode.DEGREES),
    "wrist_flex": Motor(4, "sts3215", MotorNormMode.DEGREES),
    "wrist_roll": Motor(5, "sts3215", MotorNormMode.DEGREES),
    "gripper": Motor(6, "sts3215", MotorNormMode.RANGE_0_100),
}


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Sweep Feetech motors and log tracking")
    p.add_argument("--port", default=DEFAULT_PORT, help=f"Serial port (default: {DEFAULT_PORT})")
    p.add_argument("--baudrate", type=int, default=1_000_000)
    p.add_argument("--motors", nargs="+", default=["elbow_flex", "gripper"], help="Motors to sweep")
    p.add_argument("--step", type=int, default=40, help="Ticks per movement step")
    p.add_argument("--dwell", type=float, default=0.12, help="Seconds to wait after each write")
    p.add_argument("--edge-margin", type=int, default=60, help="Keep this many ticks away from min/max")
    p.add_argument("--max-error", type=int, default=120, help="Tracking error threshold (ticks)")
    p.add_argument("--consecutive", type=int, default=4, help="Consecutive high-error steps to flag warning")
    p.add_argument("--csv", default=None, help="Optional CSV output path")
    p.add_argument(
        "--disable-torque-on-exit",
        action="store_true",
        help="Disable torque when disconnecting (default keeps current behavior)",
    )
    return p.parse_args()


def read_one(bus: FeetechMotorsBus, reg: str, motor: str) -> int:
    v = bus.read(reg, motor, normalize=False)
    if isinstance(v, dict):
        v = v[motor]
    return int(v)


def read_one_safe(bus: FeetechMotorsBus, reg: str, motor: str, retries: int = 3, retry_sleep: float = 0.03) -> int | None:
    last_exc: Exception | None = None
    for _ in range(retries):
        try:
            return read_one(bus, reg, motor)
        except Exception as exc:  # noqa: BLE001
            last_exc = exc
            time.sleep(retry_sleep)
    print(f"[warn] Failed to read {reg} on {motor} after {retries} tries: {last_exc}")
    return None


def clamp(v: int, lo: int, hi: int) -> int:
    return max(lo, min(hi, v))


def build_targets(start: int, lo: int, hi: int, step: int) -> list[int]:
    # start -> hi -> lo -> start
    targets: list[int] = []

    cur = start
    while cur < hi:
        cur = min(cur + step, hi)
        targets.append(cur)

    while cur > lo:
        cur = max(cur - step, lo)
        targets.append(cur)

    while cur < start:
        cur = min(cur + step, start)
        targets.append(cur)
    while cur > start:
        cur = max(cur - step, start)
        targets.append(cur)

    return targets


def sweep_motor(
    bus: FeetechMotorsBus,
    motor: str,
    step: int,
    dwell: float,
    edge_margin: int,
    max_error: int,
    consecutive: int,
) -> tuple[list[dict], list[str]]:
    min_lim = read_one_safe(bus, "Min_Position_Limit", motor)
    max_lim = read_one_safe(bus, "Max_Position_Limit", motor)
    start = read_one_safe(bus, "Present_Position", motor)
    if min_lim is None or max_lim is None or start is None:
        return [], [f"{motor}: could not read initial limits/position, skipping sweep"]

    lo = clamp(min_lim + edge_margin, min_lim, max_lim)
    hi = clamp(max_lim - edge_margin, min_lim, max_lim)
    start = clamp(start, lo, hi)

    targets = build_targets(start=start, lo=lo, hi=hi, step=step)
    rows: list[dict] = []
    warnings: list[str] = []

    high_err_run = 0
    prev_present = start

    print(f"\n=== Sweeping {motor} ===")
    print(f"limits=[{min_lim}, {max_lim}] safe=[{lo}, {hi}] start={start}")

    for i, goal in enumerate(targets, start=1):
        try:
            bus.write("Goal_Position", motor, int(goal), normalize=False)
        except Exception as exc:  # noqa: BLE001
            warnings.append(f"{motor}: write failed at goal={goal}: {exc}")
            print(f"[warn] {warnings[-1]}")
            break

        time.sleep(dwell)

        present = read_one_safe(bus, "Present_Position", motor)
        volts = read_one_safe(bus, "Present_Voltage", motor)
        current = read_one_safe(bus, "Present_Current", motor)
        status = read_one_safe(bus, "Status", motor)

        if present is None:
            warnings.append(f"{motor}: stopping sweep, position read failed near goal={goal}")
            print(f"[warn] {warnings[-1]}")
            break

        if volts is None:
            volts = -1
        if current is None:
            current = -1
        if status is None:
            status = -1

        err = abs(goal - present)
        progress = abs(present - prev_present)
        prev_present = present

        if err > max_error:
            high_err_run += 1
        else:
            high_err_run = 0

        if high_err_run >= consecutive and progress <= 3:
            msg = (
                f"{motor}: possible stop near present={present} while goal={goal} "
                f"(err={err}, run={high_err_run})"
            )
            if not warnings or warnings[-1] != msg:
                warnings.append(msg)

        voltage_v = None if volts < 0 else round(volts / 10.0, 2)
        row = {
            "step": i,
            "motor": motor,
            "goal": int(goal),
            "present": int(present),
            "error": int(err),
            "progress": int(progress),
            "voltage_raw": int(volts),
            "voltage_v": voltage_v,
            "current": int(current),
            "status": int(status),
        }
        rows.append(row)

        v_disp = "n/a" if voltage_v is None else f"{voltage_v:>4.1f}"
        print(
            f"step={i:03d} goal={goal:4d} present={present:4d} "
            f"err={err:4d} dpos={progress:3d} V={v_disp} status={status}"
        )

    # Return to starting position for convenience.
    try:
        bus.write("Goal_Position", motor, int(start), normalize=False)
    except Exception as exc:  # noqa: BLE001
        warnings.append(f"{motor}: failed to return to start={start}: {exc}")
        print(f"[warn] {warnings[-1]}")
    return rows, warnings


def main() -> None:
    args = parse_args()
    unknown = [m for m in args.motors if m not in MOTOR_MAP]
    if unknown:
        raise SystemExit(f"Unknown motor(s): {unknown}. Valid: {list(MOTOR_MAP.keys())}")

    bus = FeetechMotorsBus(port=args.port, motors=MOTOR_MAP)
    bus._connect(handshake=False)  # noqa: SLF001

    all_rows: list[dict] = []
    all_warnings: list[str] = []

    try:
        bus.set_baudrate(args.baudrate)

        # Best-effort mode setup: some overloaded motors may reject torque/mode writes.
        # We don't hard-fail here; the sweep can still run if position control is already active.
        for motor in args.motors:
            try:
                bus.disable_torque(motor)
                bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)
                bus.enable_torque(motor)
            except Exception as exc:  # noqa: BLE001
                print(f"[warn] Could not force POSITION mode on {motor}: {exc}")
                print(f"[warn] Continuing sweep for {motor} using current mode/torque state.")

        for motor in args.motors:
            rows, warnings = sweep_motor(
                bus=bus,
                motor=motor,
                step=args.step,
                dwell=args.dwell,
                edge_margin=args.edge_margin,
                max_error=args.max_error,
                consecutive=args.consecutive,
            )
            all_rows.extend(rows)
            all_warnings.extend(warnings)

    finally:
        if bus.is_connected:
            bus.disconnect(disable_torque=args.disable_torque_on_exit)

    print("\n=== Sweep Summary ===")
    if all_warnings:
        print("Warnings:")
        for w in all_warnings:
            print(f"- {w}")
    else:
        print("No persistent high-error stalls detected with current thresholds.")

    if args.csv:
        fieldnames = [
            "step",
            "motor",
            "goal",
            "present",
            "error",
            "progress",
            "voltage_raw",
            "voltage_v",
            "current",
            "status",
        ]
        out = Path(args.csv)
        out.parent.mkdir(parents=True, exist_ok=True)
        with out.open("w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(all_rows)
        print(f"Wrote CSV: {out}")


if __name__ == "__main__":
    main()
