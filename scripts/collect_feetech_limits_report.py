#!/usr/bin/env python3
"""
Quick diagnostic report for SO101/Feetech joint limits and health registers.

Purpose:
- Print the exact registers needed to debug "can't move past angle" issues.
- Focus on joints that are commonly problematic (shoulder_lift, elbow_flex, gripper).

Example:
  uv run scripts/collect_feetech_limits_report.py \
    --port /dev/tty.usbmodem59700737971
"""

from __future__ import annotations

import argparse
import sys
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
from lerobot.motors.feetech import FeetechMotorsBus

DEFAULT_PORT = "/dev/tty.usbmodem59700737971"
MOTOR_MAP = {
    "shoulder_pan": Motor(1, "sts3215", MotorNormMode.DEGREES),
    "shoulder_lift": Motor(2, "sts3215", MotorNormMode.DEGREES),
    "elbow_flex": Motor(3, "sts3215", MotorNormMode.DEGREES),
    "wrist_flex": Motor(4, "sts3215", MotorNormMode.DEGREES),
    "wrist_roll": Motor(5, "sts3215", MotorNormMode.DEGREES),
    "gripper": Motor(6, "sts3215", MotorNormMode.RANGE_0_100),
}
DEFAULT_MOTORS = ["shoulder_lift", "elbow_flex", "gripper"]

REGISTERS = [
    "Min_Position_Limit",
    "Max_Position_Limit",
    "Homing_Offset",
    "Goal_Position",
    "Present_Position",
    "Present_Voltage",
    "Present_Temperature",
    "Present_Current",
    "Status",
    "Unloading_Condition",
    "LED_Alarm_Condition",
    "Torque_Enable",
]


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Collect SO101 motor limit/health diagnostics")
    p.add_argument("--port", default=DEFAULT_PORT, help=f"Serial port (default: {DEFAULT_PORT})")
    p.add_argument("--baudrate", type=int, default=1_000_000, help="Bus baudrate (default: 1000000)")
    p.add_argument(
        "--motors",
        nargs="+",
        default=DEFAULT_MOTORS,
        help=f"Motor names to inspect (default: {' '.join(DEFAULT_MOTORS)})",
    )
    return p.parse_args()


def read_register_safe(bus: FeetechMotorsBus, motor: str, reg: str):
    try:
        value = bus.read(reg, motor, normalize=False)
        if isinstance(value, dict):
            value = value[motor]
        return int(value), None
    except Exception as exc:  # noqa: BLE001
        return None, str(exc)


def format_value(reg: str, value: int | None) -> str:
    if value is None:
        return "<error>"
    if reg == "Present_Voltage":
        return f"{value} (~{value / 10.0:.1f}V)"
    if reg == "Present_Temperature":
        return f"{value} (~{value}°C)"
    return str(value)


def main() -> None:
    args = parse_args()

    unknown = [m for m in args.motors if m not in MOTOR_MAP]
    if unknown:
        raise SystemExit(f"Unknown motor(s): {unknown}. Valid: {list(MOTOR_MAP.keys())}")

    bus = FeetechMotorsBus(port=args.port, motors=MOTOR_MAP)
    bus._connect(handshake=False)  # noqa: SLF001
    bus.set_baudrate(args.baudrate)

    print("=== Motor Limit/Health Report ===")
    print(f"port: {args.port}")
    print(f"baudrate: {args.baudrate}")
    print(f"motors: {', '.join(args.motors)}")

    try:
        for motor in args.motors:
            mid = MOTOR_MAP[motor].id
            print(f"\n--- {motor} (id={mid}, model=sts3215) ---")
            for reg in REGISTERS:
                value, err = read_register_safe(bus, motor, reg)
                if err:
                    print(f"{reg:22s}: <error> {err}")
                else:
                    print(f"{reg:22s}: {format_value(reg, value)}")

            min_v, _ = read_register_safe(bus, motor, "Min_Position_Limit")
            max_v, _ = read_register_safe(bus, motor, "Max_Position_Limit")
            pos_v, _ = read_register_safe(bus, motor, "Present_Position")
            if min_v is not None and max_v is not None:
                span = max_v - min_v
                print(f"{'Computed_span':22s}: {span} ticks")
                if pos_v is not None:
                    at_low = pos_v <= min_v + 5
                    at_high = pos_v >= max_v - 5
                    print(f"{'Near_limit_now':22s}: low={at_low} high={at_high}")
    finally:
        if bus.is_connected:
            bus.disconnect(disable_torque=False)


if __name__ == "__main__":
    main()
