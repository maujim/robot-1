#!/usr/bin/env python3
"""
Header:
- Purpose: Verify one expected motor is reachable and correctly configured.
- What it does: Connects to a LeRobot device bus, locates the selected motor slot,
  checks detected ID/model/baud/firmware, and reads calibration-relevant registers.
- Output: Prints a high-level summary plus full control-table read attempts.
- Safety: Read-only register inspection (no calibration writes).

Example:
  python scripts/lerobot_check_motor_details.py \
      --robot.type=so101_follower \
      --robot.port=/dev/tty.usbmodem59700737971 \
      --motor_name=shoulder_pan
"""

from dataclasses import dataclass
from pathlib import Path
from pprint import pformat
import sys

import draccus

# Allow running from repo root without requiring editable install.
REPO_ROOT = Path(__file__).resolve().parents[1]
LOCAL_SRC = REPO_ROOT / "lerobot" / "src"
try:
    if LOCAL_SRC.exists() and str(LOCAL_SRC) not in sys.path:
        sys.path.insert(0, str(LOCAL_SRC))
except OSError:
    # If path probing fails due to local FS permissions, continue and rely on installed package.
    pass

from lerobot.motors.motors_bus import get_address
from lerobot.robots import (  # noqa: F401
    RobotConfig,
    bi_so_follower,
    koch_follower,
    lekiwi,
    make_robot_from_config,
    omx_follower,
    so_follower,
)
from lerobot.teleoperators import (  # noqa: F401
    TeleoperatorConfig,
    bi_so_leader,
    koch_leader,
    make_teleoperator_from_config,
    omx_leader,
    openarm_mini,
    so_leader,
)

COMPATIBLE_DEVICES = [
    "koch_follower",
    "koch_leader",
    "omx_follower",
    "omx_leader",
    "openarm_mini",
    "so100_follower",
    "so100_leader",
    "so101_follower",
    "so101_leader",
    "lekiwi",
]


@dataclass
class InspectConfig:
    teleop: TeleoperatorConfig | None = None
    robot: RobotConfig | None = None
    motor_name: str = "shoulder_pan"

    def __post_init__(self):
        if bool(self.teleop) == bool(self.robot):
            raise ValueError("Choose either a teleop or a robot.")
        self.device = self.robot if self.robot else self.teleop


def _read_register(bus, model: str, motor_id: int, reg_name: str) -> dict:
    addr, length = get_address(bus.model_ctrl_table, model, reg_name)
    value, comm, error = bus._read(addr, length, motor_id, raise_on_error=False)  # noqa: SLF001
    decoded = bus._decode_sign(reg_name, {motor_id: value})[motor_id]  # noqa: SLF001

    entry = {
        "address": addr,
        "length": length,
        "raw": int(value),
        "decoded": int(decoded),
        "comm": bus.packet_handler.getTxRxResult(comm),
        "error": bus.packet_handler.getRxPacketError(error),
        "error_raw": f"0x{error:02X}",
        "ok": bool(bus._is_comm_success(comm) and (not bus._is_error(error))),  # noqa: SLF001
    }
    if "Voltage" in reg_name:
        entry["volts"] = round(decoded / 10.0, 2)
    if "Temperature" in reg_name:
        entry["celsius"] = int(decoded)
    return entry


def _inspect_expected_motor(device, motor_name: str) -> None:
    bus = device.bus
    if motor_name not in bus.motors:
        raise ValueError(f"Unknown motor_name='{motor_name}'. Valid names: {list(bus.motors.keys())}")

    expected = bus.motors[motor_name]

    print(f"\n=== Target motor slot: '{motor_name}' (expected id={expected.id}, model={expected.model}) ===")
    print(f"Hey are we connected to this motor? -> checking '{motor_name}'...")

    try:
        # Reuse LeRobot's own setup logic (fast and model-aware)
        found_baudrate, found_id = bus._find_single_motor(motor_name)  # noqa: SLF001
    except Exception as exc:
        print("Connected? NO")
        print(f"Reason: {exc}")
        return

    print("Connected? YES")

    bus.set_baudrate(found_baudrate)

    firmware = None
    if hasattr(bus, "_read_firmware_version"):
        firmware = bus._read_firmware_version([found_id], raise_on_error=False).get(found_id)  # noqa: SLF001

    model_number = None
    if hasattr(bus, "_read_model_number"):
        model_number = bus._read_model_number([found_id], raise_on_error=False).get(found_id)  # noqa: SLF001

    baud_reg = _read_register(bus, expected.model, found_id, "Baud_Rate")
    baud_value = int(baud_reg["decoded"])
    known_baud = {code: br for br, code in bus.model_baudrate_table[expected.model].items()}.get(baud_value)

    calibration_relevant = {
        "min_position_limit": _read_register(bus, expected.model, found_id, "Min_Position_Limit"),
        "max_position_limit": _read_register(bus, expected.model, found_id, "Max_Position_Limit"),
    }
    if "Homing_Offset" in bus.model_ctrl_table[expected.model]:
        calibration_relevant["homing_offset"] = _read_register(bus, expected.model, found_id, "Homing_Offset")

    summary = {
        "expected": {
            "name": motor_name,
            "id": expected.id,
            "model": expected.model,
            "model_number_expected": bus.model_number_table.get(expected.model),
        },
        "detected": {
            "id": found_id,
            "scan_baudrate": found_baudrate,
            "id_matches_expected": found_id == expected.id,
            "firmware_version": firmware,
            "model_number": model_number,
            "baud_register_value": baud_value,
            "baudrate_from_register": known_baud,
        },
        "calibration_relevant": calibration_relevant,
    }

    control_table = bus.model_ctrl_table[expected.model]
    registers = {
        reg_name: _read_register(bus, expected.model, found_id, reg_name)
        for reg_name in control_table
    }

    print("\nSummary:")
    print(pformat(summary, sort_dicts=False))

    print("\nAll readable details (entire control table read attempts):")
    print(pformat(registers, sort_dicts=False))


@draccus.wrap()
def inspect_motors(cfg: InspectConfig):
    if cfg.device.type not in COMPATIBLE_DEVICES:
        raise NotImplementedError(f"Unsupported device type: {cfg.device.type}")

    device = make_robot_from_config(cfg.device) if isinstance(cfg.device, RobotConfig) else make_teleoperator_from_config(cfg.device)

    if not hasattr(device, "bus"):
        raise RuntimeError("This device has no motor bus attribute.")

    bus = device.bus
    bus._connect(handshake=False)  # noqa: SLF001

    try:
        input(f"Connect only '{cfg.motor_name}' now, then press ENTER to inspect it.")
        _inspect_expected_motor(device, cfg.motor_name)
    finally:
        if bus.is_connected:
            bus.disconnect(disable_torque=False)


def main():
    inspect_motors()


if __name__ == "__main__":
    main()
