#!/usr/bin/env -S uv run --script --with rich --with draccus
"""
Header:
- Purpose: Produce a readable, non-interactive dump of one motor's state.
- What it does: Finds the selected motor, prints connection/identity summary,
  then reads the full model control table and displays results in Rich tables.
- Output: Terminal-friendly summary + per-register raw/decoded values and errors.
- Safety: Read-only register inspection.

Pipe to less:
  ./scripts/lerobot_motor_dump.py \
    --robot.type=so101_follower \
    --robot.port=/dev/tty.usbmodem59700737971 \
    --motor_name=shoulder_pan | less -R
"""

from dataclasses import dataclass
from pathlib import Path
import sys

import draccus
from rich.console import Console
from rich.table import Table

# Allow local source import without editable install.
REPO_ROOT = Path(__file__).resolve().parents[1]
LOCAL_SRC = REPO_ROOT / "lerobot" / "src"
try:
    if LOCAL_SRC.exists() and str(LOCAL_SRC) not in sys.path:
        sys.path.insert(0, str(LOCAL_SRC))
except OSError:
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
class DumpConfig:
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

    out = {
        "addr": addr,
        "len": length,
        "raw": int(value),
        "decoded": int(decoded),
        "ok": bool(bus._is_comm_success(comm) and (not bus._is_error(error))),  # noqa: SLF001
        "comm": bus.packet_handler.getTxRxResult(comm),
        "error": bus.packet_handler.getRxPacketError(error),
        "error_raw": f"0x{error:02X}",
    }
    if "Voltage" in reg_name:
        out["volts"] = round(decoded / 10.0, 2)
    if "Temperature" in reg_name:
        out["celsius"] = int(decoded)
    return out


@draccus.wrap()
def main(cfg: DumpConfig):
    console = Console()

    if cfg.device.type not in COMPATIBLE_DEVICES:
        raise NotImplementedError(f"Unsupported device type: {cfg.device.type}")

    device = make_robot_from_config(cfg.device) if isinstance(cfg.device, RobotConfig) else make_teleoperator_from_config(cfg.device)
    if not hasattr(device, "bus"):
        raise RuntimeError("This device has no motor bus attribute.")

    bus = device.bus
    if cfg.motor_name not in bus.motors:
        raise ValueError(f"Unknown motor_name='{cfg.motor_name}'. Valid: {list(bus.motors.keys())}")

    expected = bus.motors[cfg.motor_name]
    bus._connect(handshake=False)  # noqa: SLF001

    try:
        console.rule("[bold cyan]Hey are we connected to this motor?")
        found_baud, found_id = bus._find_single_motor(cfg.motor_name)  # noqa: SLF001
        bus.set_baudrate(found_baud)

        firmware = None
        if hasattr(bus, "_read_firmware_version"):
            firmware = bus._read_firmware_version([found_id], raise_on_error=False).get(found_id)  # noqa: SLF001

        model_number = None
        if hasattr(bus, "_read_model_number"):
            model_number = bus._read_model_number([found_id], raise_on_error=False).get(found_id)  # noqa: SLF001

        summary = Table(title="Summary", show_header=False)
        summary.add_column("k", style="cyan", no_wrap=True)
        summary.add_column("v")
        summary.add_row("Connected", "YES")
        summary.add_row("Motor slot", cfg.motor_name)
        summary.add_row("Expected ID", str(expected.id))
        summary.add_row("Detected ID", str(found_id))
        summary.add_row("ID set correctly", "yes" if found_id == expected.id else "no")
        summary.add_row("Expected model", expected.model)
        summary.add_row("Model number", str(model_number))
        summary.add_row("Firmware", str(firmware))
        summary.add_row("Detected baud", str(found_baud))
        console.print(summary)

        reg_table = Table(title="All readable details", header_style="bold magenta")
        reg_table.add_column("Register", no_wrap=True)
        reg_table.add_column("Addr", justify="right")
        reg_table.add_column("Len", justify="right")
        reg_table.add_column("Decoded", justify="right")
        reg_table.add_column("Raw", justify="right")
        reg_table.add_column("OK", justify="center")
        reg_table.add_column("Notes")

        for reg_name in bus.model_ctrl_table[expected.model]:
            rr = _read_register(bus, expected.model, found_id, reg_name)
            notes = []
            if "volts" in rr:
                notes.append(f"{rr['volts']}V")
            if "celsius" in rr:
                notes.append(f"{rr['celsius']}°C")
            if not rr["ok"]:
                notes.append(f"{rr['comm']} | {rr['error']} ({rr['error_raw']})")

            reg_table.add_row(
                reg_name,
                str(rr["addr"]),
                str(rr["len"]),
                str(rr["decoded"]),
                str(rr["raw"]),
                "✅" if rr["ok"] else "❌",
                " | ".join(notes),
            )

        console.print(reg_table)

    except Exception as exc:
        err = Table(title="Summary", show_header=False)
        err.add_column("k", style="cyan")
        err.add_column("v")
        err.add_row("Connected", "NO")
        err.add_row("Motor slot", cfg.motor_name)
        err.add_row("Reason", str(exc))
        console.print(err)
    finally:
        if bus.is_connected:
            bus.disconnect(disable_torque=False)


if __name__ == "__main__":
    main()
