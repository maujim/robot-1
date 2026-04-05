#!/usr/bin/env python3
"""
Header:
- Purpose: Read-only diagnostic inspector for Feetech/STS motors.
- What it does: Pings one or more motor IDs, reads key voltage/error/health registers,
  and prints both raw values and decoded hints (e.g., volts/temperature).
- Safety: Never writes to motor registers; only ping/read operations are used.

Example:
  python scripts/inspect_feetech_voltage_readonly.py \
    --port /dev/tty.usbmodem59710814421 \
    --baudrate 1000000 \
    --protocol 0

If you already know the motor ID:
  python scripts/inspect_feetech_voltage_readonly.py \
    --port /dev/tty.usbmodem59710814421 \
    --id 1
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass

import scservo_sdk as scs

# Reuse LeRobot's timeout patch and control table definitions
from lerobot.motors.feetech.feetech import patch_setPacketTimeout
from lerobot.motors.feetech.tables import STS_SMS_SERIES_CONTROL_TABLE


@dataclass
class ReadResult:
    value: int
    comm: int
    error: int


def decode_error_bits(error: int) -> list[str]:
    # SDK constants live in scservo_sdk.protocol_packet_handler.py
    bits = [
        (getattr(scs, "ERRBIT_VOLTAGE", 1), "Input voltage"),
        (getattr(scs, "ERRBIT_ANGLE", 2), "Angle"),
        (getattr(scs, "ERRBIT_OVERHEAT", 4), "Overheat"),
        (getattr(scs, "ERRBIT_OVERELE", 8), "Over-current/over-ele"),
        (getattr(scs, "ERRBIT_OVERLOAD", 32), "Overload"),
    ]
    return [name for bit, name in bits if error & bit]


def read_reg(packet_handler, port_handler, motor_id: int, addr: int, length: int) -> ReadResult:
    if length == 1:
        value, comm, error = packet_handler.read1ByteTxRx(port_handler, motor_id, addr)
    elif length == 2:
        value, comm, error = packet_handler.read2ByteTxRx(port_handler, motor_id, addr)
    elif length == 4:
        value, comm, error = packet_handler.read4ByteTxRx(port_handler, motor_id, addr)
    else:
        raise ValueError(f"Unsupported register size: {length}")
    return ReadResult(value=value, comm=comm, error=error)


def fmt_comm(packet_handler, comm: int) -> str:
    return packet_handler.getTxRxResult(comm)


def fmt_err(packet_handler, error: int) -> str:
    if error == 0:
        return "none"
    primary = packet_handler.getRxPacketError(error)
    all_flags = decode_error_bits(error)
    return f"{primary} | all_flags={all_flags} | raw=0x{error:02X}"


def scan_ids(packet_handler, port_handler, max_id: int) -> list[tuple[int, int, int, int]]:
    found = []
    for motor_id in range(max_id + 1):
        model, comm, error = packet_handler.ping(port_handler, motor_id)
        if comm == scs.COMM_SUCCESS:
            found.append((motor_id, model, comm, error))
    return found


def main() -> None:
    parser = argparse.ArgumentParser(description="Read-only Feetech voltage/error inspector")
    parser.add_argument("--port", required=True, help="Serial port (e.g. /dev/tty.usbmodemXXXX)")
    parser.add_argument("--baudrate", type=int, default=1_000_000)
    parser.add_argument("--protocol", type=int, default=0, choices=[0, 1])
    parser.add_argument("--timeout-ms", type=int, default=1000)
    parser.add_argument("--id", type=int, default=None, help="Specific motor ID to inspect")
    parser.add_argument("--scan-max-id", type=int, default=20, help="ID scan upper bound when --id not given")
    args = parser.parse_args()

    port_handler = scs.PortHandler(args.port)
    # Same monkeypatch LeRobot applies, keeps timeout behavior sane for this SDK build
    port_handler.setPacketTimeout = patch_setPacketTimeout.__get__(port_handler, scs.PortHandler)
    packet_handler = scs.PacketHandler(args.protocol)

    if not port_handler.openPort():
        raise SystemExit(f"Failed to open port: {args.port}")

    try:
        if not port_handler.setBaudRate(args.baudrate):
            raise SystemExit(f"Failed to set baudrate: {args.baudrate}")

        port_handler.setPacketTimeoutMillis(args.timeout_ms)

        print("=== Bus ===")
        print(f"port       : {args.port}")
        print(f"baudrate   : {port_handler.getBaudRate()}")
        print(f"protocol   : {args.protocol}")
        print(f"timeout_ms : {args.timeout_ms}")

        if args.id is None:
            print(f"\n=== Scanning IDs 0..{args.scan_max_id} (read-only ping) ===")
            found = scan_ids(packet_handler, port_handler, args.scan_max_id)
            if not found:
                print("No responding IDs found in scan range.")
                return

            for motor_id, model, comm, error in found:
                print(
                    f"ID {motor_id:3d} | model={model:5d} | comm={fmt_comm(packet_handler, comm)} "
                    f"| ping_error={fmt_err(packet_handler, error)}"
                )
            target_ids = [m[0] for m in found]
        else:
            model, comm, error = packet_handler.ping(port_handler, args.id)
            print("\n=== Single-ID ping ===")
            print(
                f"ID {args.id:3d} | model={model:5d} | comm={fmt_comm(packet_handler, comm)} "
                f"| ping_error={fmt_err(packet_handler, error)}"
            )
            if comm != scs.COMM_SUCCESS:
                return
            target_ids = [args.id]

        regs = [
            ("Status", *STS_SMS_SERIES_CONTROL_TABLE["Status"]),
            ("Present_Voltage", *STS_SMS_SERIES_CONTROL_TABLE["Present_Voltage"]),
            ("Min_Voltage_Limit", *STS_SMS_SERIES_CONTROL_TABLE["Min_Voltage_Limit"]),
            ("Max_Voltage_Limit", *STS_SMS_SERIES_CONTROL_TABLE["Max_Voltage_Limit"]),
            ("Present_Temperature", *STS_SMS_SERIES_CONTROL_TABLE["Present_Temperature"]),
            ("Present_Current", *STS_SMS_SERIES_CONTROL_TABLE["Present_Current"]),
            ("Unloading_Condition", *STS_SMS_SERIES_CONTROL_TABLE["Unloading_Condition"]),
            ("LED_Alarm_Condition", *STS_SMS_SERIES_CONTROL_TABLE["LED_Alarm_Condition"]),
        ]

        print("\n=== Register reads (read-only) ===")
        for motor_id in target_ids:
            print(f"\n-- ID {motor_id} --")
            for name, addr, length in regs:
                rr = read_reg(packet_handler, port_handler, motor_id, addr, length)

                # Common unit hints from Feetech docs:
                # - Present_Voltage / Min/Max_Voltage_Limit are typically in 0.1V units.
                unit_hint = ""
                if "Voltage" in name:
                    unit_hint = f" (~{rr.value / 10:.1f}V if 0.1V units)"
                elif "Temperature" in name:
                    unit_hint = f" (~{rr.value}°C)"

                print(
                    f"{name:22s} addr={addr:>3} len={length} value={rr.value:>5}{unit_hint} "
                    f"| comm={fmt_comm(packet_handler, rr.comm)} "
                    f"| error={fmt_err(packet_handler, rr.error)}"
                )

    finally:
        port_handler.closePort()
        print("\nPort closed.")


if __name__ == "__main__":
    main()
