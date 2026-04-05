# Scripts

Quick reference for utility scripts in this repo.

## `collect_feetech_limits_report.py`
- **Purpose:** One-shot diagnostic report for SO101/Feetech joint limit and health registers.
- **Use when:** A joint cannot move past certain angles, or gripper range looks limited.
- **Reads:** `Min_Position_Limit`, `Max_Position_Limit`, `Homing_Offset`, `Goal/Present_Position`, voltage/temp/current, alarm/status registers.
- **Safety:** Read-only.

Example:
```bash
.venv/bin/python scripts/collect_feetech_limits_report.py \
  --port /dev/tty.usbmodem59700737971
```

---

## `inspect_feetech_voltage_readonly.py`
- **Purpose:** Low-level bus/ID ping + voltage/error inspector for Feetech motors.
- **Use when:** You suspect power issues, communication issues, or hardware alarm flags.
- **Safety:** Read-only.

Example:
```bash
.venv/bin/python scripts/inspect_feetech_voltage_readonly.py \
  --port /dev/tty.usbmodem59700737971 \
  --id 3
```

---

## `lerobot_check_motor_details.py`
- **Purpose:** Verify one expected motor slot is reachable and correctly configured.
- **Use when:** You want to confirm ID/model/baud/firmware and dump full control-table read attempts.
- **Safety:** Read-only.

Example:
```bash
.venv/bin/python scripts/lerobot_check_motor_details.py \
  --robot.type=so101_follower \
  --robot.port=/dev/tty.usbmodem59700737971 \
  --motor_name=shoulder_pan
```

---

## `lerobot_motor_dump.py`
- **Purpose:** Rich terminal dump for one motor’s registers and decoded values.
- **Use when:** You want a readable per-register summary for a specific motor.
- **Safety:** Read-only.

Example:
```bash
.venv/bin/python scripts/lerobot_motor_dump.py \
  --robot.type=so101_follower \
  --robot.port=/dev/tty.usbmodem59700737971 \
  --motor_name=elbow_flex
```

---

## `lerobot_motor_tui.py`
- **Purpose:** Interactive keyboard jog tool for SO100/SO101 motors.
- **Use when:** You need manual incremental movement testing from terminal.
- **Safety:** Writes `Goal_Position` while running.

Example:
```bash
.venv/bin/python scripts/lerobot_motor_tui.py \
  --port /dev/tty.usbmodem59700737971
```
