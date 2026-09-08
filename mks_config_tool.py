"""
mks_config_tool.py
--------------------
Command-line tool for the MKS SERVO42D/57D (via mks_servo42d_57d_tcp.py -
the native FA/FB "function-code" protocol, NOT Modbus-RTU, carried over a
TCP socket to a TRANSPARENT-mode RS485-to-Ethernet converter) that does
two things:

  read    - snapshot every genuinely READABLE register (encoder, angle,
            speed, pulses, IO status, motor status) and save it to a
            timestamped text (JSON) file. Good for logging/diagnostics/
            "what state was it in when X happened".

  write   - push a CONFIG PROFILE (work mode, current, microstep,
            direction, home params, etc.) from a text (JSON) file to the
            driver in one go. Good for deploying the same settings to a
            new/replacement driver, or restoring known-good settings.

-------------------------------------------------------------------------
IMPORTANT LIMITATION - READ THIS FIRST
-------------------------------------------------------------------------
The MKS protocol has NO "read back current setting" commands for most
configuration (work mode, current, microstep, direction, home params,
etc.) - only "set" commands. Those settings are meant to be checked on
the driver's own OLED screen, not queried over UART. So:

  - `read` can only capture live STATUS values (position, speed, etc.),
    NOT the currently-configured settings.
  - `write` pushes a config profile YOU define into a JSON file once
    (by hand, or via the --template option below) - it is not able to
    first confirm what's currently loaded on the driver.

This is a limitation of the driver's own firmware/protocol, not this
script - there is no way around it in software alone.

-------------------------------------------------------------------------
CONVERTER SETUP
-------------------------------------------------------------------------
This talks to the driver over its native FA/FB protocol, tunneled through
an RS485-to-Ethernet converter running in TRANSPARENT/passthrough mode -
NOT a Modbus TCP<->RTU gateway. See mks_servo42d_57d_tcp.py's docstring
for converter setup details (transparent mode, matching baud rate, the
per-channel TCP port).

-------------------------------------------------------------------------
USAGE
-------------------------------------------------------------------------
  # one-off: generate a starter config file to edit by hand
  python mks_config_tool.py template --output myconfig.json

  # snapshot live status to a timestamped file
  python mks_config_tool.py read --host 192.168.1.200 --port 4196 --addr 1

  # push a config profile to the driver
  python mks_config_tool.py write --host 192.168.1.200 --port 4196 --addr 1 --input myconfig.json
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from dataclasses import asdict, dataclass, field
from datetime import datetime
from typing import Optional

# Make sure Python can find mks_servo42d_57d_tcp.py regardless of the
# current working directory the script was launched from (e.g. VS Code's
# "Run" button often uses a different CWD than the file's own folder,
# which is the #1 cause of "No such file or directory" / ModuleNotFoundError
# on Windows). This adds this script's own folder to the import path.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from mks_servo42d_57d_tcp import (
    Direction,
    EnActiveLevel,
    MksServoError,
    MksServoTCP,
    WorkMode,
)


# ===========================================================================
# Status snapshot (READ)
# ===========================================================================

def read_status_snapshot(motor: MksServoTCP) -> dict:
    """Collect every genuinely readable value into a plain dict."""
    encoder = motor.read_encoder()
    snapshot = {
        "timestamp": datetime.now().isoformat(timespec="seconds"),
        "encoder_carry": encoder.carry,
        "encoder_value": encoder.value,
        "shaft_angle_degrees": round(motor.read_shaft_angle_degrees(), 2),
        "encoder_addition_axis": motor.read_encoder_addition(),
        "speed_rpm": motor.read_speed(),
        "pulses": motor.read_pulses(),
        "io_status": motor.read_io_status(),
        "angle_error": motor.read_angle_error(),
    }
    return snapshot


def cmd_read(args):
    with MksServoTCP(args.host, port=args.port, addr=args.addr,
                      debug=args.debug) as motor:
        snapshot = read_status_snapshot(motor)

    output_path = args.output or f"mks_status_{time.strftime('%Y%m%d_%H%M%S')}.json"
    with open(output_path, "w") as f:
        json.dump(snapshot, f, indent=2)

    print(f"Status snapshot saved to {output_path}")
    for k, v in snapshot.items():
        print(f"  {k}: {v}")


# ===========================================================================
# Config profile (WRITE)
# ===========================================================================

@dataclass
class ConfigProfile:
    """
    Every settable parameter and one-shot action documented in the MKS
    SERVO42D/57D RS485 manual. All fields are Optional - leave a field as
    null/None (or omit it) in the JSON file to skip it entirely; nothing
    is written or triggered unless you explicitly set it.

    Grouped to match the manual's own sections:
      - Motor/driver parameters (Ch. 6-7): work_mode, current_ma,
        microstep, en_active_level, direction, auto_screen_off,
        stall_protection, interpolation
      - Communication parameters (Ch. 8): uart_baud_index, uart_addr,
        uart_respond_all/uart_active_report, group_addr
      - Homing (Ch. 9): home_trigger_level, home_dir, home_speed_rpm
      - Front panel: key_lock
      - One-shot ACTIONS (not persisted settings - triggered once when
        true, see write_config()'s ordering/safety notes): calibrate,
        zero_axis, restore_defaults, restart
    """
    # --- motor / driver parameters ---
    work_mode: Optional[str] = None       # WorkMode name, e.g. "SR_vFOC"
    current_ma: Optional[int] = None      # motor current in mA
    microstep: Optional[int] = None       # e.g. 16, 32, 64, 128, 256
    en_active_level: Optional[str] = None # EnActiveLevel name: "LOW"/"HIGH"/"ALWAYS_ACTIVE"
    direction: Optional[str] = None       # "CW" or "CCW"
    auto_screen_off: Optional[bool] = None    # auto-dim/off the OLED after inactivity
    stall_protection: Optional[bool] = None   # locked-rotor protection
    interpolation: Optional[bool] = None      # microstep interpolation

    # --- communication parameters ---
    uart_baud_index: Optional[int] = None  # DANGER: see write_config() - breaks
                                            # communication until the RS485-to-ETH
                                            # converter's baud is updated to match
    uart_respond_all: Optional[bool] = None    # whether the driver ACKs every
                                                # command (vs. reads only)
    uart_active_report: Optional[bool] = None  # whether the driver pushes status
                                                # unprompted (see the stale-frame
                                                # discussion - usually want False)
    group_addr: Optional[int] = None       # broadcast group address

    modbus_addr: Optional[int] = None      # driver's UART address (0x8B).
                                            # CAUTION: applied LAST - see
                                            # write_config(). Field name kept
                                            # as "modbus_addr" for backward
                                            # JSON compatibility even though
                                            # this is the native protocol's
                                            # UART address, not Modbus.

    # --- homing (all four must be set together, or all left null) ---
    home_trigger_level: Optional[int] = None   # 0 or 1
    home_dir: Optional[str] = None             # "CW" or "CCW"
    home_speed_rpm: Optional[int] = None
    home_end_limit_enable: Optional[bool] = None  # enable the end-limit switch

    # --- front panel ---
    key_lock: Optional[bool] = None       # lock the front panel buttons

    # --- one-shot actions (set true to trigger; see write_config()) ---
    calibrate: Optional[bool] = None          # run encoder calibration
                                               # (motor must be UNLOADED)
    zero_axis: Optional[bool] = None          # set_current_axis_zero(): define
                                               # current position as zero
    restore_defaults: Optional[bool] = None   # DANGER: factory reset, needs
                                               # --confirm-dangerous on the CLI
    restart: Optional[bool] = None            # DANGER: reboots the driver,
                                               # needs --confirm-dangerous

    notes: Optional[str] = None           # free text, ignored by the script


def cmd_template(args):
    """Write a starter config file with every field present so you can see
    what's available. Ordinary settings get sensible example values;
    one-shot actions (calibrate/zero_axis/restore_defaults/restart) are
    left null since you should opt into those deliberately, not by
    forgetting to delete a template value."""
    profile = ConfigProfile(
        work_mode="SR_vFOC",
        current_ma=1200,
        microstep=16,
        en_active_level="LOW",
        direction="CW",
        auto_screen_off=False,
        stall_protection=True,
        interpolation=True,
        uart_baud_index=None,       # dangerous - see field docstring, leave null unless needed
        uart_respond_all=True,
        uart_active_report=False,  # False avoids the stale-frame/resync issue
        group_addr=None,
        modbus_addr=None,
        home_trigger_level=0,
        home_dir="CW",
        home_speed_rpm=60,
        home_end_limit_enable=True,
        key_lock=False,
        calibrate=None,
        zero_axis=None,
        restore_defaults=None,
        restart=None,
        notes="Delete/null any field you don't want written/triggered. "
              "uart_baud_index, restore_defaults, and restart are DANGEROUS "
              "(see their field comments and write_config()'s docstring) - "
              "restore_defaults/restart additionally require passing "
              "--confirm-dangerous on the command line.",
    )
    output_path = args.output or "config_template.json"
    with open(output_path, "w") as f:
        json.dump(asdict(profile), f, indent=2)
    print(f"Template written to {output_path} - edit it, then run:\n"
          f"  python mks_config_tool.py write --input {output_path}")


def load_config(path: str) -> ConfigProfile:
    with open(path) as f:
        data = json.load(f)
    known_fields = {f.name for f in ConfigProfile.__dataclass_fields__.values()}
    unknown = set(data) - known_fields
    if unknown:
        print(f"Warning: ignoring unknown field(s) in {path}: {sorted(unknown)}")
        data = {k: v for k, v in data.items() if k in known_fields}
    return ConfigProfile(**data)


def write_config(motor: MksServoTCP, profile: ConfigProfile, confirm_dangerous: bool = False):
    """
    Apply every non-null field in `profile` to the driver.

    ORDERING (matters, not arbitrary):
      1. restore_defaults FIRST - if you're resetting to factory defaults,
         any other settings in this same profile should apply AFTER that
         reset, not before it (otherwise the reset would immediately wipe
         out settings you just applied).
      2. calibrate - after mode/current/microstep are in place (encoder
         calibration should happen with the driver's electrical
         parameters already correct), before cosmetic settings.
      3. Ordinary settings (work mode, current, microstep, direction,
         en_active_level, auto_screen_off, stall_protection,
         interpolation, key_lock, home params, uart_respond_all/
         uart_active_report, group_addr).
      4. zero_axis - after homing/position-related settings are in place.
      5. modbus_addr (UART address) - LAST of the "safe" settings, since
         changing it means every subsequent call in *this run* must use
         the new address. Updated on motor.addr immediately after success.
      6. uart_baud_index - LAST of everything. DANGER: changing the
         driver's UART baud rate will break ALL further communication
         over this connection until your RS485-to-Ethernet converter's
         channel baud rate is manually updated to match. This script
         does not attempt anything else after this succeeds.
      7. restart - very last, if requested: reboots the driver, which
         will drop this connection. Also gated by confirm_dangerous.

    restore_defaults and restart both require confirm_dangerous=True
    (the CLI's --confirm-dangerous flag) or they're skipped with a
    warning rather than silently applied - too easy to nuke a working
    setup by leaving a stale `true` in a config file otherwise.
    """
    applied, skipped, failed = [], [], []

    def _try(label, fn):
        try:
            fn()
            applied.append(label)
        except MksServoError as e:
            failed.append((label, str(e)))

    def _dangerous_gate(field_name, value):
        """Returns True if this dangerous action should proceed."""
        if value is not True:
            skipped.append(field_name)
            return False
        if not confirm_dangerous:
            failed.append((field_name, "requires --confirm-dangerous on the "
                                        "command line - refusing to apply "
                                        "silently"))
            return False
        return True

    # 1. restore_defaults FIRST
    if _dangerous_gate("restore_defaults", profile.restore_defaults):
        _try("restore_defaults", motor.restore_defaults)

    # 2. calibrate
    if profile.calibrate is True:
        _try("calibrate (motor must be UNLOADED)", motor.calibrate)
    else:
        skipped.append("calibrate")

    # 3. ordinary settings
    if profile.work_mode is not None:
        try:
            mode = WorkMode[profile.work_mode]
            _try(f"work_mode={profile.work_mode}", lambda: motor.set_work_mode(mode))
        except KeyError:
            failed.append((f"work_mode={profile.work_mode}",
                            f"not a valid WorkMode name, options: {[m.name for m in WorkMode]}"))
    else:
        skipped.append("work_mode")

    if profile.current_ma is not None:
        _try(f"current_ma={profile.current_ma}", lambda: motor.set_current(profile.current_ma))
    else:
        skipped.append("current_ma")

    if profile.microstep is not None:
        _try(f"microstep={profile.microstep}", lambda: motor.set_microstep(profile.microstep))
    else:
        skipped.append("microstep")

    if profile.en_active_level is not None:
        try:
            level = EnActiveLevel[profile.en_active_level]
            _try(f"en_active_level={profile.en_active_level}",
                 lambda: motor.set_en_active_level(level))
        except KeyError:
            failed.append((f"en_active_level={profile.en_active_level}",
                            f"options: {[l.name for l in EnActiveLevel]}"))
    else:
        skipped.append("en_active_level")

    if profile.direction is not None:
        try:
            d = Direction[profile.direction]
            _try(f"direction={profile.direction}", lambda: motor.set_direction(d))
        except KeyError:
            failed.append((f"direction={profile.direction}", "must be 'CW' or 'CCW'"))
    else:
        skipped.append("direction")

    if profile.auto_screen_off is not None:
        _try(f"auto_screen_off={profile.auto_screen_off}",
             lambda: motor.set_auto_screen_off(profile.auto_screen_off))
    else:
        skipped.append("auto_screen_off")

    if profile.stall_protection is not None:
        _try(f"stall_protection={profile.stall_protection}",
             lambda: motor.set_stall_protection(profile.stall_protection))
    else:
        skipped.append("stall_protection")

    if profile.interpolation is not None:
        _try(f"interpolation={profile.interpolation}",
             lambda: motor.set_interpolation(profile.interpolation))
    else:
        skipped.append("interpolation")

    if profile.key_lock is not None:
        _try(f"key_lock={profile.key_lock}", lambda: motor.set_key_lock(profile.key_lock))
    else:
        skipped.append("key_lock")

    if (profile.home_trigger_level is not None
            and profile.home_dir is not None
            and profile.home_speed_rpm is not None
            and profile.home_end_limit_enable is not None):
        try:
            hd = Direction[profile.home_dir]
            _try("home_params",
                 lambda: motor.set_home_params(profile.home_trigger_level, hd,
                                                profile.home_speed_rpm,
                                                profile.home_end_limit_enable))
        except KeyError:
            failed.append(("home_params", "home_dir must be 'CW' or 'CCW'"))
    elif any(v is not None for v in
             (profile.home_trigger_level, profile.home_dir, profile.home_speed_rpm,
              profile.home_end_limit_enable)):
        failed.append(("home_params", "home_trigger_level, home_dir, home_speed_rpm, "
                                       "and home_end_limit_enable must ALL be set "
                                       "together, or all left null"))
    else:
        skipped.append("home_params")

    if profile.uart_respond_all is not None and profile.uart_active_report is not None:
        _try(f"uart_response(respond_all={profile.uart_respond_all}, "
             f"active_report={profile.uart_active_report})",
             lambda: motor.set_uart_response(profile.uart_respond_all, profile.uart_active_report))
    elif profile.uart_respond_all is not None or profile.uart_active_report is not None:
        failed.append(("uart_response", "uart_respond_all and uart_active_report "
                                         "must BOTH be set together, or both left null"))
    else:
        skipped.append("uart_response")

    if profile.group_addr is not None:
        _try(f"group_addr={profile.group_addr}", lambda: motor.set_group_addr(profile.group_addr))
    else:
        skipped.append("group_addr")

    # 4. zero_axis
    if profile.zero_axis is True:
        _try("zero_axis", motor.set_current_axis_zero)
    else:
        skipped.append("zero_axis")

    # 5. modbus_addr (UART address) - applied late, updates motor.addr on success
    if profile.modbus_addr is not None:
        old_addr = motor.addr
        _try(f"uart_addr {old_addr}->{profile.modbus_addr}",
             lambda: motor.set_uart_addr(profile.modbus_addr))
    else:
        skipped.append("modbus_addr")

    # 6. uart_baud_index - DANGEROUS, applied last of the "settings"
    if profile.uart_baud_index is not None:
        _try(f"uart_baud_index={profile.uart_baud_index} "
             f"(!! update your RS485-to-ETH converter's baud to match, "
             f"or all further communication will fail !!)",
             lambda: motor.set_uart_baud(profile.uart_baud_index))
    else:
        skipped.append("uart_baud_index")

    # 7. restart - absolute last, gated
    if _dangerous_gate("restart", profile.restart):
        _try("restart (driver rebooting - this connection will drop)", motor.restart)

    return applied, skipped, failed


def cmd_write(args):
    profile = load_config(args.input)

    with MksServoTCP(args.host, port=args.port, addr=args.addr,
                      debug=args.debug) as motor:
        applied, skipped, failed = write_config(motor, profile, confirm_dangerous=args.confirm_dangerous)

    print(f"\nApplied ({len(applied)}):")
    for a in applied:
        print(f"  OK  {a}")

    if skipped:
        print(f"\nSkipped - not set in {args.input} ({len(skipped)}):")
        for s in skipped:
            print(f"  --  {s}")

    if failed:
        print(f"\nFAILED ({len(failed)}):")
        for label, err in failed:
            print(f"  !!  {label}: {err}")
        sys.exit(1)


# ===========================================================================
# CLI
# ===========================================================================

def build_parser():
    parser = argparse.ArgumentParser(description=__doc__,
                                      formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)

    def add_conn_args(p):
        p.add_argument("--host", default="192.168.1.200", help="RS485-to-Ethernet converter IP")
        p.add_argument("--port", type=int, default=4196,
                        help="converter's TCP port for this RS485 channel "
                             "(transparent mode - confirm on the converter's "
                             "own config page, this default is illustrative)")
        p.add_argument("--addr", type=int, default=1, help="driver's UART address")
        p.add_argument("--debug", action="store_true", help="print raw FA/FB traffic")

    p_read = sub.add_parser("read", help="snapshot live status to a JSON file")
    add_conn_args(p_read)
    p_read.add_argument("--output", help="output file path (default: timestamped)")
    p_read.set_defaults(func=cmd_read)

    p_write = sub.add_parser("write", help="push a config profile to the driver")
    add_conn_args(p_write)
    p_write.add_argument("--input", required=True, help="config JSON file to apply")
    p_write.add_argument("--confirm-dangerous", action="store_true",
                          help="required to actually apply restore_defaults or "
                               "restart if set true in the config file - "
                               "otherwise they're skipped with a warning")
    p_write.set_defaults(func=cmd_write)

    p_template = sub.add_parser("template", help="generate a starter config JSON file")
    p_template.add_argument("--output", help="output file path (default: config_template.json)")
    p_template.set_defaults(func=cmd_template)

    return parser


def main():
    parser = build_parser()
    args = parser.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()