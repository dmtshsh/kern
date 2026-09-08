"""
mks_servo42d_57d.py
--------------------
Full Python driver for the Makerbase MKS SERVO42D / SERVO57D closed-loop
stepper driver, communicating over UART / RS485 (function-code protocol,
NOT Modbus-RTU).

Reference: https://github.com/makerbase-motor/MKS-SERVO42D-57D
           (official "RS485 User Manual", function-code frame format)

-------------------------------------------------------------------------
FRAME FORMAT
-------------------------------------------------------------------------
Downlink (PC -> driver):   FA  addr  func  [data...]  CRC
Uplink   (driver -> PC):   FB  addr  func  [data...]  CRC

CRC is a simple 8-bit checksum: CRC = (sum of all preceding bytes) & 0xFF

-------------------------------------------------------------------------
IMPORTANT - READ BEFORE USE
-------------------------------------------------------------------------
1. The driver's physical "Mode" menu MUST be set to a serial control mode
   (Menu -> Mode -> SR_vFOC, or CR_RS485 on older firmware) or every
   command below will return a "fail" status. Pulse-interface modes
   (CR_OPEN / CR_CLOSE / CR_vFOC) ignore these serial motion commands.
2. Menu -> UartBaud and Menu -> UartAddr must match `baudrate`/`addr`
   below (defaults: 38400 baud, address 1).
3. Some firmware builds require you to explicitly enable() the motor over
   serial (0xF3) before it will respond to motion commands, even if the
   physical EN pin is already active.
4. Makerbase has changed byte-level details of some commands between
   firmware/manual revisions (their own changelog notes "speed and
   acceleration have been redefined" in later versions). The commands
   below follow the mainstream V1.0.4/V1.0.5-era manual. If a command
   in this file doesn't behave as expected on your unit, check the
   manual that matches your specific firmware version at:
   https://github.com/makerbase-motor/MKS-SERVO42D-57D/tree/master/User%20Manual

Requires: pyserial   (pip install pyserial)
"""

from __future__ import annotations

import struct
import time
from dataclasses import dataclass
from enum import IntEnum
from typing import Optional

import serial


# ===========================================================================
# Enums / constants
# ===========================================================================

class WorkMode(IntEnum):
    CR_OPEN = 0    # pulse interface, open loop
    CR_CLOSE = 1   # pulse interface, closed loop
    CR_vFOC = 2    # pulse interface, FOC closed loop
    SR_OPEN = 3    # serial interface, open loop
    SR_CLOSE = 4   # serial interface, closed loop
    SR_vFOC = 5    # serial interface, FOC closed loop (recommended for UART control)


class Direction(IntEnum):
    CW = 0
    CCW = 1

class States(IntEnum):
    STATE_STOP = 1
    STATE_SPEED_UP = 2
    STATE_SEED_DOWN = 3
    STATE_FULL_SPEED = 4
    STATE_HOMING = 5


class EnActiveLevel(IntEnum):
    LOW = 0
    HIGH = 1
    ALWAYS_ACTIVE = 2


class MksServoError(Exception):
    """Raised when the driver returns a fail status or a malformed reply."""


@dataclass
class EncoderReading:
    carry: int   # number of full revolutions (signed)
    value: int   # 0-0x3FFF position within the current revolution


# ===========================================================================
# Driver class
# ===========================================================================

class MksServo:
    """
    Driver for one MKS SERVO42D/57D on a UART/RS485 bus.

    Example
    -------
        with MksServo("/dev/ttyUSB0", addr=1) as motor:
            motor.enable_motor(True)
            motor.run_speed(rpm=300, direction=Direction.CW, acc=2)
            time.sleep(2)
            motor.stop_speed(acc=2)
    """

    HEAD_TX = 0xFA
    HEAD_RX = 0xFB

    def __init__(self, port: str, addr: int = 1, baudrate: int = 38400,
                 timeout: float = 0.5, debug: bool = False,
                 retries: int = 3, retry_delay: float = 0.05):
        self.addr = addr
        self.debug = debug
        self.retries = retries
        self.retry_delay = retry_delay
        self.ser = serial.Serial(port, baudrate=baudrate, timeout=timeout)

    # -- context manager -----------------------------------------------
    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()

    # ===================================================================
    # Low-level frame helpers
    # ===================================================================

    @staticmethod
    def _checksum(data: bytes) -> int:
        return sum(data) & 0xFF

    def _transport_write_read(self, frame: bytes, reply_len: int, delay: float) -> bytes:
        """
        Transport-specific: send `frame` and read up to `reply_len` bytes
        back. Serial implementation here; MksServoTCP (in
        mks_servo42d_57d_tcp.py) overrides this (and _transport_read_more)
        to use a socket instead - the retry/resync algorithm in _send()
        below is shared unchanged by both transports.
        """
        self.ser.reset_input_buffer()
        self.ser.write(frame)
        time.sleep(delay)
        return self.ser.read(reply_len)

    def _transport_read_more(self, reply_len: int) -> bytes:
        """Transport-specific: read more bytes WITHOUT sending anything -
        used by _send()'s resync step when a stale frame is detected."""
        return self.ser.read(reply_len)

    def _send(self, payload: bytes, reply_len: int = 8, delay: float = 0.02) -> bytes:
        """
        payload = [addr, func, ...data]  (head byte and CRC added here)
        Returns the raw reply bytes (may be shorter than reply_len).

        Retries the whole write+read cycle up to self.retries times if the
        reply is missing, short, or malformed (wrong head byte / wrong
        function code echoed back) - RS485 buses occasionally drop or
        corrupt a byte, and a fresh retry is usually enough to recover.
        Only returns a bad reply (letting _check_reply raise, as before)
        after every retry has been exhausted, so a genuinely dead link or
        a real driver-side fail status still surfaces as an error.

        Special-cased: a reply with the correct head byte but the WRONG
        function code is treated as a stale/unsolicited frame (some
        firmware pushes status updates for an in-progress move without
        being asked - see set_uart_response()/0x8C to disable that at the
        source) rather than corruption. In that case we try reading
        further bytes WITHOUT resending first, since the actual answer to
        THIS command may already be queued right behind the stale one -
        avoiding a redundant resend of what could be a motion command.
        """
        func = payload[1]
        last_reply = b""

        for attempt in range(1, self.retries + 1):
            frame = bytes([self.HEAD_TX]) + payload
            frame += bytes([self._checksum(frame)])
            if self.debug:
                print(f"TX (attempt {attempt}/{self.retries}):", frame.hex(" "))

            reply = self._transport_write_read(frame, reply_len, delay)
            if self.debug:
                print("RX:", reply.hex(" ") if reply else "(empty)")

            last_reply = reply
            if len(reply) >= reply_len and reply[0] == self.HEAD_RX and reply[2] == func:
                return reply

            # Correctly-headed frame, wrong function code -> likely a
            # stale/unsolicited push queued ahead of our real reply.
            # Try reading on WITHOUT resending before falling back to a
            # full retry - the real answer may already be right behind it.
            if len(reply) >= 3 and reply[0] == self.HEAD_RX and reply[2] != func:
                if self.debug:
                    print(f"  got reply for func 0x{reply[2]:02X}, expected "
                          f"0x{func:02X} - likely a stale push, reading on "
                          f"without resending...")
                more = self._transport_read_more(reply_len)
                if self.debug:
                    print("RX (resync):", more.hex(" ") if more else "(empty)")
                if len(more) >= reply_len and more[0] == self.HEAD_RX and more[2] == func:
                    return more
                last_reply = more or reply

            if attempt < self.retries:
                if self.debug:
                    print(f"  bad/short reply, retrying ({attempt}/{self.retries})...")
                time.sleep(self.retry_delay)

        # All retries exhausted - return the last attempt's reply so
        # _check_reply() still raises its normal, informative error.
        return last_reply

    @staticmethod
    def _check_reply(reply: bytes, func: int, min_len: int = 3):
        if len(reply) < min_len:
            raise MksServoError(f"No/short reply for func 0x{func:02X}: {reply.hex()}")
        if reply[0] != MksServo.HEAD_RX or reply[2] != func:
            raise MksServoError(f"Malformed reply for func 0x{func:02X}: {reply.hex()}")

    # ===================================================================
    # STATUS READS
    # ===================================================================

    def read_encoder(self) -> EncoderReading:
        """0x30 - read the (carry, value) encoder pair."""
        reply = self._send(bytes([self.addr, 0x30]), reply_len=9)
        self._check_reply(reply, 0x30, min_len=9)
        carry, value = struct.unpack(">iH", reply[3:9])
        return EncoderReading(carry=carry, value=value)

    def read_shaft_angle_degrees(self) -> float:
        """
        The single-turn shaft angle in degrees (0.0-360.0), matching the
        driver's own on-screen "0.0°" readout - documented as "calculated
        based on the read encoder value, dynamically displayed". Uses only
        the `value` field (0-16383 within the current revolution), NOT
        `carry` - so it wraps every revolution rather than accumulating.
        Use read_encoder_addition() (0x31) instead for a continuously
        accumulating multi-turn coordinate.
        """
        reading = self.read_encoder()
        return (reading.value / 16384.0) * 360.0
    
    def read_encoder_fix(self) -> EncoderReading:
        """0x30 - read the (carry, value) encoder pair."""
        reply = self._send(bytes([self.addr, 0x30]), reply_len=9)
        self._check_reply(reply, 0x30, min_len=9)
        carry, value = struct.unpack(">iH", reply[3:9])
        return  (carry * 16384 + value)


    def read_encoder_addition(self) -> int:
        """
        0x31 - read the accumulated encoder "axis" value (signed 32-bit).
        This is the coordinate used by run_position_relative_axis() (0xF4) and
        run_position_absolute_axis() (0xF5) - NOT the same as read_encoder()'s
        (carry, value) pair from 0x30.
        """
        reply = self._send(bytes([self.addr, 0x31]), reply_len=8)
        self._check_reply(reply, 0x31, min_len=8)
        return struct.unpack(">i", reply[3:7])[0]

    def read_pulses(self) -> int:
        """0x33 - read the accumulated number of pulses received (signed 32-bit)."""
        reply = self._send(bytes([self.addr, 0x33]), reply_len=8)
        self._check_reply(reply, 0x33, min_len=8)
        return struct.unpack(">i", reply[3:7])[0]

    def read_speed(self) -> int:
        """0x32 - read the real-time motor speed in RPM (signed)."""
        reply = self._send(bytes([self.addr, 0x32]), reply_len=6)
        self._check_reply(reply, 0x32, min_len=6)
        return struct.unpack(">h", reply[3:5])[0]

    def read_io_status(self) -> int:
        """0x34 - read the raw IO port status byte."""
        reply = self._send(bytes([self.addr, 0x34]), reply_len=5)
        self._check_reply(reply, 0x34, min_len=5)
        return reply[3]

    def read_angle_error(self) -> int:
        """0x39 - read the shaft angle error (encoder ticks)."""
        reply = self._send(bytes([self.addr, 0x39]), reply_len=6)
        self._check_reply(reply, 0x39, min_len=6)
        return struct.unpack(">h", reply[3:5])[0]

    def read_en_pin_status(self) -> bool:
        """0x3A - read the current EN pin logical state."""
        reply = self._send(bytes([self.addr, 0x3A]), reply_len=5)
        self._check_reply(reply, 0x3A, min_len=5)
        return bool(reply[3])

    def read_zero_status(self) -> int:
        """0x3B - read go-to-zero-on-power-up status (0=fail,1=stop,2=going,3=complete)."""
        reply = self._send(bytes([self.addr, 0x3B]), reply_len=5)
        self._check_reply(reply, 0x3B, min_len=5)
        return reply[3]

    def release_stall_protection(self) -> bool:
        """0x3D - clear a locked-rotor / stall protection trip."""
        reply = self._send(bytes([self.addr, 0x3D]), reply_len=5)
        self._check_reply(reply, 0x3D, min_len=5)
        return bool(reply[3])

    def read_stall_protection_state(self) -> bool:
        """0x3E - True if stall/locked-rotor protection is currently tripped."""
        reply = self._send(bytes([self.addr, 0x3E]), reply_len=5)
        self._check_reply(reply, 0x3E, min_len=5)
        return bool(reply[3])

    def query_motor_status(self) -> int:
        """
        0xF1 - query current motion status.
        0 = query fail, 1 = stopped, 2 = accelerating,
        3 = decelerating, 4 = running at full speed, 5 = homing, 6 = calibrating
        (exact enumeration can vary slightly by firmware).
        """
        reply = self._send(bytes([self.addr, 0xF1]), reply_len=5)
        self._check_reply(reply, 0xF1, min_len=5)
        return reply[3]

    # ===================================================================
    # PARAMETER CONFIGURATION  (write settings into the driver)
    # ===================================================================

    def calibrate(self) -> int:
        """
        0x80 - run encoder calibration. Motor must be UNLOADED.
        Returns status byte (0=fail, 1=start/success, 2=calibrating -
        poll query_motor_status() or re-read to know when it's done).
        """
        reply = self._send(bytes([self.addr, 0x80, 0x00]), reply_len=5)
        self._check_reply(reply, 0x80, min_len=5)
        return reply[3]

    def set_work_mode(self, mode: WorkMode) -> bool:
        """0x82 - set working mode (use SR_vFOC for full serial control)."""
        reply = self._send(bytes([self.addr, 0x82, int(mode)]), reply_len=5)
        self._check_reply(reply, 0x82, min_len=5)
        return bool(reply[3])

    def set_current(self, milliamps: int) -> bool:
        """0x83 - set working current in mA (check your motor's rated current!)."""
        data = struct.pack(">H", milliamps)
        reply = self._send(bytes([self.addr, 0x83]) + data, reply_len=5)
        self._check_reply(reply, 0x83, min_len=5)
        return bool(reply[3])

    def set_microstep(self, microstep: int) -> bool:
        """0x84 - set microstep subdivision (e.g. 16, 32, 64, 128, 256)."""
        reply = self._send(bytes([self.addr, 0x84, microstep & 0xFF]), reply_len=5)
        self._check_reply(reply, 0x84, min_len=5)
        return bool(reply[3])

    def set_en_active_level(self, level: EnActiveLevel) -> bool:
        """0x85 - set EN pin active level (or make it always active)."""
        reply = self._send(bytes([self.addr, 0x85, int(level)]), reply_len=5)
        self._check_reply(reply, 0x85, min_len=5)
        return bool(reply[3])

    def set_direction(self, direction: Direction) -> bool:
        """0x86 - set the motor's default rotation direction."""
        reply = self._send(bytes([self.addr, 0x86, int(direction)]), reply_len=5)
        self._check_reply(reply, 0x86, min_len=5)
        return bool(reply[3])

    def set_auto_screen_off(self, enable: bool) -> bool:
        """0x87 - auto turn off the OLED screen after inactivity."""
        reply = self._send(bytes([self.addr, 0x87, 0x01 if enable else 0x00]), reply_len=5)
        self._check_reply(reply, 0x87, min_len=5)
        return bool(reply[3])

    def set_stall_protection(self, enable: bool) -> bool:
        """0x88 - enable/disable locked-rotor (stall) protection."""
        reply = self._send(bytes([self.addr, 0x88, 0x01 if enable else 0x00]), reply_len=5)
        self._check_reply(reply, 0x88, min_len=5)
        return bool(reply[3])

    def set_interpolation(self, enable: bool) -> bool:
        """0x89 - enable/disable microstep interpolation."""
        reply = self._send(bytes([self.addr, 0x89, 0x01 if enable else 0x00]), reply_len=5)
        self._check_reply(reply, 0x89, min_len=5)
        return bool(reply[3])

    def set_uart_baud(self, baud_index: int) -> bool:
        """0x8A - change the UART baud rate index (consult manual's table)."""
        reply = self._send(bytes([self.addr, 0x8A, baud_index & 0xFF]), reply_len=5)
        self._check_reply(reply, 0x8A, min_len=5)
        return bool(reply[3])

    def set_uart_addr(self, new_addr: int) -> bool:
        """0x8B - change this driver's slave address. Updates self.addr on success."""
        reply = self._send(bytes([self.addr, 0x8B, new_addr & 0xFF]), reply_len=5)
        self._check_reply(reply, 0x8B, min_len=5)
        ok = bool(reply[3])
        if ok:
            self.addr = new_addr
        return ok

    def set_uart_response(self, respond_all: bool, active_report: bool) -> bool:
        """0x8C - "UartRSP": whether the slave replies to every command, and
        whether it actively pushes status without being asked."""
        data = (0x01 if respond_all else 0x00, 0x01 if active_report else 0x00)
        reply = self._send(bytes([self.addr, 0x8C, *data]), reply_len=5)
        self._check_reply(reply, 0x8C, min_len=5)
        return bool(reply[3])

    def set_group_addr(self, group_addr: int) -> bool:
        """0x8D - assign this driver to a group address for broadcast commands."""
        reply = self._send(bytes([self.addr, 0x8D, group_addr & 0xFF]), reply_len=5)
        self._check_reply(reply, 0x8D, min_len=5)
        return bool(reply[3])

    def set_key_lock(self, locked: bool) -> bool:
        """0x8F - lock/unlock the driver's front panel buttons."""
        reply = self._send(bytes([self.addr, 0x8F, 0x01 if locked else 0x00]), reply_len=5)
        self._check_reply(reply, 0x8F, min_len=5)
        return bool(reply[3])

    def restore_defaults(self) -> bool:
        """0x3F - factory reset. The motor will need recalibration afterwards."""
        reply = self._send(bytes([self.addr, 0x3F]), reply_len=5)
        self._check_reply(reply, 0x3F, min_len=5)
        return bool(reply[3])

    def restart(self) -> bool:
        """0x41 - restart the driver's firmware."""
        reply = self._send(bytes([self.addr, 0x41]), reply_len=5)
        self._check_reply(reply, 0x41, min_len=5)
        return bool(reply[3])

    # ===================================================================
    # HOMING
    # ===================================================================

    def set_home_params(self, trigger_level: int, home_dir: Direction,
                         home_speed_rpm: int, end_limit_enable: bool) -> bool:
        """
        0x90 - configure homing behaviour before calling go_home().
        trigger_level: 0/1 logic level that the home/limit switch triggers on.
        """
        data = bytes([
            trigger_level & 0xFF,
            int(home_dir),
            *struct.pack(">H", home_speed_rpm),
            0x01 if end_limit_enable else 0x00,
        ])
        reply = self._send(bytes([self.addr, 0x90]) + data, reply_len=5)
        self._check_reply(reply, 0x90, min_len=5)
        return bool(reply[3])

    def go_home(self) -> int:
        """0x91 - execute homing. Returns 0=fail, 1=homing started, 2=success."""
        reply = self._send(bytes([self.addr, 0x91]), reply_len=5)
        self._check_reply(reply, 0x91, min_len=5)
        return reply[3]

    def set_current_axis_zero(self) -> bool:
        """0x92 - define the current shaft position as the zero point."""
        reply = self._send(bytes([self.addr, 0x92]), reply_len=5)
        self._check_reply(reply, 0x92, min_len=5)
        return bool(reply[3])

    # ===================================================================
    # MOTION CONTROL
    # ===================================================================

    def enable_motor(self, enable: bool = True) -> bool:
        """0xF3 - enable/disable the motor over serial (decoupled from EN pin)."""
        reply = self._send(bytes([self.addr, 0xF3, 0x01 if enable else 0x00]), reply_len=5)
        self._check_reply(reply, 0xF3, min_len=5)
        return bool(reply[3])

    def emergency_stop(self) -> bool:
        """0xF7 - immediate stop, works regardless of the current mode."""
        reply = self._send(bytes([self.addr, 0xF7]), reply_len=5)
        self._check_reply(reply, 0xF7, min_len=5)
        return bool(reply[3])

    # -- speed mode ------------------------------------------------------

    def run_speed(self, rpm: int, direction: Direction = Direction.CW, acc: int = 2) -> int:
        """
        0xF6 - run continuously in speed mode.
        rpm: 0-3000 (actual achievable max depends on model/microstep)
        acc: 0-255 (0 = jump straight to target speed, higher = slower ramp)
        Returns status byte: 0=run fail, 1=run success.
        """
        rpm = max(0, min(3000, int(rpm)))
        acc = max(0, min(255, int(acc)))
        byte4 = (int(direction) << 7) | ((rpm >> 8) & 0x0F)
        byte5 = rpm & 0xFF
        reply = self._send(bytes([self.addr, 0xF6, byte4, byte5, acc]), reply_len=5)
        self._check_reply(reply, 0xF6, min_len=5)
        return reply[3]

    def stop_speed(self, acc: int = 2) -> int:
        """
        0xF6 with speed=0 - stop the motor from speed mode.
        acc=0 -> immediate stop (avoid above ~1000 RPM); acc>0 -> decelerate.
        Returns status: 0=fail, 1=starting to stop, 2=stopped.
        """
        acc = max(0, min(255, int(acc)))
        reply = self._send(bytes([self.addr, 0xF6, 0x00, 0x00, acc]), reply_len=5)
        self._check_reply(reply, 0xF6, min_len=5)
        return reply[3]

    def save_speed_state(self, save: bool) -> bool:
        """0xFF - save (or clear) current speed-mode parameters to power-on defaults."""
        reply = self._send(bytes([self.addr, 0xFF, 0x01 if save else 0x00]), reply_len=5)
        self._check_reply(reply, 0xFF, min_len=5)
        return bool(reply[3])

    # -- relative position move ------------------------------------------

    def run_position_relative(self, pulses: int, rpm: int,
                               direction: Direction = Direction.CW, acc: int = 2) -> int:
        """
        0xFD - move a relative number of pulses at a given speed.
        pulses: unsigned pulse count to move (direction sets the sign)
        Returns status: 0=fail, 1=starting, 2=complete (poll again to see 2).
        """
        rpm = max(0, min(3000, int(rpm)))
        acc = max(0, min(255, int(acc)))
        byte4 = (int(direction) << 7) | ((rpm >> 8) & 0x0F)
        byte5 = rpm & 0xFF
        pulse_bytes = pulses.to_bytes(3, byteorder="big", signed=False)
        reply = self._send(bytes([self.addr, 0xFD, byte4, byte5, acc]) + pulse_bytes, reply_len=5)
        self._check_reply(reply, 0xFD, min_len=5)
        return reply[3]

    def stop_position_relative(self) -> int:
        """0xFD with acc=0, speed=0, pulses=0 - stop a relative-move in progress."""
        reply = self._send(bytes([self.addr, 0xFD, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]), reply_len=5)
        self._check_reply(reply, 0xFD, min_len=5)
        return reply[3]

    # -- relative position move BY COORDINATE / AXIS VALUE (0xF4) --------
    # Manual section 9.3 "Relative motion according to coordinate values".
    # Distinct from run_position_relative() above (0xFD, pulse-based):
    # this one moves by a delta in the driver's internal "axis" coordinate
    # (the accumulated encoder value read by read_encoder_addition() / 0x31),
    # and direction is implied by the SIGN of axis_delta, not a direction bit.

    def run_position_relative_axis(self, axis_delta: int, rpm: int, acc: int = 2) -> int:
        """
        0xF4 - move by a relative amount in encoder "axis" coordinate units.
        axis_delta: signed int32, e.g. +0x4000 or -0x4000 (positive/negative
                    sets the direction - there's no separate direction bit).
        rpm: 0-3000
        acc: 0-255
        Returns status: 0=fail, 1=starting, 2=complete, 3=end-limit stopped
        (poll again after issuing the command to see 2 or 3).

        Example (from the manual): if current axis (read via
        read_encoder_addition()) is 0x8000, run_position_relative_axis(0x4000,
        600, 2) moves it to 0xC000; run_position_relative_axis(-0x4000, 600, 2)
        moves it to 0x4000.
        """
        rpm = max(0, min(3000, int(rpm))) & 0xFFFF
        acc = max(0, min(255, int(acc)))
        data = struct.pack(">H", rpm) + bytes([acc]) + struct.pack(">i", axis_delta)
        reply = self._send(bytes([self.addr, 0xF4]) + data, reply_len=5)
        self._check_reply(reply, 0xF4, min_len=5)
        return reply[3]

    def stop_position_relative_axis(self, acc: int = 2) -> int:
        """
        0xF4 with speed=0, axis=0 - stop a coordinate-relative move in progress.
        acc=0 -> immediate stop (avoid above ~1000 RPM); acc>0 -> decelerate.
        Returns status: 0=fail, 1=stopping, 2=stopped, 3=end-limit stopped.
        """
        acc = max(0, min(255, int(acc)))
        data = struct.pack(">H", 0) + bytes([acc]) + struct.pack(">i", 0)
        reply = self._send(bytes([self.addr, 0xF4]) + data, reply_len=5)
        self._check_reply(reply, 0xF4, min_len=5)
        return reply[3]

    # -- absolute position move BY COORDINATE / AXIS VALUE (0xF5) --------
    # Manual "Position mode3/4: absolute motion by axis". Confirmed against
    # the manual's worked examples:
    #   FA 01 F5 02 58 02 00 00 40 00 8C
    #     speed=0x0258(600RPM) acc=2 absAxis=+0x4000 -> moves TO axis 0x4000
    #     (regardless of where the shaft currently is)
    #   FA 01 F5 02 58 02 FF FF C0 00 0A
    #     absAxis=-0x4000 -> moves TO axis -0x4000
    # Byte layout: FA addr F5 speedHi speedLo acc absAxis(int32) CRC
    # Axis units are the same "encoder addition" coordinate read by
    # read_encoder_addition() (0x31) - not raw pulses.

    def run_position_absolute_axis(self, target_axis: int, rpm: int, acc: int = 2) -> int:
        """
        0xF5 - move TO an absolute axis coordinate (not a delta).
        target_axis : signed int32 target position in encoder axis units
                      (same units as read_encoder_addition() / 0x31).
        rpm         : speed, 0-3000
        acc         : acceleration, 0-255
        Returns status: 0=run fail, 1=run starting, 2=run complete,
        3=end-limit stopped (poll query_motor_status() or reissue to check).

        Note1: requires a serial working mode (SR_OPEN/SR_CLOSE/SR_vFOC).
        Note2: axis error is roughly +-15 counts; 64 microsteps recommended.
        Note3: newer firmware supports sending a new command to update
        speed/target while a previous 0xF5 move is still running.
        """
        rpm = max(0, min(3000, int(rpm))) & 0xFFFF
        acc = max(0, min(255, int(acc)))
        data = struct.pack(">H", rpm) + bytes([acc]) + struct.pack(">i", target_axis)
        reply = self._send(bytes([self.addr, 0xF5]) + data, reply_len=5)
        self._check_reply(reply, 0xF5, min_len=5)
        print("reply:", reply)
        return reply[3]

    def run_position_absolute_axis_retry(self, target_axis: int, rpm: int, acc: int = 2) -> int:
        """
        0xF5 - move TO an absolute axis coordinate (not a delta).
        target_axis : signed int32 target position in encoder axis units
                      (same units as read_encoder_addition() / 0x31).
        rpm         : speed, 0-3000
        acc         : acceleration, 0-255
        Returns status: 0=run fail, 1=run starting, 2=run complete,
        3=end-limit stopped (poll query_motor_status() or reissue to check).

        Note1: requires a serial working mode (SR_OPEN/SR_CLOSE/SR_vFOC).
        Note2: axis error is roughly +-15 counts; 64 microsteps recommended.
        Note3: newer firmware supports sending a new command to update
        speed/target while a previous 0xF5 move is still running.
        """
        rpm = max(0, min(3000, int(rpm))) & 0xFFFF
        acc = max(0, min(255, int(acc)))
        data = struct.pack(">H", rpm) + bytes([acc]) + struct.pack(">i", target_axis)
        reply = self._send(bytes([self.addr, 0xF5]) + data, reply_len=5)
        self._check_reply(reply, 0xF5, min_len=5)
        curr_state = self.query_motor_status()
        if ((curr_state == States.STATE_STOP) | (curr_state == States.STATE_SEED_DOWN) ) :
            reply = self._send(bytes([self.addr, 0xF5]) + data, reply_len=5)
        return reply[3]




    def stop_position_absolute_axis(self, acc: int = 2) -> int:
        """
        0xF5 with speed=0, axis=0 - stop a run_position_absolute_axis() move.
        acc=0 -> immediate stop (avoid above ~1000 RPM); acc>0 -> decelerate.
        Returns status: 0=fail, 1=stopping, 2=stopped, 3=end-limit stopped.
        """
        acc = max(0, min(255, int(acc)))
        data = struct.pack(">H", 0) + bytes([acc]) + struct.pack(">i", 0)
        reply = self._send(bytes([self.addr, 0xF5]) + data, reply_len=5)
        self._check_reply(reply, 0xF5, min_len=5)
        return reply[3]