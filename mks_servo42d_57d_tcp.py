"""
mks_servo42d_57d_tcp.py
-------------------------
Same driver as mks_servo42d_57d.py (the native FA/FB "function-code"
protocol - NOT Modbus-RTU) but carried over a raw TCP socket instead of a
local serial port, for use with an RS485-to-Ethernet converter running in
TRANSPARENT/passthrough mode (e.g. Waveshare's "4-CH RS485 TO POE ETH (B)"
and similar serial-server hardware).

Why this exists: on this driver, F4/F5 (position-mode speed control) has
been found to be unreliable specifically when tunneled through a Modbus-
RTU-to-TCP gateway (see mks_servo42d_57d_modbus.py's notes and the related
GitHub issues #32/#34 on makerbase-motor/MKS-SERVO42D-57D), while the
native FA/FB protocol over a real serial port works correctly. Rather than
fighting the Modbus translation layer, this uses a device's TRANSPARENT
mode instead: raw bytes in over TCP come out raw on RS485 unchanged, and
vice versa - so the exact same, already-verified-working FA/FB protocol
is used, just carried over Ethernet.

-------------------------------------------------------------------------
CONVERTER SETUP (Waveshare 4-CH RS485 TO POE ETH (B) or similar)
-------------------------------------------------------------------------
1. Leave (or set) the device's Transfer Protocol to "Transparent" /
   default passthrough - NOT "Modbus TCP <--> RTU" (that mode switches
   the port to 502 and re-encodes everything as Modbus, which is exactly
   what we're avoiding here).
2. Configure the RS485 channel's baud rate/parity/stop-bits to match the
   driver's UartBaud setting (default 38400, 8N1).
3. Note the channel's configured TCP port. In the device's default
   transparent mode, Waveshare's own example uses port 4196 per channel -
   but this is configurable per-channel via the Vircom tool or the web
   config page (http://<device-ip>), so confirm yours there rather than
   assuming 4196.
4. Set the device to TCP Server mode (the default) so this script can
   connect to it as a TCP client - or TCP Client mode pointed at this
   script's host, if you'd rather it dial out (not covered here).
5. As with the direct-serial driver: the MKS driver's own Mode menu must
   still be set to a serial control mode (SR_vFOC recommended).

Requires: nothing beyond the standard library (uses socket directly) plus
mks_servo42d_57d.py in the same folder.
"""

from __future__ import annotations

import os
import socket
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from mks_servo42d_57d import (
    Direction,
    EnActiveLevel,
    EncoderReading,
    MksServo,
    MksServoError,
    WorkMode,
)


class MksServoTCP(MksServo):
    """
    Drop-in replacement for MksServo that talks over a TCP socket to a
    transparent-mode RS485-to-Ethernet converter, instead of a local
    serial port. Every method inherited from MksServo (run_speed,
    run_position_absolute_axis, read_encoder, homing, etc.) works
    unchanged - only the transport (this class) differs.

    Example
    -------
        with MksServoTCP("192.168.1.200", port=4196, addr=1) as motor:
            motor.enable_motor(True)
            motor.run_position_absolute_axis(target_axis=100000, rpm=250, acc=2)
    """

    def __init__(self, host: str, port: int = 4196, addr: int = 1,
                 timeout: float = 0.5, debug: bool = False):
        # Deliberately NOT calling super().__init__() - it opens a local
        # serial port, which we don't want here. Set up the same instance
        # attributes MksServo's other methods rely on (self.addr, self.debug)
        # and open a TCP socket in self.sock instead of self.ser.
        self.addr = addr
        self.debug = debug
        self.host = host
        self.port = port
        self.timeout = timeout
        self.sock = socket.create_connection((host, port), timeout=timeout)
        self.sock.settimeout(timeout)

    def close(self):
        if getattr(self, "sock", None) is not None:
            try:
                self.sock.close()
            except OSError:
                pass

    def _send(self, payload: bytes, reply_len: int = 8, delay: float = 0.02) -> bytes:
        """
        Same framing as the serial version (FA head, checksum, etc.) - just
        written to/read from a TCP socket instead of a serial port.
        """
        frame = bytes([self.HEAD_TX]) + payload
        frame += bytes([self._checksum(frame)])
        if self.debug:
            print("TX:", frame.hex(" "))

        # Drain anything stale sitting in the socket buffer before sending,
        # mirroring reset_input_buffer() on the serial version - a
        # transparent-mode converter can occasionally have a stray byte or
        # two left over from a previous exchange.
        self.sock.setblocking(False)
        try:
            while self.sock.recv(4096):
                pass
        except (BlockingIOError, OSError):
            pass
        self.sock.setblocking(True)
        self.sock.settimeout(self.timeout)

        self.sock.sendall(frame)
        time.sleep(delay)

        try:
            reply = self.sock.recv(reply_len)
        except socket.timeout:
            reply = b""

        if self.debug:
            print("RX:", reply.hex(" "))
        return reply


