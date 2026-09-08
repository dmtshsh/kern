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
    unchanged, INCLUDING the retry/stale-frame-resync logic in _send() -
    only the low-level transport (_transport_write_read/_transport_read_more,
    below) differs between this and the serial version.

    Example
    -------
        with MksServoTCP("192.168.1.200", port=4196, addr=1) as motor:
            motor.enable_motor(True)
            motor.run_position_absolute_axis(target_axis=100000, rpm=250, acc=2)
    """

    def __init__(self, host: str, port: int = 4196, addr: int = 1,
                 timeout: float = 0.5, debug: bool = False,
                 retries: int = 3, retry_delay: float = 0.05):
        # Deliberately NOT calling super().__init__() - it opens a local
        # serial port, which we don't want here. Set up the same instance
        # attributes MksServo's methods rely on (self.addr, self.debug,
        # self.retries, self.retry_delay) and open a TCP socket in
        # self.sock instead of self.ser. The retry/resync algorithm itself
        # lives once in MksServo._send() and is reused unchanged - only
        # _transport_write_read/_transport_read_more below are overridden
        # for the socket transport.
        self.addr = addr
        self.debug = debug
        self.retries = retries
        self.retry_delay = retry_delay
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

    def _transport_write_read(self, frame: bytes, reply_len: int, delay: float) -> bytes:
        """
        Same framing as the serial version (handled by the shared _send()
        in MksServo) - this just does the actual bytes-over-the-wire part
        via a TCP socket instead of a serial port.
        """
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
            return self.sock.recv(reply_len)
        except socket.timeout:
            return b""

    def _transport_read_more(self, reply_len: int) -> bytes:
        """Read more bytes WITHOUT sending anything - used by _send()'s
        resync step when a stale/unsolicited frame is detected."""
        try:
            return self.sock.recv(reply_len)
        except socket.timeout:
            return b""


# ===========================================================================
# Example usage
# ===========================================================================

# if __name__ == "__main__":
#     CONVERTER_IP = "192.168.1.200"   # your RS485-to-ETH converter's IP
#     CONVERTER_PORT = 4196            # confirm the actual configured port
#     ADDR = 1

#     with MksServoTCP(CONVERTER_IP, port=CONVERTER_PORT, addr=ADDR, debug=True) as motor:
#         print("Enabling motor...")
#         print("Enable OK:", motor.enable_motor(True))

#         print("Reading encoder:", motor.read_encoder())

#         print("Moving to absolute axis 100000 at 250 RPM...")
#         status = motor.run_position_absolute_axis(target_axis=100000, rpm=250, acc=2)
#         print("Move status:", status)
#         time.sleep(3)

#         print("Current axis:", motor.read_encoder_addition())

#         print("Disabling motor.")
#         motor.enable_motor(False)