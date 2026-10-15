"""Software model of the Arduino motor controller, speaking protocol v1 (docs/interfaces.md).

This is a TEST DOUBLE and the executable reference for the behaviour the real firmware
must implement. It drives no hardware: "outputs" are just two integers.

Safety behaviour modelled (and tested):
  * Boots DISARMED with zero outputs; only ARM enables motion; ARM itself never moves motors.
  * DRV is rejected unless ARMED. Any rejected DRV while ARMED also zeroes the outputs
    (fail-safe: a misbehaving host stops the robot rather than leaving the last command).
  * Command TTL: outputs go to zero when the last accepted DRV is older than its ttl
    (inclusive boundary). The MCU stays ARMED so a fresh DRV resumes motion.
  * Link timeout: if ARMED and no VALID frame arrives for ``link_timeout_ms`` the MCU zeroes
    the outputs, becomes DISARMED and latches fault LINK_TIMEOUT until the next ARM.
  * Time is evaluated BEFORE any newly received bytes are processed, so a late frame can
    never rescue an already-expired watchdog.
  * Corrupt/oversized frames never refresh the link watchdog or change state.
"""

from __future__ import annotations

import re
from typing import List, Optional, Tuple

from . import protocol
from .clock import Clock, Deadline
from .config import RobotConfig

_SEQ = re.compile(r"[0-9]{1,5}")
_INT = re.compile(r"-?[0-9]{1,5}")
_NO_ARG_COMMANDS = ("HELLO", "ARM", "DISARM", "STOP", "PING", "STAT")


def _seq_of(token: str) -> Optional[int]:
    if _SEQ.fullmatch(token):
        value = int(token)
        if value <= 0xFFFF:
            return value
    return None


class MockMotorController:
    def __init__(
        self,
        clock: Clock,
        *,
        link_timeout_ms: int = 500,
        max_duty_permille: int = 1000,
        caps: int = 0,
        fw_id: str = "mock-0.1",
    ) -> None:
        if link_timeout_ms <= 0:
            raise ValueError("link_timeout_ms must be > 0")
        if not 1 <= max_duty_permille <= 1000:
            raise ValueError("max_duty_permille must be within 1..1000")
        if not 0 <= caps <= 0xFF:
            raise ValueError("caps must fit in one byte")
        protocol.encode("HELLO", fw_id)  # validates the token
        self._clock = clock
        self._max_duty = max_duty_permille
        self._caps = caps
        self._fw_id = fw_id
        self._asm = protocol.FrameAssembler()
        self._link = Deadline(clock, link_timeout_ms)
        self._cmd: Optional[Deadline] = None
        self._boot_ms = clock.now_ms()
        self._left = 0
        self._right = 0
        self.state = protocol.STATE_DISARMED
        self.fault = protocol.FAULT_NONE
        # (time_ms, left, right) appended every time the outputs change; starts at boot.
        self.history: List[Tuple[int, int, int]] = [(self._boot_ms, 0, 0)]

    @classmethod
    def from_config(cls, cfg: RobotConfig, clock: Clock) -> "MockMotorController":
        caps = protocol.CAP_ENCODERS if cfg.encoders.present == "yes" else 0
        return cls(clock, link_timeout_ms=cfg.limits.link_timeout_ms,
                   max_duty_permille=cfg.limits.max_duty_permille, caps=caps)

    # ---- observable state ------------------------------------------------------------

    @property
    def outputs(self) -> Tuple[int, int]:
        """What the (virtual) MDD10A channels are being asked to do: (left, right) permille."""
        return (self._left, self._right)

    @property
    def ttl_remaining_ms(self) -> int:
        return self._cmd.remaining_ms() if self._cmd is not None else 0

    # ---- time and I/O ------------------------------------------------------------------

    def tick(self) -> None:
        """Evaluate watchdogs against the clock (the firmware's loop() does this)."""
        if self.state == protocol.STATE_ARMED:
            if self._link.expired():
                self._stop_outputs()
                self.state = protocol.STATE_DISARMED
                self.fault = protocol.FAULT_LINK_TIMEOUT
            elif self._cmd is not None and self._cmd.expired():
                self._stop_outputs()

    def feed(self, data: bytes) -> bytes:
        """Receive serial bytes; return the bytes the MCU would transmit in reply."""
        self.tick()  # time first, so late frames cannot revive an expired watchdog
        replies: List[bytes] = []
        for item in self._asm.feed(data):
            if isinstance(item, protocol.ProtocolError):
                replies.append(self._nak(0, protocol.REASON_BAD_FRAME))
                continue
            self._link.reset()  # only complete, checksum-valid frames count as link activity
            replies.append(self._handle(item))
        return b"".join(replies)

    # ---- internals ---------------------------------------------------------------------

    def _set_outputs(self, left: int, right: int) -> None:
        if (left, right) != (self._left, self._right):
            self._left, self._right = left, right
            self.history.append((self._clock.now_ms(), left, right))

    def _stop_outputs(self) -> None:
        self._set_outputs(0, 0)
        self._cmd = None

    def _ack(self, seq: int) -> bytes:
        return protocol.encode("ACK", seq, self.state)

    @staticmethod
    def _nak(seq: int, reason: str) -> bytes:
        return protocol.encode("NAK", seq, reason)

    def _handle(self, frame: protocol.Frame) -> bytes:
        if frame.cmd not in protocol.HOST_COMMANDS:
            seq = _seq_of(frame.fields[0]) if frame.fields else None
            return self._nak(seq or 0, protocol.REASON_UNKNOWN_CMD)
        seq = _seq_of(frame.fields[0]) if frame.fields else None
        if seq is None:
            return self._reject_if_drv(frame.cmd, 0, protocol.REASON_BAD_ARGS)
        args = frame.fields[1:]
        if frame.cmd in _NO_ARG_COMMANDS and args:
            return self._nak(seq, protocol.REASON_BAD_ARGS)

        if frame.cmd == "DRV":
            return self._drv(seq, args)
        if frame.cmd == "HELLO":
            return protocol.encode("HELLO", seq, protocol.PROTO_VERSION, self._fw_id,
                                   "%02X" % self._caps, self.state)
        if frame.cmd == "ARM":
            self._stop_outputs()  # arming never starts motion
            self.fault = protocol.FAULT_NONE
            self.state = protocol.STATE_ARMED
            return self._ack(seq)
        if frame.cmd == "DISARM":
            self._stop_outputs()
            self.state = protocol.STATE_DISARMED
            return self._ack(seq)
        if frame.cmd == "STOP":
            self._stop_outputs()
            return self._ack(seq)
        if frame.cmd == "STAT":
            uptime = (self._clock.now_ms() - self._boot_ms) & 0xFFFFFFFF
            return protocol.encode("STAT", seq, self.state, self._left, self._right,
                                   self.ttl_remaining_ms, self.fault, uptime)
        return self._ack(seq)  # PING

    def _reject_if_drv(self, cmd: str, seq: int, reason: str) -> bytes:
        if cmd == "DRV":
            return self._reject_drv(seq, reason)
        return self._nak(seq, reason)

    def _reject_drv(self, seq: int, reason: str) -> bytes:
        if self.state == protocol.STATE_ARMED:
            self._stop_outputs()  # fail-safe
        return self._nak(seq, reason)

    def _drv(self, seq: int, args: Tuple[str, ...]) -> bytes:
        if len(args) != 3 or not all(_INT.fullmatch(a) for a in args):
            return self._reject_drv(seq, protocol.REASON_BAD_ARGS)
        left, right, ttl = (int(a) for a in args)
        if self.state != protocol.STATE_ARMED:
            return self._nak(seq, protocol.REASON_NOT_ARMED)
        if abs(left) > self._max_duty or abs(right) > self._max_duty:
            return self._reject_drv(seq, protocol.REASON_RANGE)
        if not 1 <= ttl <= protocol.MAX_TTL_MS:
            return self._reject_drv(seq, protocol.REASON_TTL)
        self._set_outputs(left, right)
        self._cmd = Deadline(self._clock, ttl)
        return self._ack(seq)
