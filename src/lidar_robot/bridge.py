"""Host-side (Raspberry Pi) client for the Arduino motor controller, protocol v1.

Safety rules implemented here (all unit-tested against the mock MCU):
  * ``safety.motors_enabled`` is false by default. While false, ``arm()`` and ``send_twist()``
    raise ``MotorsDisabledError`` BEFORE any byte is written. ``stop()`` and ``disarm()`` are
    always allowed.
  * A mock configuration is refused when the transport is not simulated, so invented
    dimensions can never drive a real robot.
  * Any failure on the drive path (bad value, missing hardware config, lost link, lost reply)
    triggers a best-effort STOP and then re-raises. ``stop()`` itself never raises.
  * DRV is never retried automatically (a stale retry could outlive its intent); STOP/DISARM
    are retried. The MCU's command TTL and link timeout remain the backstop when the link is
    gone and no STOP can be delivered.
  * Sequence numbers run 1..65535 and skip 0, which the MCU reserves for "unusable sequence".
"""

from __future__ import annotations

import re
from dataclasses import dataclass
from typing import Optional

from . import protocol
from .clock import Clock, Deadline
from .config import RobotConfig
from .kinematics import DriveCommand, Twist, twist_to_drive
from .transport import Transport, TransportError

_CAPS = re.compile(r"[0-9A-F]{2}")
_REPLY_COMMANDS = ("ACK", "NAK", "HELLO", "STAT")


class BridgeError(Exception):
    """Base class for all bridge failures."""


class MotorsDisabledError(BridgeError):
    """safety.motors_enabled is false; no motion command was sent."""


class ConfigSafetyError(BridgeError):
    """The configuration may not be used with this transport."""


class NotArmedError(BridgeError):
    """A motion command was attempted while the MCU is not known to be ARMED."""


class LinkError(BridgeError):
    """No valid reply (timeout, corrupt reply, MCU-reported bad frame)."""


class TransportDown(LinkError):
    """The byte channel itself failed; retrying is pointless."""


class ProtocolMismatch(BridgeError):
    """The MCU answered, but not in a way this host understands or trusts."""


class MotorRejected(BridgeError):
    """The MCU answered NAK."""

    def __init__(self, reason: str) -> None:
        super().__init__(f"MCU rejected the command: {reason}")
        self.reason = reason


@dataclass
class BridgeStats:
    tx_frames: int = 0
    rx_frames: int = 0
    timeouts: int = 0
    bad_frames: int = 0         # undecodable bytes received
    stale_replies: int = 0      # valid replies to some other sequence number
    bad_frame_naks: int = 0     # MCU said our frame arrived corrupted
    naks: int = 0


@dataclass(frozen=True)
class McuInfo:
    proto: int
    fw_id: str
    caps: int
    state: str

    @property
    def has_encoders(self) -> bool:
        return bool(self.caps & protocol.CAP_ENCODERS)


@dataclass(frozen=True)
class McuStatus:
    state: str
    left_permille: int
    right_permille: int
    ttl_remaining_ms: int
    fault: str
    uptime_ms: int


class MotorBridge:
    def __init__(
        self,
        transport: Transport,
        cfg: RobotConfig,
        clock: Clock,
        *,
        reply_timeout_ms: int = 50,
        poll_ms: int = 5,
        stop_attempts: int = 3,
        first_seq: int = 1,
    ) -> None:
        if reply_timeout_ms <= 0 or not 0 < poll_ms <= reply_timeout_ms:
            raise ValueError("need 0 < poll_ms <= reply_timeout_ms")
        if stop_attempts < 1:
            raise ValueError("stop_attempts must be >= 1")
        if not 1 <= first_seq <= 0xFFFF:
            raise ValueError("first_seq must be within 1..65535")
        if cfg.is_mock and not transport.is_simulated:
            raise ConfigSafetyError(
                "a mock configuration may only be used with a simulated transport")
        self._transport = transport
        self._cfg = cfg
        self._clock = clock
        self._reply_timeout_ms = reply_timeout_ms
        self._poll_ms = poll_ms
        self._stop_attempts = stop_attempts
        self._seq = first_seq
        self._asm = protocol.FrameAssembler()
        self.mcu_state: Optional[str] = None     # last state the MCU reported; None = unknown
        self.info: Optional[McuInfo] = None
        self.last_error: Optional[str] = None
        self.stats = BridgeStats()

    # ---- public API --------------------------------------------------------------------

    def connect(self) -> McuInfo:
        """Handshake only (HELLO). Never arms and never moves anything."""
        self._drain()
        reply = self._request("HELLO", attempts=3)
        if reply.cmd == "NAK":
            raise ProtocolMismatch(f"HELLO was rejected: {reply.fields[1:]}")
        if reply.cmd != "HELLO" or len(reply.fields) != 5:
            raise ProtocolMismatch(f"unexpected HELLO reply: {reply}")
        _, proto, fw_id, caps, state = reply.fields
        if not proto.isdigit() or int(proto) != protocol.PROTO_VERSION:
            raise ProtocolMismatch(
                f"MCU speaks protocol {proto!r}, host speaks {protocol.PROTO_VERSION}")
        if not _CAPS.fullmatch(caps) or state not in (protocol.STATE_ARMED, protocol.STATE_DISARMED):
            raise ProtocolMismatch(f"malformed HELLO fields: {reply.fields}")
        info = McuInfo(int(proto), fw_id, int(caps, 16), state)
        if self._cfg.encoders.present == "yes" and not info.has_encoders:
            raise ProtocolMismatch("config says encoders are present but the MCU reports none")
        self.info = info
        self.mcu_state = state
        return info

    def arm(self) -> None:
        self._require_motors_enabled()
        if self.info is None:
            raise BridgeError("call connect() before arm()")
        # Dry run of the drive pipeline: raises IncompleteHardwareConfig if any drivetrain value
        # is unknown, so an unconfigured robot is refused before a single byte is sent.
        twist_to_drive(self._cfg, Twist(0.0, 0.0))
        self._expect_ack(self._request("ARM"))
        if self.mcu_state != protocol.STATE_ARMED:
            raise ProtocolMismatch("MCU acknowledged ARM but does not report ARMED")

    def send_twist(self, twist: Twist) -> DriveCommand:
        """Convert and send one drive command with the configured TTL. Fails safe."""
        self._require_motors_enabled()
        if self.mcu_state != protocol.STATE_ARMED:
            raise NotArmedError("MCU is not known to be ARMED; call arm() first")
        try:
            cmd = twist_to_drive(self._cfg, twist)   # ValueError / IncompleteHardwareConfig
        except ValueError:
            self.stop()
            raise
        try:
            reply = self._request("DRV", cmd.left_permille, cmd.right_permille,
                                  self._cfg.limits.cmd_ttl_ms)
        except LinkError:
            self.stop()
            raise
        self._expect_ack(reply)
        return cmd

    def stop(self) -> bool:
        """Best-effort STOP. Returns True only if the MCU acknowledged. Never raises."""
        return self._best_effort("STOP")

    def disarm(self) -> bool:
        """Best-effort DISARM. Returns True only if the MCU acknowledged. Never raises."""
        return self._best_effort("DISARM")

    def ping(self) -> None:
        self._expect_ack(self._request("PING"))

    def stat(self) -> McuStatus:
        reply = self._request("STAT")
        if reply.cmd != "STAT" or len(reply.fields) != 7:
            raise ProtocolMismatch(f"unexpected STAT reply: {reply}")
        _, state, left, right, ttl, fault, uptime = reply.fields
        try:
            return McuStatus(state, int(left), int(right), int(ttl), fault, int(uptime))
        except ValueError as exc:
            raise ProtocolMismatch(f"non-numeric STAT field: {reply.fields}") from exc

    def close(self) -> None:
        """Disarm (best effort) and release the transport."""
        try:
            self.disarm()
        finally:
            self._transport.close()

    # ---- internals ---------------------------------------------------------------------

    def _require_motors_enabled(self) -> None:
        if not self._cfg.safety.motors_enabled:
            raise MotorsDisabledError(
                "safety.motors_enabled is false; refusing to arm or drive")

    def _take_seq(self) -> int:
        seq = self._seq
        self._seq = 1 if seq == 0xFFFF else seq + 1
        return seq

    def _drain(self) -> None:
        try:
            for _ in range(10_000):
                if not self._transport.read():
                    break
        except TransportError as exc:
            raise TransportDown(str(exc)) from exc
        self._asm = protocol.FrameAssembler()

    def _request(self, cmd: str, *fields: object, attempts: int = 1) -> protocol.Frame:
        for _ in range(attempts):
            seq = self._take_seq()
            try:
                self._transport.write(protocol.encode(cmd, seq, *fields))
                self.stats.tx_frames += 1
                reply = self._await_reply(seq)
            except TransportError as exc:
                self.last_error = str(exc)
                raise TransportDown(str(exc)) from exc
            if reply is not None:
                return reply
            self.stats.timeouts += 1
        self.last_error = f"no valid reply to {cmd}"
        raise LinkError(f"no valid reply to {cmd} after {attempts} attempt(s)")

    def _await_reply(self, seq: int) -> Optional[protocol.Frame]:
        wanted = str(seq)
        deadline = Deadline(self._clock, self._reply_timeout_ms)
        while True:
            # Drain everything that is available before sleeping, so a transport that hands
            # out one byte per read() cannot make a good reply look late.
            while True:
                chunk = self._transport.read()
                if not chunk:
                    break
                for item in self._asm.feed(chunk):
                    if isinstance(item, protocol.ProtocolError):
                        self.stats.bad_frames += 1
                        continue
                    self.stats.rx_frames += 1
                    if item.cmd not in _REPLY_COMMANDS or not item.fields:
                        self.stats.stale_replies += 1
                        continue
                    if item.cmd == "NAK" and item.fields[0] == "0":
                        # The MCU could not parse what we sent: our request was corrupted in transit.
                        self.stats.bad_frame_naks += 1
                        return None
                    if item.fields[0] == wanted:
                        return item
                    self.stats.stale_replies += 1
            if deadline.expired():
                return None
            self._clock.sleep_ms(self._poll_ms)

    def _expect_ack(self, reply: protocol.Frame) -> None:
        if reply.cmd == "ACK" and len(reply.fields) == 2:
            state = reply.fields[1]
            if state not in (protocol.STATE_ARMED, protocol.STATE_DISARMED):
                raise ProtocolMismatch(f"unknown MCU state {state!r}")
            self.mcu_state = state
            return
        if reply.cmd == "NAK" and len(reply.fields) == 2:
            self.stats.naks += 1
            reason = reply.fields[1]
            if reason == protocol.REASON_NOT_ARMED:
                self.mcu_state = protocol.STATE_DISARMED
            raise MotorRejected(reason)
        raise ProtocolMismatch(f"unexpected reply: {reply}")

    def _best_effort(self, cmd: str) -> bool:
        for _ in range(self._stop_attempts):
            try:
                self._expect_ack(self._request(cmd))
                return True
            except TransportDown as exc:
                self.last_error = str(exc)
                return False
            except BridgeError as exc:
                self.last_error = str(exc)
        return False
