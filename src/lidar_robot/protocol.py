"""Pi <-> Arduino serial line protocol, version 1 (framing + constants only).

Wire format (ASCII, NMEA-style, one frame per line):

    $<CMD>,<field>,<field>,...*<HH>\\n

``HH`` is the XOR of every byte between ``$`` and ``*`` as two upper-case hex digits.
A whole frame, including ``$`` and the newline, is at most MAX_FRAME_BYTES bytes.
The full command set and semantics are documented in docs/interfaces.md.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Tuple, Union

PROTO_VERSION = 1
MAX_FRAME_BYTES = 64
MAX_TTL_MS = 1000
CAP_ENCODERS = 0x01          # capability bit reported in HELLO

# Host -> MCU commands
HOST_COMMANDS = ("HELLO", "ARM", "DISARM", "DRV", "STOP", "PING", "STAT")

# NAK reason codes
REASON_BAD_FRAME = "BAD_FRAME"
REASON_BAD_ARGS = "BAD_ARGS"
REASON_UNKNOWN_CMD = "UNKNOWN_CMD"
REASON_NOT_ARMED = "NOT_ARMED"
REASON_RANGE = "RANGE"
REASON_TTL = "TTL"

# MCU states / faults
STATE_DISARMED = "DISARMED"
STATE_ARMED = "ARMED"
FAULT_NONE = "NONE"
FAULT_LINK_TIMEOUT = "LINK_TIMEOUT"

_FORBIDDEN_CHARS = frozenset("$*,\r\n")
_HEX_UPPER = frozenset(b"0123456789ABCDEF")  # iterating bytes yields ints


class ProtocolError(ValueError):
    """Malformed frame or token."""


@dataclass(frozen=True)
class Frame:
    cmd: str
    fields: Tuple[str, ...] = ()


def checksum(body: bytes) -> int:
    value = 0
    for byte in body:
        value ^= byte
    return value


def _check_token(token: str) -> None:
    if not token:
        raise ProtocolError("empty token")
    for ch in token:
        code = ord(ch)
        if ch in _FORBIDDEN_CHARS or not (0x20 < code < 0x7F):
            raise ProtocolError(f"illegal character {ch!r} in token {token!r}")


def encode(cmd: str, *fields: object) -> bytes:
    tokens = [cmd] + [str(f) for f in fields]
    for token in tokens:
        _check_token(token)
    body = ",".join(tokens).encode("ascii")
    frame = b"$" + body + b"*" + (b"%02X" % checksum(body)) + b"\n"
    if len(frame) > MAX_FRAME_BYTES:
        raise ProtocolError(f"frame too long ({len(frame)} > {MAX_FRAME_BYTES})")
    return frame


def decode(line: bytes) -> Frame:
    """Decode one complete line.

    Length rule (identical in encode(), decode() and FrameAssembler): every byte on the
    wire counts, including an optional CR, and the whole frame including the terminating
    LF is at most MAX_FRAME_BYTES. A line passed without its LF is measured as if the LF
    were present. One trailing LF, then one trailing CR, are tolerated and stripped.
    """
    wire_len = len(line) if line.endswith(b"\n") else len(line) + 1
    if wire_len > MAX_FRAME_BYTES:
        raise ProtocolError(f"frame too long ({wire_len} > {MAX_FRAME_BYTES})")
    if line.endswith(b"\n"):
        line = line[:-1]
    if line.endswith(b"\r"):
        line = line[:-1]
    if not line.startswith(b"$"):
        raise ProtocolError("missing '$' start")
    star = line.rfind(b"*")
    if star < 0:
        raise ProtocolError("missing '*' checksum delimiter")
    body, cs_text = line[1:star], line[star + 1:]
    # Strict: exactly two UPPER-CASE hex digits (int(x, 16) alone also accepts '+D', ' D', 'ab').
    if len(cs_text) != 2 or any(ch not in _HEX_UPPER for ch in cs_text):
        raise ProtocolError("checksum must be exactly two upper-case hex digits")
    expected = int(cs_text, 16)
    if expected != checksum(body):
        raise ProtocolError("checksum mismatch")
    try:
        text = body.decode("ascii")
    except UnicodeDecodeError as exc:
        raise ProtocolError("non-ASCII body") from exc
    tokens = text.split(",")
    for token in tokens:
        _check_token(token)
    return Frame(tokens[0], tuple(tokens[1:]))


class FrameAssembler:
    """Incremental stream parser: tolerates fragmentation, garbage and mid-frame restarts."""

    def __init__(self) -> None:
        self._buf = bytearray()
        self._active = False

    def _reset(self) -> None:
        self._buf = bytearray()
        self._active = False

    def feed(self, data: bytes) -> List[Union[Frame, ProtocolError]]:
        out: List[Union[Frame, ProtocolError]] = []
        for byte in data:
            if byte == 0x24:  # '$' always starts (or restarts) a frame
                if self._active:
                    out.append(ProtocolError("frame restarted before terminator"))
                self._buf = bytearray(b"$")
                self._active = True
                continue
            if not self._active:
                continue  # bytes between frames are ignored
            if byte == 0x0A:  # '\n'
                line = bytes(self._buf)
                self._reset()
                try:
                    out.append(decode(line))
                except ProtocolError as exc:
                    out.append(exc)
                continue
            self._buf.append(byte)
            if len(self._buf) > MAX_FRAME_BYTES - 1:
                out.append(ProtocolError("frame too long"))
                self._reset()
        return out
