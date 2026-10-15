"""Byte transports between the Pi-side bridge and the Arduino.

Only the interface and an in-memory loopback to the mock MCU exist. There is deliberately NO
serial-port implementation yet: opening a real port is a Phase 2 task that needs approval.

``LoopbackTransport`` carries fault-injection switches so failure handling can be tested
without hardware: disconnects, dropped requests, dropped/corrupted replies, line noise and
fragmented reads.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import List, Optional


class TransportError(Exception):
    """The byte channel itself failed (closed, unplugged, I/O error)."""


class Transport(ABC):
    @property
    @abstractmethod
    def is_simulated(self) -> bool:
        """True for in-memory test doubles; False for anything that can reach hardware."""

    @abstractmethod
    def write(self, data: bytes) -> None:
        """Send bytes. Raises TransportError on failure."""

    @abstractmethod
    def read(self) -> bytes:
        """Return whatever bytes are available now (possibly none). Never blocks."""

    @abstractmethod
    def close(self) -> None:
        """Release the channel. Must be safe to call more than once."""


class LoopbackTransport(Transport):
    def __init__(self, mcu, *, read_chunk: Optional[int] = None) -> None:
        if read_chunk is not None and read_chunk < 1:
            raise ValueError("read_chunk must be >= 1")
        self._mcu = mcu
        self._rx = bytearray()
        self._open = True
        self.read_chunk = read_chunk
        # ---- fault injection (all default to "no fault") ----
        self.connected = True      # False: write()/read() raise TransportError
        self.drop_writes = 0       # next N writes never reach the MCU
        self.drop_replies = 0      # next N MCU replies are lost on the way back
        self.corrupt_replies = 0   # next N MCU replies arrive with a flipped bit
        # ---- observation ----
        self.write_log: List[bytes] = []   # every write the host attempted, in order

    @property
    def is_simulated(self) -> bool:
        return True

    @property
    def is_open(self) -> bool:
        return self._open

    def inject_noise(self, data: bytes) -> None:
        """Make bytes arrive at the host as if the line were noisy."""
        self._rx += data

    def _require_usable(self) -> None:
        if not self._open:
            raise TransportError("transport is closed")
        if not self.connected:
            raise TransportError("transport is disconnected")

    def write(self, data: bytes) -> None:
        self._require_usable()
        data = bytes(data)
        self.write_log.append(data)
        if self.drop_writes > 0:
            self.drop_writes -= 1
            return
        reply = self._mcu.feed(data)
        if not reply:
            return
        if self.drop_replies > 0:
            self.drop_replies -= 1
            return
        if self.corrupt_replies > 0:
            self.corrupt_replies -= 1
            damaged = bytearray(reply)
            damaged[1] ^= 0x01  # inside the checksummed body, so the checksum no longer matches
            reply = bytes(damaged)
        self._rx += reply

    def read(self) -> bytes:
        self._require_usable()
        n = len(self._rx) if self.read_chunk is None else min(self.read_chunk, len(self._rx))
        out = bytes(self._rx[:n])
        del self._rx[:n]
        return out

    def close(self) -> None:
        self._open = False
