"""Small bounded queues used by multimodal GEM motion producers.

The queue deliberately rejects new requests while full. Dropping an already
queued prompt would reorder the user's commands and can make a robot jump to a
later action, so backpressure is explicit instead.
"""

from __future__ import annotations

from dataclasses import dataclass
import queue
import threading
import time


@dataclass(frozen=True)
class MotionChunkRequest:
    sequence: int
    kind: str
    value: str
    submitted_ns: int


class BoundedMotionChunkQueue:
    """Thread-safe FIFO with a fixed number of pending requests."""

    def __init__(self, max_chunks: int = 3) -> None:
        if max_chunks <= 0:
            raise ValueError("max_chunks must be positive")
        self._queue: queue.Queue[MotionChunkRequest] = queue.Queue(maxsize=max_chunks)
        self._lock = threading.Lock()
        self._next_sequence = 0
        self._closed = False

    @property
    def max_chunks(self) -> int:
        return int(self._queue.maxsize)

    def submit(self, kind: str, value: str) -> MotionChunkRequest | None:
        value = str(value).strip()
        if not value:
            raise ValueError("Motion chunk request cannot be empty")
        with self._lock:
            if self._closed:
                return None
            request = MotionChunkRequest(
                sequence=self._next_sequence,
                kind=str(kind),
                value=value,
                submitted_ns=time.time_ns(),
            )
            try:
                self._queue.put_nowait(request)
            except queue.Full:
                return None
            self._next_sequence += 1
            return request

    def get(self, timeout: float | None = None) -> MotionChunkRequest:
        return self._queue.get(timeout=timeout)

    def task_done(self) -> None:
        self._queue.task_done()

    def join(self) -> None:
        self._queue.join()

    def close(self) -> None:
        with self._lock:
            self._closed = True

    @property
    def closed(self) -> bool:
        with self._lock:
            return self._closed
