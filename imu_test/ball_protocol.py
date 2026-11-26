"""PC-side BALL v2 window. All timestamps are monotonic seconds."""

from collections import deque
from math import isfinite
from threading import Lock
from typing import NamedTuple


class BallEvent(NamedTuple):
    at: float
    session: int
    sequence: int
    outcome: str
    m1: float
    m3: float
    rtt_s: float | None = None
    mutex_us: int | None = None


class BallWindow:
    def __init__(self, ack_timeout=.15, target_age=.12, send_period=0):
        if not (0 < ack_timeout < .45 and 0 < target_age < .45 and 0 <= send_period < .45):
            raise ValueError("Invalid BALL timeout")
        self.lock = Lock()
        self.session = 0
        self.sequence = 0
        self.pending = self.in_flight = None
        self.last_sent = None
        self.streaming = False
        self.send_period = send_period
        self.ack_timeout = ack_timeout
        self.target_age = target_age
        self.events = deque(maxlen=1024)

    def begin(self, session, now, streaming=False):
        with self.lock:
            self._clear(now, "session_changed")
            self.session = session
            self.streaming = streaming
            self.sequence = 0
            self.last_sent = None

    def _clear(self, now, outcome):
        for item in (self.in_flight, self.pending):
            if item is not None:
                self.events.append(BallEvent(now, self.session, item[1], outcome, *item[2]))
        self.pending = self.in_flight = None
        self.session = 0

    def clear(self, now, outcome, session=None):
        with self.lock:
            if session is None or session == self.session:
                self._clear(now, outcome)
                return True
            return False

    def offer(self, session, values, now):
        with self.lock:
            if session != self.session or not session:
                return
            if self.pending is not None:
                _, sequence, previous = self.pending
                self.events.append(BallEvent(now, session, sequence, "replaced", *previous))
            self.sequence += 1
            self.pending = (now, self.sequence, values)
            return self.sequence

    def next_command(self, now):
        with self.lock:
            if self.in_flight is not None:
                sent, sequence, values = self.in_flight
                if now - sent < self.ack_timeout:
                    return None
                self.events.append(BallEvent(now, self.session, sequence, "timeout", *values))
                self.in_flight = None
            if self.pending is None or (self.last_sent is not None and now - self.last_sent < self.send_period):
                return None
            arrived, sequence, values = self.pending
            self.pending = None
            if now - arrived > self.target_age:
                self.events.append(BallEvent(now, self.session, sequence, "expired", *values))
                return None
            self.last_sent = now
            if not self.streaming:
                self.in_flight = (now, sequence, values)
            self.events.append(BallEvent(now, self.session, sequence, "sent", *values))
            command = "BALLFAST" if self.streaming else "BALL2"
            return f"{command} {self.session} {sequence} {values[0]:.4f} {values[1]:.4f}"

    def receive(self, line, now):
        fields = line.split(b",")
        kind = fields[0]
        if kind in (b"!BALLSTATE2", b"!BALLSTARTERR"):
            try:
                session = int(fields[1])
                if len(fields) == 3 and (kind == b"!BALLSTARTERR" or int(fields[2]) in (0, 2, 3)):
                    self.clear(now, "ended", session)
            except (IndexError, ValueError):
                pass
            return
        if kind not in (b"!BALLACK", b"!BALLNACK"):
            return
        with self.lock:
            try:
                session, sequence = map(int, fields[1:3])
                if kind == b"!BALLACK":
                    if len(fields) != 7:
                        raise ValueError
                    state = int(fields[3])
                    targets = tuple(map(float, fields[4:6]))
                    mutex_us = int(fields[6])
                    if state not in (1, 4, 5) or mutex_us < 0 or not all(map(isfinite, targets)):
                        raise ValueError
                    outcome = {1: "applied", 4: "centering", 5: "diagnostic"}[state]
                else:
                    if len(fields) != 4:
                        raise ValueError
                    outcome = "nack:" + fields[3].decode("ascii")
                    mutex_us = None
            except (ValueError, UnicodeError):
                self.events.append(BallEvent(now, 0, 0, "invalid_reply", 0, 0))
                return
            if self.streaming and kind == b"!BALLNACK" and session == self.session and 0 < sequence <= self.sequence:
                self.events.append(BallEvent(now, session, sequence, outcome, float("nan"), float("nan")))
                return
            active = self.in_flight
            if active is None or session != self.session or sequence != active[1]:
                self.events.append(BallEvent(now, session, sequence, "late_reply", 0, 0))
                return
            sent, _, values = active
            if now - sent >= self.ack_timeout:
                self.events.append(BallEvent(now, session, sequence, "timeout", *values))
                self.events.append(BallEvent(now, session, sequence, "late_reply", *values))
                self.in_flight = None
                return
            if kind == b"!BALLACK" and any(abs(a - b) > .00006 for a, b in zip(values, targets)):
                self.events.append(BallEvent(now, session, sequence, "invalid_reply", *values))
                return
            self.in_flight = None
            self.events.append(BallEvent(now, session, sequence, outcome, *values, now - sent, mutex_us))
