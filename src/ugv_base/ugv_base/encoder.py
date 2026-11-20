"""Quadrature encoder decoding (pure logic) and libgpiod event reader.

Two modes:
  x4 - both edges of channels A and B are watched as events; the levels are
       tracked from the events, so the count stays correct even if processing
       is delayed. Resolution: ppr * 4.
  x2 - only channel A edges are watched; B is read WHILE the event is PROCESSED.
       Half as many events, but since B is read later than the event it is
       only reliable at low PPR / low RPM (use x4 for the 546 PPR
       DCGM-3865). Resolution: ppr * 2.

Direction rule: when turning forward, A leads B (00 -> 10 -> 11 -> 01).
If the opposite is measured, set invert: true for that encoder.
"""

import select
import threading
import time

# (previous state, new state) -> +1; state = (A << 1) | B
_FORWARD = {(0b00, 0b10), (0b10, 0b11), (0b11, 0b01), (0b01, 0b00)}
_BACKWARD = {(new, old) for old, new in _FORWARD}

MODES = ('x4', 'x2', 'none')


def multiplier(mode: str) -> int:
    return {'x4': 4, 'x2': 2}.get(mode, 0)


class QuadratureDecoder:

    def __init__(self, mode: str = 'x4', invert: bool = False, a: int = 0, b: int = 0):
        if mode not in ('x4', 'x2'):
            raise ValueError(f"encoder mode must be x4 or x2, got: {mode!r}")
        self.mode = mode
        self.invert = invert
        self.a = a
        self.b = b
        self.count = 0
        self.errors = 0     # number of missed edges in x4

    def on_edge(self, channel: str, level: int, other_level: int = None) -> int:
        """channel: 'a' or 'b'. In x2 mode other_level is the B level read
        at the time of the A edge."""
        prev = (self.a << 1) | self.b
        if channel == 'a':
            self.a = level
            if other_level is not None:
                self.b = other_level
        else:
            self.b = level

        if self.mode == 'x2':
            delta = 1 if self.a != self.b else -1
        else:
            curr = (self.a << 1) | self.b
            if (prev, curr) in _FORWARD:
                delta = 1
            elif (prev, curr) in _BACKWARD:
                delta = -1
            else:
                # Edge arrived without a level change: an event was missed in
                # between (kernel event buffer overflowed, or noise).
                self.errors += 1
                delta = 0

        if self.invert:
            delta = -delta
        self.count += delta
        return delta


class EncoderReader:
    """Reads one or more encoders in background threads.

    encoders: [(a_pin, b_pin, invert), ...]; pin = (chip_name, line_offset).
    If the lines are on different GPIO chips the events arrive from separate
    queues; a single thread reads them all and processes them sorted by kernel
    timestamp.
    """

    def __init__(self, open_chip, encoders, mode: str = 'x4', bias: str = 'as-is'):
        self._mode = mode
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._threads = []
        self._requests = []
        self.decoders = []

        # chip -> {offset: (decoder index, channel)}
        routes = {}
        for idx, (a_pin, b_pin, invert) in enumerate(encoders):
            self.decoders.append(QuadratureDecoder(mode, invert))
            routes.setdefault(a_pin[0], {})[a_pin[1]] = (idx, 'a')
            if mode == 'x4':
                routes.setdefault(b_pin[0], {})[b_pin[1]] = (idx, 'b')
        self._b_pins = [b for _, b, _ in encoders]

        self._b_inputs = {}
        if mode == 'x2':
            by_chip = {}
            for chip_name, offset in self._b_pins:
                by_chip.setdefault(chip_name, []).append(offset)
            for chip_name, offsets in by_chip.items():
                req = open_chip(chip_name).request_inputs(offsets, bias=bias)
                self._requests.append(req)
                for offset in offsets:
                    self._b_inputs[(chip_name, offset)] = req

        level_sources = {}
        sources = []
        for chip_name, route in routes.items():
            req = open_chip(chip_name).request_edges(list(route), bias=bias)
            self._requests.append(req)
            sources.append((req, route))
            for offset in route:
                level_sources[(chip_name, offset)] = req
        self._threads.append(threading.Thread(target=self._run, args=(sources,), daemon=True))

        for idx, (a_pin, b_pin, _) in enumerate(encoders):
            dec = self.decoders[idx]
            dec.a = level_sources[a_pin].get(a_pin[1])
            b_source = level_sources.get(b_pin) or self._b_inputs[b_pin]
            dec.b = b_source.get(b_pin[1])

        for thread in self._threads:
            thread.start()

    def _run(self, sources):
        if len(sources) == 1:
            req, route = sources[0]
            while not self._stop.is_set():
                if req.wait(0.1):
                    self._apply([(edge, route) for edge in req.read()])
            return

        # Several queues: every event older than the start of the scan has
        # been read by this scan; only those are sorted and processed, later
        # ones are left for the next scan. Events are stamped with CLOCK_MONOTONIC;
        # if the clocks don't match (old kernel) events are processed without waiting.
        # libgpiod v2 queues expose an fd: select wakes up when an event arrives.
        # Without an fd (v1, mock) the queues are polled at short intervals.
        fds = [req.fileno() for req, _ in sources]
        waitable = all(fd is not None for fd in fds)
        pending = []
        while not self._stop.is_set():
            if waitable:
                # Events held back: cut again right away; otherwise sleep until one arrives.
                ready, _, _ = select.select(fds, [], [], 0.001 if pending else 0.1)
                if not ready and not pending:
                    continue
            cutoff = time.monotonic_ns()
            for req, route in sources:
                if req.wait(0.0 if waitable else 0.001):
                    pending.extend((edge, route) for edge in req.read())
            if not pending:
                continue
            pending.sort(key=lambda item: item[0].timestamp_ns)
            late = [cutoff < edge.timestamp_ns <= cutoff + 1_000_000_000 for edge, _ in pending]
            ready_items = [item for item, wait in zip(pending, late) if not wait]
            pending = [item for item, wait in zip(pending, late) if wait]
            if ready_items:
                self._apply(ready_items)

    def _apply(self, items):
        with self._lock:
            for edge, route in items:
                idx, channel = route[edge.offset]
                other = None
                if self._mode == 'x2':
                    b_pin = self._b_pins[idx]
                    other = self._b_inputs[b_pin].get(b_pin[1])
                self.decoders[idx].on_edge(channel, edge.level, other)

    def counts(self) -> list:
        with self._lock:
            return [d.count for d in self.decoders]

    def errors(self) -> list:
        with self._lock:
            return [d.errors for d in self.decoders]

    def close(self):
        self._stop.set()
        for thread in self._threads:
            thread.join(timeout=1.0)
        for req in self._requests:
            req.release()
