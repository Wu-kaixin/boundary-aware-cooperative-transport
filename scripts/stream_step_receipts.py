"""Bounded-memory reader for this project's paired_metrics JSON receipts."""
from __future__ import annotations

import json


class _Reader:
    def __init__(self, stream, chunk_size, max_value_chars):
        if chunk_size <= 0 or max_value_chars <= 0:
            raise ValueError('Reader sizes must be positive')
        self.stream, self.chunk_size, self.limit = stream, chunk_size, max_value_chars
        self.buffer, self.pos, self.eof = '', 0, False
        self.decoder = json.JSONDecoder()

    def more(self):
        self.buffer, self.pos = self.buffer[self.pos:], 0
        chunk = self.stream.read(self.chunk_size)
        self.buffer += chunk
        self.eof = not chunk

    def peek(self):
        while True:
            while self.pos < len(self.buffer) and self.buffer[self.pos] in ' \t\r\n':
                self.pos += 1
            if self.pos < len(self.buffer):
                return self.buffer[self.pos]
            if self.eof:
                return None
            self.more()

    def expect(self, token):
        if self.peek() != token:
            raise ValueError(f'Expected {token!r} in paired receipt')
        self.pos += 1

    def value(self, expected_type):
        if self.peek() is None:
            raise ValueError('Truncated paired receipt')
        while True:
            try:
                value, end = self.decoder.raw_decode(self.buffer, self.pos)
            except json.JSONDecodeError as error:
                if len(self.buffer) - self.pos > self.limit:
                    raise ValueError('Paired receipt value exceeds memory limit') from error
                if self.eof:
                    raise ValueError('Malformed or truncated paired receipt') from error
                self.more()
                continue
            if end - self.pos > self.limit:
                raise ValueError('Paired receipt value exceeds memory limit')
            if not isinstance(value, expected_type):
                raise ValueError(f'Expected {expected_type.__name__} in paired receipt')
            self.pos = end
            return value


def iter_step_receipts(stream, *, chunk_size=1024 * 1024, max_value_chars=16 * 1024 * 1024):
    """Yield rows, returning the optional summary at exhaustion.

    Accept only the owning writer's steps-first object with an optional summary.
    Prefix consumers deliberately do not validate the unseen suffix. Full consumers
    must exhaust the generator; its StopIteration.value is the parsed summary.
    Memory is bounded by one decoded row plus the configured buffer/value limit.
    """
    reader = _Reader(stream, chunk_size, max_value_chars)
    reader.expect('{')
    if reader.value(str) != 'steps':
        raise ValueError('Paired receipt must begin with steps')
    reader.expect(':')
    reader.expect('[')
    if reader.peek() != ']':
        while True:
            yield reader.value(dict)
            if reader.peek() == ']':
                break
            reader.expect(',')
    reader.expect(']')
    summary = None
    if reader.peek() == ',':
        reader.expect(',')
        if reader.value(str) != 'summary':
            raise ValueError('Unexpected or duplicate top-level receipt key')
        reader.expect(':')
        summary = reader.value(dict)
    reader.expect('}')
    if reader.peek() is not None:
        raise ValueError('Trailing data after paired receipt')
    return summary
