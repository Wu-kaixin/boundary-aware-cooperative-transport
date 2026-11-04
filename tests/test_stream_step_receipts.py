"""Scientific receipt parsing must retain every row and reject incomplete data."""
import io
import json
from pathlib import Path

import pytest


@pytest.fixture
def reader(monkeypatch):
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[1] / 'scripts'))
    from stream_step_receipts import iter_step_receipts
    return iter_step_receipts


@pytest.mark.parametrize('chunk_size', [1, 7, 64, 4096])
def test_exact_rows_across_chunk_boundaries(reader, chunk_size):
    rows = [{'frame': i, 'string': '边界 } [ \\"\n', 'nested': [None, True, {'n': -1.25e-20}],
             'commands': [[i / 7, -i / 3]]} for i in range(9)]
    encoded = json.dumps({'steps': rows, 'summary': {'tail': '} \\"', 'n': 9}}, ensure_ascii=False)
    assert list(reader(io.StringIO(encoded), chunk_size=chunk_size)) == json.loads(encoded)['steps']
    assert list(reader(io.StringIO('{"steps": []}'), chunk_size=chunk_size)) == []


@pytest.mark.parametrize('text', [
    '{"steps":[{"frame":1}', '{"steps":[{"frame":1},]}',
    '{"steps":[{"frame":1}{"frame":2}]}', '{"steps":[7]}',
    '{"steps":[],"steps":[]}', '{"steps":[],"unknown":{}}',
    '{"steps":[],"summary":{', '{"steps":[]}junk',
    '{"steps":[],"summary":null}', '{"steps":[],}',
])
def test_malformed_or_incomplete_receipts_fail_closed(reader, text):
    with pytest.raises(ValueError):
        list(reader(io.StringIO(text), chunk_size=3))


def test_reader_is_bounded_and_does_not_read_future_rows_for_prefix(reader):
    class Tracking(io.StringIO):
        requested = []
        def read(self, n=-1):
            self.requested.append(n)
            assert 0 < n <= 64
            return super().read(n)
    stream = Tracking(json.dumps({'steps': [{'frame': i, 'data': 'x' * 100} for i in range(1000)]}))
    rows = reader(stream, chunk_size=64)
    assert next(rows)['frame'] == 0
    assert stream.tell() < 1024
    with pytest.raises(ValueError, match='limit'):
        list(reader(io.StringIO('{"steps":[{"large":"' + 'x' * 1000 + '"}]}'),
                    chunk_size=32, max_value_chars=128))
