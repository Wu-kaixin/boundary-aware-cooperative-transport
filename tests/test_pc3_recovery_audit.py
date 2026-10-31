"""Unknown bytes must not be explained away by the container recovery audit."""
from io import BytesIO

import pytest

from scripts.maintenance.audit_pc3_recovery import compare_streams, sparse_pattern, PERIOD


def compare(data, reference, pattern=b"\x00\x01\x00", chunk_size=7):
    return compare_streams(BytesIO(data), BytesIO(reference), pattern, chunk_size)


def test_reference_prefix_then_fill_across_chunk_boundary():
    reference = b"original payload bytes"
    fill = (b"\x00\x01\x00" * 8)[:len(reference)]
    result = compare(reference[:10] + fill[10:], reference)
    assert result["fully_explained"]
    assert result["reference_prefix_bytes"] == result["periodic_fill_start"] == 10
    assert result["classification"] == "REFERENCE_PREFIX_AND_FILL"


def test_unique_byte_inside_fill_is_retained_as_unexplained():
    reference = b"original payload bytes"
    fill = (b"\x00\x01\x00" * 8)[:len(reference)]
    data = bytearray(reference[:10] + fill[10:])
    data[17] = 88
    result = compare(bytes(data), reference)
    assert not result["fully_explained"]
    assert result["classification"] == "UNEXPLAINED_BYTES"


def test_periodic_only_is_distinct_from_reference_identity():
    result = compare(b"\x00\x01\x00" * 6, b"x" * 18)
    assert result["fully_explained"]
    assert result["classification"] == "PERIODIC_FILL_ONLY"
    assert result["reference_prefix_bytes"] == 0


def test_exact_reference_is_identified():
    result = compare(b"verified archive", b"verified archive")
    assert result["classification"] == "EXACT_REFERENCE"
    assert result["sha256"] == result["reference_sha256"]


def test_truncation_is_not_accepted():
    with pytest.raises(ValueError, match="lengths differ"):
        compare(b"abc", b"abcd")


def test_dense_or_unrecognized_patterns_are_rejected():
    for pattern in (b"x" * PERIOD, b"\x03\x02\x01\xff" + b"x" * (PERIOD - 4), b"\0" * PERIOD):
        with pytest.raises(ValueError, match="signature"):
            sparse_pattern(pattern)
