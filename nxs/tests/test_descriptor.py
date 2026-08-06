"""Sample-descriptor parser tests."""

from nxs.descriptor import parse_sample, parse_sample_raw


def test_string_field_strips_tail_padding_only():
    """`parse_sample` must strip 0xFF/0x00 padding from the tail only.
    Interior occurrences belong to the payload (e.g. binary-text
    protocols carrying NUL bytes) and must survive — filtering them
    out is silent data corruption."""
    fields = [{'name': 'payload', 'type': 'string', 'count': 10}]
    # Interior 0x00 between 'A' and 'B' must survive; trailing 0xFF/0x00
    # padding must go.
    raw = b'A\x00B' + b'\xff' * 5 + b'\x00\x00'
    values = parse_sample(raw, fields)
    assert values['payload'] == 'A\x00B'


def test_string_field_with_no_padding_returns_full_chunk():
    fields = [{'name': 'nmea', 'type': 'string', 'count': 8}]
    raw = b'$GPGGA,1'
    values = parse_sample(raw, fields)
    assert values['nmea'] == '$GPGGA,1'


def test_sequential_field_after_overflow_is_not_misdecoded():
    """A sequentially-packed field that overflows the sample must advance the
    cursor so a later, smaller sequential field is skipped too — never decoded
    from the stale offset the overflowing field would otherwise have held."""
    fields = [
        {'name': 'first', 'type': 'uint16', 'byte_off': 0},
        {'name': 'big', 'type': 'uint32'},    # sequential at 2; 2..6 overflows
        {'name': 'tail', 'type': 'uint16'},   # sequential after big
    ]
    raw = b'\x01\x00\x99\x99\x99'   # 5 bytes: 'big' cannot fit
    for decode in (parse_sample, parse_sample_raw):
        values = decode(raw, fields)
        assert 'first' in values
        assert 'big' not in values     # overflowed → skipped
        assert 'tail' not in values    # not decoded from big's stale offset
