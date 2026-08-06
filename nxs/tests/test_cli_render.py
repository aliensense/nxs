"""Render helper tests: string fields strip trailing CR/LF and flow
their full text instead of being right-aligned into a fixed-width
column, so NMEA-style frames render readably."""

from nxs.cli import _format_sample_columns


def test_string_field_strips_trailing_crlf():
    """A complete NMEA sentence ending in \\r\\n must print without
    that terminator — otherwise every sample inserts a blank line."""
    fields = [{'name': 'nmea', 'type': 'string'}]
    values = {'nmea': '$GPGGA,123519,4807.038,N,01131.000,E*47\r\n'}
    line = _format_sample_columns(values, fields)
    assert line == '$GPGGA,123519,4807.038,N,01131.000,E*47'


def test_string_field_does_not_pad_to_fixed_width():
    """Long strings flow at their natural width; short strings don't
    get right-aligned to 10 chars (which used to obscure the column
    separator for NMEA streams)."""
    fields = [{'name': 'nmea', 'type': 'string'}]
    values = {'nmea': 'OK'}
    line = _format_sample_columns(values, fields)
    assert line == 'OK'


def test_numeric_field_keeps_fixed_width():
    fields = [{'name': 'temp', 'type': 'int16'}]
    values = {'temp': 23.5}
    line = _format_sample_columns(values, fields)
    assert line == '      23.5'


def test_numeric_field_keeps_sub_milli_values_visible():
    """Earth-field tesla values (~5e-05) must not round to zero — the
    formatter falls back to exponent notation below fixed-point range."""
    fields = [{'name': 'mag_x', 'type': 'int16'}]
    values = {'mag_x': 5.02e-05}
    line = _format_sample_columns(values, fields)
    assert line == '  5.02e-05'


def test_mixed_string_and_numeric_render():
    """Mixed schemas: numeric fields keep their column, string fields
    flow. Two-space separator between columns is preserved."""
    fields = [
        {'name': 'temp', 'type': 'int16'},
        {'name': 'status', 'type': 'string'},
    ]
    values = {'temp': 1.0, 'status': 'OK\n'}
    line = _format_sample_columns(values, fields)
    assert line == '         1  OK'


# ── Human display units (render-layer conversion, SI stays on the wire) ──


def test_human_units_convert_kelvin_to_celsius():
    from nxs.cli import _display_unit
    fields = [{'name': 'temp', 'type': 'int16', 'unit': 'kelvin'}]
    values = {'temp': 300.15}
    line = _format_sample_columns(values, fields, human=True)
    assert line == '        27'
    # The header label converts with the value — a converted number
    # never sits under an SI label.
    assert _display_unit('kelvin', True) == '°C'


def test_si_mode_passes_kelvin_through():
    from nxs.cli import _display_unit
    fields = [{'name': 'temp', 'type': 'int16', 'unit': 'kelvin'}]
    values = {'temp': 300.15}
    line = _format_sample_columns(values, fields, human=False)
    assert line == '     300.1'
    assert _display_unit('kelvin', False) == 'kelvin'


def test_human_units_leave_other_units_untouched():
    from nxs.cli import _display_unit
    fields = [
        {'name': 'accel_z', 'type': 'int16', 'unit': 'm/s^2'},
        {'name': 'nmea', 'type': 'string', 'unit': ''},
    ]
    values = {'accel_z': 9.807, 'nmea': 'OK\r\n'}
    line = _format_sample_columns(values, fields, human=True)
    assert line == '     9.807  OK'
    assert _display_unit('m/s^2', True) == 'm/s^2'
