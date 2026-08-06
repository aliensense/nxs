"""Tests for the firmware image store — the manifest pin matches image
content (the MCUboot header version), never a filename."""
import os
import struct
import tempfile

import pytest

from nxs.suite.firmware import IMAGE_MAGIC, find_image, read_image_version


def _write_image(path, version, magic=IMAGE_MAGIC):
    header = bytearray(32)
    struct.pack_into('<I', header, 0, magic)
    struct.pack_into('<BBH', header, 20, *version)
    with open(path, 'wb') as f:
        f.write(bytes(header))


def test_read_image_version_parses_the_header():
    with tempfile.TemporaryDirectory() as tmp:
        path = os.path.join(tmp, 'fw.bin')
        _write_image(path, (1, 2, 3))
        assert read_image_version(path) == (1, 2, 3)


def test_read_image_version_rejects_non_mcuboot():
    with tempfile.TemporaryDirectory() as tmp:
        path = os.path.join(tmp, 'junk.bin')
        _write_image(path, (1, 2, 3), magic=0xDEADBEEF)
        with pytest.raises(ValueError, match='magic'):
            read_image_version(path)


def test_find_image_matches_the_pin_regardless_of_filename():
    with tempfile.TemporaryDirectory() as tmp:
        _write_image(os.path.join(tmp, 'a-misleading-name.bin'), (1, 1, 0))
        _write_image(os.path.join(tmp, 'other.bin'), (1, 0, 0))
        assert find_image(tmp, '1.1.0').endswith('a-misleading-name.bin')
        assert find_image(tmp, '1.0').endswith('other.bin')


def test_find_image_error_lists_what_is_available():
    with tempfile.TemporaryDirectory() as tmp:
        _write_image(os.path.join(tmp, 'old.bin'), (1, 0, 0))
        with pytest.raises(FileNotFoundError) as e:
            find_image(tmp, '2.0.0')
        assert 'old.bin (1.0.0)' in str(e.value)
