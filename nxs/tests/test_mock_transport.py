"""Fidelity tests for MockTransport — it stands in for real hardware in
the suite tests, so its observable behavior must match the real
transports where those tests depend on it."""
from nxs.descriptor import load_driver
from nxs.image import deserialize, serialize
from nxs.transports.mock import MockTransport


def _image(name: str) -> bytes:
    return serialize(load_driver(name)().compile({}))


def test_read_driver_name_reports_the_active_slot_not_last_uploaded():
    first, second = _image('iam20680'), _image('ms5611')
    t = MockTransport()
    t.upload_image(first)
    t.save_slot(0)
    t.upload_image(second)
    t.save_slot(1)
    t.vm_run()
    # Active slot stayed 0 (the runner owns it); the name tracks the active
    # slot's driver, like I2C/Cyphal — not the most recently uploaded image.
    assert t.read_active_slot() == 0
    assert t.read_driver_name() == deserialize(first).name
    assert t.read_driver_name() != deserialize(second).name


def test_store_slots_are_stable_numeric_ids():
    """Slot numbers are stable IDs, like real firmware — a non-contiguous
    save keeps its number, and a delete doesn't renumber the others."""
    a, b = _image('iam20680'), _image('ms5611')
    t = MockTransport()
    t.upload_image(a)
    t.save_slot(3)                    # non-contiguous — must stay slot 3
    assert t.read_active_slot() == 3
    assert t.read_store_count() == 1
    assert t.read_driver_name() == deserialize(a).name

    t.upload_image(b)
    t.save_slot(5)
    assert t.read_store_count() == 2
    t.delete_slot(3)                  # active slot removed
    assert t.read_store_count() == 1
    assert t.read_active_slot() == 5  # advanced to the remaining slot, not shifted
    assert t.read_driver_name() == deserialize(b).name


def test_read_driver_name_falls_back_to_the_transient_driver():
    t = MockTransport()
    t.upload_image(_image('iam20680'))  # loaded, not saved
    assert t.read_active_slot() == 0xFF
    assert t.read_driver_name() == 'Iam20680'
    assert MockTransport().read_driver_name() == ''


def test_commission_carries_bitrate_and_term():
    """commission() accepts the full kwarg surface the CLI hands every
    transport — a stale two-argument signature raises TypeError here."""
    t = MockTransport()
    t.commission(node_addr=124, can_bitrate=(500000, 500000), can_term=1)
    assert t.read_identity()["node_addr"] == 124
    assert t.read_can_bitrate() == (500000, 500000)
    assert t.read_can_term() == 1


def test_open_client_forwards_kwargs_to_the_mock_constructor():
    # The factory contract: kwargs reach the transport constructor, so a
    # mistyped argument fails loudly instead of being silently dropped.
    from nxs import open_client

    assert isinstance(open_client("mock"), MockTransport)
    try:
        open_client("mock", bogus_kwarg=1)
    except TypeError:
        pass
    else:
        assert False, "unknown kwarg was silently dropped"
