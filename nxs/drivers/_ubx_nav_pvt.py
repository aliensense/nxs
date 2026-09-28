"""Shared UBX machinery for u-blox stream drivers.

One binary measure loop (sync, acquisition stamp, 98-byte frame, NAV-PVT
match, Fletcher check, commit), one typed output table over the 92-byte
payload, and the UBX frame and CFG-VALSET helpers. A subclass keeps its
own probe, config keys, rates, and NMEA variant. The helpers are plain
Python run at compile time inside probe()/configure().
"""

from nxs import StreamDriver
from nxs.compiler import ChecksumFletcher

_DEG_TO_RAD = 0.017453292519943295    # pi / 180

# UBX class/ID of the CFG-VALSET configuration write.
UBX_CLS_CFG = 0x06
UBX_ID_VALSET = 0x8A

# Configuration-database keys shared by the family (the value size rides
# bits 30:28 of the key ID: 0x2..=U1, 0x3..=U2).
K_RATE_MEAS = 0x30210001         # U2, measurement period [ms]
K_MSGOUT_PVT_UART1 = 0x20910007  # U1, UBX-NAV-PVT output rate on UART1


def ubx_frame(msg_class, msg_id, payload):
    """Full UBX frame: B5 62 | class id len(LE) | payload | CK_A CK_B,
    with the 8-bit Fletcher checksum over class..payload."""
    n = len(payload)
    core = bytes([msg_class, msg_id, n & 0xFF, (n >> 8) & 0xFF]) + bytes(payload)
    ck_a = 0
    ck_b = 0
    for b in core:
        ck_a = (ck_a + b) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return b'\xb5\x62' + core + bytes([ck_a, ck_b])


def valset_body(kvs):
    """CFG-VALSET payload for the RAM layer: version 0, layers 0x01,
    reserved(2), then little-endian key + value per (key, value, width)."""
    body = bytes([0x00, 0x01, 0x00, 0x00])
    for key, val, width in kvs:
        body += key.to_bytes(4, 'little') + val.to_bytes(width, 'little')
    return body


def valset_frame(kvs):
    """Complete CFG-VALSET frame for a (key, value, width) list."""
    return ubx_frame(UBX_CLS_CFG, UBX_ID_VALSET, valset_body(kvs))


class UbxNavPvtDriver(StreamDriver):
    """Base for receivers streaming UBX-NAV-PVT epochs (u-blox M9/F9)."""

    def apply_nav_pvt_outputs(self):
        """Declare the NAV-PVT output table and sample size.

        The frame commits as [class, id, len(2), payload(92), CK_A, CK_B],
        so payload byte P sits at offset 4 + P, little-endian. `altitude`
        is hMSL; the ellipsoidal height ships as `alt_ellipsoid`. Not
        published: year..sec, valid, tAcc, nano (the epoch rides `itow`),
        flags, flags2, flags3, headAcc, headVeh, magDec, magAcc.
        """
        lla = 1e-7 * _DEG_TO_RAD     # 1e-7 deg -> rad (lat/lon)
        hdg = 1e-5 * _DEG_TO_RAD     # 1e-5 deg -> rad (heading)
        mm = 0.001                    # mm -> m, mm/s -> m/s
        # 16 outputs is the descriptor table's limit; a 17th field overflows
        # it for both drivers.
        self.set_output([
            {'name': 'itow', 'type': 'uint32', 'byte_order': 'little',
             'at': 4,                                                # pl 0
             'scale': 0.001},
            {'name': 'fix_type', 'type': 'uint8', 'at': 24,          # pl 20
             'scale': 1.0, 'unit': ''},
            {'name': 'num_sv', 'type': 'uint8', 'at': 27,            # pl 23
             'scale': 1.0, 'unit': 'count'},
            {'name': 'longitude', 'type': 'int32', 'byte_order': 'little',
             'at': 28, 'scale': lla},                                 # pl 24
            {'name': 'latitude', 'type': 'int32', 'byte_order': 'little',
             'at': 32, 'scale': lla},                                 # pl 28
            {'name': 'alt_ellipsoid', 'type': 'int32', 'byte_order': 'little',
             'at': 36, 'scale': mm, 'unit': 'm'},                     # pl 32
            {'name': 'altitude', 'type': 'int32', 'byte_order': 'little',
             'at': 40, 'scale': mm},                                  # pl 36, hMSL
            {'name': 'pos_h_acc', 'type': 'uint32', 'byte_order': 'little',
             'at': 44, 'scale': mm},                                  # pl 40
            {'name': 'pos_v_acc', 'type': 'uint32', 'byte_order': 'little',
             'at': 48, 'scale': mm},                                  # pl 44
            {'name': 'vel_north', 'type': 'int32', 'byte_order': 'little',
             'at': 52, 'scale': mm},                                  # pl 48
            {'name': 'vel_east', 'type': 'int32', 'byte_order': 'little',
             'at': 56, 'scale': mm},                                  # pl 52
            {'name': 'vel_down', 'type': 'int32', 'byte_order': 'little',
             'at': 60, 'scale': mm},                                  # pl 56
            {'name': 'speed', 'type': 'int32', 'byte_order': 'little',
             'at': 64, 'scale': mm},                                  # pl 60, gSpeed
            {'name': 'heading', 'type': 'int32', 'byte_order': 'little',
             'at': 68, 'scale': hdg, 'unit': 'rad'},                  # pl 64, headMot
            {'name': 'vel_s_acc', 'type': 'uint32', 'byte_order': 'little',
             'at': 72, 'scale': mm},                                  # pl 68, sAcc
            {'name': 'pdop', 'type': 'uint16', 'byte_order': 'little',
             'at': 80, 'scale': 0.01, 'unit': ''},                    # pl 76
        ])
        self.set_sample_size(98)     # class+id+len(4) + payload(92) + CK(2)

    @StreamDriver.measure_loop(trigger="poll", sample_rate=100,
                               when=("protocol", "binary"), default=True)
    def measure_binary(self):
        self.read_until(b'\xb5\x62')            # sync to a UBX frame
        self.stamp_frame()                       # acquisition = first byte arrival
        self.read_n(98)                          # class+id+len+payload(92)+CK
        m = self.match(0x01, 0x07, 0x5c, 0x00)   # NAV-PVT (0x01 0x07), len=92 (LE)
        if m != 0:
            return None                          # not NAV-PVT -> resync next pass
        bad = self.verify_checksum(ChecksumFletcher(), 0, 96, 96)
        if bad != 0:
            return None                          # corrupt frame -> drop
        self.store_sample()                      # fixed 98-B frame at buf[0]
