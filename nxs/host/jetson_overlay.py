# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""The Jetson capture contract, generated from the sensor descriptors: one
overlay per port and lane count, carrying both virtual channels, one
``modeN`` block per entry of every sensor's ``capture.table`` in pack order,
the node's register widths, and the control table the kernel driver
interprets, one ``ctrl@N`` per sensor of the pool. A direct port's overlay
carries the one sensor node on virtual channel 0, and its modes configure
the receiver for the sensor's own lanes: no SerDes pixel clock, the clock
mode the sensor declares."""

from __future__ import annotations

import dataclasses
from typing import Any, Dict, List, Optional, Sequence

from .capture_table import REG_BITS_DEFAULT, VAL_BITS_DEFAULT, CaptureMode, PoolSensor

#: The p3768 carrier's two camera ports as the overlays wire them; `vc1` is
#: the second module / VI port / NVCSI channel added for virtual channel 1.
#: The capture stack numbers its sensors by module order. Each port's
#: `module` is the one the mux overlay already declares (the overlay
#: targets it), `vc1.module` the one this overlay creates. A port's link
#: A rides VC1 (link B is the dual's primary on VC0), so the declared,
#: lower-numbered module carries the VC1 node and the created one the
#: VC0 node: cam0 A is capture id 1 and B is 2, cam1 A is 0 and B is 3.
#: `enable` is the connector's camera-enable line (NVIDIA's CAM0_PWDN and
#: CAM1_PWDN) and the hog the foundation overlay holds it low with.
#: Behind a hub each channel's node carries a host alias (`sensor_addr`,
#: `vc1.sensor_addr`), an address no part on the port straps: the link's
#: serializer maps it to the sensor's own, so the kernel's per-frame
#: controls for one channel reach one head. A direct port's node carries
#: the sensor's own address.
PORTS: Dict[str, Dict[str, Any]] = {
    "cam1": dict(
        module=0, i2c_bus=9, mux="cam_i2cmux_i2c1", mux_path="i2c@1",
        node="c", sensor_addr=0x1C, badge="universal_rear_aliensense",
        vi_port=2, csi_ch=2, port_index=2, devnode="video1",
        physical=("12.00", "9.30"), sinterface="serial_c", lane_polarity=0,
        enable=dict(gpio=("AC", 0), hog="som_gpio_cam1_rst"),
        vc1=dict(module=3, position="topright", node="d", sensor_addr=0x1D,
                 vi_port=3, csi_ch=3, endpoints=(6, 7)),
    ),
    "cam0": dict(
        module=1, i2c_bus=10, mux="cam_i2cmux_i2c0", mux_path="i2c@0",
        node="b", sensor_addr=0x1B, badge="universal_centerright_aliensense",
        vi_port=1, csi_ch=1, port_index=1, devnode="video0",
        physical=("7.22", "7.05"), sinterface="serial_b",
        # The carrier inverts CAM0's two data lanes (NVIDIA's own imx219-A
        # overlay says so); without it VI times out.
        lane_polarity=6,
        enable=dict(gpio=("H", 6), hog="som_gpio_cam0_rst"),
        vc1=dict(module=2, position="bottomright", node="e", sensor_addr=0x1E,
                 vi_port=0, csi_ch=0, endpoints=(0, 1)),
    ),
}

#: The main GPIO controller the enable lines hang off, and its port indices
#: (dt-bindings/gpio/tegra234-gpio.h): TEGRA234_MAIN_GPIO(port, pin) is the
#: port's index times the pins per port, plus the pin.
MAIN_GPIO_PATH = "/bus@0/gpio@2200000"
_MAIN_GPIO_PORTS = {"H": 7, "AC": 20}
_MAIN_GPIO_PINS_PER_PORT = 8

#: TEGRA234_CLK_EXTPERIPH1 (dt-bindings/clock/tegra234-clock.h): the sensor
#: MCLK the carrier routes to the camera connectors.
CLK_EXTPERIPH1 = 36
#: JETSON_COMPATIBLE_P3768 (dt-bindings/tegra234-p3767-0000-common.h).
COMPATIBLE_P3768 = (
    "nvidia,p3768-0000+p3767-0000", "nvidia,p3768-0000+p3767-0001",
    "nvidia,p3768-0000+p3767-0003", "nvidia,p3768-0000+p3767-0004",
    "nvidia,p3768-0000+p3767-0005", "nvidia,p3768-0000+p3767-0000-super",
    "nvidia,p3768-0000+p3767-0001-super", "nvidia,p3768-0000+p3767-0003-super",
    "nvidia,p3768-0000+p3767-0004-super", "nvidia,p3768-0000+p3767-0005-super",
)
#: The deserializer's nominal pixel clock the driver predefines for GMSL.
SERDES_PIX_CLK_HZ = 12_000_000_000
#: The clock a pod feeds its sensor: the node's clock behind a hub.
POD_INCK_HZ = 37_125_000


def _fps(value: float, factor: int) -> str:
    return str(int(round(float(value) * int(factor))))


def mode_properties(mode: CaptureMode, port: Dict[str, Any], lanes: int,
                    vc: int, direct: bool = False) -> List[tuple]:
    """A ``modeN`` block's properties in the driver's order, values as strings.
    The port's lane polarity overrides the sensor's own (an explicit 0 included);
    with neither, no property is written. Behind a hub the receiver sees the
    deserializer's output: a continuous clock at the SerDes pixel clock. A
    direct port's receiver sees the sensor: its declared clock mode, and no
    SerDes pixel clock."""
    port_polarity = port.get("lane_polarity")
    if port_polarity is None:
        polarity = mode.lane_polarity
    elif int(port_polarity):
        polarity = int(port_polarity)
    else:
        # A non-inverted port: the sensor's declaration cannot invert the
        # lanes; it only decides whether the property is written.
        polarity = 0 if mode.lane_polarity is not None else None
    props: List[tuple] = [
        ("aliensense_sensors_pool", f"{mode.pool};"),
        ("mclk_khz", str(mode.mclk_khz)),
        ("num_lanes", str(int(lanes))),
        ("tegra_sinterface", str(port["sinterface"])),
        ("vc_id", str(int(vc))),
        ("phy_mode", "DPHY"),
        ("discontinuous_clk", "yes" if direct and mode.clock_noncontinuous else "no"),
        ("dpcm_enable", "false"),
        ("cil_settletime", "0"),
    ]
    if polarity is not None:
        props.append(("lane_polarity", str(int(polarity))))
    fr = mode.framerate
    ex = mode.exposure
    g = mode.gain
    max_exp_us = mode.exposure_ceiling_us()
    props += [
        ("active_w", str(mode.width)),
        ("active_h", str(mode.height)),
        ("dynamic_pixel_bit_depth", str(mode.bit_depth)),
        ("csi_pixel_bit_depth", str(mode.bit_depth)),
        ("mode_type", "bayer"),
        ("pixel_phase", mode.pixel_phase),
        ("readout_orientation", "0"),
        ("line_length", str(mode.line_length)),
        ("inherent_gain", "1"),
        ("pix_clk_hz", str(mode.pix_clk_hz)),
    ]
    if not direct:
        props.append(("serdes_pix_clk_hz", str(SERDES_PIX_CLK_HZ)))
    props += [
        ("gain_factor", str(int(g["factor"]))),
        ("min_gain_val", str(int(g["min"]))),
        ("max_gain_val", str(int(g["max"]))),
        ("step_gain_val", str(int(g["step"]))),
        ("default_gain", str(int(g["default"]))),
        ("min_hdr_ratio", str(int(mode.hdr_ratio["min"]))),
        ("max_hdr_ratio", str(int(mode.hdr_ratio["max"]))),
        ("framerate_factor", str(int(fr["factor"]))),
        ("min_framerate", _fps(fr["min_fps"], fr["factor"])),
        ("max_framerate", _fps(mode.max_fps, fr["factor"])),
        ("step_framerate", str(int(fr["step"]))),
        ("default_framerate", _fps(mode.default_fps if mode.default_fps is not None
                                   else mode.max_fps, fr["factor"])),
        ("exposure_factor", str(int(ex["factor"]))),
        ("min_exp_time", str(int(mode.min_exp_us))),
        ("max_exp_time", str(max_exp_us)),
        ("step_exp_time", str(int(ex["step"]))),
        # The kernel refuses a control range whose default lies outside
        # it, and it takes the first row's at probe.
        ("default_exp_time", str(min(max(int(ex["default_us"]), int(mode.min_exp_us)),
                                     max_exp_us))),
        ("embedded_metadata_height", str(int(mode.embedded_lines))),
    ]
    return props


def _mode_block(index: int, props: List[tuple], indent: str) -> str:
    lines = [f"{indent}mode{index} {{"]
    for key, value in props:
        lines.append(f'{indent}\t{key} = "{value}";')
    lines.append(f"{indent}}};")
    return "\n".join(lines) + "\n"


def _control_node(pools: Sequence[PoolSensor], indent: str) -> str:
    """The ``aliensense-ctrl`` child: one ``ctrl@N`` per pool sensor with
    its identity, clock, and control rows."""
    if not pools:
        return ""
    i = indent
    lines = [f"{i}aliensense-ctrl {{",
             f"{i}\t#address-cells = <1>;",
             f"{i}\t#size-cells = <0>;"]
    for n, sensor in enumerate(pools):
        lines += [f"{i}\tctrl@{n} {{",
                  f"{i}\t\treg = <{n}>;",
                  f'{i}\t\taliensense,sensor = "{sensor.compatible}";']
        if sensor.inck_hz is not None:
            lines.append(f"{i}\t\taliensense,inck-hz = <{sensor.inck_hz}>;")
        for name, row in sensor.controls:
            cells = " ".join([f"0x{row[0]:x}", *(str(v) for v in row[1:])])
            lines.append(f"{i}\t\taliensense,ctrl-{name} = <{cells}>;")
        lines.append(f"{i}\t}};")
    lines.append(f"{i}}};")
    return "\n".join(lines) + "\n"


def _sensor_node(port: Dict[str, Any], label: str, node: str, addr: int,
                 table: Sequence[CaptureMode], lanes: int, vc: int,
                 csi_in_label: str, out_label: str, indent: str,
                 reg_bits: int = REG_BITS_DEFAULT, val_bits: int = VAL_BITS_DEFAULT,
                 pools: Sequence[PoolSensor] = (), direct: bool = False) -> str:
    """The port's sensor node: driver identity, register widths, clocks,
    the mode table, the control table, and the endpoint into the NVCSI
    channel. Behind a hub the clock is the pods' INCK; a direct port's is
    the lead sensor's own."""
    i = indent
    w, h = port["physical"]
    clock_hz = int(table[0].mclk_khz) * 1000 if direct else POD_INCK_HZ
    head = f'''{i}{label}: universal_{node}@{addr:x} {{
{i}\tcompatible = "aliensense,universal";
{i}\treg = <0x{addr:x}>;
{i}\taliensense,reg-bits = <{reg_bits}>;
{i}\taliensense,val-bits = <{val_bits}>;
{i}\tclocks = <&bpmp {CLK_EXTPERIPH1}>, <&bpmp {CLK_EXTPERIPH1}>;
{i}\tclock-names = "extperiph1", "pllp_grtba";
{i}\tmclk = "extperiph1";
{i}\tclock-frequency = <{clock_hz}>;
{i}\tdevnode = "{port['devnode']}";
{i}\tphysical_w = "{w}";
{i}\tphysical_h = "{h}";
{i}\tsensor_model = "universal";
{i}\tpost_crop_frame_drop = "0";
{i}\tuse_decibel_gain = "true";
{i}\tdelayed_gain = "false";
{i}\tuse_sensor_mode_id = "true";
'''
    modes = "".join(_mode_block(n, mode_properties(m, port, lanes, vc, direct), i + "\t")
                    for n, m in enumerate(table))
    modes += _control_node(pools, i + "\t")
    tail = f'''{i}\tports {{
{i}\t\t#address-cells = <1>;
{i}\t\t#size-cells = <0>;
{i}\t\tport@0 {{
{i}\t\t\treg = <0>;
{i}\t\t\t{out_label}: endpoint {{
{i}\t\t\t\tvc-id = <{vc}>;
{i}\t\t\t\tport-index = <{port['port_index']}>;
{i}\t\t\t\tbus-width = <{lanes}>;
{i}\t\t\t\tremote-endpoint = <&{csi_in_label}>;
{i}\t\t\t}};
{i}\t\t}};
{i}\t}};
{i}}};
'''
    return head + modes + tail


def overlay_name(port_name: str, lanes: int, direct: bool = False) -> str:
    return (f"Universal-camera-{port_name.upper()}-{lanes}Lane"
            + ("-Direct" if direct else ""))


def dtbo_basename(port_name: str, lanes: int, direct: bool = False) -> str:
    """The shipped file name for this overlay. A direct port's keeps the
    `-<port>-` the boot label's bookkeeping finds a port's overlays by."""
    return (f"tegra234-p3767-camera-p3768-aliensense_universal-{port_name}-"
            f"{lanes}lane-{'direct-' if direct else ''}overlay.dtbo")


def _header(port_name: str, lanes: int, direct: bool = False) -> str:
    compat = ", ".join(f'"{c}"' for c in COMPATIBLE_P3768)
    return f'''/*
 * Generated by nxs (nxs switch) from the descriptor pack's sensor
 * capture tables — do not edit; regenerate.
 */

/dts-v1/;
/plugin/;

/ {{
\toverlay-name = "{overlay_name(port_name, lanes, direct)}";
\tjetson-header-name = "Jetson 24pin CSI Connector";
\tcompatible = {compat};

'''


@dataclasses.dataclass(frozen=True)
class NodeIdentity:
    """What the driver's register map is configured from: the VC0 node's
    address and the register and value widths of every node."""

    addr: int
    reg_bits: int
    val_bits: int


def node_identity(port: Dict[str, Any], pools: Sequence[PoolSensor],
                  direct: bool = False, node_addr: Optional[int] = None) -> NodeIdentity:
    """The pool's lead sensor names the node's widths; every other pool
    sensor must share them (one register map serves the pool). The
    address is the port template's alias behind a hub and, on a direct
    port, ``node_addr`` (where the wiring says the sensor answers) else
    the lead sensor's own. Without a pool the template's address and the
    default widths."""
    if not pools:
        return NodeIdentity(int(port["sensor_addr"]), REG_BITS_DEFAULT, VAL_BITS_DEFAULT)
    lead = pools[0]
    for other in pools[1:]:
        if (other.reg_bits, other.val_bits) != (lead.reg_bits, lead.val_bits):
            raise ValueError(
                f"one register map cannot serve {lead.compatible} "
                f"({lead.reg_bits}/{lead.val_bits}-bit) and {other.compatible} "
                f"({other.reg_bits}/{other.val_bits}-bit) on one node")
    if direct:
        addr = int(node_addr) if node_addr is not None else lead.i2c_addr
    else:
        addr = int(port["sensor_addr"])
    return NodeIdentity(addr, lead.reg_bits, lead.val_bits)


def node_aliases(port_name: str) -> Dict[int, int]:
    """The host alias each channel's node carries on a hub port, {vc: addr}."""
    port = PORTS[port_name]
    return {0: int(port["sensor_addr"]), 1: int(port["vc1"]["sensor_addr"])}


def _vc0_fragments(port: Dict[str, Any], lanes: int,
                   table: Sequence[CaptureMode], identity: NodeIdentity,
                   pools: Sequence[PoolSensor], direct: bool = False) -> str:
    """Fragments 0-8: the carrier's own module (declared by the mux
    overlay; it takes the VC1 node, link A's, so link A gets the lower
    capture id), VI port and NVCSI channel switched on, and the VC0
    sensor node. A direct port has the one node, so its module takes it."""
    n = port["node"]
    vi, ch = port["vi_port"], port["csi_ch"]
    addr = identity.addr
    v = port["vc1"]
    module_node, module_addr = (n, addr) if direct else (v["node"], v["sensor_addr"])
    body = f'''\tfragment@0 {{
\t\ttarget = <&sensor_module{port['module']}>;
\t\t__overlay__ {{
\t\t\tstatus = "okay";
\t\t\tbadge = "{port['badge']}";
\t\t\tdrivernode0 {{
\t\t\t\tstatus = "okay";
\t\t\t\tdevname = "universal {port['i2c_bus']}-00{module_addr:02x}";
\t\t\t\tsysfs-device-tree = "/sys/firmware/devicetree/base/bus@0/cam_i2cmux/{port['mux_path']}/universal_{module_node}@{module_addr:x}";
\t\t\t}};
\t\t}};
\t}};

\tfragment@1 {{
\t\ttarget = <&sensor_vi_port{vi}>;
\t\t__overlay__ {{
\t\t\tstatus = "okay";
\t\t}};
\t}};

\tfragment@2 {{
\t\ttarget = <&sensor_vi_in{vi}>;
\t\t__overlay__ {{
\t\t\tstatus = "okay";
\t\t\tbus-width = <{lanes}>;
\t\t}};
\t}};

\tfragment@3 {{
\t\ttarget = <&sensor_csi_ch{ch}>;
\t\t__overlay__ {{
\t\t\tstatus = "okay";
\t\t\tports {{
\t\t\t\tstatus = "okay";
\t\t\t}};
\t\t}};
\t}};

\tfragment@4 {{
\t\ttarget = <&sensor_csi_ch{ch}_port0>;
\t\t__overlay__ {{
\t\t\tstatus = "okay";
\t\t}};
\t}};

\tfragment@5 {{
\t\ttarget = <&sensor_csi_ch{ch}_in>;
\t\t__overlay__ {{
\t\t\tstatus = "okay";
\t\t\tbus-width = <{lanes}>;
\t\t\tremote-endpoint = <&sensor_out_{n}>;
\t\t}};
\t}};

\tfragment@6 {{
\t\ttarget = <&sensor_csi_ch{ch}_port1>;
\t\t__overlay__ {{
\t\t\tstatus = "okay";
\t\t}};
\t}};

\tfragment@7 {{
\t\ttarget = <&sensor_csi_ch{ch}_out>;
\t\t__overlay__ {{
\t\t\tstatus = "okay";
\t\t}};
\t}};

\tfragment@8 {{
\t\ttarget = <&{port['mux']}>;
\t\t__overlay__ {{
'''
    body += _sensor_node(port, f"sensor_{n}", n, addr, table, lanes, 0,
                         f"sensor_csi_ch{ch}_in", f"sensor_out_{n}", "\t\t\t",
                         identity.reg_bits, identity.val_bits, pools, direct)
    body += "\t\t};\n\t};\n"
    return body


def _enable_fragment(port_name: str, port: Dict[str, Any]) -> str:
    """Fragment 9 of a direct overlay: the connector's enable line held high.
    The foundation overlay holds it low for a hub; a camera wired to the
    connector is powered by it, as NVIDIA's own camera overlays drive it.
    An overlay cannot delete a property, so the low hog is switched off and
    a high one takes the line."""
    enable = port["enable"]
    gpio_port, pin = enable["gpio"]
    line = _MAIN_GPIO_PORTS[gpio_port] * _MAIN_GPIO_PINS_PER_PORT + int(pin)
    return f'''\tfragment@9 {{
\t\ttarget-path = "{MAIN_GPIO_PATH}";
\t\t__overlay__ {{
\t\t\t{enable['hog']} {{
\t\t\t\tstatus = "disabled";
\t\t\t}};
\t\t\tdirect_{port_name}_enable {{
\t\t\t\tgpio-hog;
\t\t\t\toutput-high;
\t\t\t\tgpios = <{line} 0>;
\t\t\t\tlabel = "{port_name}_enable";
\t\t\t\tstatus = "okay";
\t\t\t}};
\t\t}};
\t}};
'''


def _vc1_fragments(port: Dict[str, Any], lanes: int,
                   table: Sequence[CaptureMode], identity: NodeIdentity,
                   pools: Sequence[PoolSensor]) -> str:
    """Fragments 9-11: a second module (created here; it takes the VC0
    node, link B's), VI port and NVCSI channel, and the VC1 sensor node at
    the port's alias address."""
    v = port["vc1"]
    n = v["node"]
    vi, ch = v["vi_port"], v["csi_ch"]
    e_in, e_out = v["endpoints"]
    addr = v["sensor_addr"]
    pi = port["port_index"]
    body = f'''\tfragment@9 {{
\t\ttarget-path = "/";
\t\t__overlay__ {{
\t\t\ttegra-camera-platform {{
\t\t\t\tmodules {{
\t\t\t\t\tstatus = "okay";
\t\t\t\t\tsensor_module{v['module']}: module{v['module']} {{
\t\t\t\t\t\tstatus = "okay";
\t\t\t\t\t\tposition = "{v['position']}";
\t\t\t\t\t\torientation = "1";
\t\t\t\t\t\tbadge = "{port['badge']}";
\t\t\t\t\t\tdrivernode0 {{
\t\t\t\t\t\t\tstatus = "okay";
\t\t\t\t\t\t\tpcl_id = "v4l2_sensor";
\t\t\t\t\t\t\tdevname = "universal {port['i2c_bus']}-00{identity.addr:02x}";
\t\t\t\t\t\t\tsysfs-device-tree = "/sys/firmware/devicetree/base/bus@0/cam_i2cmux/{port['mux_path']}/universal_{port['node']}@{identity.addr:x}";
\t\t\t\t\t\t}};
\t\t\t\t\t}};
\t\t\t\t}};
\t\t\t}};

\t\t\ttegra-capture-vi {{
\t\t\t\tnum-channels = <4>;
\t\t\t\tports {{
\t\t\t\t\t#address-cells = <1>;
\t\t\t\t\t#size-cells = <0>;
\t\t\t\t\tstatus = "okay";
\t\t\t\t\tsensor_vi_port{vi}: port@{vi} {{
\t\t\t\t\t\treg = <{vi}>;
\t\t\t\t\t\tstatus = "okay";
\t\t\t\t\t\tsensor_vi_in{vi}: endpoint {{
\t\t\t\t\t\t\tstatus = "okay";
\t\t\t\t\t\t\tvc-id = <1>;
\t\t\t\t\t\t\tport-index = <{pi}>;
\t\t\t\t\t\t\tbus-width = <{lanes}>;
\t\t\t\t\t\t\tremote-endpoint = <&sensor_csi_ch{ch}_out>;
\t\t\t\t\t\t}};
\t\t\t\t\t}};
\t\t\t\t}};
\t\t\t}};
\t\t}};
\t}};

\tfragment@10 {{
\t\ttarget-path = "/bus@0";
\t\t__overlay__ {{
\t\t\thost1x@13e00000 {{
\t\t\t\tnvcsi@15a00000 {{
\t\t\t\t\tnum-channels = <4>;
\t\t\t\t\t#address-cells = <1>;
\t\t\t\t\t#size-cells = <0>;
\t\t\t\t\tsensor_csi_ch{ch}: channel@{ch} {{
\t\t\t\t\t\treg = <{ch}>;
\t\t\t\t\t\tstatus = "okay";
\t\t\t\t\t\tports {{
\t\t\t\t\t\t\t#address-cells = <1>;
\t\t\t\t\t\t\t#size-cells = <0>;
\t\t\t\t\t\t\tstatus = "okay";
\t\t\t\t\t\t\tsensor_csi_ch{ch}_port0: port@0 {{
\t\t\t\t\t\t\t\treg = <0>;
\t\t\t\t\t\t\t\tstatus = "okay";
\t\t\t\t\t\t\t\tsensor_csi_ch{ch}_in: endpoint@{e_in} {{
\t\t\t\t\t\t\t\t\tstatus = "okay";
\t\t\t\t\t\t\t\t\tport-index = <{pi}>;
\t\t\t\t\t\t\t\t\tbus-width = <{lanes}>;
\t\t\t\t\t\t\t\t\tremote-endpoint = <&sensor_out_{n}>;
\t\t\t\t\t\t\t\t}};
\t\t\t\t\t\t\t}};
\t\t\t\t\t\t\tsensor_csi_ch{ch}_port1: port@1 {{
\t\t\t\t\t\t\t\treg = <1>;
\t\t\t\t\t\t\t\tstatus = "okay";
\t\t\t\t\t\t\t\tsensor_csi_ch{ch}_out: endpoint@{e_out} {{
\t\t\t\t\t\t\t\t\tstatus = "okay";
\t\t\t\t\t\t\t\t\tremote-endpoint = <&sensor_vi_in{vi}>;
\t\t\t\t\t\t\t\t}};
\t\t\t\t\t\t\t}};
\t\t\t\t\t\t}};
\t\t\t\t\t}};
\t\t\t\t}};
\t\t\t}};
\t\t}};
\t}};

\tfragment@11 {{
\t\ttarget = <&{port['mux']}>;
\t\t__overlay__ {{
'''
    body += _sensor_node(port, f"sensor_{n}", n, addr, table, lanes, 1,
                         f"sensor_csi_ch{ch}_in", f"sensor_out_{n}", "\t\t\t",
                         identity.reg_bits, identity.val_bits, pools)
    body += "\t\t};\n\t};\n"
    return body


def overlay_dts(port_name: str, lanes: int, table: Sequence[CaptureMode],
                pools: Sequence[PoolSensor] = (), direct: bool = False,
                node_addr: Optional[int] = None) -> str:
    """The overlay source for a port, lane count, capture table, and sensor
    pool, both virtual channels included; a direct port's carries virtual
    channel 0 alone. Compiles with ``dtc -@`` alone."""
    if port_name not in PORTS:
        raise ValueError(f"no such camera port {port_name!r}; have {sorted(PORTS)}")
    if int(lanes) not in (2, 4):
        raise ValueError(f"a CSI port carries 2 or 4 lanes, got {lanes}")
    if not table:
        raise ValueError("an empty capture table makes no overlay")
    port = PORTS[port_name]
    identity = node_identity(port, pools, direct, node_addr)
    head = (_header(port_name, int(lanes), direct)
            + _vc0_fragments(port, int(lanes), table, identity, pools, direct))
    if direct:
        return head + "\n" + _enable_fragment(port_name, port) + "};\n"
    return head + "\n" + _vc1_fragments(port, int(lanes), table, identity, pools) + "};\n"
