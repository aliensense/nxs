# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""The knob model of `nxs tune`: a field carries the options a node offers,
a section groups them, and a channel is one node of the rig."""


class Field:
    """One panel knob: a name and its stepped options."""

    def __init__(self, name, options, index, render=str, unit=None, bounds=None,
                 pinned=None):
        self.name = name
        self.options = options
        #: (lo, hi) for a range parameter: any integer inside is a lawful
        #: value, offered or not.
        self.bounds = bounds
        #: An option that stays offered whatever the scope: the value the
        #: manifest declares, shown even when no hub offers it.
        self.pinned = pinned
        self.index = max(0, index)
        self.render = render
        self.unit = unit
        #: A parameter's compiled default: a save writes a value that differs
        #: from it, or one the file already carries.
        self.default = None

    @property
    def value(self):
        return self.options[self.index]

    def step(self, delta):
        # A stepped knob is the operator's word: a save spells it out.
        self.default = None
        self.index = (self.index + delta) % len(self.options)

class Section:
    """One group of a channel's knobs: its DECLARE knobs, or one node's rows
    (a hub's, a camera's, a unit's, a click's on a unit)."""

    def __init__(self, label, fields, kind, unit_name=None,
                 sensor_index=None, rebuild=None):
        self.label = label
        self.fields = fields
        self.kind = kind  # "declare" | "rows"
        self.unit_name = unit_name
        self.sensor_index = sensor_index
        #: Re-derives the fields that depend on others after a step; returns
        #: (note, snapped): an operator line and whether a value changed.
        self.rebuild = rebuild

def refresh(section):
    """Run a section's rebuild after one of its knobs stepped."""
    if section is None or section.rebuild is None:
        return None, False
    return section.rebuild()

#: The option that declares nothing: a hub not chosen, a link without a
#: sensor, a unit without a personality.
NONE = "(none)"


def token(option):
    """The addressable value of an option: the first of a (token, label) pair."""
    return option[0] if isinstance(option, tuple) else option

class Channel:
    """One channel = one node of the rig: a PORT, one of its LINKs, or a UNIT
    (a pod on a link, or a unit on a bus). An undeclared node opens with a
    DECLARE section whose knobs write its entry at save."""

    def __init__(self, name, sections, kind="port", declared=True,
                 presence="unknown", template=None, note=""):
        self.name = name
        self.sections = sections
        self.kind = kind  # "port" | "link" | "unit"
        self.declared = declared
        self.presence = presence  # "present" | "absent" | "unknown"
        #: What an undeclared node is shaped on: a port's platform port (a
        #: Topology), a unit's (route, serial).
        self.template = template
        self.note = note
        #: A declared port's spec.
        self.port = None
        #: The PortFamily a port or link channel belongs to.
        self.family = None
        #: A link channel's letter and port.
        self.letter = None
        self.port_name = None
        #: A link's declared mode token and free-run rate.
        self.declared_mode = None
        self.declared_fps = None
        #: A declared unit's spec, the link a pod rides ("cam1/A"), and the
        #: click personality a found unit reports running.
        self.unit_spec = None
        self.link_ref = None
        self.running = None
        #: A found unit `generate` declares whatever it runs.
        self.adopt = False

    @property
    def label(self):
        tag = "" if self.declared else " · new"
        return f"{self.kind.upper()} {self.name}{tag}"

    def declare(self):
        """The DECLARE section, or None for a node with nothing to declare."""
        return next((s for s in self.sections if s.kind == "declare"), None)

    def knobs(self):
        """The DECLARE knobs' tokens by name."""
        declare = self.declare()
        return {f.name: token(f.value) for f in declare.fields} if declare else {}

    @property
    def fields(self):
        return [f for sec in self.sections for f in sec.fields]

    def section_of(self, flat_index):
        for sec in self.sections:
            if flat_index < len(sec.fields):
                return sec
            flat_index -= len(sec.fields)
        return self.sections[-1] if self.sections else None

class _EmptyConfig:
    """What a host without a manifest declares: nothing."""
    ports: dict = {}
    units: list = []

def _platform_port_buses():
    """{port name: bus} from the host layer; {} off-target."""
    from nxs import host as host_layer

    try:
        return dict(host_layer.current().camera_buses())
    except Exception:
        return {}

def _platform_ports():
    """{port name: Topology}: the hub's port shape per platform port."""
    from nxs.cam import topology as cam_topo

    try:
        ports = cam_topo._ports_from_platform()
    except Exception:
        return {}
    if not ports:
        return {}
    return {t.carrier.rsplit("/", 1)[-1]: t for t in ports[0].values()}

def _unit_name(link):
    from nxs.suite.scan import _suggest_name
    return _suggest_name(link)
