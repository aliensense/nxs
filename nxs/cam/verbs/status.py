"""`status`: presence first, then the diagnosis sections with the unit's personality line."""

from __future__ import annotations

import argparse
import json
import time
from typing import Any, Dict, List, Optional, Tuple



from nxs.cam.descriptors import to_int
from nxs.cam.contracts import ContractError, LinkSpec, Topology
from nxs import host as host_layer
from nxs.finding import Finding
from nxs.cam import port_state
from nxs.cam import unit_source
from nxs.cam import run as cam_run
from nxs.cam import identity as cam_identity
from nxs.cam.follow import HEARTBEAT_STALE_S
from nxs.cam.identity import (_identity_read, answering_address, detect_sensor,
                              sensor_identity_line)
from nxs.cam.select import _pack_for, _port_name, select_port_links
from nxs.cam.verbs.sync import PULSE_EXPOSURE, sync_text
from nxs.cam.verbs.verify import verified_line

#: Why a presence walk read nothing: the bus lock stayed another run's past
#: the wait, most often nxsd's while it brings the ports up after boot.
BUS_HELD = ("another nxs run holds the bus after {wait:g} s of waiting "
            "(nxsd brings the ports up after boot)")
#: The wait before the hub's first register is read again: a hub that has
#: just come up can miss the first read.
HUB_RETRY_S = 0.1


def _hub_answers(i2c, reg: int, addr: int) -> bool:
    """Whether the hub answers its first register, read twice at most."""
    for attempt in range(2):
        if attempt:
            time.sleep(HUB_RETRY_S)
        try:
            i2c.read_reg(reg, reg_width=16, data_width=8, addr=hex(addr))
            return True
        except Exception:
            continue
    return False


def _presence_hub(payload: Dict[str, Any], pack, topology: Topology, i2c) -> bool:
    """The hub's part of a presence walk, written into `payload`: presence,
    then identity. True when the links may be walked."""
    desd = pack.descriptor(topology.des_compatible)
    # Presence: the hub's first register (REG0 where the descriptor names
    # it, address 0 otherwise) answers.
    first = desd.registers.get("REG0", {}).get("addr", 0)
    if not _hub_answers(i2c, to_int(first), topology.des_addr):
        payload["ok"] = False
        return False
    payload["hub"]["present"] = True
    facts = cam_identity.identity_facts(desd)
    if facts is None:
        # A hub the pack cannot identify is never walked (window
        # selection writes CTRL0): the probe says why and stops.
        payload["ok"] = False
        payload["error"] = (f"hub descriptor {topology.des_compatible} declares "
                            f"no identity register")
        return False
    id_reg, id_want, width = facts
    got = _identity_read(i2c, topology.des_addr, id_reg, width)
    verified = got == id_want
    payload["hub"].update({"id": int(got), "verified": verified})
    if not verified:
        # Not the silicon the pack describes: no window write and no walk;
        # the report carries the id it read.
        payload["ok"] = False
        return False
    # Window selection writes CTRL0; a kernel-owned hub gets only the
    # directly readable state above.
    return topology.hub_driver == "nxs"


def bus_held_text() -> str:
    """The held-bus line, naming the wait it outlasted."""
    return BUS_HELD.format(wait=port_state.HELD_WAIT_S)


def _hold_bus(payload: Dict[str, Any]) -> Optional[port_state.BusLock]:
    """The bus lock for a read of the port, waited for up to HELD_WAIT_S, or
    None with `payload` marked held: the lock stayed another run's, most
    often nxsd's after boot."""
    try:
        with port_state.held_wait():
            return port_state.BusLock().__enter__()
    except port_state.BusHeld:
        payload.update({"ok": False, "held": True, "error": bus_held_text()})
        return None


def presence_payload(topology: Topology,
                     selected: Optional[List[LinkSpec]] = None,
                     held: bool = False) -> Dict[str, Any]:
    """The presence walk of one port: hub identity, then each link's SER,
    SEN, and unit alias through the link window, and what each present
    unit holds and last ran. `held`: another port's walk already found the
    bus lock held, so this one is marked held without waiting again."""
    from nxs.schemas import CONTRACT

    pack = _pack_for(topology)
    flows = pack.flows()
    direct = topology.is_direct
    payload: Dict[str, Any] = {
        "contract": CONTRACT, "port": _port_name(topology),
        "bus": topology.i2c_bus, "ok": True,
        "walked": False, "links": [],
        # The port's CSI lane count against the booted overlay's (None
        # when the device tree does not say).
        "csi_lanes": int(topology.csi_lanes),
        "dt_lanes": host_layer.current().booted_lanes(topology.i2c_bus),
    }
    if payload["dt_lanes"] is not None and payload["dt_lanes"] != payload["csi_lanes"]:
        payload["ok"] = False
    if not direct:
        payload["hub"] = {"present": False, "id": None, "verified": None}
    if held:
        payload.update({"ok": False, "held": True, "error": bus_held_text()})
        return payload
    i2c = cam_run.CamI2c(addr=hex(topology.des_addr), bus=topology.i2c_bus)
    try:
        # The bus lock and the open are part of the probe: a held bus, a missing alias,
        # or a permission refusal is a structured payload naming why, never a no-ACK.
        lock = _hold_bus(payload)
        if lock is None:
            return payload
        try:
            i2c.open()
        except Exception as exc:
            payload["ok"] = False
            payload["error"] = f"cannot open {topology.i2c_bus}: {exc}"
            lock.__exit__(None, None, None)
            return payload
    except Exception as exc:  # the lock file itself
        payload["ok"] = False
        payload["error"] = str(exc)
        return payload
    try:
        if not direct and not _presence_hub(payload, pack, topology, i2c):
            return payload
        payload["walked"] = True
        found_sensors: Dict[str, str] = {}
        walked: List[Tuple[LinkSpec, Dict[str, Any]]] = []
        for link in selected or topology.links:
            flows.open_window(pack, i2c, topology, link)
            entry: Dict[str, Any] = {"name": link.name, "units": []}
            walked.append((link, entry))
            sen_addr, mapped = answering_address(pack, i2c, link)
            entry["sen_addr"] = sen_addr
            entry["mapped"] = mapped
            chain = [("sen", sen_addr)]
            if not direct:
                chain.insert(0, ("ser", link.ser_addr))
            for key, addr in chain:
                try:
                    i2c.read_reg(0x0000, reg_width=16, data_width=8,
                                 addr=hex(addr))
                    entry[key] = "present"
                except Exception:
                    entry[key] = "no ACK"
                    # A pod alone declares no head: silence there is the fact.
                    if key != "sen" or link.has_camera:
                        payload["ok"] = False
            if link.has_camera:
                detected, detail = detect_sensor(pack, i2c, link)
                ok, text = sensor_identity_line(
                    link, pack.descriptor(link.sensor_compatible), detected, detail)
                entry["sensor"] = {"declared": link.sensor_compatible,
                                   "detected": detected, "detail": detail,
                                   "ok": ok, "text": text}
                if detected:
                    found_sensors[link.name] = detected
                if not ok and entry["sen"] == "present":
                    payload["ok"] = False
            for unit in link.nxs_units:
                node: Dict[str, Any] = {"addr": int(unit.alias_addr), "state": "present"}
                try:
                    i2c.read_reg(0x0000, reg_width=16, data_width=8,
                                 addr=hex(unit.alias_addr))
                except Exception:
                    node["state"] = "no ACK"
                    payload["ok"] = False
                    # A pod silent at its alias is sought where it straps: a
                    # hub that lost its aliases (a power cycle) leaves it there.
                    if int(unit.target_addr) != int(unit.alias_addr):
                        node["strap"] = int(unit.target_addr)
                        try:
                            i2c.read_reg(0x0000, reg_width=16, data_width=8,
                                         addr=hex(unit.target_addr))
                            node["answers_at"] = int(unit.target_addr)
                        except Exception:
                            pass
                entry["units"].append(node)
            payload["links"].append(entry)
        flows.close_windows(pack, i2c, topology)
        if found_sensors:
            # Detection is remembered: the next `on` runs the sensor
            # that answered, no flag needed.
            port_state.set_sensors(topology, found_sensors)
    finally:
        i2c.close()
        lock.__exit__(None, None, None)
    # The units' personalities and their last run, read after the walk
    # (the read takes the bus lock itself) and cached for `on` and `pack_for`.
    for link, entry in walked:
        for unit in entry["units"]:
            if unit["state"] != "present":
                continue
            try:
                # The read takes the lock anew, under the same wait.
                with port_state.held_wait():
                    unit["personalities"] = unit_source.unit_personalities(topology, link)
            except port_state.BusHeld:
                payload.update({"ok": False, "held": True, "error": bus_held_text()})
                return payload
            except Exception as exc:
                unit["error"] = f"personalities unreadable ({exc})"
    return payload


def presence_rows(payload: Dict[str, Any]) -> List[str]:
    """The text rendering of a presence walk."""
    hub = payload.get("hub")
    rows: List[str] = []
    if payload.get("held"):
        # Whichever read met the lock, the walk's word is the held bus.
        return [bus_held_text() if hub is None else f"HUB: {bus_held_text()}"]
    if hub is None:
        if payload.get("error"):
            return [payload["error"]]
    elif not hub["present"]:
        return ["HUB: no ACK"]
    elif hub.get("id") is not None:
        rows.append(f"HUB: id 0x{hub['id']:02X} "
                    f"{'ok' if hub['verified'] else 'MISMATCH'}")
    else:
        rows.append("HUB: present")
    if "csi_lanes" in payload:
        booted = payload.get("dt_lanes")
        if booted is None:
            rows.append(f"CSI: port {payload['csi_lanes']}-lane "
                        "(booted overlay: device tree silent)")
        elif booted == payload["csi_lanes"]:
            rows.append(f"CSI: {booted}-lane (port and booted overlay agree)")
        else:
            rows.append(f"CSI: port {payload['csi_lanes']}-lane but the "
                        f"booted overlay declares {booted} lanes: MISMATCH")
    if not payload["walked"]:
        rows.append("links: not walked (kernel-owned hub)")
        return rows
    for link in payload["links"]:
        if "ser" in link:
            rows.append(f"link {link['name']} SER: {link['ser']}")
        sensor = link.get("sensor")
        if sensor and link["sen"] == "present":
            rows.append(f"link {link['name']} SEN: {sensor['text']}")
        elif sensor is None and link.get("sen_addr") is not None:
            rows.append(f"link {link['name']} SEN: none declared ({link['sen']} at "
                        f"{int(link['sen_addr']):#04x})")
        else:
            rows.append(f"link {link['name']} SEN: {link['sen']}")
        for unit in link["units"]:
            where = f"link {link['name']} NXS@{hex(unit['addr'])}"
            rows.append(f"{where}: {unit['state']}")
            # The unit's personalities, the measuring one and the camera one alike.
            rows.extend(f"{where}: {held['text']}" for held in unit.get("personalities", []))
            if unit.get("error"):
                rows.append(f"{where}: {unit['error']}")
    return rows


def status_payload(topology: Topology, sections, presence: Optional[Dict[str, Any]] = None
                   ) -> Dict[str, Any]:
    """The `status` surface: the presence walk and the diagnosis sections
    as data."""
    from nxs.schemas import CONTRACT

    out = []
    for title, results, derived in sections:
        out.append({
            "title": title,
            "results": [{"name": r.name, "text": r.text,
                         "raw": (int(r.raw) if r.raw is not None else None),
                         "ok": r.ok, "desc": getattr(r, "desc", "") or ""}
                        for r in results],
            "derived": [str(line) for line in derived],
        })
    bad = any(r.ok is False for _, results, _ in sections for r in results)
    payload = {"contract": CONTRACT, "port": _port_name(topology),
               "bus": topology.i2c_bus, "sync": topology.sync.source,
               "sync_live": port_state.port_sync(topology),
               "verified": port_state.verified(topology),
               "ae": pair_gain(topology),
               "ok": not bad and (presence is None or bool(presence["ok"])),
               "sections": out}
    if presence is not None:
        payload["presence"] = presence
    return payload


#: What the gap of a followed pair names next: the follower runs in nxsd.
FOLLOW_NEXT = "sudo systemctl restart nxsd"


def pair_gain(topology: Topology) -> Optional[Dict[str, Any]]:
    """Who decides the gain of the port's camera links, as `status --json`
    carries it (`ae`): the part the recorded sync names (the pack's
    `pair_ae`), `following` while nxsd copies a followed pair's gain (its
    heartbeat says so and is at most HEARTBEAT_STALE_S old), the
    heartbeat's age, the gain it copies and a stopped follower's reason.
    None where the record names no part."""
    ae = (port_state.port_sync(topology) or {}).get("ae")
    if not ae:
        return None
    gain: Dict[str, Any] = dict(ae, following=False, heartbeat_age_s=None)
    if ae.get("mode") != "follow":
        return gain
    gain["gain_db"] = None
    beat = port_state.read_follow(_port_name(topology)) or {}
    try:
        age = max(0.0, time.time() - float(beat["at"]))
    except (KeyError, ValueError):
        return gain
    gain["heartbeat_age_s"] = round(age, 1)
    if beat.get("state") == "stopped" and beat.get("reason"):
        gain["reason"] = beat["reason"]
    if beat.get("state") == "following" and age <= HEARTBEAT_STALE_S:
        gain["following"] = True
        if beat.get("gain_db") is not None:
            gain["gain_db"] = float(beat["gain_db"])
    return gain


def _pair_up(topology: Topology, gain: Dict[str, Any]) -> bool:
    """Whether both links of a followed pair are up: nxsd runs a follower
    on no other pair."""
    try:
        links = [topology.link(str(gain[part])) for part in ("leader", "follower")]
    except ContractError:
        return False
    return all(port_state.link_state(topology, link) == port_state.STATE_UP for link in links)


def follow_gap(topology: Topology, gain: Optional[Dict[str, Any]]) -> Optional[Finding]:
    """The gap of a followed pair whose links are up while nxsd copies no
    gain (`pair_gain` not `following`: no heartbeat, a stale one, or a
    follower its faults stopped, with the reason), and the next command;
    None otherwise."""
    if not gain or gain.get("mode") != "follow" or gain["following"]:
        return None
    if not _pair_up(topology, gain):
        return None
    port = _port_name(topology)
    why = f": {gain['reason']}" if gain.get("reason") else ""
    return Finding(f"{port}/{gain['follower']}",
                   f"does not follow {gain['leader']} (nxsd copies no gain for {port}{why})",
                   [FOLLOW_NEXT])


def pair_gain_lines(topology: Topology, gain: Optional[Dict[str, Any]],
                    sync: Dict[str, Any]) -> List[str]:
    """The lines under the sync line on who decides the gain of the port's
    camera links (`pair_gain`): a followed pair's leader and the gain nxsd
    copies, a declared lock with the exposure it runs, or each link's own
    loop on a pair; the gap and its next command where nxsd copies nothing
    (`follow_gap`). No line for one camera's own loop, nor for a followed
    pair whose links are not up, which the verdict names."""
    if not gain:
        return []
    port = _port_name(topology)
    mode = gain.get("mode")
    if mode == "follow":
        if not _pair_up(topology, gain):
            return []
        if not gain["following"]:
            return str(follow_gap(topology, gain)).splitlines()
        copied = f": {gain['gain_db']:.1f} dB on both" if gain.get("gain_db") is not None else ""
        return [f"{port} ae: {gain['leader']} leads, {gain['follower']} follows "
                f"(nxsd copies the gain each frame{copied})"]
    if mode == "locked":
        parts = ["locked"]
        if sync.get("source") == "fsync":
            if sync.get("pulse_exposure"):
                parts.append(f"exposure {PULSE_EXPOSURE}")
        elif topology.camera_exposure_us is not None:
            parts.append(f"exposure {float(topology.camera_exposure_us) / 1000:.1f} ms")
        parts.append(f"gain {float(gain['gain_db']):.1f} dB on {' and '.join(gain['links'])}")
        return [f"{port} ae: " + ", ".join(parts)]
    if topology.is_dual:
        return [f"{port} ae: per link ({gain.get('reason')})"]
    return []


def _unit_line(link: LinkSpec, entry: Dict[str, Any]) -> str:
    """One line per link on its unit: the alias's presence, the camera
    personality it holds and what the last run recorded; a link that
    declares none says so."""
    if not entry["units"]:
        return f"link {link.name} NXS: none declared"
    parts = []
    for unit in entry["units"]:
        text = f"link {link.name} NXS@0x{unit['addr']:02x}: {unit['state']}"
        for held in unit.get("personalities", []):
            text += f", {held['text']}"
        parts.append(text)
    return "\n".join(parts)


def cmd_status(args: argparse.Namespace) -> int:
    """Presence first (the hub, each link's chain and unit), then the
    descriptor-driven diagnosis of the selected links."""
    from nxs.cam.diag import probe_topology, render

    topology, selected = select_port_links(args)
    # The laws judge the port at the line the delivery check found for it.
    topology = port_state.with_found_lines(topology)
    pack = _pack_for(topology)
    as_json = getattr(args, "json", False)
    presence = presence_payload(topology, selected or None)
    sections = []
    hub = presence.get("hub")
    # The diagnosis reads what answered: behind a present hub, or the
    # walked sensor on the port's own bus. A bus the walk left held is
    # not waited for a second time.
    answered = hub["present"] if hub is not None else presence["walked"]
    if answered and not presence.get("held"):
        i2c = cam_run.CamI2c(addr=hex(topology.des_addr), bus=topology.i2c_bus)
        # The walk released the lock: a run that took it since holds the bus.
        lock = _hold_bus(presence)
        if lock is not None:
            try:
                i2c.open()
                try:
                    sections = probe_topology(i2c, topology, pack,
                                              links=selected or None)
                finally:
                    i2c.close()
            finally:
                lock.__exit__(None, None, None)
    payload = status_payload(topology, sections, presence)
    verdict, whole = verdict_lines(topology, pack, presence, selected or None)
    payload["verdict"] = {"ok": whole, "lines": verdict}
    # A pair's gain is the port's: nothing copying it fails the port as a
    # link down does.
    copied = follow_gap(topology, payload["ae"]) is None
    if as_json:
        print(json.dumps(payload, indent=2))
        return 0 if payload["ok"] and whole and copied else 1
    for line in verdict:
        print(line)
    if presence.get("held"):
        return 1
    live = port_state.port_sync(topology)
    declared = topology.sync.source
    if live:
        text = sync_text(live)
        if live["source"] != declared:
            text += f" (declared {declared})"
    else:
        text = f"{declared} (declared; no port state recorded)"
    print(f"{_port_name(topology)} on {topology.i2c_bus}, sync {text}")
    for line in pair_gain_lines(topology, payload["ae"], live or {}):
        print(line)
    seen = verified_line(topology, pack)
    if seen:
        print(seen)
    rows = presence_rows(presence)
    for row in rows:
        if row.startswith(("HUB:", "CSI:", "links:")):
            print(row)
    if "error" in presence:
        print(presence["error"])
    print()
    by_link: Dict[str, list] = {}
    port_sections = []
    for section in sections:
        title = section[0]
        if title.startswith("link "):
            by_link.setdefault(title.split()[1], []).append(section)
        else:
            port_sections.append(section)
    render(port_sections)
    entries = {e["name"]: e for e in presence["links"]}
    for link in selected or topology.links:
        entry = entries.get(link.name)
        if entry is not None:
            if "ser" in entry:
                print(f"link {link.name} SER: {entry['ser']}")
            sensor = entry.get("sensor")
            print(f"link {link.name} SEN: "
                  f"{sensor['text'] if sensor and entry['sen'] == 'present' else entry['sen']}")
            print(_unit_line(link, entry))
        render(by_link.get(link.name, []))
    return 0 if payload["ok"] and whole and copied else 1


def _pod_names(topology: Topology) -> Dict[str, str]:
    """Link name -> the declared pod's name, from the manifest when one
    declares this port."""
    import os

    from nxs.suite import default_config_path
    from nxs.suite.schema import ManifestError, load_suite_config

    path = default_config_path()
    if not os.path.exists(path):
        return {}
    try:
        port = load_suite_config(path).ports.get(_port_name(topology))
    except (ManifestError, OSError):
        return {}
    if port is None:
        return {}
    return {l.name: l.unit.name for l in port.links if l.unit is not None}


def verdict_lines(topology: Topology, pack, presence: Dict[str, Any],
                  selected) -> Tuple[List[str], bool]:
    """Declared against actual, one line for the port and one per declared
    link, and whether every declared link is up on a ready capture stack:
    `cam0: hub <compatible> ok, capture stack ready` and
    `cam0/A: <sensor> 1920x1080 RAW10 59.9 fps, up, pod unit-cam0-a
    (<personality>, head ok)`."""
    from nxs.cam.descriptors import mode_label
    from nxs.host.cli import capture_stack_state

    port = _port_name(topology)
    hub = presence.get("hub")
    lines: List[str] = []
    ok = True
    # A host without its camera kernel package boots no capture node: the
    # fact and the next command come first. A port of pods alone streams
    # no video: no package, no capture stack.
    cameras = bool(topology.camera_links)
    missing = host_layer.current().kernel_package_missing() if cameras else None
    if missing:
        lines += [missing, f"  - {host_layer.current().kernel_package_next()}"]
        ok = False
    if presence.get("held"):
        # Nothing was read: the port's line names why, and no link has a state.
        return lines + [f"{port}: {bus_held_text()}"], False
    if hub is not None:
        if hub.get("present") and hub.get("verified", True):
            head = f"{port}: hub {topology.des_compatible} ok"
        else:
            head = f"{port}: the hub does not answer at {topology.des_addr:#04x}"
            ok = False
    else:
        head = f"{port}: the sensor on the port's bus"
    try:
        stack = (capture_stack_state(host_layer.current(), port, topology)
                 if cameras else "ready")
    except Exception:        # noqa: BLE001 (a host that cannot say)
        stack = "ready"
    words = {"ready": "capture stack ready", "preparing": "preparing the capture stack",
             "missing": "capture stack not configured"}
    lines.append(f"{head}, {words[stack]}" if cameras else head)
    if cameras and stack == "missing":
        # A build the host cannot run says why before `switch` is asked.
        needed = host_layer.current().tuning_prerequisite_missing()
        if needed:
            lines += [needed, f"  - {host_layer.current().tuning_prerequisite_next()}"]
    ok = ok and stack == "ready"
    record = port_state.port_record(topology)
    modes = record.get("modes") or {}
    entries = {e["name"]: e for e in presence.get("links") or []}
    names = _pod_names(topology)
    for link in selected or topology.links:
        state = port_state.link_state(topology, link)
        parts = [link.sensor_compatible] if link.has_camera else []
        mode = modes.get(link.name)
        if mode and link.has_camera:
            try:
                parts.append(mode_label(pack.descriptor(link.sensor_compatible), mode))
            except Exception:        # noqa: BLE001 (a mode the pack no longer names)
                parts.append(str(mode))
        rate = port_state.running_rate(record, link.name)
        if rate is not None and link.has_camera:
            parts.append(f"{float(rate):.1f} fps")
        entry = entries.get(link.name) or {}
        if not link.has_camera:
            # A pod alone: the pod's name and personality, then the state.
            who = names.get(link.name) or (f"@{link.nxs_units[0].alias_addr:#04x}"
                                           if link.nxs_units else "the pod")
            held = (unit_source.cached_record(topology, link) or {}).get("personality") or {}
            personality = held.get("name") or "no personality read"
            parts = [f"pod {who} ({personality})"]
        text = f"{port}/{link.name}: {' '.join(parts)}, {state}"
        # The walk outranks the record: a pod or a head that does not
        # answer now is the line, whatever `on` recorded.
        silent = next((u for u in entry.get("units") or []
                       if u.get("state") != "present"), None)
        if silent is not None:
            alias = int(silent["addr"])
            who = names.get(link.name) or f"@{alias:#04x}"
            if silent.get("answers_at") is not None:
                # The hub lost its aliases: `switch` brings the port up and maps them.
                lines += [f"{port}/{link.name}: pod {who} does not answer at its alias "
                          f"{alias:#04x}; a pod answers at {int(silent['answers_at']):#04x}",
                          "  - nxs switch"]
            else:
                sought = [alias] + ([int(silent["strap"])] if "strap" in silent else [])
                where = " or at ".join(f"{a:#04x}" for a in sought)
                lines.append(f"{port}/{link.name}: no pod answers at {where} ({who})")
            ok = False
            continue
        if entry.get("sen") == "no ACK" and link.has_camera:
            addr = entry.get("sen_addr")
            where = f" at {int(addr):#04x}" if addr is not None else ""
            lines.append(f"{port}/{link.name}: no head answers{where}")
            ok = False
            continue
        if link.nxs_units and link.has_camera:
            who = names.get(link.name) or f"@{link.nxs_units[0].alias_addr:#04x}"
            held = (unit_source.cached_record(topology, link) or {}).get("personality") or {}
            personality = held.get("name") or "no personality read"
            head_ok = "head ok" if entry.get("sen") == "present" else "head silent"
            text += f", pod {who} ({personality}, {head_ok})"
        lines.append(text)
        ok = ok and state == port_state.STATE_UP
    return lines, ok


