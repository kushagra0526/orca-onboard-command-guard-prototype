"""Spacecraft resource model and plan execution.

Goals (from the AI layer) are expanded by the deterministic planner into commands with a mode and a time window:
  CAPTURE(target)  point the camera at a target at its opportunity step (slew first), capture 1 image
  DOWNLINK(station) point at a station for a window, send data while it is above 10 deg
  BURN(conjunction) collision-avoidance burn, 2 min, attitude to the burn direction
  SUN               sun-point (default when idle; panels face the sun)

The same model runs as the guard's *nominal* model and, with perturbed parameters, as the *truth* simulator.
"""
import math
from dataclasses import dataclass, field, replace

from .orbit import angle, scale, sub, unit


@dataclass(frozen=True)
class Params:
    battery_wh: float = 45.0
    solar_w: float = 34.0
    bus_w: float = 12.0
    camera_w: float = 28.0         # payload on through the 2-min settle + capture
    radio_w: float = 24.0
    heater_w: float = 12.0         # in eclipse
    slew_w: float = 5.0
    burn_w: float = 30.0
    slew_deg_s: float = 1.0
    storage_gb: float = 4.0
    image_gb: float = 0.45
    downlink_gb_s: float = 0.004
    sun_keepout_deg: float = 50.0  # stray-light / sensor keep-out around the camera boresight
    low_soc: float = 0.20          # below this: LOW_BATTERY violation
    safe_soc: float = 0.12         # below this the vehicle drops to safe mode until 50%
    burn_min_soc: float = 0.40


def perturbed(p, rng, k=0.10):
    f = lambda: 1 + rng.uniform(-k, k)
    return replace(p, solar_w=p.solar_w * f(), bus_w=p.bus_w * f(), camera_w=p.camera_w * f(), radio_w=p.radio_w * f(),
                   heater_w=p.heater_w * f(), battery_wh=p.battery_wh * (1 - rng.uniform(0, k)))


@dataclass
class State:
    k: int = 0
    soc_wh: float = 0.0
    stored_gb: float = 0.0
    queue: list = field(default_factory=list)     # [(gb_left, priority)] FIFO of images on board
    pointing: tuple = None
    safe_mode: bool = False

    def copy(self):
        return State(self.k, self.soc_wh, self.stored_gb, [list(x) for x in self.queue], self.pointing, self.safe_mode)


def boresight(g, cmd, k):
    r = g.r[k]
    if cmd is None or cmd["mode"] == "SUN":
        return scale(g.sun, -1)                     # panels (back face) to the sun, camera away from it
    if cmd["mode"] == "CAPTURE":
        lat, lon, _ = g.targets[cmd["target"]]
        return unit(sub(g.ground_point(lat, lon, k), r))
    if cmd["mode"] == "DOWNLINK":
        lat, lon = g.stations[cmd["station"]]
        return unit(sub(g.ground_point(lat, lon, k), r))
    if cmd["mode"] == "BURN":
        return unit(scale(r, -1)) if cmd.get("radial_in") else unit(sub((0, 0, 0), g.sun))
    return scale(g.sun, -1)


def step(g, p, s, cmd, out, recording=True):
    """Advance one step under command `cmd` (or idle). Mutates s; appends events to out."""
    k = s.k
    b = boresight(g, cmd, k)
    slew_deg = angle(b, s.pointing) if s.pointing else 0.0
    slewing = slew_deg > p.slew_deg_s * g.dt
    s.pointing = b
    lit = not g.eclipse[k]
    panel_cos = max(0.0, math.cos(math.radians(angle(scale(b, -1), g.sun))))
    gen = p.solar_w * panel_cos if lit else 0.0
    load = p.bus_w + (p.heater_w if not lit else 0.0) + (p.slew_w if slew_deg > 1 else 0.0)
    mode = cmd["mode"] if cmd else "SUN"
    if mode == "CAPTURE":
        load += p.camera_w
    if mode == "DOWNLINK":
        load += p.radio_w
    if mode == "BURN":
        load += p.burn_w
    s.soc_wh = max(0.0, min(p.battery_wh, s.soc_wh + (gen - load) * g.dt / 3600.0))
    # pointing constraint: camera boresight inside the sun keep-out while sunlit
    if mode in ("CAPTURE", "DOWNLINK") and lit and angle(b, g.sun) < p.sun_keepout_deg:
        out.append(("SUN_IN_CAMERA", k, cmd.get("id")))
    if mode == "CAPTURE" and k == cmd["at"]:
        if slewing:
            out.append(("CAPTURE_MISSED", k, cmd.get("id")))
        elif s.stored_gb + p.image_gb > p.storage_gb + 1e-9:
            out.append(("DATA_LOST", k, cmd.get("id")))
        else:
            s.stored_gb += p.image_gb
            s.queue.append([p.image_gb, cmd["priority"]])
            out.append(("CAPTURED", k, cmd.get("id")))
    if mode == "DOWNLINK" and g.visible[cmd["station"]][k]:
        send = p.downlink_gb_s * g.dt
        out.append(("LINK", k, cmd.get("id")))
        while send > 1e-12 and s.queue:
            q = s.queue[0]
            d = min(send, q[0])
            q[0] -= d
            send -= d
            s.stored_gb -= d
            if q[0] <= 1e-9:
                out.append(("DELIVERED", k, q[1]))
                s.queue.pop(0)
    if mode == "BURN" and k == cmd["at"]:
        out.append(("BURN_DONE" if s.soc_wh >= p.burn_min_soc * p.battery_wh else "BURN_FAILED", k, cmd.get("conj")))
    if s.soc_wh < p.low_soc * p.battery_wh:
        out.append(("LOW_BATTERY", k, None))
    s.k += 1


def command_at(plan, k):
    for c in plan:
        if c["start"] <= k < c["end"]:
            return c
    return None


def expand(goal, g, p):
    """Deterministic planner: goal -> command with a window (slew lead time before the action)."""
    lead = 4                                              # 2 min to slew and settle
    if goal["kind"] == "capture":
        return {"id": goal["id"], "mode": "CAPTURE", "target": goal["target"], "priority": goal["priority"],
                "at": goal["step"], "start": max(0, goal["step"] - lead), "end": goal["step"] + 1}
    if goal["kind"] == "downlink":
        return {"id": goal["id"], "mode": "DOWNLINK", "station": goal["station"], "start": max(0, goal["start"] - lead), "end": goal["end"]}
    if goal["kind"] == "burn":
        return {"id": goal["id"], "mode": "BURN", "conj": goal["conj"], "at": goal["step"], "start": max(0, goal["step"] - lead), "end": goal["step"] + 4}
    if goal["kind"] == "sun":
        return {"id": goal["id"], "mode": "SUN", "start": goal["start"], "end": goal["end"]}
    raise ValueError(goal)


def violations(events, conjunctions, g, p_dt=30):
    """Count unsafe outcomes from an event list."""
    V = {"LOW_BATTERY": 0, "SAFE_MODE": 0, "SUN_IN_CAMERA": 0, "DATA_LOST": 0, "NO_LINK": 0, "CONJUNCTION": 0}
    last_low = -10
    seen_sun = set()
    for e, k, ref in events:
        if e == "LOW_BATTERY":
            if k - last_low > 1:
                V["LOW_BATTERY"] += 1
            last_low = k
        elif e == "SUN_IN_CAMERA" and ref not in seen_sun:
            seen_sun.add(ref)
            V["SUN_IN_CAMERA"] += 1
        elif e in ("DATA_LOST", "SAFE_MODE", "NO_LINK"):
            V[e] += 1
    for c in conjunctions:
        ok = any(e == "BURN_DONE" and ref == c["id"] and c["tca"] - 360 <= k <= c["tca"] - 60 for e, k, ref in events)
        if not ok:
            V["CONJUNCTION"] += 1
    return V
