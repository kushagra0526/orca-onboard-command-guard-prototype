"""Command guards between the AI layer and the vehicle, and the day runner.

Each goal reaches the guard when its command would start. The guard sees the vehicle's current state (telemetry),
the geometry (ephemeris) and the commands it has already approved. Policies:

  none       approve everything
  static     per-command limit checks on the current state: SoC >= 30 %, room for one image, station in view at
             start, commanded pointing vs the sun at the capture moment, SoC >= 40 % for a burn
  predictive roll the vehicle forward with the guard's *nominal* model through the command and one more orbit;
             deny if the command creates a violation (low battery in the next eclipse, sun in the keep-out, data
             lost, no link, a conjunction burn that can no longer be powered); re-time a downlink to the next real
             pass; reflex: schedule the collision-avoidance burn itself if none is approved in the window
"""
from .sim import Params, State, command_at, expand, step, violations

LOOKAHEAD = 190                 # ~1 orbit of 30 s steps
MARGIN_SOC = 0.27               # guard's own floor (truth violation is 20 %): covers +-10 % model error


def rollout(g, p, s, plan, k0, k1, conjs):
    s = s.copy()
    ev = []
    for k in range(k0, min(k1, g.n)):
        s.k = k
        c = command_at(plan, k)
        step(g, p, s, c, ev)
    return s, ev


def _soc_floor(g, p, s, plan, k0, k1):
    s = s.copy()
    lo = s.soc_wh
    for k in range(k0, min(k1, g.n)):
        s.k = k
        step(g, p, s, command_at(plan, k), [])
        lo = min(lo, s.soc_wh)
    return lo / p.battery_wh


class Predictive:
    name = "predictive"

    def __init__(self, g, nominal: Params, conjs):
        self.g, self.p, self.conjs = g, nominal, conjs
        self.log = []

    def decide(self, s, k, cmd, plan, pending=()):
        verdict, out, why = self._check(s, k, cmd, plan, pending)
        if verdict == "DENY" and cmd["mode"] == "DOWNLINK":
            # keep the part of the pass that is safe: in view, sun outside the keep-out; then try shorter
            for v in self._trims(cmd, k):
                if self._check(s, k, v, plan, pending, quiet=True)[0] == "APPROVE":
                    self.log.append((k, cmd["id"], "MODIFY", f"{why}; trimmed to steps {v['start']}-{v['end']}"))
                    return "MODIFY", v
        if verdict == "DENY":
            self.log.append((k, cmd["id"], "DENY", why))
        elif verdict == "APPROVE":
            self.log.append((k, cmd["id"], "APPROVE", ""))
        return verdict, out

    def _trims(self, cmd, k):
        from .orbit import angle
        from .sim import boresight
        g, p = self.g, self.p
        ok = [kk for kk in range(max(k, cmd["start"]), min(cmd["end"], g.n)) if g.visible[cmd["station"]][kk]
              and (g.eclipse[kk] or angle(boresight(g, cmd, kk), g.sun) >= p.sun_keepout_deg + 2)]
        segs, cur = [], []
        for kk in ok:
            if cur and kk != cur[-1] + 1:
                segs.append(cur)
                cur = []
            cur.append(kk)
        if cur:
            segs.append(cur)
        out = []
        for seg in sorted(segs, key=len, reverse=True)[:2]:
            for frac in (1.0, 0.5):
                n = max(4, int(len(seg) * frac))
                if len(seg) >= 4:
                    out.append(dict(cmd, start=max(k, seg[0] - 2), end=seg[0] + n, trimmed=True))
        return out

    def _check(self, s, k, cmd, plan, pending=(), quiet=False):
        g, p = self.g, self.p
        horizon = cmd["end"] + LOOKAHEAD
        base_floor = _soc_floor(g, p, s, plan, k, horizon)
        with_cmd = plan + [cmd]
        floor = _soc_floor(g, p, s, with_cmd, k, horizon)
        why = []
        if cmd["mode"] in ("CAPTURE", "DOWNLINK"):
            _, ev = rollout(g, p, s, with_cmd, k, cmd["end"], self.conjs)
            if any(e == "SUN_IN_CAMERA" and ref == cmd["id"] for e, _, ref in ev):
                why.append(f"sun inside the {p.sun_keepout_deg:g} deg keep-out while pointing")
            if any(e == "DATA_LOST" for e, _, _ in ev):
                why.append("storage full at capture (downlink first)")
        if cmd["mode"] == "DOWNLINK" and not any(g.visible[cmd["station"]][kk] for kk in range(cmd["start"], cmd["end"])):
            nxt = next(((st, a, b) for st, a, b in g.passes if a >= k and not command_at(plan, a)), None)
            if nxt and command_at(list(pending), nxt[1]):
                why.append(f"{cmd['station']} not in view during the window; the next pass is already planned")
                nxt = None
            elif nxt:
                st, a, b = nxt
                new = dict(cmd, station=st, start=max(k, a - 4), end=b, retimed=True)
                if not quiet:
                    self.log.append((k, cmd["id"], "MODIFY", f"{cmd['station']} not in view {cmd['start']}-{cmd['end']}; re-timed to {st} pass at step {a}"))
                return "MODIFY", new, ""
            if not why:
                why.append(f"{cmd['station']} not in view during the window")
        if floor < MARGIN_SOC and floor < base_floor - 1e-9:
            why.append(f"SoC would fall to {floor:.0%} within the next orbit (floor {MARGIN_SOC:.0%})")
        # keep power for an open conjunction window with no burn done yet
        for c in self.conjs:
            if c["tca"] - 360 <= horizon and k <= c["tca"] - 60 and not c.get("done") and cmd["mode"] != "BURN":
                f2 = _soc_floor(g, p, s, with_cmd, k, min(horizon, c["tca"] - 60))
                if f2 < p.burn_min_soc + 0.05 and f2 < _soc_floor(g, p, s, plan, k, min(horizon, c["tca"] - 60)) - 1e-9:
                    why.append(f"keeps less than {p.burn_min_soc + 0.05:.0%} SoC for the collision-avoidance burn before TCA (step {c['tca']})")
        if cmd["mode"] == "BURN":
            c = next(c for c in self.conjs if c["id"] == cmd["conj"])
            if not (c["tca"] - 360 <= cmd["at"] <= c["tca"] - 60):
                why.append(f"burn at step {cmd['at']} is outside the window [{c['tca'] - 360}, {c['tca'] - 60}] for TCA {c['tca']}")
        if why:
            return "DENY", None, "; ".join(why)
        return "APPROVE", cmd, ""

    def reflex(self, s, k, plan):
        """Schedule the conjunction burn if the window is open and no valid burn is approved."""
        out = []
        for c in self.conjs:
            lo, hi = c["tca"] - 360, c["tca"] - 66
            if c.get("done") or not (lo <= k <= hi):
                continue
            if any(x["mode"] == "BURN" and x.get("conj") == c["id"] and lo <= x["at"] <= c["tca"] - 60 and x["at"] >= k for x in plan):
                continue
            if s.soc_wh >= (self.p.burn_min_soc + 0.08) * self.p.battery_wh or k == hi:
                cmd = {"id": f"reflex-burn-{c['id']}", "mode": "BURN", "conj": c["id"], "at": k + 4, "start": k, "end": k + 8}
                plan[:] = [x for x in plan if x["end"] <= k or x["start"] >= k + 8]
                out.append(cmd)
                self.log.append((k, cmd["id"], "REFLEX", f"no valid burn approved for conjunction {c['id']} (TCA step {c['tca']}); burning now"))
            elif k == lo:
                self.log.append((k, f"hold-{c['id']}", "REFLEX", "conjunction window open; holding SoC for the burn"))
        return out


class Static:
    name = "static"

    def __init__(self, g, nominal: Params, conjs):
        self.g, self.p, self.conjs, self.log = g, nominal, conjs, []

    def decide(self, s, k, cmd, plan, pending=()):
        g, p = self.g, self.p
        soc = s.soc_wh / p.battery_wh
        ok = True
        if cmd["mode"] == "CAPTURE":
            from .sim import boresight
            from .orbit import angle
            ok = soc >= 0.30 and s.stored_gb + p.image_gb <= p.storage_gb and angle(boresight(g, cmd, cmd["at"]), g.sun) >= p.sun_keepout_deg
        elif cmd["mode"] == "DOWNLINK":
            ok = soc >= 0.30 and g.visible[cmd["station"]][min(g.n - 1, cmd["start"] + 4)]
        elif cmd["mode"] == "BURN":
            ok = soc >= p.burn_min_soc
        self.log.append((k, cmd["id"], "APPROVE" if ok else "DENY", ""))
        return ("APPROVE", cmd) if ok else ("DENY", None)

    def reflex(self, s, k, plan):
        return []


class NoGuard(Static):
    name = "none"

    def decide(self, s, k, cmd, plan, pending=()):
        return "APPROVE", cmd


def run_day(g, goals, conjs, truth: Params, nominal: Params, policy_cls, soc0=0.75):
    conjs = [dict(c) for c in conjs]
    pol = policy_cls(g, nominal, conjs)
    s = State(0, soc0 * truth.battery_wh, 0.9, [[0.45, 1], [0.45, 1]], None, False)
    cmds = sorted((expand(x, g, nominal) for x in goals), key=lambda c: c["start"])
    due = {}
    for c in cmds:
        due.setdefault(c["start"], []).append(c)
    plan, events, denied, approved, trace = [], [], 0, 0, []
    for k in range(g.n):
        s.k = k
        for c in due.pop(k, []):
            verdict, c2 = pol.decide(s, k, c, plan, [x for v in due.values() for x in v])
            if verdict == "DENY":
                denied += 1
                continue
            if verdict == "MODIFY" and c2["start"] > k:
                due.setdefault(c2["start"], []).append(dict(c2, retimed=False))
                continue
            approved += 1
            plan.append(c2)
        plan += pol.reflex(s, k, plan)
        if s.safe_mode and s.soc_wh >= 0.5 * truth.battery_wh:
            s.safe_mode = False
        if not s.safe_mode and s.soc_wh < truth.safe_soc * truth.battery_wh:
            s.safe_mode = True
            events.append(("SAFE_MODE", k, None))
        cmd = None if s.safe_mode else command_at(plan, k)
        n0 = len(events)
        step(g, truth, s, cmd, events)
        for e, _, ref in events[n0:]:
            if e == "BURN_DONE":
                for c in conjs:
                    if c["id"] == ref:
                        c["done"] = True
        trace.append((round(s.soc_wh / truth.battery_wh, 3), cmd["mode"] if cmd else ("SAFE" if s.safe_mode else "SUN")))
    for c in plan:     # a downlink commanded while its station is never in view
        if c["mode"] == "DOWNLINK" and not any(g.visible[c["station"]][kk] for kk in range(c["start"], min(c["end"], g.n))):
            events.append(("NO_LINK", c["start"], c["id"]))
    V = violations(events, conjs, g)
    delivered = sum(ref for e, _, ref in events if e == "DELIVERED")
    captured = sum(1 for e, _, _ in events if e == "CAPTURED")
    return {"violations": V, "delivered_value": delivered, "captured": captured, "denied": denied, "approved": approved,
            "log": pol.log, "events": events, "end_soc": s.soc_wh / truth.battery_wh, "trace": trace}


class IdleReference(Predictive):
    """Reference floor, not a policy: deny every AI goal, keep only the conjunction reflex.
    Whatever still goes wrong here would go wrong with any guard (e.g. a vehicle that is power-negative even idle)."""
    name = "idle_ref"

    def decide(self, s, k, cmd, plan, pending=()):
        return "DENY", None
