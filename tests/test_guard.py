import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from orcaguard.guard import NoGuard, Predictive, Static, run_day
from orcaguard.orbit import build
from orcaguard.scenario import DEV, make
from orcaguard.sim import Params, State, expand, step

STATIONS = {"Svalbard": (78.2, 15.4), "Bangalore": (13.0, 77.6)}
G = build(520, 97.5, 30, 10, STATIONS, {"A": (40.0, 10.0, 3), "B": (-20.0, 140.0, 2)})


def test_geometry_is_plausible():
    assert 93 < G.period / 60 < 96                       # 520 km LEO
    assert 0.30 < sum(G.eclipse) / G.n < 0.40            # low beta angle: about a third in shadow
    assert len(G.passes) >= 8 and all(b - a >= 4 for _, a, b in G.passes)
    assert G.opportunities and all(o["off_nadir"] < 30 for o in G.opportunities)


def test_capture_with_sun_in_keepout_is_denied_and_static_also_sees_it():
    p = Params()
    o = min(G.opportunities, key=lambda o: o["sun_angle"])
    p2 = Params(sun_keepout_deg=o["sun_angle"] + 5)   # make this opportunity violate the keep-out
    cmd = expand({"id": "c", "kind": "capture", "target": o["target"], "step": o["step"], "priority": 3}, G, p2)
    s = State(cmd["start"], 0.9 * p2.battery_wh, 0.0, [], None, False)
    assert Predictive(G, p2, []).decide(s, cmd["start"], cmd, [])[0] == "DENY"
    assert Static(G, p2, []).decide(s, cmd["start"], cmd, [])[0] == "DENY"


def test_downlink_one_orbit_off_is_retimed_to_a_real_pass():
    st, a, b = G.passes[2]
    per = round(G.period / G.dt)
    cmd = expand({"id": "d", "kind": "downlink", "station": st, "start": a + per // 2, "end": b + per // 2}, G, Params())
    if any(G.visible[st][k] for k in range(cmd["start"], cmd["end"])):
        return
    s = State(cmd["start"], 0.9 * Params().battery_wh, 1.0, [[0.45, 1]], None, False)
    verdict, new = Predictive(G, Params(), []).decide(s, cmd["start"], cmd, [])
    assert verdict == "MODIFY" and any(G.visible[new["station"]][k] for k in range(new["start"], new["end"]))


def test_capture_that_drains_the_next_eclipse_is_denied_only_by_lookahead():
    p = Params(battery_wh=20.0)
    o = next(o for o in G.opportunities if o["sun_angle"] > 60)      # the next eclipse is always within one orbit
    cmd = expand({"id": "c", "kind": "capture", "target": o["target"], "step": o["step"], "priority": 3}, G, p)
    s = State(cmd["start"], 0.36 * p.battery_wh, 0.0, [], None, False)
    assert Static(G, p, []).decide(s, cmd["start"], cmd, [])[0] == "APPROVE"            # 36 % now: limit check passes
    verdict, _ = Predictive(G, p, []).decide(s, cmd["start"], cmd, [])
    assert verdict == "DENY"


def test_late_burn_denied_and_reflex_burns_in_window():
    sc = make(3, DEV)
    conj = [{"id": "C9", "tca": 1500, "miss_m": 150}]
    goals = [{"id": "burn-late", "kind": "burn", "conj": "C9", "step": 1520}]
    r = run_day(sc["geom"], goals, conj, sc["nominal"], sc["nominal"], Predictive)
    assert r["violations"]["CONJUNCTION"] == 0
    assert any(v == "DENY" and cid == "burn-late" for _, cid, v, _ in r["log"])
    assert sum(1 for _, cid, v, _ in r["log"] if v == "REFLEX" and cid.startswith("reflex-burn")) == 1
    assert run_day(sc["geom"], goals, conj, sc["nominal"], sc["nominal"], NoGuard)["violations"]["CONJUNCTION"] == 1


def test_battery_never_negative_and_safe_mode_recovers():
    sc = make(1, DEV)
    r = run_day(sc["geom"], sc["goals"], sc["conjs"], sc["truth"], sc["nominal"], NoGuard)
    assert min(t[0] for t in r["trace"]) >= 0.0
    assert any(m == "SAFE" for _, m in r["trace"]) and r["trace"][-1][1] != "SAFE" or r["violations"]["SAFE_MODE"] == 0


def test_predictive_beats_both_baselines_on_dev_days():
    tot = {"none": 0, "static": 0, "predictive": 0}
    for seed in range(4):
        sc = make(seed, DEV)
        for P in (NoGuard, Static, Predictive):
            tot[P.name] += sum(run_day(sc["geom"], sc["goals"], sc["conjs"], sc["truth"], sc["nominal"], P)["violations"].values())
    assert tot["predictive"] < tot["static"] < tot["none"]
