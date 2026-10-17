"""Scenarios (orbit, stations, targets, conjunctions, truth parameters) and the simulated AI operator.

The operator stands in for the AI decision layer: it picks goals greedily by priority and makes the mistakes a goal
picker without a resource model makes. Fault rates are part of the scenario, so the bank decides how often it errs:
  p_sun_blind  ignores the camera keep-out when choosing targets
  p_shift      a downlink placed one orbit off (no station in view)
  p_forget     no burn for a conjunction;  p_late  burn placed after TCA
  greed        how many captures it stacks (captures ignore battery and storage)
"""
import math
import random

from .orbit import build
from .sim import Params, perturbed

STATIONS_DEV = {"Svalbard": (78.2, 15.4), "Punta Arenas": (-53.1, -70.9), "Bangalore": (13.0, 77.6), "Fairbanks": (64.8, -147.7),
                "Hartebeesthoek": (-25.9, 27.7)}


def make(seed, cfg):
    rng = random.Random(seed)
    stations = dict(rng.sample(sorted(cfg["stations"].items()), rng.randint(*cfg["n_stations"])))
    targets = {f"T{i:02d}": (round(rng.uniform(-60, 70), 1), round(rng.uniform(-180, 180), 1), rng.choice([1, 1, 2, 2, 3]))
               for i in range(rng.randint(*cfg["n_targets"]))}
    g = build(rng.uniform(*cfg["alt"]), rng.uniform(*cfg["inc"]), rng.uniform(0, 360), rng.uniform(*cfg["beta"]), stations, targets,
              u0_deg=rng.uniform(0, 360))
    conjs = [{"id": f"C{j}", "tca": rng.randint(500, g.n - 100), "miss_m": rng.randint(80, 600)} for j in range(rng.choice(cfg["n_conj"]))]
    nominal = Params(battery_wh=rng.choice(cfg["battery"]), solar_w=rng.choice(cfg["solar"]))
    truth = perturbed(nominal, rng, cfg["model_error"])
    goals = operator(g, conjs, nominal, rng, cfg)
    return {"seed": seed, "geom": g, "conjs": conjs, "nominal": nominal, "truth": truth, "goals": goals}


def operator(g, conjs, p, rng, cfg):
    goals, busy = [], []
    period = round(g.period / g.dt)

    def free(a, b):
        return all(b <= x or a >= y for x, y in busy)

    for st, a, b in g.passes:
        if rng.random() < 0.9:
            s, e = (a, b)
            if rng.random() < cfg["p_shift"]:
                d = period * rng.choice([-1, 1])
                s, e = a + d, b + d
            if 0 <= s and e < g.n and free(s - 4, e):
                goals.append({"id": f"dl-{st[:4]}-{a}", "kind": "downlink", "station": st, "start": s, "end": e})
                busy.append((s - 4, e))
    take = {1: cfg["greed"] * 0.35, 2: cfg["greed"] * 0.65, 3: cfg["greed"] * 0.95}
    for o in g.opportunities:
        if o["sun_angle"] < p.sun_keepout_deg and rng.random() > cfg["p_sun_blind"]:
            continue
        if rng.random() < take[o["priority"]] and free(o["step"] - 6, o["step"] + 2):
            goals.append({"id": f"cap-{o['target']}-{o['step']}", "kind": "capture", "target": o["target"], "step": o["step"], "priority": o["priority"]})
            busy.append((o["step"] - 6, o["step"] + 2))
    for c in conjs:
        r = rng.random()
        if r < cfg["p_forget"]:
            continue
        at = c["tca"] + rng.randint(5, 40) if r < cfg["p_forget"] + cfg["p_late"] else c["tca"] - rng.randint(90, 330)
        at = max(5, min(g.n - 10, at))
        goals.append({"id": f"burn-{c['id']}", "kind": "burn", "conj": c["id"], "step": at})
    return sorted(goals, key=lambda x: x.get("step", x.get("start")))


DEV = {"stations": STATIONS_DEV, "n_stations": (2, 3), "n_targets": (20, 35), "alt": (480, 560), "inc": (96, 99), "beta": (0, 45),
       "n_conj": [0, 1, 1, 2], "battery": [28.0, 34.0, 40.0], "solar": [27.0, 31.0, 35.0], "model_error": 0.10,
       "p_sun_blind": 0.6, "p_shift": 0.15, "p_forget": 0.3, "p_late": 0.3, "greed": 1.0}
