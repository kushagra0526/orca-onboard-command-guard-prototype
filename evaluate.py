"""Run each policy over a bank of simulated days and total unsafe outcomes vs mission value.

python evaluate.py --dev     # dev scenarios (orcaguard.scenario.DEV), 60 days
python evaluate.py           # held-out scenarios (eval/heldout.py, frozen in git first), 200 days
"""
import collections
import json
import sys
from pathlib import Path

from orcaguard.guard import IdleReference, NoGuard, Predictive, Static, run_day
from orcaguard.scenario import DEV, make

POLICIES = [NoGuard, Static, Predictive, IdleReference]
VKEYS = ["LOW_BATTERY", "SAFE_MODE", "SUN_IN_CAMERA", "DATA_LOST", "NO_LINK", "CONJUNCTION"]


def run(cfg, seeds):
    T = {P.name: collections.Counter() for P in POLICIES}
    worst = []
    for seed in seeds:
        sc = make(seed, cfg)
        T["_"] = T.get("_", collections.Counter())
        T["_"]["days"] += 1
        T["_"]["conjunctions"] += len(sc["conjs"])
        T["_"]["goals"] += len(sc["goals"])
        for P in POLICIES:
            r = run_day(sc["geom"], sc["goals"], sc["conjs"], sc["truth"], sc["nominal"], P)
            c = T[P.name]
            c.update(r["violations"])
            c["violations"] += sum(r["violations"].values())
            c["delivered_value"] += r["delivered_value"]
            c["captured"] += r["captured"]
            c["denied"] += r["denied"]
            c["days_with_violation"] += 1 if sum(r["violations"].values()) else 0
            if P is Predictive and sum(r["violations"].values()):
                worst.append((seed, r["violations"], round(sc["truth"].solar_w / sc["nominal"].solar_w, 3),
                              round(sc["truth"].battery_wh / sc["nominal"].battery_wh, 3)))
    return {k: dict(v) for k, v in T.items()}, worst


def main():
    dev = "--dev" in sys.argv
    if dev:
        cfg, seeds = DEV, range(0, 60)
    else:
        from eval.heldout import HELDOUT, SEEDS
        cfg, seeds = HELDOUT, SEEDS
    T, worst = run(cfg, seeds)
    meta = T.pop("_")
    print(f"{'dev' if dev else 'held-out'}: {meta['days']} days, {meta['goals']} AI goals, {meta['conjunctions']} conjunctions")
    print(f"  {'policy':11s} " + " ".join(f"{k[:11]:>11s}" for k in VKEYS) + f" {'total':>6s} {'bad days':>8s} {'captured':>8s} {'delivered':>9s} {'denied':>6s}")
    for p, c in T.items():
        print(f"  {p:11s} " + " ".join(f"{c.get(k, 0):>11d}" for k in VKEYS) +
              f" {c.get('violations', 0):>6d} {c.get('days_with_violation', 0):>8d} {c.get('captured', 0):>8d} {c.get('delivered_value', 0):>9d} {c.get('denied', 0):>6d}")
    print("\npredictive: days that still had a violation (seed, counts, truth/nominal solar, truth/nominal battery):")
    for w in worst[:20]:
        print("  ", w)
    Path("out").mkdir(exist_ok=True)
    Path(f"out/{'dev' if dev else 'heldout'}.json").write_text(json.dumps({"meta": meta, "summary": T, "predictive_bad_days": worst}, indent=1), encoding="utf-8")


if __name__ == "__main__":
    main()
