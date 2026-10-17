"""One held-out day: the AI layer's goals through three guards; the predictive guard's decision log.

python demo.py            # held-out day 70020
python demo.py 70003
"""
import sys

from eval.heldout import HELDOUT
from orcaguard.guard import NoGuard, Predictive, Static, run_day
from orcaguard.scenario import make


def hhmm(k, dt=30):
    m = k * dt // 60
    return f"{m // 60:02d}:{m % 60:02d}"


def main():
    seed = int(sys.argv[1]) if len(sys.argv) > 1 else 70020
    sc = make(seed, HELDOUT)
    g = sc["geom"]
    kinds = {}
    for x in sc["goals"]:
        kinds[x["kind"]] = kinds.get(x["kind"], 0) + 1
    print(f"day {seed}: period {g.period / 60:.1f} min, eclipse {sum(g.eclipse) / g.n:.0%} of the day, {len(g.passes)} passes, "
          f"{len(sc['conjs'])} conjunction(s) at {', '.join(hhmm(c['tca']) for c in sc['conjs'])}")
    print(f"AI goals: {kinds}; truth vs guard model: solar x{sc['truth'].solar_w / sc['nominal'].solar_w:.2f}, battery x{sc['truth'].battery_wh / sc['nominal'].battery_wh:.2f}\n")
    for P in (NoGuard, Static, Predictive):
        r = run_day(g, sc["goals"], sc["conjs"], sc["truth"], sc["nominal"], P)
        v = {k: n for k, n in r["violations"].items() if n}
        print(f"{P.name:11s} violations {sum(v.values()):3d} {v}  captured {r['captured']}  delivered value {r['delivered_value']}  "
              f"denied {r['denied']}  lowest SoC {min(t[0] for t in r['trace']):.0%}")
        if P is Predictive:
            log = r["log"]
    print("\npredictive guard log (first denials, every modify/reflex):")
    shown = 0
    for k, cid, verdict, why in log:
        if verdict in ("MODIFY", "REFLEX") or (verdict == "DENY" and shown < 8):
            print(f"  {hhmm(k)}  {verdict:7s} {cid:22s} {why}")
            shown += verdict == "DENY"


if __name__ == "__main__":
    main()
