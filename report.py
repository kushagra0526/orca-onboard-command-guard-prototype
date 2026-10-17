"""out/report.html: battery state of charge over one held-out day under each guard + the held-out table (python report.py)."""
import json
from pathlib import Path

from eval.heldout import HELDOUT
from orcaguard.guard import NoGuard, Predictive, Static, run_day
from orcaguard.scenario import make

OUT = Path(__file__).parent / "out"
SEED = 70020
W, H, PAD = 1100, 300, 40


def svg(g, runs):
    n = g.n
    x = lambda k: PAD + (W - 2 * PAD) * k / n
    y = lambda v: PAD + (H - 2 * PAD) * (1 - v)
    parts = [f'<svg width="{W}" height="{H}" xmlns="http://www.w3.org/2000/svg" font-family="Segoe UI" font-size="12">']
    k = 0
    while k < n:                                   # eclipse shading
        if g.eclipse[k]:
            j = k
            while j < n and g.eclipse[j]:
                j += 1
            parts.append(f'<rect x="{x(k):.1f}" y="{PAD}" width="{x(j) - x(k):.1f}" height="{H - 2 * PAD}" fill="#e9ecf2"/>')
            k = j
        else:
            k += 1
    for v, lab in ((0.2, "20 % (low battery)"), (0.12, "12 % safe mode")):
        parts.append(f'<line x1="{PAD}" x2="{W - PAD}" y1="{y(v):.1f}" y2="{y(v):.1f}" stroke="#c0392b" stroke-dasharray="4 3"/>'
                     f'<text x="{PAD + 4}" y="{y(v) - 4 if v > 0.15 else y(v) + 13:.1f}" fill="#c0392b">{lab}</text>')
    col = {"none": "#999", "static": "#c98a12", "predictive": "#1d6b3c"}
    for name, r in runs.items():
        pts = " ".join(f"{x(i):.1f},{y(v):.1f}" for i, (v, _) in enumerate(r["trace"]) if i % 2 == 0)
        parts.append(f'<polyline points="{pts}" fill="none" stroke="{col[name]}" stroke-width="{2.4 if name == "predictive" else 1.6}"/>')
    for h in range(0, 25, 3):
        parts.append(f'<text x="{x(h * 120) - 8:.1f}" y="{H - 14}" fill="#666">{h:02d}h</text>')
    for v in (0, 0.5, 1):
        parts.append(f'<text x="4" y="{y(v) + 4:.1f}" fill="#666">{int(v * 100)}%</text>')
    parts.append("</svg>")
    return "".join(parts)


def main():
    sc = make(SEED, HELDOUT)
    runs = {P.name: run_day(sc["geom"], sc["goals"], sc["conjs"], sc["truth"], sc["nominal"], P) for P in (NoGuard, Static, Predictive)}
    F = json.loads((OUT / "heldout_first_run.json").read_text())["summary"]
    A = json.loads((OUT / "heldout.json").read_text())["summary"]
    keys = ["LOW_BATTERY", "SAFE_MODE", "SUN_IN_CAMERA", "DATA_LOST", "NO_LINK", "CONJUNCTION"]
    names = {"none": "No guard (execute every AI goal)", "static": "Per-command limit checks on current state", "predictive": "Predictive guard (after burn-reflex fix)",
             "idle_ref": "Reference: deny everything, keep burn reflex"}
    rows = "".join(f"<tr{' style=\"font-weight:700;color:#1d6b3c\"' if p == 'predictive' else ''}><td>{names[p]}</td>" + "".join(f"<td>{A[p].get(k, 0):,}</td>" for k in keys)
                   + f"<td><b>{A[p]['violations']:,}</b></td><td>{A[p]['captured']:,}</td><td>{A[p]['delivered_value']:,}</td></tr>" for p in ("none", "static", "predictive", "idle_ref"))
    rows += (f"<tr style='color:#666'><td>Predictive guard, first run</td>" + "".join(f"<td>{F['predictive'].get(k, 0):,}</td>" for k in keys)
             + f"<td>{F['predictive']['violations']:,}</td><td>{F['predictive']['captured']:,}</td><td>{F['predictive']['delivered_value']:,}</td></tr>")
    legend = " · ".join(f"<span style='color:{c}'>■</span> {n}: {sum(runs[n]['violations'].values())} violations, delivered value {runs[n]['delivered_value']}"
                        for n, c in (("none", "#999"), ("static", "#c98a12"), ("predictive", "#1d6b3c")))
    page = f"""<!doctype html><meta charset="utf-8"><title>Onboard command guard</title>
<style>body{{font-family:Segoe UI,sans-serif;margin:24px;color:#1b1f24;width:1180px}}td,th{{padding:6px 9px;border-bottom:1px solid #ddd;text-align:right;font-size:13px}}
td:first-child,th:first-child{{text-align:left}}.m{{color:#555;font-size:13px}}</style>
<h2>AI goal → deterministic planner → guard → vehicle: one simulated held-out day (grey = eclipse)</h2>
<p class="m">{legend}</p>{svg(sc['geom'], runs)}
<h3>Held-out: 200 simulated days, 13,587 AI goals, 311 conjunctions (scenarios frozen in git before the first run)</h3>
<table><tr><th></th><th>low battery</th><th>safe mode</th><th>sun in keep-out</th><th>data lost</th><th>no link</th><th>conjunction unmitigated</th><th>total</th><th>images captured</th><th>delivered value</th></tr>{rows}</table>"""
    (OUT / "report.html").write_text(page, encoding="utf-8")
    print("wrote out/report.html")


if __name__ == "__main__":
    main()
