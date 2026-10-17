"""Circular-orbit geometry for one day: eclipse, ground-station passes, imaging opportunities.

ECI frame, Earth rotating at its sidereal rate, sun vector fixed for the day and set by the beta angle
(angle between the sun and the orbit plane). Two-body circular orbit, no J2: enough to place eclipses, passes
and target windows realistically for a guard test, not for navigation.
"""
import math
from dataclasses import dataclass, field

MU = 398600.4418         # km^3/s^2
RE = 6378.137            # km
WE = 7.2921159e-5        # rad/s


def norm(v):
    return math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])


def unit(v):
    n = norm(v)
    return (v[0] / n, v[1] / n, v[2] / n)


def dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def scale(a, k):
    return (a[0] * k, a[1] * k, a[2] * k)


def angle(a, b):
    return math.degrees(math.acos(max(-1.0, min(1.0, dot(unit(a), unit(b))))))


@dataclass
class Geometry:
    dt: int
    n: int
    period: float
    sun: tuple
    r: list                      # satellite position per step
    eclipse: list                # bool per step
    stations: dict               # name -> (lat, lon)
    visible: dict                # station -> [bool per step]
    passes: list                 # [(station, start_step, end_step)]
    opportunities: list          # [{"target", "step", "off_nadir", "sun_angle", "priority"}]
    targets: dict = field(default_factory=dict)

    def ground_point(self, lat, lon, k):
        th = math.radians(lon) + WE * k * self.dt
        la = math.radians(lat)
        return (RE * math.cos(la) * math.cos(th), RE * math.cos(la) * math.sin(th), RE * math.sin(la))


def build(alt_km, inc_deg, raan_deg, beta_deg, stations, targets, hours=24, dt=30, u0_deg=0.0):
    a = RE + alt_km
    nmo = math.sqrt(MU / a ** 3)
    i, O = math.radians(inc_deg), math.radians(raan_deg)
    h = (math.sin(O) * math.sin(i), -math.cos(O) * math.sin(i), math.cos(i))
    e1 = (math.cos(O), math.sin(O), 0.0)
    b = math.radians(beta_deg)
    sun = unit((math.cos(b) * e1[0] + math.sin(b) * h[0], math.cos(b) * e1[1] + math.sin(b) * h[1], math.cos(b) * e1[2] + math.sin(b) * h[2]))
    n = int(hours * 3600 / dt)
    g = Geometry(dt, n, 2 * math.pi / nmo, sun, [], [], dict(stations), {s: [] for s in stations}, [], [], dict(targets))
    for k in range(n):
        u = math.radians(u0_deg) + nmo * k * dt
        r = (a * (math.cos(O) * math.cos(u) - math.sin(O) * math.sin(u) * math.cos(i)),
             a * (math.sin(O) * math.cos(u) + math.cos(O) * math.sin(u) * math.cos(i)),
             a * math.sin(u) * math.sin(i))
        g.r.append(r)
        rs = dot(r, sun)
        g.eclipse.append(rs < 0 and norm(sub(r, scale(sun, rs))) < RE)
        for s, (lat, lon) in stations.items():
            p = g.ground_point(lat, lon, k)
            rho = sub(r, p)
            g.visible[s].append(dot(rho, unit(p)) / norm(rho) > math.sin(math.radians(10)))
    for s, vis in g.visible.items():
        k = 0
        while k < n:
            if vis[k]:
                j = k
                while j < n and vis[j]:
                    j += 1
                if j - k >= 4:                         # at least 2 minutes above 10 deg
                    g.passes.append((s, k, j))
                k = j
            else:
                k += 1
    g.passes.sort(key=lambda p: p[1])
    for t, (lat, lon, pri) in targets.items():
        best = None
        for k in range(n):
            p = g.ground_point(lat, lon, k)
            if dot(p, sun) < RE * math.sin(math.radians(10)):     # target in darkness
                if best:
                    g.opportunities.append(best)
                    best = None
                continue
            look = sub(p, g.r[k])
            off = angle(look, scale(g.r[k], -1))
            if off < 30 and not g.eclipse[k]:
                cand = {"target": t, "step": k, "off_nadir": round(off, 1), "sun_angle": round(angle(look, sun), 1), "priority": pri}
                if best is None or cand["off_nadir"] < best["off_nadir"]:
                    best = cand
            elif best:
                g.opportunities.append(best)
                best = None
        if best:
            g.opportunities.append(best)
    g.opportunities.sort(key=lambda o: o["step"])
    return g
