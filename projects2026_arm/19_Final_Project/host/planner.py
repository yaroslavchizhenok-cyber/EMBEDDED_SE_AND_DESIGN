#!/usr/bin/env python3
"""
planner.py - a show file in, a timed track for every drone out.

    uv run planner.py shows/parade.show            the plan, stage by stage,
                                                    and the separation check
    uv run planner.py shows/parade.show --csv plan.csv
                                                    every drone's planned
                                                    position, 10 times a second

This is the part of a drone light show that happens on a laptop the day
before.  Nothing here flies: it decides WHERE every drone must be at every
moment, and proves, before anything takes off, that no two drones are ever
planned closer than `sep` metres.

The show, stage by stage:
    pads      everyone on the ground, on their pads, motors armed
    takeoff   straight up to `takeoff` metres above the pads
    <form>    MOVE to the next formation, then HOLD it for its hold time
    ...
    home      MOVE back above the pads (any drone to any pad)
    land      the drones' own LAND mode: straight down, then disarm

A MOVE is planned in three steps:
  1. WHO GOES WHERE.  Each drone gets one slot of the next formation, chosen
     so that the SUM OF SQUARED distances flown is smallest (the Hungarian
     algorithm, below).  Squared, not plain: a known result (Turpin, Michael
     and Kumar's "CAPT", 2014) says that with this choice and straight lines
     flown in step, two paths never cross - if the formations themselves are
     spaced well enough.
  2. HOW FAST.  The longest path gets a trapezoidal speed profile - speed up
     at `amax`, cruise at `vmax`, brake at `amax` - and every other drone
     flies the SAME profile scaled to its own distance, so all start and
     arrive together.
  3. CHECK.  Every pair, every 0.05 s.  A pair planned closer than `sep` gets
     repaired: one of the two flies over (or under) the other - up by `sep`,
     across, down - and the move is timed again.  If no repair works, the
     show is refused.

Standard library only.
"""

import argparse
import itertools
import math
import sys

# --------------------------------------------------------------------------
# The show file
# --------------------------------------------------------------------------
DEFAULTS = dict(drones=5, pads=4.0, takeoff=5.0, vmax=3.0, vzmax=1.5, amax=1.0, sep=3.0, safe=1.0,
                wind=(0.0, 0.0), settle=1.0, gps="rtk")


class Formation:
    def __init__(self, shape, hold, opts, points=None, line_no=0):
        self.shape, self.hold, self.opts, self.points, self.line_no = shape, hold, opts, points, line_no

    @property
    def name(self):
        return self.opts.get("name", self.shape)


class Show:
    def __init__(self):
        self.cfg = dict(DEFAULTS)
        self.forms = []
        self.title = "show"


def _num(s, where):
    try:
        return float(s)
    except ValueError:
        raise SystemExit(f"{where}: '{s}' is not a number")


def load_show(path):
    show = Show()
    show.title = path.replace("\\", "/").split("/")[-1].rsplit(".", 1)[0]
    with open(path, encoding="utf-8") as fh:
        for no, raw in enumerate(fh, 1):
            line = raw.split("#", 1)[0].strip()
            if not line:
                continue
            where = f"{path}:{no}"
            w = line.split()
            key = w[0].lower()
            if key in ("drones",):
                show.cfg[key] = int(_num(w[1], where))
            elif key in ("pads", "takeoff", "vmax", "vzmax", "amax", "sep", "safe", "settle"):
                show.cfg[key] = _num(w[1], where)
            elif key == "gps":
                if w[1] not in ("rtk", "plain"):
                    raise SystemExit(f"{where}: gps rtk | plain")
                show.cfg["gps"] = w[1]
            elif key == "wind":
                show.cfg["wind"] = (_num(w[1], where), _num(w[2], where) if len(w) > 2 else 0.0)
            elif key == "form":
                if len(w) < 3:
                    raise SystemExit(f"{where}: form <shape> <hold s> [key=value ...]")
                opts = {}
                for kv in w[3:]:
                    if "=" not in kv:
                        raise SystemExit(f"{where}: '{kv}' should be key=value")
                    k, v = kv.split("=", 1)
                    opts[k] = v if k in ("plane", "name") else _num(v, where)
                if w[1] not in SHAPES:
                    raise SystemExit(f"{where}: unknown shape '{w[1]}' (have: {', '.join(SHAPES)})")
                show.forms.append(Formation(w[1], _num(w[2], where), opts, line_no=no))
            elif key == "points":
                pts = []
                for xyz in w[2:]:
                    c = xyz.split(",")
                    if len(c) != 3:
                        raise SystemExit(f"{where}: '{xyz}' should be x,y,z")
                    pts.append(tuple(_num(v, where) for v in c))
                show.forms.append(Formation("points", _num(w[1], where), {}, points=pts, line_no=no))
            else:
                raise SystemExit(f"{where}: unknown keyword '{w[0]}'")
    return show


# --------------------------------------------------------------------------
# Shapes: n points in a local (u, w) plane, centred on 0, about `size` across
# --------------------------------------------------------------------------
def shape_line(n, s):
    return [(-s / 2 + s * k / max(1, n - 1), 0.0) for k in range(n)]


def shape_v(n, s):
    # the apex, then pairs further out on alternate arms
    ranks = max(1, math.ceil((n - 1) / 2))
    du = (s / 2) / ranks
    pts = [(0.0, 0.0)]
    for k in range(1, n):
        r = (k + 1) // 2
        pts.append((du * r * (1 if k % 2 else -1), 1.2 * du * r))
    top = max(p[1] for p in pts)
    return [(u, w - top / 2) for u, w in pts]


def shape_circle(n, s):
    return [((s / 2) * math.cos(math.pi / 2 + 2 * math.pi * k / n),
             (s / 2) * math.sin(math.pi / 2 + 2 * math.pi * k / n)) for k in range(n)]


def shape_cross(n, s):
    pts = [(0.0, 0.0)]
    per_arm = max(1, math.ceil((n - 1) / 4))
    step = (s / 2) / per_arm
    dirs = [(1, 0), (0, 1), (-1, 0), (0, -1)]
    for k in range(1, n):
        a, r = (k - 1) % 4, (k - 1) // 4 + 1
        pts.append((dirs[a][0] * r * step, dirs[a][1] * r * step))
    return pts


def shape_grid(n, s):
    cols = math.ceil(math.sqrt(n))
    rows = math.ceil(n / cols)
    d = s / max(1, cols - 1)
    pts = []
    for k in range(n):
        r, c = divmod(k, cols)
        in_row = min(cols, n - r * cols)               # a short last row is centred
        pts.append(((c - (in_row - 1) / 2) * d, ((rows - 1) / 2 - r) * d))
    return pts


def shape_heart(n, s):
    """The classic heart curve, with the drones spaced evenly ALONG it -
    even steps in the curve's parameter would bunch them at the notch and
    the tip.  The bottom tip and the top notch are both sharp corners: two
    drones either side of one would nearly touch.  So one drone sits ON
    each, and the rest share the two sides between them."""
    def curve(t):
        return (16 * math.sin(t) ** 3,
                13 * math.cos(t) - 5 * math.cos(2 * t) - 2 * math.cos(3 * t) - math.cos(4 * t))

    def along(t0, t1, m):
        """m points strictly between curve(t0) and curve(t1), evenly by length."""
        fine = [curve(t0 + (t1 - t0) * k / 1000) for k in range(1001)]
        run = [0.0]
        for a, b in zip(fine, fine[1:]):
            run.append(run[-1] + math.dist(a, b))
        pts, j = [], 0
        for k in range(1, m + 1):
            want = run[-1] * k / (m + 1)
            while run[j + 1] < want:
                j += 1
            pts.append(fine[j])
        return pts

    def gap(pts):
        return min((math.dist(a, b) for a, b in itertools.combinations(pts, 2)), default=1e9)

    tip, notch = curve(math.pi), curve(0.0)
    if n == 1:
        raw = [tip]
    else:
        left = (n - 2) // 2                                  # an odd one goes right
        raw = [tip] + along(math.pi, 2 * math.pi, n - 2 - left) + [notch] + along(0.0, math.pi, left)
        if n % 2:
            # An odd count makes that lopsided.  The symmetric choice leaves
            # the notch empty, which is fine while the drones are far apart
            # (few of them) - so take it unless it costs real spacing.
            half = (n - 1) // 2
            even = [tip] + along(math.pi, 2 * math.pi, half) + along(0.0, math.pi, half)
            if gap(even) >= 0.9 * gap(raw):
                raw = even
    box = [curve(2 * math.pi * k / 400) for k in range(400)]
    us, ws = [p[0] for p in box], [p[1] for p in box]
    k = s / max(max(us) - min(us), max(ws) - min(ws))
    cu, cw = (max(us) + min(us)) / 2, (max(ws) + min(ws)) / 2
    return [((u - cu) * k, (w - cw) * k) for u, w in raw]


SHAPES = {"line": shape_line, "v": shape_v, "circle": shape_circle, "cross": shape_cross,
          "grid": shape_grid, "heart": shape_heart, "points": None}


def slots(form, n):
    """A formation's n slots in world coordinates (x east, y north, z up)."""
    if form.shape == "points":
        if len(form.points) != n:
            raise SystemExit(f"line {form.line_no}: 'points' has {len(form.points)} points for {n} drones")
        return list(form.points)
    o = form.opts
    local = SHAPES[form.shape](n, o.get("size", 10.0))
    cx, cy, cz = o.get("x", 0.0), o.get("y", 10.0), o.get("z", 10.0)
    turn = math.radians(o.get("turn", 0.0))
    c, s = math.cos(turn), math.sin(turn)
    out = []
    for u, w in local:
        if o.get("plane", "flat") == "wall":          # standing up, facing south
            out.append((cx + u * c, cy + u * s, cz + w))
        else:                                         # lying flat, seen from above
            out.append((cx + u * c - w * s, cy + u * s + w * c, cz))
    return out


# --------------------------------------------------------------------------
# Who goes where: the Hungarian algorithm (Kuhn 1955, Munkres 1957)
# --------------------------------------------------------------------------
def hungarian(cost):
    """The assignment row -> column with the smallest total cost, for a square
    matrix.  O(n^3), the potentials form (as in e-maxx.ru's write-up):
    u[] and v[] are prices on rows and columns kept so that every cost minus
    its row and column price is >= 0; rows are added one at a time, each
    along the cheapest augmenting path in those reduced costs."""
    n = len(cost)
    INF = float("inf")
    u, v = [0.0] * (n + 1), [0.0] * (n + 1)
    owner = [0] * (n + 1)               # owner[col] = the row holding it (1-based)
    way = [0] * (n + 1)
    for row in range(1, n + 1):
        owner[0] = row
        col0 = 0
        best = [INF] * (n + 1)
        used = [False] * (n + 1)
        while True:
            used[col0] = True
            r0, delta, col1 = owner[col0], INF, 0
            for col in range(1, n + 1):
                if not used[col]:
                    cur = cost[r0 - 1][col - 1] - u[r0] - v[col]
                    if cur < best[col]:
                        best[col], way[col] = cur, col0
                    if best[col] < delta:
                        delta, col1 = best[col], col
            for col in range(n + 1):
                if used[col]:
                    u[owner[col]] += delta
                    v[col] -= delta
                else:
                    best[col] -= delta
            col0 = col1
            if owner[col0] == 0:
                break
        while col0:                      # flip the augmenting path
            col1 = way[col0]
            owner[col0] = owner[col1]
            col0 = col1
    match = [0] * n
    for col in range(1, n + 1):
        match[owner[col] - 1] = col - 1
    return match


def assign(starts, goals):
    cost = [[sum((a - b) ** 2 for a, b in zip(p, q)) for q in goals] for p in starts]
    return hungarian(cost)


# --------------------------------------------------------------------------
# How fast: a trapezoidal profile, shared by every drone in a move
# --------------------------------------------------------------------------
class Profile:
    """Distance along the LONGEST path as a function of time, as a fraction
    0..1 - every drone in the move multiplies it by its own path length."""

    def __init__(self, length, vmax, amax, shortest=2.0):
        self.L = length
        if length < 1e-6:
            self.T, self.ta, self.vp = shortest, shortest / 2, 0.0
            return
        if length >= vmax * vmax / amax:                 # reaches cruise speed
            self.vp, self.ta = vmax, vmax / amax
            self.T = length / vmax + vmax / amax
        else:                                            # a triangle: brakes at once
            self.vp = math.sqrt(length * amax)
            self.ta = self.vp / amax
            self.T = 2 * self.ta
        if self.T < shortest:                            # stretch very short moves
            k = self.T / shortest
            self.vp *= k
            self.ta /= k
            self.T = shortest

    def at(self, t):
        """(fraction done, fraction per second) at t seconds into the move."""
        if self.L < 1e-6:
            return (min(1.0, max(0.0, t / self.T)), 0.0)
        t = min(max(t, 0.0), self.T)
        a = self.vp / self.ta
        if t < self.ta:
            d, v = 0.5 * a * t * t, a * t
        elif t < self.T - self.ta:
            d, v = 0.5 * a * self.ta ** 2 + self.vp * (t - self.ta), self.vp
        else:
            r = self.T - t
            d, v = self.L - 0.5 * a * r * r, a * r
        return d / self.L, v / self.L


# --------------------------------------------------------------------------
# The plan
# --------------------------------------------------------------------------
class Stage:
    """One part of the show.  Each drone flies a polyline in [t0, t1]."""

    def __init__(self, kind, name, t0, paths, profile=None, hold=0.0):
        self.kind, self.name, self.t0 = kind, name, t0
        self.paths = [list(p) for p in paths]
        self.profile, self.hold = profile, hold
        self.repairs = []                                # (drone, other, how)
        self._index()

    def _index(self):
        self.lens = []
        for p in self.paths:
            seg = [math.dist(p[k], p[k + 1]) for k in range(len(p) - 1)]
            self.lens.append(seg)

    def retime(self, vmax, amax, vzmax):
        """One profile for the whole move, sized so that NO drone breaks a
        limit: the profile is planned for an 'effective length' - the longest
        path, or longer if some drone's path is so steep that flying it at
        vmax would climb or sink faster than vzmax."""
        self._index()
        eff = 0.0
        for path, seg in zip(self.paths, self.lens):
            total = sum(seg)
            steep = max((abs(path[k + 1][2] - path[k][2]) / L for k, L in enumerate(seg) if L > 1e-9),
                        default=0.0)
            eff = max(eff, total, total * steep * vmax / vzmax)
        self.profile = Profile(eff, vmax, amax)
        self.eff = eff

    @property
    def T(self):
        return self.profile.T if self.profile else self.hold

    @property
    def t1(self):
        return self.t0 + self.T

    def ends(self):
        return [p[-1] for p in self.paths]

    def sample(self, i, t):
        """Drone i at absolute time t: (position, velocity)."""
        path, seg = self.paths[i], self.lens[i]
        if self.profile is None or len(path) == 1:
            return path[-1], (0.0, 0.0, 0.0)
        frac, rate = self.profile.at(t - self.t0)       # of the profile's length
        total = sum(seg)
        d, speed = frac * total, rate * total            # ...scaled to this path
        for k, L in enumerate(seg):
            if d <= L or k == len(seg) - 1:
                a, b = path[k], path[k + 1]
                if L < 1e-9:
                    return b, (0.0, 0.0, 0.0)
                f = min(1.0, d / L)
                pos = tuple(a[j] + (b[j] - a[j]) * f for j in range(3))
                vel = tuple((b[j] - a[j]) / L * speed for j in range(3))
                return pos, vel
            d -= L
        return path[-1], (0.0, 0.0, 0.0)


class Plan:
    def __init__(self, show, stages, pads):
        self.show, self.stages, self.pads = show, stages, pads
        self.n = len(pads)

    @property
    def duration(self):
        return self.stages[-1].t1

    def stage_at(self, t):
        for k, s in enumerate(self.stages):
            if t < s.t1:
                return k, s
        return len(self.stages) - 1, self.stages[-1]

    def sample(self, t):
        """Every drone at time t: a list of (position, velocity)."""
        _, s = self.stage_at(t)
        return [s.sample(i, t) for i in range(self.n)]

    def stage_by_kind(self, kind):
        return [s for s in self.stages if s.kind == kind]


def _closest(stage, dt=0.05):
    """The closest planned pass of each pair in a stage: {(i, j): (d, t)}."""
    n = len(stage.paths)
    best = {}
    steps = max(1, int(math.ceil(stage.T / dt)))
    for k in range(steps + 1):
        t = stage.t0 + min(stage.T, k * dt)
        pos = [stage.sample(i, t)[0] for i in range(n)]
        for i, j in itertools.combinations(range(n), 2):
            d = math.dist(pos[i], pos[j])
            if (i, j) not in best or d < best[(i, j)][0]:
                best[(i, j)] = (d, t)
    return best


def _conflicts(stage, sep):
    return sorted(((d, t, i, j) for (i, j), (d, t) in _closest(stage).items() if d < sep))


def _lifted(path, h):
    a, b = path[0], path[-1]
    return [a, (a[0], a[1], a[2] + h), (b[0], b[1], b[2] + h), b]


def _repair(stage, sep, vmax, amax, vzmax, floor=2.0, tries=None):
    """Fly one of a too-close pair over (or under) the other until the stage
    is clean.  Returns True if it got there."""
    for _ in range(tries or 4 + len(stage.paths)):
        bad = _conflicts(stage, sep)
        if not bad:
            return True
        _, _, i, j = bad[0]
        best = None
        for who in (j, i):
            for h in (sep, -sep, 2 * sep, -2 * sep):
                a, b = stage.paths[who][0], stage.paths[who][-1]
                if min(a[2], b[2]) + h < floor:          # never below `floor` m
                    continue
                old = stage.paths[who]
                stage.paths[who] = _lifted([a, b], h)
                stage.retime(vmax, amax, vzmax)
                left = _conflicts(stage, sep)
                still = any((p, q) == (min(i, j), max(i, j)) for _, _, p, q in left)
                score = (still, len(left), abs(h), stage.T)
                if best is None or score < best[0]:
                    best = (score, who, h)
                stage.paths[who] = old
                stage.retime(vmax, amax, vzmax)
        if best is None:
            return False
        _, who, h = best
        a, b = stage.paths[who][0], stage.paths[who][-1]
        stage.paths[who] = _lifted([a, b], h)
        stage.retime(vmax, amax, vzmax)
        stage.repairs.append((who, i if who == j else j, "over" if h > 0 else "under", abs(h)))
    return not _conflicts(stage, sep)


class PlanError(Exception):
    pass


def plan_show(show):
    c = show.cfg
    n, sep, vmax, amax, vzmax = c["drones"], c["sep"], c["vmax"], c["amax"], c["vzmax"]
    if n < 1 or n > 64:
        raise PlanError("drones: 1 .. 64")
    if c["pads"] < sep:
        raise PlanError(f"pads are {c['pads']} m apart, closer than sep {sep} m")
    pads = [((k - (n - 1) / 2) * c["pads"], 0.0, 0.0) for k in range(n)]
    up = [(x, y, c["takeoff"]) for x, y, _ in pads]

    stages = []
    t = 0.0
    stages.append(Stage("pads", "pads", t, [[p] for p in pads], hold=1.0))
    t = stages[-1].t1
    s = Stage("takeoff", "takeoff", t, [[a, b] for a, b in zip(pads, up)])
    s.retime(vmax, amax, vzmax)
    stages.append(s)
    t, here = s.t1, up

    targets = [(f, slots(f, n)) for f in show.forms] + [(None, up)]
    for form, goal in targets:
        for (i, p), (j, q) in itertools.combinations(enumerate(goal), 2):
            if math.dist(p, q) < sep - 1e-6:
                where = f"line {form.line_no} ({form.name})" if form else "home"
                raise PlanError(f"{where}: slots {i} and {j} are {math.dist(p, q):.2f} m apart, "
                                f"closer than sep {sep} m - make the formation bigger")
        for p in goal:
            if p[2] < 2.0:
                raise PlanError(f"line {form.line_no}: a slot at {p[2]:.1f} m - keep every slot above 2 m")
        match = assign(here, goal)
        dest = [goal[match[i]] for i in range(n)]
        name = form.name if form else "home"
        mv = Stage("move", f"to {name}", t, [[a, b] for a, b in zip(here, dest)])
        mv.retime(vmax, amax, vzmax)
        if not _repair(mv, sep, vmax, amax, vzmax):
            d, tt, i, j = _conflicts(mv, sep)[0]
            raise PlanError(f"move to {name}: drones {i} and {j} would pass {d:.2f} m apart "
                            f"at t = {tt:.1f} s and no repair worked")
        stages.append(mv)
        t = mv.t1
        hold = form.hold if form else 1.0
        stages.append(Stage("hold" if form else "home", name, t, [[p] for p in dest], hold=hold))
        t = stages[-1].t1
        here = dest
    stages.append(Stage("land", "land", t, [[p] for p in here], hold=8.0))
    return Plan(show, stages, pads)


def closest_planned(plan):
    """The closest any two drones are ever planned, once off the ground."""
    best = (float("inf"), 0.0, 0, 0)
    for s in plan.stages:
        if s.kind in ("pads", "land"):
            continue
        for (i, j), (d, t) in _closest(s).items():
            if d < best[0]:
                best = (d, t, i, j)
    return best


# --------------------------------------------------------------------------
def describe(plan, out=sys.stdout):
    c = plan.show.cfg
    print(f"{plan.show.title}: {plan.n} drones, {len(plan.show.forms)} formations, "
          f"{plan.duration:.1f} s;  vmax {c['vmax']:g} m/s, vzmax {c['vzmax']:g} m/s, "
          f"amax {c['amax']:g} m/s^2, sep {c['sep']:g} m", file=out)
    print(f"  {'#':>2}  {'stage':<8} {'name':<16} {'start':>6} {'len':>6}  note", file=out)
    for k, s in enumerate(plan.stages):
        note = ""
        if s.kind in ("move", "takeoff"):
            longest = max(sum(L) for L in s.lens)
            peak = s.profile.vp * longest / s.eff if s.eff > 0 else 0.0
            note = f"longest path {longest:.1f} m, peak {peak:.1f} m/s"
        for who, other, how, h in s.repairs:
            note += f"; drone {who} flies {how} drone {other} ({h:.0f} m)"
        print(f"  {k:>2}  {s.kind:<8} {s.name:<16} {s.t0:6.1f} {s.T:6.1f}  {note}", file=out)
    d, t, i, j = closest_planned(plan)
    print(f"  closest planned pass: drones {i} and {j}, {d:.2f} m at t = {t:.1f} s "
          f"({'OK' if d >= c['sep'] - 1e-6 else 'TOO CLOSE'}, sep {c['sep']} m)", file=out)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("show")
    ap.add_argument("--csv", help="write every drone's planned position, 10 per second")
    a = ap.parse_args()
    try:
        plan = plan_show(load_show(a.show))
    except PlanError as e:
        print(f"REFUSED: {e}")
        return 1
    describe(plan)
    if a.csv:
        with open(a.csv, "w", encoding="utf-8") as fh:
            fh.write("t,drone,x,y,z,vx,vy,vz,stage\n")
            for k in range(int(plan.duration * 10) + 1):
                t = k / 10
                idx, _ = plan.stage_at(t)
                for i, (p, v) in enumerate(plan.sample(t)):
                    fh.write(f"{t:.1f},{i},{p[0]:.3f},{p[1]:.3f},{p[2]:.3f},"
                             f"{v[0]:.3f},{v[1]:.3f},{v[2]:.3f},{idx}\n")
        print(f"wrote {a.csv}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
