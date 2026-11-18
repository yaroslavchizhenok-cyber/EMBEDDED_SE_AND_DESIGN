#!/usr/bin/env python3
"""
test_swarm.py - the swarm's host tests.  Exit code 0 = every check passed.

    uv run test_swarm.py          (test.bat builds swarm.dll first, then runs this)

What is checked, and why each matters:
  1. the DLL and fleet.py agree on swarm_drone_t      (else: wrong numbers, no crash)
  2. hungarian() = brute force on 200 random cases     (else: a worse plan, silently)
  3. every shape gives n distinct slots, for several n
  4. a move that MUST cross gets repaired, and the repaired plan really is clean
  5. a formation too small for `sep` is refused before take-off
  6. a drone follows a moving setpoint, lands and disarms
  7. a silent ground station makes the drones HOVER, not fly off
  8. shows/parade.show flies: PASS, and identically twice with one seed
"""

import itertools
import math
import os
import random
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import fleet                       # noqa: E402
import planner                     # noqa: E402
import show as showmod             # noqa: E402

failed = 0
checks = 0


def check(ok, what):
    global failed, checks
    checks += 1
    print(f"  {'ok  ' if ok else 'FAIL'}  {what}")
    if not ok:
        failed += 1


def mini_show(text):
    path = os.path.join(HERE, "out", "test.show")
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as fh:
        fh.write(text)
    return planner.load_show(path)


print("1. layout")
try:
    lib = fleet.load()
    check(True, f"swarm_drone_t: {lib.swarm_drone_size()} B, {len(fleet.Drone._fields_)} fields agree")
except Exception as e:  # noqa: BLE001
    check(False, f"fleet.load(): {e}")
    sys.exit(1)

print("2. hungarian")
rng = random.Random(3050)
bad = 0
for case in range(200):
    n = rng.randint(1, 6)
    cost = [[rng.uniform(0, 100) for _ in range(n)] for _ in range(n)]
    m = planner.hungarian(cost)
    got = sum(cost[i][m[i]] for i in range(n))
    best = min(sum(cost[i][p[i]] for i in range(n)) for p in itertools.permutations(range(n)))
    if sorted(m) != list(range(n)) or abs(got - best) > 1e-9:
        bad += 1
check(bad == 0, f"200 random matrices up to 6 x 6: {200 - bad} optimal")

print("3. shapes")
for name, fn in planner.SHAPES.items():
    if fn is None:
        continue
    for n in (2, 5, 9, 16):
        pts = fn(n, 12.0)
        d = min(math.dist(a, b) for a, b in itertools.combinations(pts, 2))
        if len(pts) != n or d < 1e-6:
            check(False, f"{name}({n}): {len(pts)} points, closest {d:.2f} m")
            break
    else:
        check(True, f"{name}: n points, all distinct")

print("4. a crossing that must be repaired")
# A row across x turning into a column along y, both 3 m apart: whichever
# drone takes which slot, two straight paths pass closer than sep.
s = mini_show("drones 3\npads 4\nsep 3\ntakeoff 5\n"
              "points 2  -3,10,8  0,10,8  3,10,8\n"
              "points 2  0,7,8  0,10,8  0,13,8\n")
p = planner.plan_show(s)
moves = [st for st in p.stages if st.kind == "move"]
repaired = sum(len(st.repairs) for st in moves)
d, t, i, j = planner.closest_planned(p)
check(repaired > 0, f"line -> column across it: {repaired} repair(s) made")
check(d >= 3.0 - 1e-6, f"repaired plan's closest pass {d:.2f} m >= sep 3 m")

print("5. refusal")
try:
    planner.plan_show(mini_show("drones 5\nsep 3\nform circle 5 z=10 size=4\n"))
    check(False, "a 4 m circle of 5 drones with sep 3 was accepted")
except planner.PlanError as e:
    check("closer than sep" in str(e), f"refused: {e}")

print("6. one drone: follow, land, disarm")
f = fleet.Fleet(1, seed=5)
f.gps(*fleet.GPS["rtk"])
f.step(200)
check(f.arm(0) == 0, "arms after 2 s of GPS")
worst = 0.0
for k in range(1500):                         # 15 s: climb, then cruise east at 2 m/s
    t = k * 0.01
    z, vz = (min(8.0, 1.0 * t), 1.0 if t < 8 else 0.0)
    x, vx = (2.0 * max(0.0, t - 9), 2.0 if t > 9 else 0.0)
    if k % 10 == 0:
        f.target(0, (x, 0.0, z), (vx, 0.0, vz))
    f.step()
    d = f.state(0)
    if t > 10:
        worst = max(worst, math.dist((d.x, d.y, d.z), (x, 0.0, z)))
check(worst < 1.0, f"cruising at 2 m/s it stays within {worst:.2f} m of the moving setpoint")
f.land(0)
f.step(3000)
d = f.state(0)
check(d.mode == fleet.IDLE and d.grounded, f"landed and disarmed ({fleet.MODES[d.mode]}, z {d.z:.2f} m)")
check(d.hardest_landing < 1.0, f"touchdown {d.hardest_landing:.2f} m/s")

print("7. silent ground station")
f = fleet.Fleet(2, seed=9)
f.place(0, -2, 0)
f.place(1, 2, 0)
f.step(200)
f.arm(0), f.arm(1)
for k in range(800):
    if k % 10 == 0:
        t = k * 0.01
        f.target(0, (-2, 0, min(6.0, t)), (0, 0, 1.0 if t < 6 else 0.0))
        f.target(1, (2 + 1.5 * max(0, t - 6), 0, min(6.0, t)), (1.5 if t > 6 else 0, 0, 1.0 if t < 6 else 0))
    f.step()
x_then = f.state(1).x
f.step(1500)                                  # 15 s and not a word
d0, d1 = f.state(0), f.state(1)
check(d1.mode == fleet.HOVER, f"no target for 15 s -> {fleet.MODES[d1.mode]}")
check(abs(d1.x - x_then) < 4.0, f"and it stopped: drifted {d1.x - x_then:+.1f} m after the link went quiet")

print("8. the parade")
plan = planner.plan_show(planner.load_show(os.path.join(HERE, "shows", "parade.show")))
cfg = plan.show.cfg
a = showmod.fly(plan, 1, cfg["wind"], cfg["gps"])
b = showmod.fly(plan, 1, cfg["wind"], cfg["gps"])
c = showmod.fly(plan, 2, cfg["wind"], cfg["gps"])
rms, worst, frms, fworst = a.overall()
check(a.passed(), f"PASS: closest {a.closest[0]:.2f} m (safe {cfg['safe']:g}), landed {a.landed}/{plan.n}")
check(frms < 0.5, f"formation RMS {frms:.2f} m with RTK in {cfg['wind'][0]:g} m/s wind")
check(a.summary() == b.summary(), "seed 1 twice: identical")
check(a.summary() != c.summary(), "seed 2: different gusts, different numbers")

print(f"\n{checks} checks, {failed} failed")
sys.exit(1 if failed else 0)
