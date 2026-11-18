#!/usr/bin/env python3
"""
show.py - fly a show: plan it, check it, fly every drone on one clock, score it.

    uv run show.py shows/parade.show                 as fast as the PC can, then the score
    uv run show.py shows/parade.show --seed 7 --wind 6 90
    uv run show.py shows/parade.show --serve 4002    in real time, to the station:
    uv run tools/station/station.py --port socket://localhost:4002     (from the repo root)
    uv run show.py shows/parade.show --twice         fly it twice: same seed, same numbers?
    uv run show.py shows/parade.show --log flight.csv
    uv run show.py shows/parade.show --no-feedforward   positions only: watch the lag

The orchestrator's loop is the whole idea of a light show's ground station:

    every 10 ms of show time:
        if a 100 ms boundary:  each drone's target = the plan at time t
                               (position AND velocity - the velocity is the
                               pilot's feed-forward)
        swarm_step()           every drone, one tick, the same tick
        measure                how far each drone is from where the plan
                               says it should be, and how close any two are

No drone talks to another, and none knows the plan: each is only ever told
"be here, moving like this", ten times a second.  Formation is the plan's
job; staying in it is each pilot's.

Talking to the station (--serve): a TCP server speaking the board's frames,
so tools/station connects to it exactly as it connects to Wokwi:

    $SWM,t_ms,n,stage,stages,name,kind,show_ms,wind_cms,wind_deg,rtk,ff,state
                                                        10 Hz  where the show is, its settings
    $DRN,i,mode,x,y,z,gx,gy,gz,sx,sy,sz,bat,ax,ay       10 Hz  per drone, cm (ax ay: air, cm/s):
                                                               truth, planned now, slot
    $SEP,t_ms,now_cm,i,j,min_cm,mi,mj,sep_cm,safe_cm    10 Hz  closest pair now / so far
    $SCO,rms_cm,max_cm,closest_cm,landed,n,pass         at the end
Typed in the station's command box (or its buttons): abort, pause, go, wind MS [DEG].
"""

import argparse
import io
import itertools
import math
import os
import socket
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import planner                      # noqa: E402
from fleet import GPS, IDLE, MODES, TICK_MS, Fleet   # noqa: E402

TARGET_EVERY = 10                   # ticks: a new target every 100 ms (10 Hz)
FRAME_EVERY = 10                    # ticks: frames to the station at 10 Hz
GPS_WARMUP_S = 2.0                  # seconds on the pads before arming


# --------------------------------------------------------------------------
# The station link: a one-client TCP server, the board's $BODY*CS frames
# --------------------------------------------------------------------------
def frame(body):
    cs = 0
    for b in body.encode("ascii"):
        cs ^= b
    return f"${body}*{cs:02X}\r\n"


class Link:
    def __init__(self, port):
        self.srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.srv.bind(("127.0.0.1", port))
        self.srv.listen(1)
        print(f"waiting for the station:  uv run tools/station/station.py --port socket://localhost:{port}")
        self.c, _ = self.srv.accept()
        self.c.setblocking(False)
        self.buf = b""
        self.closed = False
        print("station connected")

    def send(self, text):
        try:
            self.c.sendall(text.encode("ascii"))
        except OSError:
            pass

    def lines(self):
        """Whatever the station typed since last time - its $KEY / $MIRROR
        frames are read and dropped, or its writes would block one day."""
        try:
            while True:
                data = self.c.recv(4096)
                if not data:
                    self.closed = True               # the station went away
                    break
                self.buf += data
        except BlockingIOError:
            pass
        except OSError:
            self.closed = True
        out = []
        while b"\n" in self.buf:
            raw, self.buf = self.buf.split(b"\n", 1)
            line = raw.decode("ascii", "replace").strip()
            if line and not line.startswith("$"):
                out.append(line.lower())
        return out

    def linger(self):
        """After the show: keep the connection, so the station keeps its
        picture and the score, until the station closes or Ctrl+C."""
        print("show over - the station keeps the picture; close it, or press Ctrl+C here")
        try:
            while not self.closed:
                self.lines()
                time.sleep(0.2)
        except KeyboardInterrupt:
            pass

    def close(self):
        """Close politely.  Closing a socket with unread input makes Windows
        send a reset, and the other side can lose what was still in flight -
        the score, for instance.  So: say we are done, read what is left,
        then close."""
        try:
            self.c.shutdown(socket.SHUT_WR)
            self.c.settimeout(1.0)
            while self.c.recv(4096):
                pass
        except OSError:
            pass
        self.c.close()
        self.srv.close()


# --------------------------------------------------------------------------
# The score
# --------------------------------------------------------------------------
class Score:
    def __init__(self, plan):
        self.plan, self.n = plan, plan.n
        self.sq = [[0.0, 0] for _ in range(self.n)]       # moves + holds: sum of err^2, count
        self.worst = [0.0] * self.n
        self.form_sq = [[0.0, 0] for _ in range(self.n)]  # holds only
        self.form_worst = [0.0] * self.n
        self.closest = (float("inf"), 0.0, 0, 0)
        self.settle = {}                                  # stage index -> seconds, or None
        self._last_bad = {}                               # stage index -> last time out of tolerance
        self.aborted_at = None                            # s, if the show was aborted
        self.flown = 0.0                                  # s of show time

    def add(self, t, k, stage, states, wanted, following=True):
        """One tick.  `following` is False once the drones were told to land
        (the plan's end, or an abort): from then on the plan is no yardstick."""
        tol = self.plan.show.cfg["settle"]
        self.flown = t
        if following and stage.kind not in ("pads", "land"):
            errs = []
            for i, (d, (p, _)) in enumerate(zip(states, wanted)):
                e = math.dist((d.x, d.y, d.z), p)
                errs.append(e)
                self.sq[i][0] += e * e
                self.sq[i][1] += 1
                self.worst[i] = max(self.worst[i], e)
                if stage.kind == "hold":
                    self.form_sq[i][0] += e * e
                    self.form_sq[i][1] += 1
                    self.form_worst[i] = max(self.form_worst[i], e)
            if stage.kind == "hold":
                if max(errs) > tol:
                    self._last_bad[k] = t
                self.settle.setdefault(k, None)
        for i, j in itertools.combinations(range(self.n), 2):
            a, b = states[i], states[j]
            if a.grounded and b.grounded:
                continue
            d = math.dist((a.x, a.y, a.z), (b.x, b.y, b.z))
            if d < self.closest[0]:
                self.closest = (d, t, i, j)

    def finish(self, states):
        for k in self.settle:
            s = self.plan.stages[k]
            last = self._last_bad.get(k)
            if last is None:
                self.settle[k] = 0.0
            elif last < s.t1 - 0.5:          # settled with at least half a second to spare
                self.settle[k] = last - s.t0 + 0.01
            else:
                self.settle[k] = None
        self.landed = sum(1 for d in states if d.mode == IDLE and d.grounded)
        self.states = states

    @staticmethod
    def rms(acc):
        return math.sqrt(acc[0] / acc[1]) if acc[1] else 0.0

    def overall(self):
        tot = [sum(a[0] for a in self.sq), sum(a[1] for a in self.sq)]
        ftot = [sum(a[0] for a in self.form_sq), sum(a[1] for a in self.form_sq)]
        return self.rms(tot), max(self.worst), self.rms(ftot), max(self.form_worst)

    def passed(self):
        cfg = self.plan.show.cfg
        return self.aborted_at is None and self.closest[0] >= cfg["safe"] and self.landed == self.n

    def summary(self):
        """The numbers that must repeat exactly for one seed."""
        return (tuple(round(x, 6) for x in self.overall()), round(self.closest[0], 6),
                self.landed, tuple(self.settle.get(k) for k in sorted(self.settle)))

    def report(self, out=sys.stdout):
        cfg = self.plan.show.cfg
        print("", file=out)
        print(f"  drone   track RMS   track max   formation RMS   formation max   battery   landing", file=out)
        for i in range(self.n):
            d = self.states[i]
            print(f"  {i:>5}   {self.rms(self.sq[i]):7.2f} m   {self.worst[i]:7.2f} m   "
                  f"{self.rms(self.form_sq[i]):11.2f} m   {self.form_worst[i]:11.2f} m   "
                  f"{d.charge:5.1f} %   {d.hardest_landing:.2f} m/s", file=out)
        rms, worst, frms, fworst = self.overall()
        print(f"    all   {rms:7.2f} m   {worst:7.2f} m   {frms:11.2f} m   {fworst:11.2f} m", file=out)
        print("", file=out)
        print(f"  settle (every drone within {cfg['settle']:g} m of its slot, and staying there):", file=out)
        for k in sorted(self.settle):
            s = self.plan.stages[k]
            v = self.settle[k]
            txt = f"{v:.1f} s" if v is not None else f"NOT settled in its {s.T:.0f} s hold"
            print(f"    {s.name:<12} {txt}", file=out)
        d, t, i, j = self.closest
        verdict = "OK" if d >= cfg["safe"] else "TOO CLOSE"
        print("", file=out)
        print(f"  closest approach: drones {i} and {j}, {d:.2f} m at t = {t:.1f} s "
              f"(safe {cfg['safe']:g} m: {verdict}; planned >= {cfg['sep']:g} m)", file=out)
        print(f"  landed: {self.landed} of {self.n}", file=out)
        if self.aborted_at is not None:
            print(f"  ABORTED at t = {self.aborted_at:.1f} s: nothing after it is scored", file=out)
        print(f"  {'PASS' if self.passed() else 'FAIL'}", file=out)


# --------------------------------------------------------------------------
# One flight
# --------------------------------------------------------------------------
def fly(plan, seed, wind, gps="rtk", link=None, speed=1.0, log=None, feedforward=True):
    n = plan.n
    fleet = Fleet(n, seed)
    for i, (x, y, _) in enumerate(plan.pads):
        fleet.place(i, x, y)
    fleet.wind(wind[0], math.radians(wind[1]))
    fleet.gps(*GPS[gps])
    fleet.step(int(GPS_WARMUP_S * 1000 / TICK_MS))          # let every GPS settle
    for i in range(n):
        r = fleet.arm(i)
        if r != 0:
            raise RuntimeError(f"drone {i} refused to arm ({'no GPS yet' if r == 2 else r})")
    t_arm = fleet.t_ms

    score = Score(plan)
    landing_sent = False
    aborted = paused = False
    tick = 0
    wall0 = time.monotonic()
    end_ms = int((plan.duration + 2.0) * 1000)
    logf = open(log, "w", encoding="utf-8") if log else None
    if logf:
        logf.write("t,drone,mode,x,y,z,ex,ey,ez,px,py,pz,err\n")

    while True:
        t_ms = fleet.t_ms - t_arm
        t = t_ms / 1000.0
        k, stage = plan.stage_at(t)
        wanted = plan.sample(t)

        if tick % TARGET_EVERY == 0 and not landing_sent and not aborted:
            if stage.kind == "land":
                for i in range(n):
                    fleet.land(i)
                landing_sent = True
            else:
                for i, (p, v) in enumerate(wanted):
                    fleet.target(i, p, v if feedforward else (0.0, 0.0, 0.0))

        fleet.step()
        tick += 1
        states = fleet.states()
        score.add(t, k, stage, states, wanted, following=not (landing_sent or aborted))

        if logf and tick % FRAME_EVERY == 0:
            for i, d in enumerate(states):
                p = wanted[i][0]
                logf.write(f"{t:.2f},{i},{MODES[d.mode]},{d.x:.3f},{d.y:.3f},{d.z:.3f},"
                           f"{d.ex:.3f},{d.ey:.3f},{d.ez:.3f},{p[0]:.3f},{p[1]:.3f},{p[2]:.3f},"
                           f"{math.dist((d.x, d.y, d.z), p):.3f}\n")

        if link and tick % FRAME_EVERY == 0:
            state = "PAUSED" if paused else "LANDING" if (landing_sent or aborted) else "FLYING"
            look = (wind, gps, feedforward, state)
            _frames(link, plan, t_ms, k, stage, states, wanted, score, aborted, look)
            cmds = link.lines()
            while cmds or paused:
                cmd = cmds.pop(0) if cmds else None
                if cmd == "abort" and not aborted:
                    aborted, paused = True, False
                    score.aborted_at = t
                    for i in range(n):
                        fleet.land(i)
                    link.send("abort: every drone lands where it is\r\n")
                elif cmd == "pause" and not paused:
                    paused = True
                    link.send("paused - type go\r\n")
                    _frames(link, plan, t_ms, k, stage, states, wanted, score, aborted,
                            (wind, gps, feedforward, "PAUSED"))
                elif cmd and cmd.split()[0] == "wind":       # the station's WIND buttons
                    w = cmd.split()
                    try:
                        wind = (float(w[1]), float(w[2]) if len(w) > 2 else wind[1])
                        fleet.wind(wind[0], math.radians(wind[1]))
                    except (IndexError, ValueError):
                        link.send("wind MS [DEG]\r\n")
                elif cmd == "go" and paused:
                    paused = False
                    wall0 = time.monotonic() - (tick * TICK_MS / 1000.0) / speed
                elif cmd is not None and cmd not in ("abort", "pause", "go", "start"):
                    link.send(f"? {cmd} - this show knows: abort, pause, go, wind MS [DEG]\r\n")
                if paused and not cmds:
                    # Paused, the show clock stops - and the swarm with it,
                    # since nothing steps it.  Time simply stands still.
                    time.sleep(0.1)
                    cmds = link.lines()
                    if link.closed:
                        paused = False
            ahead = (tick * TICK_MS / 1000.0) / speed - (time.monotonic() - wall0)
            if ahead > 0:
                time.sleep(ahead)

        everyone_down = all(d.mode == IDLE for d in states)
        if (landing_sent or aborted) and everyone_down:
            break
        if t_ms > end_ms + 20000:                       # 20 s past the plan: give up
            break
        if link and link.closed and not (aborted or landing_sent):
            aborted = True                              # the station went away: land
            score.aborted_at = t
            for i in range(n):
                fleet.land(i)
    if logf:
        logf.close()
    score.finish(fleet.states())
    if link:
        t_ms = fleet.t_ms - t_arm                       # the final state: everyone down
        k, stage = plan.stage_at(t_ms / 1000.0)
        _frames(link, plan, t_ms, k, stage, score.states, plan.sample(t_ms / 1000.0), score, aborted,
                (wind, gps, feedforward, "DONE"))
        rms, worst, _, _ = score.overall()
        link.send(frame(f"SCO,{rms * 100:.0f},{worst * 100:.0f},{score.closest[0] * 100:.0f},"
                        f"{score.landed},{n},{1 if score.passed() else 0}"))
        buf = io.StringIO()
        score.report(buf)
        for line in buf.getvalue().splitlines():
            link.send(line + "\r\n")
    return score


def _frames(link, plan, t_ms, k, stage, states, wanted, score, aborted=False, look=None):
    cm = lambda v: int(round(v * 100))                  # noqa: E731
    name, kind = ("ABORTED", "land") if aborted else (stage.name.replace(" ", "_"), stage.kind)
    wind, gps, ff, state = look or ((0.0, 0.0), "rtk", True, "FLYING")
    link.send(frame(f"SWM,{t_ms},{plan.n},{k},{len(plan.stages)},{name},{kind},{int(plan.duration * 1000)},"
                    f"{cm(wind[0])},{int(wind[1]) % 360},{1 if gps == 'rtk' else 0},{1 if ff else 0},{state}"))
    ends = stage.ends()
    for i, d in enumerate(states):
        p, s = wanted[i][0], ends[i]
        link.send(frame(f"DRN,{i},{MODES[d.mode]},{cm(d.x)},{cm(d.y)},{cm(d.z)},"
                        f"{cm(p[0])},{cm(p[1])},{cm(p[2])},{cm(s[0])},{cm(s[1])},{cm(s[2])},{d.charge:.0f},"
                        f"{cm(d.air_x)},{cm(d.air_y)}"))
    best = (float("inf"), 0, 0)
    for i, j in itertools.combinations(range(plan.n), 2):
        a, b = states[i], states[j]
        if a.grounded and b.grounded:
            continue
        dd = math.dist((a.x, a.y, a.z), (b.x, b.y, b.z))
        if dd < best[0]:
            best = (dd, i, j)
    if best[0] < float("inf"):
        c = score.closest
        cfg = plan.show.cfg
        link.send(frame(f"SEP,{t_ms},{cm(best[0])},{best[1]},{best[2]},{cm(c[0])},{c[2]},{c[3]},"
                        f"{cm(cfg['sep'])},{cm(cfg['safe'])}"))


# --------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("show")
    ap.add_argument("--seed", type=int, default=1, help="one seed = one exact flight (default 1)")
    ap.add_argument("--wind", type=float, nargs=2, metavar=("MS", "DEG"),
                    help="steady wind, m/s and the direction it blows toward (default: the show file's)")
    ap.add_argument("--gps", choices=sorted(GPS), help="the receivers: rtk (2 cm) or plain (lesson 14's, "
                    "~0.5 m) - default: the show file's, else rtk")
    ap.add_argument("--serve", type=int, metavar="PORT", help="real time, to the station on this TCP port")
    ap.add_argument("--speed", type=float, default=1.0, help="with --serve: 2 = twice real time")
    ap.add_argument("--log", help="CSV: every drone, 10 times a second")
    ap.add_argument("--twice", action="store_true", help="fly twice and check the two agree exactly")
    ap.add_argument("--no-feedforward", action="store_true",
                    help="send positions only, no velocities: every drone then trails its moving slot")
    a = ap.parse_args()

    show = planner.load_show(a.show)
    try:
        plan = planner.plan_show(show)
    except planner.PlanError as e:
        print(f"REFUSED before take-off: {e}")
        return 1
    planner.describe(plan)
    wind = tuple(a.wind) if a.wind else show.cfg["wind"]
    gps = a.gps or show.cfg["gps"]
    print(f"\nflying: seed {a.seed}, wind {wind[0]:g} m/s toward {wind[1]:g} deg, {gps} GPS")

    link = Link(a.serve) if a.serve else None
    w0 = time.perf_counter()
    ff = not a.no_feedforward
    score = fly(plan, a.seed, wind, gps, link=link, speed=a.speed, log=a.log, feedforward=ff)
    took = time.perf_counter() - w0
    score.report()
    print(f"  ({score.flown:.0f} s of show flown in {took:.1f} s of PC time)")
    if link:
        link.linger()
        link.close()
    ok = score.passed()

    if a.twice:
        again = fly(plan, a.seed, wind, gps, feedforward=ff)
        same = again.summary() == score.summary()
        print(f"\n  flown again with seed {a.seed}: {'IDENTICAL' if same else 'DIFFERENT'}")
        ok = ok and same
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
