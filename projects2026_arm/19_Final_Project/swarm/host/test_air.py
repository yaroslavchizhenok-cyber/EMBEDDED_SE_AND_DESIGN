"""
test_air.py - the station's air and ground radio, alone and with drones on it.

    uv run test_air.py          (test.bat builds out/drone_pc.exe first)

1. The medium by itself: range, relay gain, loss, fade, cut radios, the
   ground radio, $PHY parsing.
2. Five drone_pc.exe stand-ins (the chip's agent.c, on the PC, in real time)
   joined by the station's own Air class - the same code swarm_station.py
   runs between Wokwi windows.  The operator posts a SURVEY through the
   ground radio and the test only WATCHES: the drones must take off, split
   the work, finish it and land, with nothing else from the station.  The
   air must never write anything to a drone but radio frames that a drone
   (or the ground radio) transmitted.

Exit code 0 = every check passed.
"""

import os
import queue
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "..", "..", "tools", "station"))

from radio_medium import GROUND, Medium   # noqa: E402
import swarm_station                      # noqa: E402

fails = checks = 0


def check(ok, what):
    global fails, checks
    checks += 1
    fails += not ok
    print(f"  {'ok  ' if ok else 'FAIL'}  {what}")


print("== the medium alone ==")
m = Medium(range_m=10.0, loss=0.0, fade=0.0, seed=1, ground=(0.0, -20.0))
m.truth(1, ["1", "0", "0", "400", "0", "0", "0", "2", "0"])
m.truth(2, ["2", "800", "0", "400", "0", "0", "0", "2", "0"])
m.truth(3, ["3", "1500", "0", "400", "0", "0", "0", "2", "0"])
check(m.carry(1, [1, 2, 3]) == [2], "8 m away hears it, 15 m away does not, the sender does not")
m.truth(3, ["3", "1500", "0", "400", "0", "0", "0", "2", "2"])     # drone 3 is a relay now
check(m.carry(1, [1, 2, 3]) == [2, 3], "a relay's bigger radio: the 1-3 link reaches 15 m (10 x (1 + 2) / 2)")
check(m.carry(GROUND, [1, 2, 3]) == [], "the ground radio, 20 m away, reaches nobody")
m.where[GROUND] = (0.0, -5.0, 1.0)
check(m.carry(GROUND, [1, 2, 3]) == [1, 2], "moved to 5 m: it reaches drones 1 and 2")
m.cut.add(2)
check(m.carry(1, [1, 2, 3]) == [3] and m.carry(2, [1, 2, 3]) == [], "a cut radio neither hears nor sends")
m.cut.clear()
m.loss = 0.3
got = sum(len(m.carry(1, [1, 2])) for _ in range(5000)) / 5000
check(0.66 < got < 0.74, f"30 % loss: {got * 100:.1f} % arrive")
m.loss, m.fade = 0.0, 0.25
m.truth(2, ["2", "950", "0", "400", "0", "0", "0", "2", "0"])
near = sum(len(m.carry(1, [1, 2])) for _ in range(4000)) / 4000
check(0.38 < near < 0.50, f"near the edge (9.5 of 10 m) the link fades: {near * 100:.0f} % arrive")
m.truth(9, ["9", "x"])
check(9 not in m.where, "a malformed $PHY is ignored")
m.forget(3)
check(m.carry(3, [1, 2, 3]) == [], "a drone that went away is not carried")

print("== five drone_pc.exe and the station's air: a SURVEY, start to finish ==")
exe = os.path.join(HERE, "out", "drone_pc.exe")
ports = [4101, 4102, 4103, 4104, 4105]
types = [0, 0, 1, 2, 1]                         # mapper mapper sensor relay sensor
procs = [subprocess.Popen([exe, str(i + 1), str(p), str(t)], stdout=subprocess.DEVNULL)
         for i, (p, t) in enumerate(zip(ports, types))]
written, sent = [], set()
try:
    time.sleep(0.5)
    medium = Medium(range_m=12.0, loss=0.0, ground=(6.0, -4.0))
    air = swarm_station.Air(ports, "socket", 115200, medium)
    for ln in air.links.values():               # record every line the air writes to a drone
        orig = ln.write

        def spy(text, orig=orig, port=ln.port):
            written.append((port, text.strip()))
            return orig(text)
        ln.write = spy
    orig_line = air._on_line

    def tap(port, line, orig_line=orig_line):   # and every line a drone transmits
        sent.add(line.strip())
        orig_line(port, line)
    for ln in air.links.values():
        ln.on_line = tap
    orig_tx = air.transmit

    def gtap(body, orig_tx=orig_tx):           # and every line the ground radio transmits
        sent.add(swarm_station.make_frame(body))
        orig_tx(body)
    air.transmit = gtap

    state = {}                                  # port -> (mode, x, y, z)
    texts = []

    def pump(seconds, until=None):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            try:
                kind, port, data = air.ui.get(timeout=0.05)
            except queue.Empty:
                continue
            if kind == "PHY":
                state[port] = (int(data[7]), int(data[1]) / 100, int(data[2]) / 100, int(data[3]) / 100)
            elif kind == "text":
                texts.append((port, data))
            if until and until():
                return True
        return False

    pump(3.0)
    check(len(state) == 5 and all(s[0] == 1 for s in state.values()), "five drones up, on their pads")
    t0 = time.monotonic()
    mid = air.post(2, (0, 10, 20, 30), [(3, 27), (17, 13), (10, 20)])
    pump(8.0)
    check(all(s[0] == 2 for s in state.values()), "the mission reached them through the ground radio: all five took off")
    done = pump(150.0, until=lambda: len(state) == 5 and all(s[0] == 1 and s[3] < 0.2 for s in state.values())
                and sum("going home: mission complete" in t for _, t in texts) == 5)
    took = time.monotonic() - t0
    readings = [t for _, t in texts if "done: reading" in t]
    check(done, f"survey complete and all five landed again, {took:.0f} s after posting (real time)")
    check(len(readings) == 3, f"three sensing tasks done, once each: {len(readings)} readings")
    who = {p for p, t in texts if "done: reading" in t}
    check(who <= {4103, 4105}, "only the sensors (ports 4103, 4105) did sensing tasks")
    on_pad = all(abs(state[p][1] - 3 * i) < 1.0 and abs(state[p][2]) < 1.0 for i, p in enumerate(ports))
    check(on_pad, "every drone landed on its own pad")
    bad = [t for _, t in written if not t.startswith(("$BCN,", "$MSN,", "$TSK,", "$MAP,", "$GND,"))]
    check(not bad, f"the air wrote {len(written)} lines to the drones, all radio frames")
    forged = [t for _, t in written if t not in sent]
    check(not forged, "every one of them was a drone's or the ground radio's own transmission, unchanged")
    check(air.carried > 1000, f"{air.carried} frames carried on the air")
finally:
    for p in procs:
        p.kill()

print("== five SLOW chips (1000 B/s, the 512-byte ring of link.c): the station unpaced, then paced ==")
# What a Wokwi window that runs slower than real time looks like to the air:
# bytes arrive faster than the chip takes them, and a lost byte costs a frame.


def slow_run(pace, base):
    ports = [base + i for i in range(5)]
    procs = [subprocess.Popen([exe, str(i + 1), str(p), "0", "1000", "512"], stdout=subprocess.DEVNULL)
             for i, p in enumerate(ports)]
    try:
        time.sleep(0.5)
        medium = Medium(range_m=30.0, loss=0.0, ground=(6.0, -4.0))
        air = swarm_station.Air(ports, "socket", 115200, medium, pace=pace)
        hlt, quality, flaps, most = {}, [], 0, 0
        end, settle = time.monotonic() + 35.0, time.monotonic() + 15.0   # judge the last 20 s
        first = {}
        while time.monotonic() < end:
            try:
                kind, port, data = air.ui.get(timeout=0.05)
            except queue.Empty:
                continue
            now = time.monotonic()
            for ln in air.links.values():                  # bytes on their way, not yet taken in
                if ln.credit:
                    most = max(most, ln.sent - ln.taken)
            if kind == "HLT":
                tot = [int(v) for v in data[1:5]]
                if now >= settle and port not in first:
                    first[port] = tot
                hlt[port] = tot
            elif kind == "NBT" and now >= settle:
                pairs = [int(v) for v in data[2:]]
                quality += pairs[1::2]
            elif kind == "text" and now >= settle and "-nbr" in data:
                flaps += 1
        span = 20.0
        lines = sum(hlt[p][0] - first[p][0] for p in first) / span / len(first)
        broken = sum((hlt[p][1] - first[p][1]) + (hlt[p][2] - first[p][2]) for p in first) / span / len(first)
        q = sum(quality) / len(quality) if quality else 0
        return lines, broken, q, flaps, most
    finally:
        for p in procs:
            p.kill()


u_lines, u_broken, u_q, u_flaps, _ = slow_run(False, 4111)
p_lines, p_broken, p_q, p_flaps, p_most = slow_run(True, 4121)
print(f"    per drone, over 20 s:          unpaced          paced")
print(f"      whole frames heard /s     {u_lines:8.1f}      {p_lines:8.1f}")
print(f"      frames lost or broken /s  {u_broken:8.1f}      {p_broken:8.1f}")
print(f"      mean link quality %       {u_q:8.0f}      {p_q:8.0f}")
print(f"      neighbours dropped        {u_flaps:8d}      {p_flaps:8d}")
print(f"    paced: at most {p_most} bytes were ever on their way to a drone, not yet taken in")
check(u_broken > 1.0, f"unpaced, a slow chip loses frames: {u_broken:.1f} a second each (the Wokwi symptom)")
check(p_broken < 0.2, f"paced by credit, it loses almost none: {p_broken:.2f} a second")
check(p_lines > u_lines, f"and hears MORE whole frames: {p_lines:.1f} a second against {u_lines:.1f}")
check(p_q > u_q + 8, f"links are stronger: quality {p_q:.0f} % against {u_q:.0f} %")
check(p_flaps <= u_flaps, f"and no more neighbours dropped ({p_flaps} against {u_flaps})")
check(0 < p_most <= swarm_station.DroneLink.WINDOW,
      f"the credit held: never more than {p_most} of {swarm_station.DroneLink.WINDOW} bytes outstanding")

print(f"\n{checks - fails} of {checks} checks passed")
sys.exit(1 if fails else 0)
