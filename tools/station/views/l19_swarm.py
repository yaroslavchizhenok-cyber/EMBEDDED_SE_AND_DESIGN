"""
l19_swarm.py - lesson 19, the drone-swarm parade (projects2026_arm/19_Final_Project).

Five drones fly a light show inside the board: the chip plans the show,
flies every drone on one clock, scores it, and sends these frames:

    $SWM,t_ms,n,stage,stages,name,kind,show_ms,wind_cms,wind_deg,rtk,ff,state
                                                        where the show is, and its settings
    $DRN,i,mode,x,y,z,gx,gy,gz,sx,sy,sz,bat,ax,ay       cm: truth, planned now, slot;
                                                        ax ay: the air at the drone, cm/s
    $SEP,t_ms,now_cm,i,j,min_cm,mi,mj,sep_cm,safe_cm    closest pair now / so far
    $SCO,rms_cm,max_cm,closest_cm,landed,n,pass         the verdict, at the end

The board sends one line per 10 ms tick (all at once would overflow its
256 B transmit ring), so the drones' $DRN lines arrive one after another.
The lesson's run.bat opens this view once Wokwi is running.  The PC version,
host/show.py --serve, speaks the same frames (up to 64 drones).

Left: the show as the audience sees it, from the south, in 3D (scene3d.py).
Right: the same from above, then the numbers.  A line joins any two drones
closer than the planned spacing - orange - or than the safe limit - red.
Under the 3D view, every drone in numbers: position, error, battery, the
wind where it is.  On the right, the WIND and CONTROL menus: click a button
and the station types the matching shell command for you (wind 6 90,
start, gps plain ...).  Arrows turn and tip the camera, + - zoom, r resets
it: these keys stay here.  a, b, s and 1-5 go to the board.
"""

import math
import time
from collections import deque

from . import scene3d as s3
from .base import BG, COLOURS, DIM, FG, GRID, View, num

WARN, BAD, GOOD = "#ffb454", "#f07178", "#9ad36a"
MODE_COLOUR = {"IDLE": DIM, "FOLLOW": GOOD, "HOVER": WARN, "LAND": "#7fdbca"}
TRAIL_S = 3.0                     # seconds of path drawn behind each drone
CENTRE = (0.0, 10.0, 11.0)        # what the camera orbits: the middle of the show


def _compass(deg):
    """The compass point a direction (degrees, counter-clockwise from east) is nearest."""
    names = ("E", "NE", "N", "NW", "W", "SW", "S", "SE")
    return names[int(((deg % 360) + 22.5) // 45) % 8]


def _cm(f, k):
    v = [num(x) for x in f[k:]]
    return None if any(not isinstance(x, (int, float)) or not math.isfinite(x) for x in v) else [x / 100.0 for x in v]


class Swarm(View):
    name = "swarm"
    title = "Lesson 19 - drone swarm parade"
    frames = ("SWM", "DRN", "SEP", "SCO")
    help = "click the WIND / CONTROL buttons, or keys a start, b abort, s pause, 1-5 wind;  arrows + - r = camera"
    layout = "view"

    def __init__(self, canvas, station):
        super().__init__(canvas, station)
        self.cam = s3.Camera(fov_deg=55.0)
        self.board_lines = deque(maxlen=6)    # the board's last text lines (not frames)
        self._cmds = {}                       # button tag -> shell command, rebuilt each draw
        self.sent = None                      # (command, when) - shown for 2 s
        canvas.tag_bind("btn", "<Button-1>", self._click)
        canvas.tag_bind("btn", "<Enter>", lambda e: canvas.configure(cursor="hand2"))
        canvas.tag_bind("btn", "<Leave>", lambda e: canvas.configure(cursor=""))
        self._cam_reset()
        self._reset()

    def text(self, line):
        """A line the board printed that is not a frame: shown while no drones have arrived."""
        line = line.strip()
        if line:
            self.board_lines.append(line[:90])

    def _cam_reset(self):
        self.heading, self.tip, self.dist = 0.0, 8.0, 40.0

    def _reset(self):
        self.n = 0
        self.t_ms = 0
        self.stage = ("", "", 0, 0)            # name, kind, index, count
        self.show_ms = 0
        self.drones = {}                       # i -> dict
        self.trails = {}                       # i -> deque of (t_ms, x, y, z)
        self.sep = None                        # (now, i, j, min, mi, mj, sep, safe)
        self.score = None
        self.settings = None                   # wind, deg, rtk, ff, state - from $SWM

    # ---- frames -----------------------------------------------------------
    def frame(self, name, f, t):
        if name == "SWM" and len(f) >= 7:
            tm, n, k, ks, sm = num(f[0]), num(f[1]), num(f[2]), num(f[3]), num(f[6])
            if not all(isinstance(v, int) for v in (tm, n, k, ks, sm)) or not 0 < n <= 64:
                return
            if tm < self.t_ms - 500 or n != self.n:      # a new show
                self._reset()
            self.t_ms, self.n, self.show_ms = tm, n, sm
            self.stage = (f[4].replace("_", " ")[:20], f[5][:8], k, ks)
            if len(f) >= 12:                             # the settings (lesson 19, 2026-11-18 on)
                wcm, wdeg, rtk, ff = num(f[7]), num(f[8]), num(f[9]), num(f[10])
                if all(isinstance(v, int) for v in (wcm, wdeg, rtk, ff)):
                    self.settings = {"wind": wcm / 100.0, "deg": float(wdeg), "rtk": bool(rtk),
                                     "ff": bool(ff), "state": f[11][:8]}
        elif name == "DRN" and len(f) >= 12:
            i = num(f[0])
            v = _cm(f[:11], 2)
            bat = num(f[11])
            if not isinstance(i, int) or not 0 <= i < 64 or v is None:
                return
            d = {"mode": f[1][:7], "pos": tuple(v[0:3]), "plan": tuple(v[3:6]), "slot": tuple(v[6:9]),
                 "bat": bat if isinstance(bat, (int, float)) else None}
            d["err"] = math.dist(d["pos"], d["plan"])
            if len(f) >= 14:                             # the air at this drone, cm/s
                ax, ay = num(f[12]), num(f[13])
                if isinstance(ax, int) and isinstance(ay, int):
                    d["air"] = (ax / 100.0, ay / 100.0)
            self.drones[i] = d
            tr = self.trails.setdefault(i, deque())
            tr.append((self.t_ms, *d["pos"]))
            while tr and tr[0][0] < self.t_ms - TRAIL_S * 1000:
                tr.popleft()
        elif name == "SEP" and len(f) >= 9:
            now, mn, sp, sf = (num(f[k]) for k in (1, 4, 7, 8))
            ij = [num(f[k]) for k in (2, 3, 5, 6)]
            if all(isinstance(v, (int, float)) for v in (now, mn, sp, sf)) and all(isinstance(v, int) for v in ij):
                self.sep = (now / 100.0, ij[0], ij[1], mn / 100.0, ij[2], ij[3], sp / 100.0, sf / 100.0)
        elif name == "SCO" and len(f) >= 6:
            v = [num(x) for x in f[:6]]
            if all(isinstance(x, int) for x in v):
                self.score = v

    # ---- keys: the camera -------------------------------------------------
    def key(self, keysym, char):
        if keysym == "Left":
            self.heading -= 10
        elif keysym == "Right":
            self.heading += 10
        elif keysym == "Up":
            self.tip = min(80.0, self.tip + 5)
        elif keysym == "Down":
            self.tip = max(-5.0, self.tip - 5)
        elif char in ("+", "="):
            self.dist = max(10.0, self.dist * 0.85)
        elif char == "-":
            self.dist = min(150.0, self.dist / 0.85)
        elif char == "r":
            self._cam_reset()
        else:
            return False
        return True

    # ---- picture ----------------------------------------------------------
    def draw(self):
        c, w, h = self.c, self.w, self.h
        c.delete("all")
        c.configure(bg=BG)
        self._cmds = {}
        if not self.drones:
            if self.board_lines:                 # the board talks, but sends no drones yet
                said = "\n".join(self.board_lines)
                msg = ("no drones from the board yet - it plans the show at power-up,\n"
                       "then warms its GPS up (slower in Wokwi).  The board says:\n\n" + said)
            else:
                msg = ("nothing from the board yet.\n\n"
                       "Is the simulator running with THIS lesson's wokwi.toml\n"
                       "(F1 - Wokwi: Select Config File)?  The status line, top right,\n"
                       "says whether the station is connected to localhost:4000.")
            c.create_text(w / 2, h / 2, fill=FG, font=self.fnt(13), justify="center", text=msg)
            return
        right = max(320, int(w * 0.30))
        lw = w - right
        ft = self.fnt(12)
        lh = int(ft[1] * 1.75)
        rows = min(len(self.drones), 12)
        table_h = lh * (rows + 1) + 18
        scene_h = h - table_h
        self._scene(0, 0, lw, scene_h)
        self._table(10, scene_h + 8, lw - 20, ft, lh)
        if self.score:
            self._verdict(0, 0, lw, scene_h)
        rx, rw = lw + 8, right - 16
        ms = min(rw, int(h * 0.30))
        self._map(rx, 6, ms)
        y = self._wind(rx, ms + 16, rw)
        y = self._controls(rx, y + 10, rw)
        self._closest(rx, y + 8, rw)

    # ---- the menu: buttons that type shell commands for you ------------------
    def _button(self, x, y, w, h, label, cmd, active=False, enabled=True, colour=None):
        """A clickable box.  Clicking sends `cmd` to the board as if typed in the
        command box.  `active` = the setting in force now, drawn filled."""
        c = self.c
        col = colour or FG
        tags = ()
        if enabled and cmd:
            k = len(self._cmds)
            self._cmds[f"btn{k}"] = cmd
            tags = ("btn", f"btn{k}")
        if not enabled:
            fill, out, txt = "#14181d", GRID, DIM
        elif active:
            fill, out, txt = s3.blend(col, BG, 0.25), col, BG
        else:
            fill, out, txt = "#161b21", s3.blend(col, BG, 0.4), col
        c.create_rectangle(x, y, x + w, y + h, fill=fill, outline=out, width=2 if active else 1, tags=tags)
        c.create_text(x + w / 2, y + h / 2, text=label, fill=txt, font=self.fnt(10, True), tags=tags)

    def _click(self, _event):
        item = self.c.find_withtag("current")
        if not item:
            return
        for tag in self.c.gettags(item[0]):
            cmd = self._cmds.get(tag)
            if cmd:
                self.station.send_line(cmd)
                self.sent = (cmd, time.monotonic())
                return

    # ---- the drones, in numbers ---------------------------------------------------
    def _table(self, x, y, w, f, lh):
        c = self.c
        cols = [("#", 0.00), ("mode", 0.05), ("x m", 0.19), ("y m", 0.29), ("z m", 0.39),
                ("error m", 0.50), ("battery", 0.63), ("local wind", 0.78)]
        c.create_rectangle(x - 6, y - 4, x + w + 6, y + lh * (min(len(self.drones), 12) + 1) + 4,
                           outline=GRID, fill="#0d1115")
        for label, fx in cols:
            c.create_text(x + fx * w, y, anchor="nw", fill=DIM, font=f, text=label)
        y += lh
        for n, (i, d) in enumerate(sorted(self.drones.items())):
            if n >= 12:
                break
            col = COLOURS[i % len(COLOURS)]
            px, py, pz = d["pos"]
            flying = d["mode"] in ("FOLLOW", "HOVER")
            err = d["err"] if flying else None
            m = lambda v, n: f"{v:{n}.1f}".replace("-0.0", " 0.0")          # noqa: E731 - no "-0.0"
            cells = [(f"{i}", col), (d["mode"], MODE_COLOUR.get(d["mode"], FG)),
                     (m(px, 6), FG), (m(py, 6), FG), (m(pz, 5), FG),
                     (f"{err:5.2f}" if err is not None else "  -", BAD if (err or 0) > 1.0 else WARN if (err or 0) > 0.5 else FG)]
            for (text, colr), (_, fx) in zip(cells, cols):
                c.create_text(x + fx * w, y, anchor="nw", fill=colr, font=f, text=text)
            bat = d["bat"]                                      # a battery bar
            bx, bw = x + 0.63 * w, 0.12 * w
            if isinstance(bat, (int, float)):
                b = max(0.0, min(100.0, float(bat)))
                c.create_rectangle(bx, y + 3, bx + bw, y + lh - 6, outline=DIM)
                c.create_rectangle(bx, y + 3, bx + bw * b / 100.0, y + lh - 6, outline="",
                                   fill=BAD if b < 25 else WARN if b < 50 else GOOD)
                c.create_text(bx + bw + 6, y, anchor="nw", fill=FG, font=self.fnt(10), text=f"{b:.0f}%")
            air = d.get("air")
            if air:
                sp = math.hypot(*air)
                ax = x + 0.78 * w + 10
                ay = y + lh / 2 - 2
                k = 1.6 * min(1.0, sp / 10.0) * 10 + 4                   # arrow length, px
                if sp > 0.05:
                    c.create_line(ax - air[0] / sp * k / 2, ay + air[1] / sp * k / 2,
                                  ax + air[0] / sp * k / 2, ay - air[1] / sp * k / 2,
                                  fill=col, width=2, arrow="last")
                c.create_text(ax + 16, y, anchor="nw", fill=FG, font=f, text=f"{sp:4.1f} m/s")
            y += lh

    # ---- the wind: what it is, and buttons to change it ------------------------------
    def _wind(self, x, y, w):
        c = self.c
        st = self.settings
        c.create_text(x, y, anchor="nw", fill=FG, font=self.fnt(13, True), text="WIND")
        y += int(self.fnt(13)[1] * 2.0)
        if st is None:
            c.create_text(x, y, anchor="nw", fill=DIM, font=self.fnt(10),
                          text="(this board sends no wind data - rebuild lesson 19)")
            return y + 20
        speed, deg = st["wind"], st["deg"]
        r = max(30, min(int(w * 0.20), 62))
        cx, cy = x + r + 4, y + r + 4
        c.create_oval(cx - r, cy - r, cx + r, cy + r, outline=GRID, width=2)
        for lab, ang in (("N", 90), ("E", 0), ("S", 270), ("W", 180)):
            a = math.radians(ang)
            c.create_text(cx + (r + 9) * math.cos(a), cy - (r + 9) * math.sin(a), text=lab, fill=DIM,
                          font=self.fnt(9, True))
        scale = r / 10.0                                         # px per m/s: 10 m/s = the rim
        for i, d in sorted(self.drones.items()):                 # each drone's own air: gusts
            air = d.get("air")
            if air and math.hypot(*air) > 0.05:
                c.create_line(cx, cy, cx + air[0] * scale, cy - air[1] * scale,
                              fill=s3.blend(COLOURS[i % len(COLOURS)], BG, 0.35), width=1, arrow="last")
        if speed > 0.05:
            a = math.radians(deg)
            ln = min(r, speed * scale)
            c.create_line(cx, cy, cx + ln * math.cos(a), cy - ln * math.sin(a), fill=FG, width=4,
                          arrow="last", arrowshape=(12, 14, 5))
        tx = x + 2 * r + 34                                    # clear of the rim's E
        c.create_text(tx, y, anchor="nw", fill=FG, font=self.fnt(20, True), text=f"{speed:.1f} m/s")
        c.create_text(tx, y + self.fnt(20)[1] * 1.9, anchor="nw", fill=FG, font=self.fnt(11),
                      text=f"blowing toward {deg % 360:.0f}° ({_compass(deg)})")
        airs = [math.hypot(*d["air"]) for d in self.drones.values() if d.get("air")]
        if airs:
            c.create_text(tx, y + self.fnt(20)[1] * 1.9 + self.fnt(11)[1] * 1.8, anchor="nw", fill=DIM,
                          font=self.fnt(10), text=f"at the drones, with gusts:\n{min(airs):.1f} - {max(airs):.1f} m/s")
        y = cy + r + 14
        bh = int(self.fnt(10)[1] * 2.3)
        c.create_text(x, y, anchor="nw", fill=DIM, font=self.fnt(10), text="speed, m/s")
        y += int(self.fnt(10)[1] * 1.7)
        speeds = (0, 1, 3, 5, 7, 9)                         # keys 1-5 set 1 3 5 7 9
        bw = (w - 5 * 4) / len(speeds)
        for k, v in enumerate(speeds):
            self._button(x + k * (bw + 4), y, bw, bh, str(v), f"wind {v} {int(deg) % 360}",
                         active=abs(speed - v) < 1.0)
        y += bh + 8
        c.create_text(x, y, anchor="nw", fill=DIM, font=self.fnt(10), text="blowing toward")
        y += int(self.fnt(10)[1] * 1.7)
        dirs = (("N", 90), ("NE", 45), ("E", 0), ("SE", 315), ("S", 270), ("SW", 225), ("W", 180), ("NW", 135))
        bw = (w - 7 * 4) / len(dirs)
        for k, (lab, ang) in enumerate(dirs):
            diff = abs((deg - ang + 180) % 360 - 180)
            self._button(x + k * (bw + 4), y, bw, bh, lab, f"wind {int(round(speed))} {ang}", active=diff < 22.5)
        return y + bh

    # ---- the show: state, start/pause/abort, GPS, feed-forward -------------------------
    def _controls(self, x, y, w):
        c = self.c
        st = self.settings
        c.create_text(x, y, anchor="nw", fill=FG, font=self.fnt(13, True), text="CONTROL")
        state = st["state"] if st else "?"
        sc = {"FLYING": GOOD, "LANDING": "#7fdbca", "PAUSED": WARN, "READY": FG, "DONE": FG,
              "REFUSED": BAD, "WARMUP": DIM}.get(state, FG)
        c.create_text(x + w, y + 2, anchor="ne", fill=sc, font=self.fnt(12, True), text=state)
        y += int(self.fnt(13)[1] * 2.0)
        bh = int(self.fnt(10)[1] * 2.6)
        bw = (w - 2 * 6) / 3
        flying = state in ("FLYING", "LANDING", "PAUSED")
        self._button(x, y, bw, bh, "GO" if state == "PAUSED" else "START  (a)", "start", colour=GOOD,
                     enabled=state in ("READY", "DONE", "PAUSED", "WARMUP"))
        self._button(x + bw + 6, y, bw, bh, "PAUSE  (s)", "pause", colour=WARN, enabled=state in ("FLYING", "LANDING"))
        self._button(x + 2 * (bw + 6), y, bw, bh, "ABORT  (b)", "abort", colour=BAD, enabled=flying)
        y += bh + 8
        if st:
            bw2 = (w - 6) / 2
            ground = not flying
            self._button(x, y, bw2, bh, "GPS: RTK" if st["rtk"] else "GPS: plain",
                         "gps plain" if st["rtk"] else "gps rtk", active=st["rtk"], enabled=ground)
            self._button(x + bw2 + 6, y, bw2, bh, "feed-forward: on" if st["ff"] else "feed-forward: off",
                         "ff off" if st["ff"] else "ff on", active=st["ff"], enabled=ground)
            y += bh + 4
            c.create_text(x, y, anchor="nw", fill=DIM, font=self.fnt(9),
                          text="GPS and feed-forward change on the ground only; wind any time")
            y += int(self.fnt(9)[1] * 1.8)
        if self.sent and time.monotonic() - self.sent[1] < 2.0:
            c.create_text(x, y, anchor="nw", fill=GOOD, font=self.fnt(10), text=f"sent:  {self.sent[0]}")
        return y + int(self.fnt(10)[1] * 1.8)

    def _closest(self, x, y, w):
        if not self.sep:
            return
        c = self.c
        f = self.fnt(11)
        lh = int(f[1] * 1.8)
        now, i, j, mn, mi, mj, sep, safe = self.sep
        col = BAD if now < safe else WARN if now < sep else GOOD
        c.create_text(x, y, anchor="nw", fill=col, font=f, text=f"closest now   {now:5.2f} m  (drones {i}-{j})")
        col = BAD if mn < safe else WARN if mn < sep else GOOD
        c.create_text(x, y + lh, anchor="nw", fill=col, font=f, text=f"closest ever  {mn:5.2f} m  (drones {mi}-{mj})")
        c.create_text(x, y + 2 * lh, anchor="nw", fill=DIM, font=self.fnt(10),
                      text=f"planned >= {sep:g} m, safe >= {safe:g} m")

    def _pairs(self):
        """Pairs closer than the planned spacing, with how close."""
        if not self.sep:
            return []
        sep = self.sep[6]
        out = []
        items = sorted(self.drones.items())
        for a in range(len(items)):
            for b in range(a + 1, len(items)):
                (i, di), (j, dj) = items[a], items[b]
                if di["pos"][2] < 0.2 and dj["pos"][2] < 0.2:
                    continue
                d = math.dist(di["pos"], dj["pos"])
                if d < sep:
                    out.append((d, i, j))
        return out

    def _scene(self, x0, y0, w, h):
        c, cam = self.c, self.cam
        hd, tp = math.radians(self.heading), math.radians(self.tip)
        fwd = (math.sin(hd) * math.cos(tp), math.cos(hd) * math.cos(tp), -math.sin(tp))
        cam.pos = s3.sub(CENTRE, s3.mul(fwd, self.dist))
        cam.heading_deg, cam.pitch_deg = self.heading, self.tip
        cam.fit(w, h, x0, y0)

        hy = cam.horizon_y()                                  # sky and ground
        if hy > y0:
            c.create_rectangle(x0, y0, x0 + w, min(y0 + h, hy), fill="#0b0f19", outline="")
        if hy < y0 + h:
            c.create_rectangle(x0, max(y0, hy), x0 + w, y0 + h, fill="#0f1612", outline="")
        for g in range(-30, 31, 5):                            # the field, 5 m squares
            for a, b in (((g, -15, 0), (g, 40, 0)), ((-30, g + 10, 0), (30, g + 10, 0))):
                seg = cam.segment(a, b)
                if seg:
                    c.create_line(*seg[:4], fill="#1d2a22")

        items = []                                             # (depth, kind, data) - far first
        for i, d in self.drones.items():
            col = COLOURS[i % len(COLOURS)]
            p, s = d["pos"], d["slot"]
            items.append((cam.project((p[0], p[1], 0.0)), "shadow", col))
            sl = cam.project(s)
            if sl and d["mode"] != "IDLE" and math.dist(p, s) > 0.3:
                items.append((sl, "slot", col))
            tr = self.trails.get(i)
            if tr and len(tr) > 1:
                for run in cam.polyline([q[1:] for q in tr]):
                    c.create_line(*run, fill=s3.blend(col, BG, 0.55), width=2)
            items.append((cam.project(p), "drone", (i, col, d)))
        for dd, i, j in self._pairs():
            seg = cam.segment(self.drones[i]["pos"], self.drones[j]["pos"])
            if seg:
                bad = dd < self.sep[7]
                c.create_line(*seg[:4], fill=BAD if bad else WARN, width=4 if bad else 2, dash=() if bad else (5, 3))
                c.create_text((seg[0] + seg[2]) / 2, (seg[1] + seg[3]) / 2 - 8, fill=BAD if bad else WARN,
                              font=self.fnt(9), text=f"{dd:.1f} m")
        items = [it for it in items if it[0] is not None]
        items.sort(key=lambda it: -it[0][2])
        for (sx, sy, depth), kind, data in items:
            r = max(3.0, min(16.0, cam.f * 0.25 / depth))
            if kind == "shadow":
                c.create_oval(sx - r, sy - r * 0.35, sx + r, sy + r * 0.35, fill="#070a08", outline="")
            elif kind == "slot":
                c.create_oval(sx - r, sy - r, sx + r, sy + r, outline=data, dash=(3, 3), width=1)
            else:
                i, col, d = data
                lit = d["mode"] in ("FOLLOW", "HOVER", "LAND")
                if lit:                                        # the light, glowing
                    g = r * 2.2
                    c.create_oval(sx - g, sy - g, sx + g, sy + g, fill=s3.blend(col, BG, 0.78), outline="")
                c.create_oval(sx - r, sy - r, sx + r, sy + r, fill=col if lit else BG, outline=col, width=2)
                c.create_text(sx, sy - r - 4, anchor="s", fill=col, font=self.fnt(9), text=str(i))

        name, kind, k, ks = self.stage
        c.create_text(x0 + 12, y0 + 10, anchor="nw", fill=FG, font=self.fnt(20, True), text=name.upper())
        c.create_text(x0 + 12, y0 + 10 + self.fnt(20)[1] * 1.8, anchor="nw", fill=DIM, font=self.fnt(11),
                      text=f"{kind}  -  stage {k + 1} of {ks}  -  t {self.t_ms / 1000:.1f} s of "
                           f"{self.show_ms / 1000:.0f} s  -  seen from the south")
        if kind == "pads" and not self.score:                  # waiting to start
            c.create_text(x0 + w / 2, y0 + h * 0.45, fill=GOOD, font=self.fnt(16, True), justify="center",
                          text="on the pads - press START (right) or  a  to start the show")
        if self.show_ms:                                       # progress
            frac = max(0.0, min(1.0, self.t_ms / self.show_ms))
            c.create_rectangle(x0 + 12, y0 + h - 14, x0 + w - 12, y0 + h - 8, outline=GRID)
            c.create_rectangle(x0 + 12, y0 + h - 14, x0 + 12 + (w - 24) * frac, y0 + h - 8, outline="", fill=GOOD)

    def _map(self, x0, y0, s):
        c = self.c
        c.create_rectangle(x0, y0, x0 + s, y0 + s, outline=GRID, fill="#0d1115")
        pts = [d["pos"] for d in self.drones.values()] + [d["slot"] for d in self.drones.values()]
        xs, ys = [p[0] for p in pts] + [-10, 10], [p[1] for p in pts] + [-2, 14]
        cx, cy = (max(xs) + min(xs)) / 2, (max(ys) + min(ys)) / 2
        span = max(max(xs) - min(xs), max(ys) - min(ys)) * 1.25 + 2
        k = s / span

        def P(x, y):
            return x0 + s / 2 + (x - cx) * k, y0 + s / 2 - (y - cy) * k

        step = 5 if 5 * k >= 18 else 10
        for g in range(int((cx - span) // step) * step, int(cx + span) + step, step):
            gx = P(g, 0)[0]
            if x0 < gx < x0 + s:
                c.create_line(gx, y0, gx, y0 + s, fill=GRID)
        for g in range(int((cy - span) // step) * step, int(cy + span) + step, step):
            gy = P(0, g)[1]
            if y0 < gy < y0 + s:
                c.create_line(x0, gy, x0 + s, gy, fill=GRID)
        c.create_text(x0 + 6, y0 + 4, anchor="nw", fill=DIM, font=self.fnt(9), text=f"from above, grid {step} m, N up")
        for dd, i, j in self._pairs():
            a, b = self.drones[i]["pos"], self.drones[j]["pos"]
            c.create_line(*P(a[0], a[1]), *P(b[0], b[1]), fill=BAD if dd < self.sep[7] else WARN, width=2)
        for i, d in sorted(self.drones.items()):
            col = COLOURS[i % len(COLOURS)]
            tr = self.trails.get(i)
            if tr and len(tr) > 1:
                c.create_line(*[q for p in tr for q in P(p[1], p[2])], fill=s3.blend(col, BG, 0.5))
            sx, sy = P(*d["slot"][:2])
            c.create_rectangle(sx - 4, sy - 4, sx + 4, sy + 4, outline=col)
            px, py = P(*d["pos"][:2])
            c.create_oval(px - 5, py - 5, px + 5, py + 5, fill=col, outline="")

    def _verdict(self, x0, y0, w, h):
        c = self.c
        rms, worst, closest, landed, n, ok = self.score
        col = GOOD if ok else BAD
        bw, bh = min(w - 40, 560), int(self.fnt(22)[1] * 5.5)
        bx, by = x0 + (w - bw) / 2, y0 + h * 0.22
        c.create_rectangle(bx, by, bx + bw, by + bh, fill="#0b0e11", outline=col, width=3)
        c.create_text(bx + bw / 2, by + 10, anchor="n", fill=col, font=self.fnt(22, True),
                      text="PASS" if ok else "FAIL")
        c.create_text(bx + bw / 2, by + bh - 10, anchor="s", fill=FG, font=self.fnt(11), justify="center",
                      text=f"tracking RMS {rms / 100:.2f} m, worst {worst / 100:.2f} m\n"
                           f"closest approach {closest / 100:.2f} m   -   landed {landed} of {n}")
