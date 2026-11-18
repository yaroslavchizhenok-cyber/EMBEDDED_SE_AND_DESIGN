# Swarm parade — instructions

## What it is

A drone light show, flown inside the board. Five simulated drones take off
together, fly seven formations (line, V, circle, heart, cross, arrow, a wide
circle), fly home and land. Each drone has its own flight computer, its own
gusts and its own GPS error. No drone talks to another: each is only told
"be here, moving like this" ten times a second.

When the board powers up, it **plans the whole show** and proves that no two
drones are ever planned closer than 3 m. If it cannot, the show is
**refused** and the serial monitor says why. Then it waits for you. In
flight it keeps everyone on one clock, measures how far each drone is from
where it should be and how close any two come, and at the end it prints a
score: **PASS** or **FAIL**.

The station on the PC draws the show: a 3D view from the audience (south of
the pads, looking north), a map from above, and every drone's numbers.

## Start it

1. **Build:** `build.bat` in this folder.
2. **Simulate:** in VS Code press F1, choose **Wokwi: Select Config File**
   and pick this folder's `wokwi.toml`. Then F1, **Wokwi: Start Simulator**.
3. **Station:** `run.bat` in this folder.
4. **Fly:** wait until the five drones appear on their pads and the
   picture says *press a to start* (the board plans the show and warms its
   GPS up first, a few seconds in Wokwi). Click the picture and press `a`.

   It does not matter which you start first, the simulator or the station:
   while nothing is moving, the board sends its picture again every second.
   If the station still says *nothing from the board yet*, it is not
   connected: check the status line (top right) and that Wokwi was started
   with this folder's `wokwi.toml`.

To fly again after the score, press `a` again. To start from scratch, stop
and start the simulator; the station reconnects by itself.

**In Wokwi the show runs in slow motion**, because simulating five drones is
more work than Wokwi's CPU can do in real time. Nothing is wrong: the show
clock in the station's title line is the board's own clock. Type `stats` to
see how fast it is running. On a real board it runs in real time.

## Controls

### The station's menu (right side, click with the mouse)

| Panel | Buttons | Does |
|---|---|---|
| **WIND** | speed `0` `1` `3` `5` `7` `9` | steady wind speed, m/s. The one in force is lit |
| | toward `N` `NE` `E` `SE` `S` `SW` `W` `NW` | the direction the wind blows **toward**; keeps the speed |
| **CONTROL** | `START` / `GO` | start the show; after a pause, carry on; after the score, fly again |
| | `PAUSE` | the show clock stands still |
| | `ABORT` | every drone lands where it is (scored FAIL) |
| | `GPS: RTK` / `GPS: plain` | switch the receivers (on the ground only) |
| | `feed-forward: on` / `off` | switch the pilots' feed-forward (on the ground only) |

A button only types the matching shell command for you (`wind 5 90`,
`start`, `gps plain` ...) and shows it under the panel for 2 s. A greyed
button cannot be used now: PAUSE before the start, GPS during a flight.

### Keys (click the station's picture first)

| Key | Does |
|---|---|
| `a` | start the show; after a pause, carry on; after the score, fly it again |
| `b` (or `p`) | **abort**: every drone lands straight down where it is. The run is scored FAIL |
| `s` | pause (the show clock stands still); press again to carry on |
| `1` `2` `3` `4` `5` | wind 1, 3, 5, 7, 9 m/s |
| `?` | print the list of keys and commands |
| Left / Right | turn the camera around the show |
| Up / Down | tip the camera up or down |
| `+` / `-` | zoom in / out |
| `r` | reset the camera |

The camera keys stay in the station; the others go to the board.

### The board in Wokwi

| Control | Does |
|---|---|
| button **A** | start / carry on / fly again (same as `a`) |
| button **B** | abort (same as `b`) |
| joystick **press** | pause / carry on (same as `s`) |
| the **knob** | wind, 0 to 10 m/s. It takes over only once you turn it |
| **LD4** (the green LED) | blinks slowly on the ground, fast in flight: the chip is alive |

The OLED stays dark. This firmware does not use it, because the display
code did not fit in the chip's flash alongside five drones; the station
draws the show instead.

### Commands (type in the station's command box, or Wokwi's serial monitor)

| Command | Does |
|---|---|
| `start`, `pause`, `go`, `abort` | as the keys above |
| `plan` | print the plan: every stage, its start and length, any repairs |
| `score` | print the last score again |
| `stats` | how fast the show runs compared with real time, the cost of one tick, each task's CPU and stack, serial-port counters |
| `wind MS [DEG]` | steady wind in m/s, blowing toward DEG degrees (0 = east, 90 = north) |
| `seed N` | a different set of gusts and GPS errors. The same seed always gives the same score |
| `gps rtk` / `gps plain` | the GPS receivers: RTK (about 2 cm, as show drones use) or plain (about 0.5 m, as lesson 14) |
| `ff on` / `ff off` | velocity feed-forward. Off sends positions only, and the drones trail their slots |
| `speed 0\|1\|2\|5\|10` | times real time. 0 = as fast as the chip can. In Wokwi it is already slower than 1 |
| `reset` | back on the pads |
| `quiet` / `loud` | stop / restart the telemetry frames |
| `help` | the list |

`seed`, `gps`, `ff` and `reset` work only on the ground. `wind` works any time.

### What the station shows

- **Coloured dots** are the drones, numbered, with a 3 s tail. A **dashed
  ring** is where a moving drone is heading.
- An **orange line** joins two drones closer than the planned 3 m; a **red
  line**, closer than the 1 m safety limit.
- **Under the 3D view, the drone table:** for every drone its mode, its
  position x (east), y (north), z (height) in metres, its error (its
  distance from where the plan says it should be), its battery as a bar,
  and the **local wind**: the air where that drone is, the steady wind
  plus its own gust, as an arrow (from above, north up) and a speed.
- **On the right:** the map from above; the **WIND** panel, with a compass
  whose thick arrow is the steady wind and whose thin coloured arrows are
  each drone's local wind, the speed, the direction it blows toward, and
  the range of gusts the drones feel now; the **CONTROL** panel, with the
  state (WARMUP, READY, FLYING, PAUSED, LANDING, DONE), the buttons above,
  and the closest pair now and ever.
- **At the end:** PASS or FAIL, the tracking error and the closest approach.
  The full score table is in the console.

**The score:** PASS means no two drones ever came closer than 1 m, every
drone landed, and nobody aborted. Below that come the tracking error (RMS
and worst), the formation error during holds, and the settle time of each
formation: how long until every drone was within 0.5 m of its slot.

## Make your own show

Edit **`show.c`**, then `build.bat` and restart the simulator. Each line of
the table is one formation:

```
{ SH_HEART, WALL, 8.0f,  0.0f, 10.0f, 13.0f, 14.0f,  0.0f, "heart", 0 },
  shape     plane hold   x     y      z      size   turn   name
```

| Field | Meaning |
|---|---|
| shape | `SH_LINE` `SH_V` `SH_CIRCLE` `SH_CROSS` `SH_GRID` `SH_HEART`, or `SH_POINTS` with your own list of points (`show_w` in show.c is an example) |
| plane | `WALL` stands up facing the audience; `FLAT` lies flat, seen from above |
| hold | seconds to hold it |
| x y z | its centre, metres: x east, y north, z up. The pads are along x at y = 0 |
| size | metres across |
| turn | degrees, about the vertical |
| name | shown in the station (no spaces) |

Above the table are the show's rules: the number of drones (at most 5 on
the chip), the pad spacing, the take-off height, the speed limits (`vmax`,
`vzmax`, `amax`), `sep` (the planned minimum distance), `safe` (the flown
minimum), `settle`, the wind and the GPS. Up to 10 formations.

Every slot must be at least 2 m up and at least `sep` from every other slot.
If a show is refused, the console says which formation and which drones:
make that formation bigger.

## Without the board (host/)

| | |
|---|---|
| `host\test.bat` | builds the show and the swarm for the PC and runs every check (31 + 21) |
| `host\out\test_show --print` | flies `show.c`'s show on the PC in under a second and prints what the chip would print |
| `host\out\test_show --frames` | the same, but prints every frame: a capture you can play into the station without Wokwi (see below) |
| `uv run host\show.py host\shows\parade.show` | the PC version: Python planner, up to 64 drones (`host\shows\big.show` has 25). Options: `--seed N`, `--wind MS DEG`, `--gps rtk\|plain`, `--no-feedforward`, `--twice`, `--log FILE`, `--serve PORT` |
| `uv run host\planner.py <file.show>` | check a show without flying it |

To watch a capture in the station with no simulator, from the repository root:

```
projects2026_arm\19_Final_Project\host\out\test_show --frames > show.txt
uv run tools/station/fakeboard.py --replay show.txt --rate 100
uv run tools/station/station.py --port socket://localhost:4001 --view swarm
```

The `.show` files in `host\shows\` use the same rules as `show.c`, written
as text: `drones`, `pads`, `takeoff`, `vmax`, `vzmax`, `amax`, `sep`, `safe`,
`settle`, `wind MS DEG`, `gps`, then one `form <shape> <hold> key=value ...`
per formation (keys `x y z size turn plane=wall|flat name`), or
`points <hold> x,y,z x,y,z ...`.
