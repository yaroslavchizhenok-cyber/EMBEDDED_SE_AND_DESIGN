# 19 — Swarm parade

**Part 3, the last lesson.**
Target: ST Nucleo-C031C6 (STM32C031C6, Cortex-M0+ at 48 MHz) on the app board.

Five simulated drones fly a light show **inside the chip**: take off, a
line, a V, a circle, a heart, a cross, an arrow, a wide circle, home, land.
Each drone has its own flight computer, reworked from lesson 14's, with its
own gusts and its own GPS error.

At power-up the chip plans the whole show and proves that no two drones are
ever planned closer than 3 m, or refuses the show. In flight it keeps every
drone on one clock and scores the run: tracking error, formation error,
settle time, closest approach, PASS or FAIL. The station draws it all in 3D.

**[INSTRUCTIONS.md](INSTRUCTIONS.md) lists every control.**

## Run it

```
build.bat                  # FLASH 32 496 B / 32 KB (99.2 %), RAM 10 608 B / 12 KB, zero warnings
```

Then in VS Code: F1, **Wokwi: Select Config File**, `wokwi.toml`; F1,
**Wokwi: Start Simulator**. Then:

```
run.bat                    # the station, on the swarm view
```

Click the picture and press `a`. **In Wokwi the show runs in slow motion.**
Five drones are more work than Wokwi emulates in real time, so the firmware
lets its show clock fall behind instead of skipping work (see "Three tasks"
below). `stats` says how fast it is running.

```
host\test.bat              # the same show on the PC: 31 + 21 checks, exit code 0
```

## How it works

```
 show.c ──► plan.c ──► a timed track for every drone   (at power-up; refused if
                │        checked: no pair ever < sep    any pair would be < sep)
                ▼
          director.c ── every 10 ms: fleet_step()   (all drones, one tick)
                │        every 100 ms: each drone's target = the plan at t
                │                      (position AND velocity)
                ▼
            fleet.c ── 5 x [ craft.c (airframe, air, sensors) + pilot.c (flight computer) ]
                │
                ▼
     $SWM $DRN $SEP $SCO ──► USART2 ──► run.bat: tools/station, views/l19_swarm.py
```

This is how real drone light shows work: no swarm intelligence, no drone
talking to another. Every drone follows its own timed track, one ground
station keeps everyone on one clock, and the spacing is checked before
take-off. Here the ground station is the chip itself.

**The pilot follows a moving setpoint.** Lesson 14's drone flies "to the
waypoint at cruise speed", so its arrival time depends on the wind. A
formation needs every drone at its slot at a given moment, so `pilot.c` is
handed a goal position *and the goal's velocity*:

```
want_v = goal_v + k_pos * (goal - estimate)      (the correction capped at 2.5 m/s)
         ^^^^^^ feed-forward                     ^^^ feedback
```

Without feed-forward a drone trails its slot by `goal_v / k_pos`: `ff off`
shows it. If no target arrives for 1 s, the pilot hovers.

**The planner** (`plan.c`) handles each change of formation in three steps:

1. **Who goes where.** The Hungarian algorithm picks the assignment with the
   smallest sum of *squared* distances. Turpin, Michael and Kumar (2014,
   "CAPT") showed that with this choice, straight paths flown in step do not
   cross if both formations are spaced about 1.4 × `sep`.
2. **How fast.** One trapezoidal speed profile, sized for the longest path,
   is shared by every drone and scaled to its own distance, so all of them
   arrive together. `vmax`, `vzmax` and `amax` hold for every drone.
3. **Check and repair.** Every pair is checked every 0.2 s. A pair planned
   closer than `sep` is repaired by flying one drone over or under the other.
   If no repair works, the show is refused.

The plan is small: the drones' positions at each formation, 13 × 5 × 12 B.
`plan_sample()` works out any moment from them.

### Three tasks

| task | prio | runs | does |
|---|---|---|---|
| `link` | 3 | per byte | the shell, the station's `$KEY` presses |
| `ui` | 2 | 20 ms | buttons, joystick press, knob → commands; LD4 |
| `fly` | 1 | 10 ms | `director_tick()`: the fleet, the score, the frames |

`fly` is the **lowest** priority on purpose. In Wokwi a tick of five drones
takes longer than 10 ms, so `fly` never sleeps. A higher-priority task that
never sleeps starves everything below it; that is how lesson 13 froze. At
the bottom it uses only the time the others leave: the show slows down and
the controls keep working. On silicon `fly` sleeps until each 10 ms is due.
One lock, `dir_lock`, guards the director; `print_lock` keeps lines whole,
and is always taken second.

**A station that connects late still gets the picture.** While nothing
moves (READY on the pads, paused, DONE, REFUSED) the show itself sends
nothing, and `run.bat` usually starts after the simulator. So `fly` calls
`director_heartbeat()` every 100 ms when idle, and the board resends its
picture one line at a time, a whole set every second (a refusal: its text
every 5 s). The first version lacked this, and a late station waited forever.

**Frames are spread one per tick:** `$SWM` on tick 0, one `$DRN` per drone
on ticks 1–5, `$SEP` on tick 6. All seven at once are about 390 B, more
than `uart.c`'s 256 B transmit ring. The last drones' lines would never fit,
and the view would never see them.

## Files

| | |
|---|---|
| `INSTRUCTIONS.md` | every control, how to start, how to write a show |
| `show.c`, `show.h` | **the show**: its rules and its formations. Edit `show.c` to make your own |
| `plan.c`, `plan.h` | the planner: shapes, Hungarian assignment, profiles, check, repair, refusal |
| `director.c`, `director.h` | runs the show: states, clock, targets, score, frames, the report |
| `fleet.c`, `fleet.h` | N drones on one clock (`FLEET_MAX` 5 on the chip, 64 on the PC) |
| `pilot.c`, `pilot.h` | one drone's flight computer: **every function takes a `pilot_t *`** |
| `craft.c`, `craft.h` | one drone's airframe, its air and sensors. `sky_t` is the steady wind every drone shares, gusts are per drone, `gps_grade_t` selects RTK or plain GPS |
| `contract.h` | what an airframe and its pilot exchange: `meas_t` in, `cmd_t` out |
| `fmt.c`, `fmt.h` | a small `snprintf`: integers and strings, 2 KB less than newlib's |
| `trig.c`, `trig.h` | `fmath.h`'s sin/cos/sqrt/atan2, compiled once |
| `Main.c` | the hardware half: three tasks, the shell, the keys, LD4 |
| `build.bat` | `LIBS=retarget os uart proto adc pad` |
| `wokwi.toml`, `diagram.json` | the app board, unchanged; serial port forwarded to TCP 4000 for the station |
| `run.bat` | the station, `--view swarm` |
| `simulate.bat` | rebuilds, then prints the Wokwi steps |
| `host/test_show.c` | the firmware's show flown on the PC: `--print` shows the chip's output, `--frames` a capture for `fakeboard.py --replay` |
| `host/swarm.c` | `fleet.c` as a DLL, 64 drones, for Python |
| `host/fleet.py`, `planner.py`, `show.py`, `test_swarm.py`, `shows/` | the PC version: Python planner, 25-drone shows, fast scoring, `--serve` to the station |
| `host/test.bat`, `test.sh` | build both, run both test sets |
| `recom.txt` | the brief this grew from. Its verdict was "run the swarm on the PC"; the lesson now runs it on the chip, for 5 drones, at the cost of the OLED (below) |
| `tools/station/views/l19_swarm.py` | the station view: the show in 3D, the drone table (position, error, battery, local wind) and the WIND / CONTROL menu, whose buttons send shell commands |

Everything except `Main.c` is pure C, with no chip header. `host/test.bat`
compiles it with the PC's gcc, and would fail if any of it included one.

## Fitting in 32 KB

The first build was **43 192 B of flash** for a 32 KB chip, and 13 824 B of
RAM for 12 KB. What it took:

| cut | flash saved |
|---|---|
| no `strtok`/`strtoul`/`atoi`: newlib's `strtok` calls `assert`, which prints through all of stdio | ~3.5 KB |
| the station's `$KEY` presses read in `Main.c`, not via `_lib`'s `kbd.c` + `keys.c` | ~1.4 KB |
| `fmt.c` instead of `vsnprintf` (and the `malloc` it drags in) | ~2 KB |
| **no OLED**: `oled`, `i2c`, the picture and the station mirror | ~4 KB |
| sin/cos/sqrt compiled once (`trig.c`) | ~0.7 KB |
| RAM: 5 drones not 6, a sensor reading not kept per drone, repair heights as bytes, a 12-step (not 16) estimator history | ~1.5 KB of RAM |

**The OLED stays dark.** It was the largest single cut and the least
missed: the station draws the show far better than 128 × 64 pixels could.
LD4 blinks instead, slowly on the pads and fast in flight. The headroom left
is about 270 B of flash. Editing `show.c`'s table costs none; new code does.

Task stacks were sized from `-fstack-usage` along the deepest call chains,
not measured running. `fly`'s worst is about 990 B of its 1408 B. `stats`
prints each task's real high-water mark: check it the first time it runs.

## Rewritten from lesson 14, not copied

The physics and the estimator are lesson 14's model with the same
constants, so a drone of the swarm flies like lesson 14's drone. The code
is rewritten around a different structure, and the names say so:

| lesson 14 | here | what changed |
|---|---|---|
| `world_t`, `world.c` | `craft_t`, `craft.c` | the steady wind is a shared `sky_t`. Each drone keeps only its own gusts, and has its own IMU bias from its seed |
| xorshift32 `rnd()` + 3 uniforms | LCG `roll_dice()` + 4 uniforms | a different generator. Each drone gets its own stream via a MurmurHash3-style `mix(seed, i)` |
| `static fc_t fc` + `fc_get()` | `pilot_t *p`, one per drone | no state in the file; the pilot is the argument |
| `control_step()` | `pilot_step(p, m, out)` | stages: believe / guard / aim / steer |
| TAKEOFF HOLD MISSION RTL LAND | IDLE FOLLOW HOVER LAND | no missions, fence or `$PARAM`; a show drone does not decide where to go |
| `hold_xy()`, `leg()` | `aim()` | feed-forward plus a capped correction, in all three axes |

Lesson 14 itself is untouched.

## Measured on the host

`host\out\test_show.exe` runs the show from `show.c`: 5 drones, 3 m/s wind
toward 30°, RTK GPS, seed 1. 2026-11-17. The Python version
(`host\show.py`) gives the same figures.

| run | track RMS | track max | closest pass | |
|---|---|---|---|---|
| as written | 0.20 m | 0.58 m | 3.61 m | PASS |
| `gps plain` (lesson 14's receiver) | 0.73 m | | | PASS |
| `ff off` | | **5.42 m** | | PASS |
| `seed 2` | 0.19 m | | | PASS |
| `abort` 20 s in | | | | FAIL, all 5 landed in 15 s |

- **GPS is the limit, not the control loop.** With plain GPS each pilot holds
  its *estimate* on the slot, and the estimate is about 0.6 m off. That is
  why real show drones carry RTK GPS.
- **Feed-forward is what keeps a formation.** Without it the drones trail
  their slots by metres in every move.
- The show is 125.5 s, planned with no repairs. A row turning into a column
  across it needs 9 repairs, and the repaired plan never comes closer than
  3.00 m.
- The same seed always gives the same numbers, to the last bit.

## Status

- **Built:** zero warnings, FLASH 32 496 B, RAM 10 608 B. All host checks pass.
- **Power-up is the slow part in Wokwi.** Checking every pair through the
  whole show is millions of soft-float operations: about a second on a real
  chip, many times that in Wokwi, which runs float-heavy code at least 6x
  slower (lesson 13 measured it). The first version also checked the whole
  show twice, every 0.1 s, and said nothing while it did, so the station
  showed a black screen for what could be most of a minute. Now: one pass,
  every 0.2 s (5x less work; `host/test_show` re-checks every plan at 20 ms),
  a line per checked move (`planning: 3 of 8 moves checked`), and a 1.3 s
  warm-up instead of 2 s.
- **The station view** was checked by rendering it off-screen from a
  capture of the firmware's own frames (`test_show --frames`). `run.bat`
  starts the station with `--view swarm` on Wokwi's port.
- **Not yet watched running in Wokwi.** Nothing here could run the
  simulator. Things to look at the first time:
  - `stats`: each task's stack high-water mark, and how fast the show runs
    against real time;
  - that the `$DRN` lines reach the station for all five drones;
  - that holding `s` pauses once and does not toggle.
