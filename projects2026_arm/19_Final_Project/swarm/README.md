# The swarm — one drone per chip

The swarm parade one folder up is a light show. One chip plans every drone's
track and keeps them all on one clock. That is how real shows work, and it
is not a swarm.

This folder is a swarm. **Every drone is its own STM32C031 in its own Wokwi
window, running the same firmware.** No chip, program or person flies a drone,
assigns it work or keeps the swarm's clock. The operator posts a *goal*
("survey this area, take readings at these points"). The drones work out the
rest among themselves, by radio, from what each one hears.

| property | how it is met | tested by (`host\test.bat`) |
|---|---|---|
| **Decentralised control** | Each chip decides everything in `agent.c`, from its own sensors and the beacons it hears. There is no plan, no leader and no member list anywhere. | every scenario; `test_air.py` checks that the station only ever carries radio frames |
| **Peer-to-peer** | Broadcast beacons 5×/s. The mission is flooded drone to drone. Tasks and the map spread by gossip. Hop counts back to the ground radio are stamped with sequence numbers, as in the DSDV routing protocol. | *neighbours*, *multi-hop* |
| **Emergent behaviour** | Separation, look-ahead avoidance, goal, cohesion and alignment give a flock with no shape written anywhere. Coverage happens because each mapper avoids the cells its neighbours are heading for. | *avoid*, *flock*, *survey* |
| **Fault tolerance** | A silent drone is dropped after 3 s. A dead winner's task claim goes stale after 4 s and is auctioned again. Lost link or no route home: the drone flies back toward home, and resumes if it hears the swarm again. | *faults*, *lostlink*, *loss*, *clocks* |
| **Scalability** | 8 neighbour rows, at most 64 cells, at most 4 tasks: **1,520 B per drone at any swarm size**, and about 1 KB/s of radio per drone. | *scale*: 40 drones, 9 hops |
| **Dynamic task allocation** | An auction with no auctioneer: the lowest bid wins, and the bid is distance plus a battery penalty. A drone on low battery gives its task up, flies home, recharges and rejoins. | *battery*, *rejoin*, *faults* |
| **Heterogeneity** | Each board's straps make it a mapper (camera), a sensor or a relay (double-range radio). Only sensors bid for sensing tasks. Relays line up between the ground radio and the work. | *survey*, *hetero*, *relay* |

## Run it

```
build.bat                  # one Main.hex for every drone: FLASH 29.9 KB / 32 KB, RAM 5.2 KB / 12 KB, no warnings
```

Open **`drone1\`** as a folder in its own VS Code window and run
F1 → **Wokwi: Start Simulator**. Do the same for `drone2\` … `drone7\`, as
many as the laptop carries. Then start the station:

```
run.bat                    # the air, the ground radio and the screen
```

Press **`s`** on the map. The drones take off by themselves, survey the
dashed area, take the three sensing readings, fly home and land.

| drone | type | lane | Wokwi port |
|---|---|---|---|
| `drone1` | mapper | 4 m | 4001 |
| `drone2` | mapper | 5.5 m | 4002 |
| `drone3` | sensor | 7 m | 4003 |
| `drone4` | relay | 4 m | 4004 |
| `drone5` | sensor | 5.5 m | 4005 |
| `drone6` | mapper | 7 m | 4006 |
| `drone7` | sensor | 4 m | 4007 |

**Station** (click the map first):

| key | action |
|---|---|
| `s` | survey the area: right-drag to redraw it, shift-click to add a sensing point, `c` to clear the points |
| `g` | go to the mouse (flocking) |
| `f` | **follow** the mouse: on/off (see below) |
| `x` | hold |
| `h` | home |
| `1`–`7` or click | select a drone: the map then shows *its* copy of the coverage map and task auction |
| `k` | cut the selected drone's radio |
| sliders | radio range and packet loss |
| command box | the selected drone's shell: `nbr`, `msn`, `kill`, `bat 25`, `stats`, `help` |

**Follow the mouse (`f`).** While follow is on, the station posts a new GOTO
mission at the pointer each time the pointer has moved 1 m, at most every
0.4 s. Each goal is an ordinary mission, flooded drone to drone, so the
firmware needs no change for this.
- The map stops rescaling while following, and the crosshair marks the
  current goal.
- The drones' `mission N: GOTO` messages are hidden from the console.
- Any other mission key turns follow off.
- In a test with five stand-ins chasing a pointer moving at 1.5 m/s around a
  10 m circle, the swarm trailed about 6.5 m behind (up to 12 m on the
  curve), and the closest two drones came was 1.6 m.

A drone treats a GOTO that only moves the goal as the same job, so the new
goals don't restart its staggered take-off (ID × 0.3 s after a mission
appears). Every drone launches on time, and one that lands during follow
(battery) rejoins by itself once recharged.

**A goal outside the radio zone.** The goal crosshair turns red. A swarm
can't hear goals beyond the ground radio's reach. Before this fix, it flew
to the last goal it heard, stopped there out of touch, and never heard the
goal come back. Now each drone learns the edge for itself:
- If the ground beacon numbers stop rising for 3 s, the drone is out of
  touch. It flies back toward the ground radio until it hears it again.
- Where it gets back in touch, less 1 m, becomes its edge. A goal beyond the
  edge is held *at* the edge, so the swarm waits there, in touch, and
  follows the goal the moment it comes back.
- It probes the edge 1 m further at intervals: every 2 s while probes
  succeed (relays extending the network are followed), up to every 32 s
  after failed ones (a goal that is simply out of reach).

The first time out, the swarm overshoots once, to about 23 m on a 15 m
radio, before it learns the edge.

**Landing by hand is an order.** A drone landed with B, `down` or the
station's `b` stays down. It won't rejoin the mission when charged, and a
follow goal that only moves doesn't wake it. It takes off again when you
take it off by hand, or post a new mission. `msn` shows `parked`.

**Each drone's Wokwi window:**

| control | action |
|---|---|
| **A** | take off (when no mission is on) |
| **B** | land |
| joystick | move it by hand |
| joystick press | crash it |
| knob | wind |
| LD4 | blinks once per neighbour |

**Things to try:**
- Crash a sensor mid-task (`kill`, or press its joystick). Its task is
  auctioned again, and another sensor finishes it.
- Type `bat 25` to a working drone. It gives its task up, lands, recharges
  and comes back.
- Cut one radio with `k`.
- Slide the loss up to 30%.

## How it works

```
 Wokwi #1 ─ rfc2217:4001 ─┐         $BCN $MSN $TSK $MAP   radio: carried only to drones in range
 Wokwi #2 ─ rfc2217:4002 ─┼─ swarm_station.py  ($GND $MSN   the ground radio, near the pads; $GND 5x a second)
     ...                  │         $PHY                  the world's truth: only the air reads it
 Wokwi #7 ─ rfc2217:4007 ─┘         $NBT                  what a drone believes: only the screen reads it
```

**Each drone runs the same loop**, 100 times a second (`agent_tick`):

- **Fly.** Its own airframe and pilot (`../craft.c`, `../pilot.c`) run.
  Every 50 ms the local rules (`steer.c`) produce a velocity from its goal and
  its neighbours' beacons. Safety comes first: separation and avoidance take
  the speed budget first, and the goal gets what is left. A predicted conflict
  also makes it brake. Each drone cruises in its own altitude lane (4, 5.5
  or 7 m), so most crossings are 1.5 m apart vertically. It lands or takes
  off only when the column above its pad is clear.
- **Think,** 4 times a second. If the battery is low, it gives up its task and
  goes home. If it has heard nobody for 15 s, or had no route to the ground
  radio for 60 s, it heads home, and turns back to work if it gets back in
  touch. The second no-route trip in one mission means the goal is out of
  radio reach, so it stops trying. Otherwise it does its job:
  - **mapper:** the nearest uncovered cell that a nearer mapper is not
    already heading for (`cover.c`);
  - **sensor:** bids in the auction (`tasks.c`), flies to the point it won and
    hovers there for 5 s;
  - **relay:** moves out along the line from the ground radio to the work, as
    far as its link back allows (`relay_spot`).

  When its own copy of the map is full and its copy of the tasks all done, it
  flies home and lands.
- **Talk.** It sends a beacon 5 times a second, relays the mission every 2 s,
  and gossips its task table and coverage map once a second. It also sends
  its task table at once whenever it changes.

### The link: from the chip's UART to the station

The first Wokwi run showed weak links even between drones 3 m apart, and a
frame that arrived without its `$`. Bytes were being lost on the way into
the chips. Two changes fix that, one at each end of the wire.

**On the chip, `link.c` replaces the course's shared `uart.c`.**
- **Before:** the interrupt passed each byte through a 64-byte kernel queue
  to a `link` task, which meant a task switch per byte. A burst of neighbour
  frames overflowed the queue mid-frame.
- **Now:** the interrupt assembles whole lines in a 512-byte ring. A line
  becomes readable only when its `\n` arrives, and a line that doesn't fit
  is dropped whole and counted, never cut in half.
- **Radio frames are never skipped.** A skipped beacon would look to every
  neighbour like a lost one. Telemetry (`$PHY` at 5 Hz, `$NBT`, `$HLT`)
  goes out only when there's room to spare.
- **Damaged frames are counted silently.** A frame that lost its `$` no
  longer comes back as `? key,right,...`.

**On the station, each drone gets its own queue, merging and credit**
(`DroneLink` in `swarm_station.py`):
- A newer frame from the same source replaces an older one still waiting.
  A slow drone gets fewer frames, but each one whole and fresh.
- **Credit:** ten times a second the drone reports how many bytes it has
  taken in (`$HLT`). The station never has more than 384 bytes on their
  way to a drone that it hasn't taken in yet. That is less than the
  512-byte ring, so a slow Wokwi window cannot overflow it. This is TCP's
  window, used for the same reason.
- The table's **link** column shows each drone's rate and frames lost per
  second. `run.bat --no-pace` turns pacing off, for comparison.

Measured with five stand-ins that act like slow chips (1,000 B/s,
`host\test_air.py`):

| per drone, over 20 s | unpaced | paced with credit |
|---|---|---|
| whole frames heard /s | 16.3 | 20.1 |
| frames lost or broken /s | 2.1 | 0.0 |
| link quality | 84% | 98% |

**RAM went from 7.8 KB to 5.1 KB** (-34%):

| item | saved |
|---|---|
| no `link` task: its stack and the 64-byte queue | 704 B |
| 8 × 120-byte inbox replaced by one 512-byte ring | 448 B |
| task stacks resized to measured use (fly 1,280 → 1,024 B, ui 768 → 384 B) | 640 B |
| heap reserve (nothing calls `malloc`) and main stack (1,024 → 768 B), in a swarm-only `link.ld` | 768 B |
| smaller neighbour rows | 64 B |

Worst-case stack use, measured with `-fstack-usage`: fly about 640 of
1,024 B, ui about 110 of 384 B, main about 400 of 768 B.

A neighbour is now dropped after **3 s** of silence (was 2 s), so one at
the fading edge of range no longer flaps in and out of the table.

**Why it holds together without a coordinator:**
- **The map can only grow.** Bits are only ever set, so copies merge by OR in
  any order, and every copy ends up the same.
- **Every drone applies the same auction rule** (lowest bid wins) to the same
  claims, so the copies agree once the gossip has spread. Only the *age* of a
  claim travels, never a time, because clocks are not synchronised.
- **A route has to be fresh.** The ground radio's beacon number travels with
  each hop count, and only a route whose number keeps rising is believed. A
  group cut off from home therefore cannot keep a fake route alive by quoting
  each other.

## Test results

`host\test.bat` runs 16 scenarios under 10 random seeds, 790 checks, all
passing. It then runs 24 checks on the station's own air code, all
passing: a real-time survey, and the slow-chip comparison above. The
scenarios also passed under 20 seeds.

| scenario | result (seed 1) |
|---|---|
| protocol | 200,000 mangled frames: no crash |
| avoid | head-on swap passed 2.4 m apart; 8 drones swapping across a circle came no closer than 2.3 m, with no deadlock (9 s) |
| flock | 6 drones gathered within 3.3 m of the point; closest 1.8 m |
| flock, out of reach | 44 m out with no relay, they came home by themselves after 60 s without a route |
| multi-hop | mission reached all 7 drones in 0.33 s, though only 3 hear the ground radio; a broken chain is detected in about 4 s and heals |
| survey | 7 drones, 16 cells, 3 tasks done once each, all home on their own pads in 52 s |
| faults | a mapper and a working sensor crash: the task is re-auctioned and the mission completes |
| battery, rejoin | low-battery hand-over, then recharge and rejoin by itself |
| loss, clocks | completes under 30% packet loss, and with chips running at 100 / 85 / 70% speed |
| relay | a point 39 m out (radio 12 m): with no relay the workers wait at the edge; with two, the relays settle about 14 m and 34 m out and the workers spend 65–69% of 4 minutes at the point, linked home 100% of that time |
| follow | a new GOTO every 0.4 s: all six airborne in 4.3 s, trailing the goal by 5.8 m; a drone whose battery fails goes home, recharges and rejoins by itself |
| outofreach | the follow goal dragged 50 m out (radio 15 m), then back: the swarm waits at the edge (farthest about 23 m), in touch 96–100% of the time, and is at the returned goal in 5–7 s; a drone landed by hand stays down through 60 s of follow goals and takes off for a new mission |
| scale | 40 drones: survey done in 84 s; at most 8 table rows; about 960 B/s radio per drone; closest 1.7 m |

**Closest any two airborne drones came, across all scenarios and 20 seeds:
1.47 m.**

**Limits, plainly:**
- **Duplicate work during a partition.** Under a radio partition, two drones
  can both do the same task before they hear of each other. Gossiping on
  every change made this rare, but it is the known price of agreeing without
  a coordinator.
- **The relay rule is a heuristic.** It holds a two-relay chain linked 78–93%
  of the time in the test geometry, not 100%.
- **The swarm needs a sensor.** With no sensor in the swarm, sensing tasks
  stay undone. The mappers finish their part and go home.

## Not yet checked in Wokwi

Everything above was tested on the PC, running the chips' own C files. The
firmware builds for the chip, but three things only a Wokwi run will show:
- whether the new link fixes the losses there. Each drone's `stats` command
  and the station's **link** column will show it;
- that the Nucleo-C031C6 part takes strap wires from PB0, PA7, PA15, PB1 and
  PB2 to GND;
- that `firmware = '../Main.hex'` works from a drone sub-folder;
- how fast each window really runs. The *clocks* scenario shows the swarm
  copes with 100 / 85 / 70% speed.

## Files

| | |
|---|---|
| `Main.c` | the chip: ui and fly tasks, straps, LD4, `$HLT` |
| `link.c`, `link.h` | the radio link: whole lines assembled by the interrupt, the transmit ring |
| `link.ld` | the shared linker script, with no heap reserve and a smaller main stack |
| `agent.c`, `agent.h` | one drone: flying, thinking, talking, its shell |
| `steer.c`, `steer.h` | the local rules: separation, look-ahead avoidance, goal, flocking |
| `nbr.c`, `nbr.h` | the neighbour table (8 rows, keeps the nearest) |
| `tasks.c`, `tasks.h` | the auction with no auctioneer |
| `cover.c`, `cover.h` | the survey map: a grow-only set |
| `radio.c`, `radio.h` | the frames |
| `shared_*.c` | the show's own `craft.c`, `pilot.c`, `trig.c`, `fmt.c` and `softfloat.c`, from `..\` |
| `drone1\` … `drone7\` | each drone's `wokwi.toml` (its port) and `diagram.json` (its straps) |
| `host\test_swarm.c` | the swarm on the PC: 16 scenarios (`-s SEED`, `-v`, `-c` close calls, `-t ID` trace) |
| `host\test_air.py` | the station's air with five `drone_pc.exe`, in real time; slow chips, paced and unpaced |
| `host\drone_pc.c` | a test stand-in for one Wokwi window. It can act like a slow chip. Not part of the swarm |
| `..\..\..\tools\station\swarm_station.py` | the station |
| `..\..\..\tools\station\radio_medium.py` | the air by itself |
