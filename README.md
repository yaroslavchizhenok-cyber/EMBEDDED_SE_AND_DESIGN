# Swarm parade: five drones, one clock

Five simulated drones fly a light show **inside an STM32C031C6**
(Cortex-M0+, 48 MHz, 32 KB flash, 12 KB RAM): take off, a line, a V, a
circle, a heart, a cross, an arrow, a wide circle, home, land. Each drone
has its own flight computer, gusts and GPS error.

At power-up the chip plans the whole show and proves that no two drones are
ever planned closer than 3 m, or refuses the show. In flight it keeps every
drone on one clock and scores the run. A ground station on the PC draws it
in 3D, with the drones' numbers and a menu for the wind and the show.

It runs in the **Wokwi** simulator on the Nucleo-C031C6 with the SOC3050
"app board" (buttons, joystick, knob). No hardware is needed.

## Start here

- **[projects2026_arm/19_Final_Project/INSTRUCTIONS.md](projects2026_arm/19_Final_Project/INSTRUCTIONS.md)**:
  every control, how to start, how to write your own show.
- **[projects2026_arm/19_Final_Project/README.md](projects2026_arm/19_Final_Project/README.md)**:
  how it works, the files, what it took to fit in 32 KB, measured results.
- **[projects2026_arm/19_Final_Project/Slide.md](projects2026_arm/19_Final_Project/Slide.md)**:
  the lecture deck.

## Set up (once, on Windows)

1. **The ARM toolchain** (not in this repository: about 230 MB).
   Download xPack `arm-none-eabi-gcc` **15.2.1-1.1**, `win32-x64` zip, from
   <https://github.com/xpack-dev-tools/arm-none-eabi-gcc-xpack/releases>,
   unpack it, and rename the folder so that this file exists:
   `tools/arm-toolchain/bin/arm-none-eabi-gcc.exe`.
   `.gitignore` keeps it out of git.
2. **uv** for the Python side: <https://docs.astral.sh/uv/>, then in this
   folder `uv sync` (uv fetches Python 3.12 with tkinter, and pyserial).
3. **VS Code** with the **Wokwi Simulator** extension.
4. *(Optional, for the PC tests in `host/`)* gcc for Windows (MinGW-w64) on PATH.

## Run

```
cd projects2026_arm\19_Final_Project
build.bat
```

In VS Code: F1, **Wokwi: Select Config File**, choose
`projects2026_arm/19_Final_Project/wokwi.toml`. Then F1, **Wokwi: Start
Simulator**. Then:

```
run.bat
```

The station opens. Wait for the five drones on their pads, then press
**START** (or click the picture and press `a`). In Wokwi the show runs in
slow motion; that is expected.

## What is in this repository

```
projects2026_arm/
  19_Final_Project/      the project: firmware (Main.c + pure-C modules), host/ tests and PC tools, docs
  _build/                build-lesson.bat - the build engine build.bat calls
  _startup/c031c6/       startup code and linker script for the STM32C031C6
  _targets/c031c6.bat    compiler flags for that chip
  _lib/                  the shared drivers the project links: os (RTOS), uart, proto, adc, pad, retarget
tools/
  cmsis/                 ARM CMSIS and ST's STM32C0 device headers
  station/               the PC ground station (station.py) and its views; fakeboard.py replays captures
  arm-toolchain/         (you unpack it here - see "Set up")
```

The folder layout matches the SOC3050 course repository this project was
built in, so the build scripts' relative paths work unchanged. The build
engine, the shared drivers (`_lib`), the startup code and the station come
from that course framework; the swarm parade, everything in
`19_Final_Project/` and the station's `l19_swarm.py` view, is this project.
