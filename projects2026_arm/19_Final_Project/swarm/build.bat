@echo off
REM ============================================================================
REM  19_Final_Project\swarm - ONE drone of the swarm, for one chip - build
REM
REM  Every board of the swarm flies this same Main.hex; its ID straps in its
REM  own diagram.json (drone1\ ... drone5\) say which drone it is.
REM
REM    Main.c     the hardware half: ui and fly tasks; the straps; LD4
REM    link.c     the radio link: USART2, whole lines assembled by the interrupt
REM               (in place of the shared uart.c)
REM    agent.c    the drone: flies, beacons, listens, keeps its table
REM    nbr.c      its neighbour table      radio.c  the beacon, to a line and back
REM    shared_*.c the show's own craft.c, pilot.c, trig.c, fmt.c and softfloat.c,
REM               included from ..\ so there is one copy of each
REM
REM  Then open drone1\ (and drone2\, drone3\ ...) each in its own VS Code
REM  window, F1 - "Wokwi: Start Simulator" in each, then run.bat here.
REM ============================================================================
setlocal
cd /d "%~dp0"
set TARGET=c031c6
set LIBS=retarget os proto adc pad
call "%~dp0..\..\_build\build-lesson.bat"
