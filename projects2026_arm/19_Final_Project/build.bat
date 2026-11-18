@echo off
REM ============================================================================
REM  19_Final_Project - Swarm parade - build
REM
REM  Every *.c in this folder is compiled (host\ is not: that is the PC side):
REM    Main.c      the hardware half: three RTOS tasks, the shell
REM    director.c  runs the show: the clock, the targets, the score, the frames
REM    plan.c      plans the show and proves it safe, at power-up
REM    show.c      THE SHOW - edit it to make your own
REM    fleet.c     N drones on one clock
REM    pilot.c     one drone's flight computer      craft.c  one drone's airframe
REM    fmt.c       a small snprintf     trig.c  sin/cos/sqrt, compiled once
REM  From _lib\:
REM    retarget  the C library stubs (its weak polled _write is replaced by uart's)
REM    os        the kernel         uart proto   interrupt UART, $frames
REM    adc pad   buttons, joystick, knob
REM  Not used, to fit 32 KB of flash (Main.c says how): i2c oled mirror
REM  (the OLED stays dark; the station draws the show), keys kbd.
REM
REM  Then: Wokwi (F1 - "Wokwi: Select Config File" - this folder's wokwi.toml,
REM  then "Wokwi: Start Simulator"), then run.bat for the station.
REM ============================================================================
set TARGET=c031c6
set LIBS=retarget os uart proto adc pad
call "%~dp0..\_build\build-lesson.bat"
