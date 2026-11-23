@echo off
REM ============================================================================
REM  19_Final_Project\swarm\host - the whole swarm on the PC: build and test
REM
REM  Needs gcc on PATH (MinGW-w64) and uv.  Exit code 0 = every check passed.
REM  Takes about two minutes (the last part runs in real time).
REM
REM    out\test_swarm.exe  the chip's agent.c, nbr.c, radio.c, steer.c, tasks.c
REM                        and cover.c - one per drone - over a model of the
REM                        station's air: 16 scenarios, 79 checks, run here
REM                        under 10 different random seeds (790 checks)
REM    out\drone_pc.exe    ONE drone behind a TCP port - a test stand-in for a
REM                        Wokwi window.  Not the swarm: the swarm is the chips.
REM    test_air.py         the station's own air and ground radio with five
REM                        drone_pc.exe on it, flying a survey in real time;
REM                        then five SLOW chips, the station unpaced and paced
REM ============================================================================
setlocal
cd /d "%~dp0"
if not exist out mkdir out
set "SRC=..\agent.c ..\nbr.c ..\radio.c ..\steer.c ..\tasks.c ..\cover.c ..\..\craft.c ..\..\pilot.c ..\..\trig.c ..\..\fmt.c ..\..\..\_lib\proto.c"
set "INC=-I.. -I..\.. -I..\..\..\_lib"

echo == build ==
gcc -std=c11 -O2 -Wall -Wextra -Wno-format-truncation %INC% -o out\test_swarm.exe test_swarm.c %SRC% -lm
if errorlevel 1 exit /b 1
gcc -std=c11 -O2 -Wall -Wextra -Wno-format-truncation %INC% -o out\drone_pc.exe drone_pc.c %SRC% -lm -lws2_32
if errorlevel 1 exit /b 1

echo.
echo == the swarm, simulated: 16 scenarios x 10 seeds ==
for %%S in (1 2 3 4 5 6 7 8 9 10) do (
    out\test_swarm.exe -s %%S > out\seed%%S.txt
    if errorlevel 1 (
        type out\seed%%S.txt | findstr /C:"FAIL" /C:"=="
        echo seed %%S FAILED - full output in out\seed%%S.txt
        exit /b 1
    )
    findstr /C:"checks passed" out\seed%%S.txt
)
echo (scenario output: out\seed1.txt ... out\seed10.txt;  out\test_swarm.exe -v NAME shows every drone's words)

echo.
echo == the station's air, with five drones on it ==
uv run --project "%~dp0..\..\..\.." python "%~dp0test_air.py"
exit /b %errorlevel%
