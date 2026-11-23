@echo off
REM ============================================================================
REM  19_Final_Project\host - the swarm on the PC: build and test
REM
REM  Needs gcc on PATH (MinGW-w64) and uv.  Exit code 0 = every check passed.
REM
REM    out\test_show.exe   the firmware's show, flown on the PC: the SAME
REM                        plan.c, director.c, fleet.c, pilot.c, craft.c,
REM                        show.c and fmt.c the chip runs (32 checks)
REM    out\swarm.dll       the firmware's fleet for Python, 64 drones:
REM                        show.py, planner.py (test_swarm.py: 21 checks)
REM
REM  No stm32c031xx.h anywhere: if one of those files ever includes it, this
REM  build fails - which is the point.
REM ============================================================================
setlocal
cd /d "%~dp0"
if not exist "%~dp0out" mkdir "%~dp0out"
set "SIM=..\fleet.c ..\pilot.c ..\craft.c ..\trig.c"

echo == build: the firmware's show, for the PC ==
gcc -std=c11 -O2 -Wall -Wextra -Wno-format-truncation -I.. -I..\..\_lib -o "%~dp0out\test_show.exe" test_show.c ..\plan.c ..\director.c ..\show.c ..\fmt.c ..\..\_lib\proto.c %SIM% -lm
if errorlevel 1 exit /b 1
echo == build: swarm.dll, 64 drones ==
gcc -std=c11 -O2 -Wall -Wextra -Werror -shared -static-libgcc -DFLEET_MAX=64 -I.. -I..\..\_lib -o "%~dp0out\swarm.dll" swarm.c %SIM%
if errorlevel 1 exit /b 1

echo.
echo == the firmware's show ==
"%~dp0out\test_show.exe"
if errorlevel 1 exit /b 1

echo.
echo == the PC tools ==
uv run "%~dp0test_swarm.py"
exit /b %errorlevel%
