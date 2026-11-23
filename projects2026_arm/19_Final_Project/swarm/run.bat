@echo off
REM ============================================================================
REM  19_Final_Project\swarm - the swarm station: the air between the drones,
REM  and the operator's screen
REM
REM    1. build.bat
REM    2. open drone1\ in its own VS Code window, F1 - "Wokwi: Start Simulator";
REM       the same for drone2\, drone3\ ... (as many as the laptop carries)
REM    3. run.bat   <- this.  Drones are picked up as their windows start.
REM
REM  Extra arguments pass through:  run.bat --ports 4001-4003 --range 8 --loss 10
REM  Needs uv (the repository's Python environment: uv sync, once).
REM ============================================================================
setlocal
set "ROOT=%~dp0..\..\.."
where uv >nul 2>nul
if errorlevel 1 (
    echo uv was not found.  Install it once, then run "uv sync" at the repository root.
    exit /b 1
)
uv run --project "%ROOT%" python "%ROOT%\tools\station\swarm_station.py" %*
exit /b %errorlevel%
