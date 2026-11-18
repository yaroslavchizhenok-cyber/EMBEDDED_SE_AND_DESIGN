@echo off
REM ============================================================================
REM  19_Final_Project - Swarm parade - the station
REM
REM  Run this AFTER the simulation is running in VS Code:
REM    1. build.bat
REM    2. F1 - "Wokwi: Select Config File" - this folder's wokwi.toml
REM       F1 - "Wokwi: Start Simulator"
REM    3. run.bat   <- this
REM
REM  It opens tools\station\station.py on the swarm view.  The station reaches
REM  the simulated board through wokwi.toml's rfc2217ServerPort = 4000, and
REM  reconnects by itself if the simulator is restarted.  Extra arguments pass
REM  through, e.g.  run.bat --port COM5  for a real board.
REM
REM  Needs uv (the repository's Python environment: uv sync, once).
REM ============================================================================
setlocal
set "ROOT=%~dp0..\.."
where uv >nul 2>nul
if errorlevel 1 (
    echo uv was not found.  Install it once, then run "uv sync" at the repository root:
    echo     powershell -ExecutionPolicy ByPass -c "irm https://astral.sh/uv/install.ps1 ^| iex"
    exit /b 1
)
uv run --project "%ROOT%" python "%ROOT%\tools\station\station.py" --view swarm %*
exit /b %errorlevel%
