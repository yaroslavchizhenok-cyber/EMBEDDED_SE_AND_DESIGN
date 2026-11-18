@echo off
REM ============================================================================
REM  19_Final_Project - Swarm parade - build, then how to simulate
REM
REM  ALWAYS rebuilds first.  The AVR edition shipped a simulate.bat that only
REM  built when Main.hex was missing, so every fix after the first build was
REM  invisible and people debugged firmware they had already replaced.
REM  See CLAUDE.md section 9e.  Do not "optimise" this back.
REM ============================================================================
call "%~dp0build.bat"
if errorlevel 1 exit /b 1

echo.
echo ============================================================
echo  Simulate in VS Code (the station needs the extension's
echo  serial port forwarding - the browser route has none)
echo ============================================================
echo.
echo   1. F1 - "Wokwi: Select Config File" - choose
echo        projects2026_arm/19_Final_Project/wokwi.toml
echo   2. F1 - "Wokwi: Start Simulator"
echo   3. In a terminal, in this folder:  run.bat   (the station)
echo   4. Click the station's picture, press a: the show starts.
echo      b aborts, s pauses.  INSTRUCTIONS.md lists every control.
echo.
exit /b 0
