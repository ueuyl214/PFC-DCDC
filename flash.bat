@echo off
setlocal
cd /d "%~dp0"
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0flash.ps1" %*
set ERR=%ERRORLEVEL%
echo.
if "%ERR%"=="0" (
  echo Flash script finished successfully.
) else (
  echo Flash script failed with exit code %ERR%.
)
pause
exit /b %ERR%
