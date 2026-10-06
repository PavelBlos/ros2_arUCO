@echo off
cd /d "%~dp0"
where pyw.exe >nul 2>nul
if %errorlevel%==0 (
    start "" pyw.exe -3 "%~dp0find_raspberry_pi.pyw"
) else (
    start "" pythonw.exe "%~dp0find_raspberry_pi.pyw"
)
