@echo off
copy "%~dp0start_acc.ps1" "%TEMP%\start_acc.ps1" /Y >nul
PowerShell.exe -ExecutionPolicy Bypass -File "%TEMP%\start_acc.ps1"
timeout /t 3 /nobreak >nul
del "%TEMP%\start_acc.ps1" >nul 2>&1