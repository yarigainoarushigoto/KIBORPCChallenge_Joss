@echo off
REM Kibo-RPC 7 - install WSL2 + Ubuntu 24.04 (Programming Manual 2.4.1)
REM Klik 2x. Akan minta izin Administrator.
net session >nul 2>&1
if %errorlevel% neq 0 (
  powershell -Command "Start-Process '%~f0' -Verb RunAs"
  exit /b
)
echo Menginstall WSL2 + Ubuntu-24.04 ...
wsl --install -d Ubuntu-24.04
echo.
echo Kalau diminta restart, restart Windows dulu.
echo Setelah itu buka "Ubuntu" dari Start Menu, buat username + password.
echo Cek versi: wsl --list --verbose  (VERSION harus 2)
pause
