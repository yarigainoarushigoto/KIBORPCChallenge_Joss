@echo off
chcp 65001 >nul
title Kibo-RPC 7 - download semua bahan
cd /d "%~dp0"
echo ==================================================
echo   Kibo-RPC 7 : download dokumen + simulator resmi
echo   Sumber: https://jaxa.krpc.jp/download
echo ==================================================

if not exist "01_Dokumen" mkdir "01_Dokumen"
echo [..] The 7th Kibo-RPC_Guidebook.pdf
curl.exe -L --fail --retry 3 -s -S -o "01_Dokumen\The 7th Kibo-RPC_Guidebook.pdf" "https://jaxa.krpc.jp/static/ed119ca818940fcb2fe4bc4ed03f2303/The%%207th%%20Kibo-RPC_Guidebook.pdf"
if errorlevel 1 (echo [GAGAL] The 7th Kibo-RPC_Guidebook.pdf) else (echo [OK]  The 7th Kibo-RPC_Guidebook.pdf)

if not exist "01_Dokumen" mkdir "01_Dokumen"
echo [..] The 7th Kibo-RPC_Guidance_Session.pdf
curl.exe -L --fail --retry 3 -s -S -o "01_Dokumen\The 7th Kibo-RPC_Guidance_Session.pdf" "https://jaxa.krpc.jp/static/d89afd41d2ea4595ea43fcc8946dffc2/The%%207th%%20Kibo-RPC_Guidance_Session.pdf"
if errorlevel 1 (echo [GAGAL] The 7th Kibo-RPC_Guidance_Session.pdf) else (echo [OK]  The 7th Kibo-RPC_Guidance_Session.pdf)

if not exist "01_Dokumen" mkdir "01_Dokumen"
echo [..] 7thKibo-RPC_Rulebook.pdf
curl.exe -L --fail --retry 3 -s -S -o "01_Dokumen\7thKibo-RPC_Rulebook.pdf" "https://jaxa.krpc.jp/static/2881600683dab1897ef65741461f8618/7thKibo-RPC_Rulebook.pdf"
if errorlevel 1 (echo [GAGAL] 7thKibo-RPC_Rulebook.pdf) else (echo [OK]  7thKibo-RPC_Rulebook.pdf)

if not exist "01_Dokumen" mkdir "01_Dokumen"
echo [..] 7thKibo-RPC_BeginnerCourseRulebook.pdf
curl.exe -L --fail --retry 3 -s -S -o "01_Dokumen\7thKibo-RPC_BeginnerCourseRulebook.pdf" "https://jaxa.krpc.jp/static/6fae6038e074dda360f3e390675dad8f/7thKibo-RPC_BeginnerCourseRulebook.pdf"
if errorlevel 1 (echo [GAGAL] 7thKibo-RPC_BeginnerCourseRulebook.pdf) else (echo [OK]  7thKibo-RPC_BeginnerCourseRulebook.pdf)

if not exist "01_Dokumen" mkdir "01_Dokumen"
echo [..] ProgrammingManual.pdf
curl.exe -L --fail --retry 3 -s -S -o "01_Dokumen\ProgrammingManual.pdf" "https://jaxa.krpc.jp/static/b60d4a181401f2875ca8d8371f6c0c5a/ProgrammingManual.pdf"
if errorlevel 1 (echo [GAGAL] ProgrammingManual.pdf) else (echo [OK]  ProgrammingManual.pdf)

if not exist "01_Dokumen" mkdir "01_Dokumen"
echo [..] 7thKiboRPC_poster_A4.pdf
curl.exe -L --fail --retry 3 -s -S -o "01_Dokumen\7thKiboRPC_poster_A4.pdf" "https://jaxa.krpc.jp/static/5d17e79659e3ce54563228f023e03404/7thKiboRPC_poster_A4.pdf"
if errorlevel 1 (echo [GAGAL] 7thKiboRPC_poster_A4.pdf) else (echo [OK]  7thKiboRPC_poster_A4.pdf)

if not exist "02_Simulator" mkdir "02_Simulator"
echo [..] krpc7th-simulator.tar.gz
curl.exe -L --fail --retry 3 -s -S -o "02_Simulator\krpc7th-simulator.tar.gz" "https://jaxa.krpc.jp/static/c90937a3712dd94a0a8b8449327080cc/krpc7th-simulator.tar.gz"
if errorlevel 1 (echo [GAGAL] krpc7th-simulator.tar.gz) else (echo [OK]  krpc7th-simulator.tar.gz)

if not exist "03_Sample_Code" mkdir "03_Sample_Code"
echo [..] krpc7th-sample-source-beta.tar.gz
curl.exe -L --fail --retry 3 -s -S -o "03_Sample_Code\krpc7th-sample-source-beta.tar.gz" "https://jaxa.krpc.jp/static/fba02f400a3b7fc190cb6e77d41b2787/krpc7th-sample-source-beta.tar.gz"
if errorlevel 1 (echo [GAGAL] krpc7th-sample-source-beta.tar.gz) else (echo [OK]  krpc7th-sample-source-beta.tar.gz)

echo.
echo Selesai. Cek folder 01_Dokumen, 02_Simulator, 03_Sample_Code.
echo Jendela ini akan tertutup sendiri dalam 15 detik...
timeout /t 15 >nul
