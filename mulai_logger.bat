@echo off
REM Klik dua kali file ini untuk mulai merekam data SWH.
REM Pastikan laptop sudah tersambung ke WiFi SWH_PID_Agung.

set "ACT="
for %%D in ("%USERPROFILE%\anaconda3" "%USERPROFILE%\miniconda3" "C:\ProgramData\anaconda3" "C:\ProgramData\miniconda3") do (
    if not defined ACT if exist "%%~D\Scripts\activate.bat" set "ACT=%%~D\Scripts\activate.bat"
)
if not defined ACT (
    echo [ERROR] Anaconda/Miniconda tidak ditemukan.
    pause
    exit /b 1
)

call "%ACT%" swh
if errorlevel 1 (
    echo [ERROR] Environment "swh" tidak ditemukan. Jalankan: conda create -n swh python=3.11 -y
    pause
    exit /b 1
)

cd /d "%~dp0"
REM Logger otomatis membuka jendela grafik (swh_plot.py).
python swh_logger.py %*
pause
