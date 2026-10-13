@echo off
:: Find pythonw.exe next to whichever python is on PATH
for /f "delims=" %%i in ('py -3 -c "import sys,os; print(os.path.join(os.path.dirname(sys.executable),'pythonw.exe'))" 2^>nul') do set PYW=%%i
if exist "%PYW%" (
    start "" "%PYW%" "%~dp0gui_main.py"
    exit /b
)
:: Fallback: run via py launcher (no pythonw needed, py is always in C:\Windows)
start "" py -3 "%~dp0gui_main.py"
