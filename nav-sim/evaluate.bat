@echo off
REM Score a trained policy in sweep.sh's own columns, on HELD-OUT seeds.
REM Run rl_bench.exe first for the classical baselines to compare against.
setlocal
set PYTHONPATH=%~dp0build;%~dp0python
python "%~dp0python\evaluate.py" %*
pause
