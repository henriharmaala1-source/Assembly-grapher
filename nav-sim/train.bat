@echo off
REM Train the primitive-ranking policy. See docs/RL_HARNESS_PLAN.md.
REM
REM Sized for a 6 P-core / 8 E-core desktop: 16 workers, the rest for the
REM learner and the OS. --stereo is the honest setting and roughly 3x slower;
REM leave it off only for plumbing runs.
setlocal
set PYTHONPATH=%~dp0build;%~dp0python
python "%~dp0python\train.py" --workers 16 --steps 10000000 --stereo %*
pause
