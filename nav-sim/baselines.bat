@echo off
REM The non-RL side of the comparison, through the SAME environment the policy
REM uses -- random / freeM / goal / the classical weighted score.
"%~dp0build\Release\rl_bench.exe" --worlds forest maze --seeds 101 108 %*
pause
