#!/usr/bin/env bash
# One `kestrel sitl` flight against a fresh ArduCopter SITL, for CI
# (.github/workflows/sitl-linux.yml) or by hand from the repo root:
#
#   nav-sim/sitl/ci_fly.sh gps|vio [SEED] [SECONDS]
#
# Expects ardupilot/build/sitl/bin/arducopter, ardupilot/Tools (for the
# default parameters) and build/kestrel. Writes sitl_ci/<mode>.log, the CSV row
# and the pictures. Exit: kestrel's, except that 1 (the flight touched a wall)
# is reported and passes -- see the workflow for why.
set -uo pipefail
MODE=${1:?gps or vio}; SEED=${2:-101}; SECS=${3:-30}
ROOT=$(pwd)
AP=${AP:-$ROOT/ardupilot}
K=${K:-$ROOT/build/kestrel}
OUT=$ROOT/sitl_ci
mkdir -p "$OUT"

PARMS=$AP/Tools/autotest/default_params/copter.parm,$ROOT/nav-sim/sitl/kestrel_sitl.parm
FLAG=""
if [ "$MODE" = vio ]; then PARMS=$PARMS,$ROOT/nav-sim/sitl/kestrel_vio.parm; FLAG=--vio; fi

pkill -x arducopter 2>/dev/null; sleep 1
( cd "$OUT" && exec "$AP/build/sitl/bin/arducopter" --model + --speedup 1 -I0 -w \
      --defaults "$PARMS" > "$OUT/arducopter_$MODE.log" 2>&1 ) &
SITL=$!

"$K" sitl $FLAG --lowres --seconds "$SECS" --seed "$SEED" \
    --csv "$OUT/sitl.csv" --shot "$OUT/$MODE" > "$OUT/$MODE.log" 2>&1
RC=$?
echo "exit=$RC" >> "$OUT/$MODE.log"
kill $SITL 2>/dev/null; wait $SITL 2>/dev/null
grep -E '^\[sitl\] ([0-9]+ s,|holding|GNSS|soft|COLLISION)' "$OUT/$MODE.log"

case $RC in
  0) exit 0 ;;
  1) echo "::warning::kestrel sitl $MODE: the flight touched a wall (one seed; judged across seeds, not here)"; exit 0 ;;
  *) echo "::error::kestrel sitl $MODE exited $RC"; tail -30 "$OUT/$MODE.log"; exit "$RC" ;;
esac
