#!/usr/bin/env bash
# Can a wide lens stand in for the IMU? Vision-only attitude (PnpVo, 6 DOF)
# against DepthVio on IMU attitude (0.3 deg noise, and 1 / 2 deg bias), by lens,
# both depth sources, level and manoeuvring. 4 seeds.
#   nav-sim/test/downvio_noimu.sh BUILD_DIR OUT.csv [JOBS]
set -euo pipefail
B=${1:?build dir}; OUT=${2:?out.csv}; J=${3:-$(nproc)}
T=$B/downvio_sweep
jobs=$(mktemp)
for fov in 60 90 120 140; do for d in plane stereo; do
  for fl in "--tilt 0" "--tilt 15 --wobble 10 --wobble-hz 1.5"; do
    for att in "--attitude imu" "--attitude imu --att-bias 1" "--attitude imu --att-bias 2" "--attitude vision"; do
      for s in 1 2 3 4; do echo "--fov $fov --depth $d $fl $att --dist 15 --seed $s" >> "$jobs"; done
    done
  done
done; done
echo "$(wc -l < "$jobs") runs on $J cores"
"$T" --header > "$OUT"
xargs -P "$J" -L 1 "$T" < "$jobs" >> "$OUT"
rm -f "$jobs"; echo "wrote $OUT"
