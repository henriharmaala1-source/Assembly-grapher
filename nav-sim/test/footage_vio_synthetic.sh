#!/bin/sh
# footage_vio end to end, without a drone: downvio_sweep renders a looped
# flight (gimbal straight down, 3 m, 25 m circle that ends where it started)
# to a video and DJI-style subtitles with the truth in them; footage_vio runs
# on the video like any recording.
#
#   1. height from the subtitles: the loop must close and match the GPS
#      track to a few percent;
#   2. a custom --height 20 % too high: the track comes out 20 % long, and
#      the report must say so -- the height it implies within 5 % of 3 m.
#
# The ground is navcore's noise model, so no textures are needed. Exit 77
# (skipped) when this OpenCV cannot write video.
#
#   sh footage_vio_synthetic.sh DOWNVIO_SWEEP FOOTAGE_VIO WORKDIR
set -u
SWEEP=$1 FV=$2 W=$3
mkdir -p "$W" || exit 2
"$SWEEP" --gimbal -90 --path circle --alt 3 --fov 90 --speed 3 --dist 25 --res 640x400 --seed 1 \
         --write-video "$W/loop.mp4" --write-srt "$W/loop.SRT" > /dev/null
rc=$?
[ $rc -eq 77 ] && { echo "no video writer: skipped"; exit 77; }
[ $rc -ne 0 ] && { echo "downvio_sweep failed ($rc)"; exit 1; }
rm -f "$W/summary.csv"
"$FV" --video "$W/loop.mp4" --hfov 90 --width 640 --summary "$W/summary.csv" --check-drift 5 || exit 1
"$FV" --video "$W/loop.mp4" --hfov 90 --width 640 --height 3.6 --summary "$W/summary.csv" --quiet || exit 1
# row 3 = the custom-height run; implied_height_m is the 24th column
awk -F, 'NR == 3 { h = $24; print "implied height " h " m (true 3.0)"; exit (h > 2.85 && h < 3.15) ? 0 : 1 }' \
    "$W/summary.csv"
