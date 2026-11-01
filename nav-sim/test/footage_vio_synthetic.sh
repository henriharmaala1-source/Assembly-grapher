#!/bin/sh
# footage_vio end to end, without a drone: downvio_sweep renders a looped
# flight (gimbal straight down, 3 m, 25 m circle that ends where it started,
# then hovers 3 s there, as the recording protocol does)
# to a video and DJI-style subtitles with the truth in them; footage_vio runs
# on the video like any recording.
#
#   1. height from the subtitles: the loop must close and match the GPS
#      track to a few percent;
#   2. a custom --height 20 % too high: the track comes out 20 % long, and
#      the report must say so -- the height it implies within 5 % of 3 m;
#   3. --loop: the landmark memory recognises the start when the loop returns;
#   4. the clock: with the subtitles deliberately shifted by 0.5 s
#      (--srt-offset), the run must estimate the shift back to within a frame.
#
# The ground is navcore's noise model, so no textures are needed. Exit 77
# (skipped) when this OpenCV cannot write video.
#
#   sh footage_vio_synthetic.sh DOWNVIO_SWEEP FOOTAGE_VIO WORKDIR
set -u
SWEEP=$1 FV=$2 W=$3
mkdir -p "$W" || exit 2
"$SWEEP" --gimbal -90 --path circle --alt 3 --fov 90 --speed 3 --dist 25 --res 640x400 --seed 1 --hover-end 3 \
         --write-video "$W/loop.mp4" --write-srt "$W/loop.SRT" > /dev/null
rc=$?
[ $rc -eq 77 ] && { echo "no video writer: skipped"; exit 77; }
[ $rc -ne 0 ] && { echo "downvio_sweep failed ($rc)"; exit 1; }
rm -f "$W/summary.csv"
"$FV" --video "$W/loop.mp4" --hfov 90 --width 640 --summary "$W/summary.csv" --check-drift 5 || exit 1
"$FV" --video "$W/loop.mp4" --hfov 90 --width 640 --height 3.6 --summary "$W/summary.csv" --quiet || exit 1
# row 3 = the custom-height run; implied_height_m is the 24th column
awk -F, 'NR == 3 { h = $24; print "implied height " h " m (true 3.0)"; exit (h > 2.85 && h < 3.15) ? 0 : 1 }' \
    "$W/summary.csv" || exit 1
# 3. the landmark memory: the loop ends over its start, so it must recognise
#    the start (returns, column 29) and stay within the drift bound. The loop
#    takes 9 s before the hover, hence the short minimum age.
"$FV" --video "$W/loop.mp4" --hfov 90 --width 640 --loop --loop-min-age 5 --summary "$W/summary.csv" \
      --check-drift 5 --quiet || exit 1
awk -F, 'NR == 4 { print "landmark memory: " $29 " return(s) recognised, start-to-end " $30 " -> " $15 " m";
                   exit ($29 >= 1) ? 0 : 1 }' "$W/summary.csv" || exit 1
"$FV" --video "$W/loop.mp4" --hfov 90 --width 640 --srt-offset 0.5 --summary "$W/summary.csv" --quiet || exit 1
# row 5: srt_lag_s is the 32nd column; 0.5 s injected, so -0.5 expected
awk -F, 'NR == 5 { l = $32; print "clock offset estimated " l " s (injected +0.5)"; exit (l > -0.54 && l < -0.46) ? 0 : 1 }' \
    "$W/summary.csv"
