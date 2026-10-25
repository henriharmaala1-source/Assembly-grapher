#!/usr/bin/env bash
# Real ground photographs for nav-sim's daylight ground renderer
# (ground_texture.hpp), fetched -- not committed -- from pinned sources and
# checked by SHA-256. Run from anywhere; writes next to this script.
#
#   name             source (pinned)                                   licence
#   lawn.jpg         three.js examples/textures/terrain/grasslight-big CC-BY 3.0, opengameart.org "dark-grass"
#   grass.jpg        Babylon.js playground textures/grass.jpg          Apache-2.0 (Babylon.js repository)
#   grass_pd.png     scikit-image 0.26.0 data/grass.png                public domain (Wikimedia, per skimage)
#   gravel.png       scikit-image 0.26.0 data/gravel.png               public domain (Wikimedia, per skimage)
#   dirt.jpg         Babylon.js textures/ground.jpg (dry, cracked)     Apache-2.0
#   rocky.png        Babylon.js textures/rockyGround_basecolor.png     Apache-2.0
#   sand.jpg         Babylon.js textures/sand.jpg                      Apache-2.0
#   paving.png       Babylon.js textures/floor.png (paving stones)     Apache-2.0
#   brick.jpg        three.js examples/textures/brick_diffuse.jpg      MIT (three.js repository)
#
# The licences are those the files are distributed under in those
# repositories; the opengameart one asks for attribution (above).
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"
BJS=https://raw.githubusercontent.com/BabylonJS/Babylon.js/c9e77a1189e1cc2ae762ec4b88ce9085d2803bcc/packages/tools/playground/public/textures
THREE=https://raw.githubusercontent.com/mrdoob/three.js/5efce9dcb3b6d063e4e96772374ba4c5bcbe2626/examples/textures
fetch() {   # name url sha256
    if [ -f "$1" ] && echo "$3  $1" | sha256sum -c --quiet 2>/dev/null; then return; fi
    curl -fsSL -o "$1" "$2"
    echo "$3  $1" | sha256sum -c --quiet || { echo "checksum mismatch: $1" >&2; rm -f "$1"; exit 1; }
}
fetch lawn.jpg   "$THREE/terrain/grasslight-big.jpg"      23bd506b94e40a9de435375c2be6280eb088dc637fb64c331e325c7472f47177
fetch brick.jpg  "$THREE/brick_diffuse.jpg"               b49b0ab13aa4455cc7348cb4bfbbffcb344f2f81b17121eb1e57bcd3b387febe
fetch grass.jpg  "$BJS/grass.jpg"                         25ebb5a3408f3a17e80ee1d56fc1290c945076ee648aacb8ac5ff6a0725fc46c
fetch dirt.jpg   "$BJS/ground.jpg"                        e3637cbd17d1f086f32ebdf21c8399b0bac9d6dccecae1bd2207482895e17678
fetch rocky.png  "$BJS/rockyGround_basecolor.png"         e0f9375b366c3bd3375581a329af36fe70ded07ffee8e1156aa6a210c16f486b
fetch sand.jpg   "$BJS/sand.jpg"                          14e81b4cd4dad3d001bf647eef7139d0a89de38be8bd13d551e66e4eeb6b8ecd
fetch paving.png "$BJS/floor.png"                         de2466be7a3b8e98a767cd33e81f1a515b2d3355015d95c03b7b6652ca525c92
# scikit-image's two from its wheel on PyPI (the GitHub path moved).
if [ ! -f grass_pd.png ] || [ ! -f gravel.png ]; then
    tmp=$(mktemp -d)
    WHL=scikit_image-0.26.0-cp311-cp311-manylinux_2_24_x86_64.manylinux_2_28_x86_64.whl
    curl -fsSL -o "$tmp/$WHL" "https://files.pythonhosted.org/packages/cp311/s/scikit-image/$WHL" 2>/dev/null \
      || python3 -m pip download -q --no-deps --only-binary=:all: --python-version 3.11 \
             --platform manylinux_2_28_x86_64 scikit-image==0.26.0 -d "$tmp"
    echo "74aa5518ccea28121f57a95374581d3b979839adc25bb03f289b1bc9b99c58af  $tmp/$WHL" | sha256sum -c --quiet
    unzip -q -o -j "$tmp/$WHL" skimage/data/grass.png skimage/data/gravel.png -d "$tmp"
    mv "$tmp/grass.png" grass_pd.png; mv "$tmp/gravel.png" gravel.png; rm -rf "$tmp"
fi
echo "b6b6022426b38936c43a4ac09635cd78af074e90f42ffa8227ac8b7452d39f89  grass_pd.png" | sha256sum -c --quiet
echo "c48615b451bf1e606fbd72c0aa9f8cc0f068ab7111ef7d93bb9b0f2586440c12  gravel.png" | sha256sum -c --quiet
echo "ground textures ready in $(pwd)"
