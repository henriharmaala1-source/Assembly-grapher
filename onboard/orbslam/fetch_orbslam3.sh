#!/usr/bin/env bash
# Fetch, patch and build ORB-SLAM3 HEADLESS, for the kestrel-orbslam bridge.
#
#   onboard/orbslam/fetch_orbslam3.sh [DEST]     (default: onboard/orbslam/third_party)
#
# PINNED to one upstream commit, so what was measured is what gets built. The
# patch is small and touches the build, not the SLAM:
#   - no Pangolin/OpenGL: the viewer is replaced by headless/headless_stubs.cc
#     and <pangolin/pangolin.h> by a one-type shim (the aircraft has no screen)
#   - C++14 instead of C++11 (Eigen 3.4 / GCC 13 warnings), examples not built
#   - one read-only accessor, System::GetCurrentMapIdHeadless()
#   - the settings printout removed: upstream segfaults printing a Rectified
#     stereo configuration (it dereferences a second calibration it never made)
#   - the vocabulary is unpacked next to the library
# ORB-SLAM3 is GPLv3. It is built into its own process, kestrel-orbslam, which
# talks to the rest of onboard over a socket (slam_link.hpp) -- see README.md.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEST="${1:-$HERE/third_party}"
COMMIT=4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4        # upstream master, 2022-02-10
JOBS="${JOBS:-$(nproc)}"

mkdir -p "$DEST"
SRC="$DEST/ORB_SLAM3"
if [ ! -d "$SRC/.git" ]; then
    git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git "$SRC"
fi
git -C "$SRC" fetch -q origin "$COMMIT" 2>/dev/null || true
git -C "$SRC" checkout -q -f "$COMMIT"
git -C "$SRC" clean -q -fdx -e Vocabulary/ORBvoc.txt

# ---- the headless patch --------------------------------------------------
python3 - "$SRC" "$HERE/headless" <<'PY'
import re, sys
src, headless = sys.argv[1], sys.argv[2]
p = src + '/CMakeLists.txt'
s = open(p).read()
s = s.replace('find_package(Pangolin REQUIRED)', '')
s = s.replace('find_package(realsense2)', '')
s = s.replace('${Pangolin_INCLUDE_DIRS}', '')
s = s.replace('${Pangolin_LIBRARIES}', '')
s = s.replace('-std=c++11', '-std=c++14')
s = s.replace('include_directories(\n${PROJECT_SOURCE_DIR}\n',
              'include_directories(\n' + headless + '\n${PROJECT_SOURCE_DIR}\n', 1)
s = s.replace('src/Viewer.cc\n', '')
s = s.replace('src/MapDrawer.cc\n', headless + '/headless_stubs.cc\n')
cut = s.find('# Build examples')
if cut > 0:
    s = s[:cut]
open(p, 'w').write(s)
# One accessor, so the bridge can report WHICH map is active: a new map
# (after a loss) is a new coordinate frame, and the consumer must re-anchor.
p = src + '/include/System.h'
s = open(p).read()
anchor = '    int GetTrackingState();'
if 'GetCurrentMapIdHeadless' not in s:
    s = s.replace(anchor, anchor + '\n    long GetCurrentMapIdHeadless();', 1)
open(p, 'w').write(s)
p = src + '/src/System.cc'
s = open(p).read()
# Upstream bug: Settings' printer dereferences the second camera's calibration,
# which a "Rectified" stereo pair never creates -- a segfault on startup.
s = s.replace('        cout << (*settings_) << endl;',
              '        // (settings printout removed: segfaults for Rectified stereo)')
if 'GetCurrentMapIdHeadless' not in s:
    s += ('\nnamespace ORB_SLAM3 {\nlong System::GetCurrentMapIdHeadless() {\n'
          '    Map* m = mpAtlas ? mpAtlas->GetCurrentMap() : nullptr;\n'
          '    return m ? long(m->GetId()) : -1;\n}\n}\n')
open(p, 'w').write(s)
PY

# ---- third-party parts, as upstream build.sh does ------------------------
for part in DBoW2 g2o Sophus; do
    cmake -S "$SRC/Thirdparty/$part" -B "$SRC/Thirdparty/$part/build" \
          -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF -DBUILD_SOPHUS_TESTS=OFF >/dev/null
    cmake --build "$SRC/Thirdparty/$part/build" -j "$JOBS"
done
if [ ! -f "$SRC/Vocabulary/ORBvoc.txt" ]; then
    tar -xzf "$SRC/Vocabulary/ORBvoc.txt.tar.gz" -C "$SRC/Vocabulary"
fi
cmake -S "$SRC" -B "$SRC/build" -DCMAKE_BUILD_TYPE=Release >/dev/null
cmake --build "$SRC/build" -j "$JOBS"
echo "ORB-SLAM3 built: $SRC/lib/libORB_SLAM3.so"
echo "Now: cmake -S $HERE -B $HERE/build -DORB_SLAM3_DIR=$SRC && cmake --build $HERE/build"
