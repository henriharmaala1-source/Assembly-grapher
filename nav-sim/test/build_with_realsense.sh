#!/usr/bin/env bash
# Compile AND RUN the --live path without a camera, and without the SDK.
#
# The --live branch sits behind #ifdef NAVSIM_HAVE_REALSENSE, so on any machine
# without librealsense it is invisible to the compiler -- it can rot
# indefinitely and every build stays green. It did: the first version called
# prof.get_device().first_depth_sensor(), which is a PYTHON binding and not the
# C++ API, and nothing anywhere would have caught it before the one machine
# that could actually run it tried to build it.
#
# The trick: pyrealsense2's wheel exports the whole rs2_* C API (456 symbols),
# and librealsense's C++ interface is a header-only inline wrapper over exactly
# that. So headers from git plus the wheel's .so gives a real link. It will not
# see a USB camera through this path, but it compiles the branch, runs it, and
# proves the graceful "no device" behaviour -- which is everything except the
# camera itself.
#
#   pip install pyrealsense2
#   bash test/build_with_realsense.sh
set -e
cd "$(dirname "$0")/.."

HDR=${RS_HEADERS:-/tmp/rsheaders}
if [ ! -f "$HDR/include/librealsense2/rs.hpp" ]; then
  echo "fetching librealsense headers -> $HDR"
  git clone --depth 1 --filter=blob:none --sparse \
      https://github.com/realsenseai/librealsense.git "$HDR" >/dev/null
  (cd "$HDR" && git sparse-checkout set include >/dev/null)
fi

RS=$(python3 -c "import pyrealsense2,glob,os;d=os.path.dirname(pyrealsense2.__file__);print(glob.glob(d+'/pyrealsense2*.so')[0])")
RSDIR=$(dirname "$RS")
PYLIB=$(python3 -c "import sysconfig;print(sysconfig.get_config_var('LIBDIR'))")
PYVER=$(python3 -c "import sysconfig;print(sysconfig.get_config_var('LDVERSION'))")

# The wheel's soname is versioned; give the loader something to find.
mkdir -p /tmp/rslib
ln -sf "$RS" "/tmp/rslib/$(basename "$RS").2.58" 2>/dev/null || true

echo "building voxel_live with NAVSIM_HAVE_REALSENSE=1"
g++ -O2 -std=c++17 -DSIM_HAVE_HIGHGUI=1 -DNAVSIM_HAVE_REALSENSE=1 \
    -I. -I"$HDR/include" -I/usr/include/opencv4 \
    voxel_live.cpp frame_source.cpp depth_record.cpp depth_camera.cpp \
    voxel_map.cpp voxel_world.cpp voxel_traj.cpp voxel_planner.cpp \
    -lopencv_core -lopencv_imgproc -lopencv_imgcodecs -lopencv_highgui \
    "$RS" -L"$PYLIB" -lpython"$PYVER" \
    -Wl,-rpath,"$RSDIR" -Wl,-rpath,"$PYLIB" -Wl,-rpath,/tmp/rslib \
    -o /tmp/voxel_live_rs
echo "  compiled and linked"

export LD_LIBRARY_PATH=/tmp/rslib:$RSDIR:$LD_LIBRARY_PATH
/tmp/voxel_live_rs --menu-preview /tmp/menu_realsense.png >/dev/null
echo "  menu rendered with LIVE enabled -> /tmp/menu_realsense.png"

echo "running --live (expected: a clean 'No device connected')"
/tmp/voxel_live_rs --live --frames 3 --headless --out /tmp/vlive
echo "OK -- the live path compiles, links, runs and fails gracefully"
