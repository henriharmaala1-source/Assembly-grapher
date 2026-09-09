# NDK build for the native nav core. The whole reason this app exists is to run
# the REAL move-stop-sense controller against live phone data, so we compile the
# actual nav-sim source file rather than copy or reimplement it — single source
# of truth. move_stop_sense.cpp is fully self-contained (only <algorithm>/<cmath>,
# no OpenCV, no other nav-sim files), which is exactly what makes this clean.
#
# If Android Studio can't resolve the out-of-tree path below, the fallback is to
# symlink or copy nav-sim/move_stop_sense.{cpp,hpp} into this cpp/ dir — but the
# out-of-tree reference is preferred so the phone always runs the same code the
# sim and drone do.

cmake_minimum_required(VERSION 3.22.1)
project(navviz CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Repo root, five levels up from android/app/src/main/cpp/.
get_filename_component(KESTREL_REPO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../../../.." ABSOLUTE)
set(NAVSIM_DIR "${KESTREL_REPO_ROOT}/nav-sim")

if(NOT EXISTS "${NAVSIM_DIR}/move_stop_sense.cpp")
    message(FATAL_ERROR
        "Cannot find ${NAVSIM_DIR}/move_stop_sense.cpp — the app module must live "
        "at <repo>/android/ so the out-of-tree nav-sim reference resolves. If you "
        "relocated it, copy move_stop_sense.{cpp,hpp} into this cpp/ dir instead.")
endif()

add_library(navviz SHARED
    nav_bridge.cpp
    "${NAVSIM_DIR}/move_stop_sense.cpp")

target_include_directories(navviz PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}"
    "${NAVSIM_DIR}")

# JNI needs no extra find_package on Android; jni.h is on the NDK include path.
target_link_libraries(navviz android log)

# 16 KB page-size support. The Magic V5 (and all Android 15+ devices on the
# newer SoCs) uses 16 KB memory pages; native .so's must be aligned to a 16 KB
# ELF max-page-size or the loader rejects them at install/launch. NDK r28+ does
# this by default, but set it explicitly so an older NDK also produces a
# compliant library. (This aligns OUR lib; the ONNX Runtime .so's ship from
# onnxruntime-android 1.26.0, which is recent enough to already be 16 KB-aligned.)
target_link_options(navviz PRIVATE
    "-Wl,-z,max-page-size=16384"
    "-Wl,-z,common-page-size=16384")
