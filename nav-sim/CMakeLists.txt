cmake_minimum_required(VERSION 3.16)
project(nav_sim CXX)

# Standalone path-planning testbench. No dependency on any other project — this
# directory is self-contained and can be lifted into its own git repository as-is.

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)
find_package(OpenCV QUIET COMPONENTS highgui)

add_executable(nav_sim
  sim_main.cpp
  sim_world.cpp
  occupancy_grid.cpp
  planners.cpp
  move_stop_sense.cpp
  explore.cpp)

target_include_directories(nav_sim PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(nav_sim PRIVATE ${OpenCV_LIBS})

if(OpenCV_highgui_FOUND OR TARGET opencv_highgui)
  target_compile_definitions(nav_sim PRIVATE SIM_HAVE_HIGHGUI=1)
  message(STATUS "nav_sim: highgui found — --display enabled")
else()
  message(STATUS "nav_sim: no highgui — headless only (--save= PNG dumps still work)")
endif()

# ---------------------------------------------------------------------------
# Test suite: exercises the planners AND the ported move-stop-sense controller
# headlessly, asserting they reach the goal and keep obstacle standoff.
#   cmake -B build -DBUILD_TESTS=ON && cmake --build build && ctest --test-dir build
# ---------------------------------------------------------------------------
option(BUILD_TESTS "Build the nav-sim test suite" OFF)
if(BUILD_TESTS)
  enable_testing()
  add_executable(test_nav
    test/test_nav.cpp
    sim_world.cpp
    occupancy_grid.cpp
    planners.cpp
    move_stop_sense.cpp
    explore.cpp)
  target_include_directories(test_nav PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(test_nav PRIVATE ${OpenCV_LIBS})
  add_test(NAME test_nav COMMAND test_nav)
endif()

# --- 3D voxel world / stereo depth / voxel map -------------------------------
add_executable(voxel_demo
  voxel_demo.cpp
  voxel_world.cpp
  depth_camera.cpp
  voxel_map.cpp)
target_include_directories(voxel_demo PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(voxel_demo PRIVATE ${OpenCV_LIBS})

# Headless renderer check. No window needed, so it runs in CI and over ssh --
# which matters, because the defect it exists to catch (one-pixel cubes) is
# invisible in every other test and obvious in its output.
add_executable(iso_render_check
  test/iso_render_check.cpp
  voxel_world.cpp
  depth_camera.cpp
  voxel_map.cpp)
target_include_directories(iso_render_check PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(iso_render_check PRIVATE ${OpenCV_LIBS})

# Side-by-side of what the fine and coarse maps know, with range rings. The
# claim the coarse map makes is about RANGE, and a first-person view cannot
# show range -- it shows what is close, which is the part both maps agree on.
add_executable(far_map_demo
  test/far_map_demo.cpp
  voxel_world.cpp
  depth_camera.cpp
  voxel_map.cpp)
target_include_directories(far_map_demo PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(far_map_demo PRIVATE ${OpenCV_LIBS})

# Depth improver checks, including a brute-force differential test of the
# O(1)-per-pixel window minimum. Needs only opencv core, so it runs anywhere.
add_executable(depth_improve_check
  test/depth_improve_check.cpp
  depth_improve.cpp)
target_include_directories(depth_improve_check PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(depth_improve_check PRIVATE ${OpenCV_LIBS})
if(BUILD_TESTS)
  add_test(NAME depth_improve_check COMMAND depth_improve_check)
endif()

# Yaw-coupled sideslip: asserts |v| is preserved, straight primitives are
# untouched, and the lateral gain is where the header claims it is.
add_executable(sideslip_check test/sideslip_check.cpp)
target_include_directories(sideslip_check PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(sideslip_check PRIVATE ${OpenCV_LIBS})
if(BUILD_TESTS)
  add_test(NAME sideslip_check COMMAND sideslip_check)
endif()

# Calibrates the forest world's trunk-visibility parameter against a REAL
# photograph, so that number stops being a guess read off someone's screenshot.
add_executable(bark_contrast test/bark_contrast.cpp)
target_include_directories(bark_contrast PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(bark_contrast PRIVATE ${OpenCV_LIBS})

# Defined after the OMPL probe below, since it flies a real planned run.

find_package(ompl QUIET)
if(ompl_FOUND)
  message(STATUS "nav_sim: OMPL found - forward planner uses RRTConnect")
else()
  message(STATUS "nav_sim: no OMPL - forward planner falls back to a straight-line probe")
endif()

add_executable(voxel_sim
  voxel_sim.cpp
  ompl_planner.cpp
  voxel_world.cpp
  depth_camera.cpp
  depth_improve.cpp
  voxel_map.cpp
  voxel_planner.cpp
  voxel_traj.cpp)
target_include_directories(voxel_sim PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(voxel_sim PRIVATE ${OpenCV_LIBS})
if(OpenCV_highgui_FOUND OR TARGET opencv_highgui)
  target_compile_definitions(voxel_sim PRIVATE SIM_HAVE_HIGHGUI=1)
endif()
if(ompl_FOUND)
  target_compile_definitions(voxel_sim PRIVATE NAVSIM_HAVE_OMPL=1)
  target_include_directories(voxel_sim PRIVATE ${OMPL_INCLUDE_DIRS})
  target_link_libraries(voxel_sim PRIVATE ${OMPL_LIBRARIES})
endif()

# Headless render of the GUI's flight layout. The window is the one artefact in
# this tree that cannot be checked over ssh or in CI, so this writes the same
# 2x2 composite to a PNG that anyone can look at without building the GUI.
add_executable(gui_preview
  test/gui_preview.cpp
  ompl_planner.cpp
  voxel_world.cpp
  depth_camera.cpp
  voxel_map.cpp
  voxel_planner.cpp
  voxel_traj.cpp)
target_include_directories(gui_preview PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(gui_preview PRIVATE ${OpenCV_LIBS})
if(ompl_FOUND)
  target_compile_definitions(gui_preview PRIVATE NAVSIM_HAVE_OMPL=1)
  target_include_directories(gui_preview PRIVATE ${OMPL_INCLUDE_DIRS})
  target_link_libraries(gui_preview PRIVATE ${OMPL_LIBRARIES})
endif()

# --- point-and-click front end (needs highgui) --------------------------------
if(OpenCV_highgui_FOUND OR TARGET opencv_highgui)
  add_executable(voxel_gui
    voxel_gui.cpp
    voxel_world.cpp
    depth_camera.cpp
    voxel_map.cpp
    voxel_planner.cpp
    voxel_traj.cpp
    ompl_planner.cpp)
  target_include_directories(voxel_gui PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(voxel_gui PRIVATE ${OpenCV_LIBS})
  if(ompl_FOUND)
    target_compile_definitions(voxel_gui PRIVATE NAVSIM_HAVE_OMPL=1)
    target_include_directories(voxel_gui PRIVATE ${OMPL_INCLUDE_DIRS})
    target_link_libraries(voxel_gui PRIVATE ${OMPL_LIBRARIES})
  endif()
  message(STATUS "nav_sim: voxel_gui enabled")
endif()

# --- voxel_live: the real map and planner over REAL depth --------------------
# Optional librealsense. Without it the target still builds and --replay works
# on recordings, which is the mode that needs no camera at all; --live then
# reports that this build has no SDK rather than silently doing nothing.
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/find_realsense.cmake)

add_executable(voxel_live
  voxel_live.cpp
  frame_source.cpp
  depth_record.cpp
  voxel_world.cpp
  depth_camera.cpp
  voxel_map.cpp
  voxel_planner.cpp
  voxel_traj.cpp)
target_include_directories(voxel_live PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(voxel_live PRIVATE ${OpenCV_LIBS})
if(OpenCV_highgui_FOUND OR TARGET opencv_highgui)
  target_compile_definitions(voxel_live PRIVATE SIM_HAVE_HIGHGUI=1)
endif()
if(realsense2_FOUND)
  target_compile_definitions(voxel_live PRIVATE NAVSIM_HAVE_REALSENSE=1)
  target_link_libraries(voxel_live PRIVATE realsense2::realsense2)
  message(STATUS "nav_sim: librealsense found -- voxel_live --live enabled")
else()
  # Loud and actionable. The old message said only that the SDK was missing,
  # which is exactly wrong on the common case where it is installed and merely
  # not on CMake's search path.
  message(WARNING
    "nav_sim: librealsense NOT found -- voxel_live will build with --replay only "
    "and the LIVE CAMERA button will be disabled.\n"
    "  If the RealSense SDK IS installed, CMake is simply not looking in the "
    "right place. Point it there:\n"
    "    cmake -B build -Drealsense2_DIR=\"C:/Program Files (x86)/Intel RealSense "
    "SDK 2.0/lib/cmake/realsense2\"\n"
    "  or set the REALSENSE2_DIR environment variable to the SDK root.\n"
    "  If it is NOT installed: https://github.com/realsenseai/librealsense/releases/latest "
    "-> RealSense.SDK.exe")
endif()

# THE DLL. On Windows the link succeeds and the program then fails to start,
# because realsense2.dll is not beside the exe. Copy it there. This is the
# classic "it built, why will it not run" and it is one line to prevent.
if(realsense2_FOUND AND WIN32)
  find_file(RS2_DLL realsense2.dll
            HINTS ${_rs_hints} PATH_SUFFIXES bin bin/x64)
  if(RS2_DLL)
    add_custom_command(TARGET voxel_live POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy_if_different
              "${RS2_DLL}" "$<TARGET_FILE_DIR:voxel_live>")
    message(STATUS "nav_sim: will copy ${RS2_DLL} beside voxel_live.exe")
  else()
    message(STATUS "nav_sim: realsense2.dll not located -- if voxel_live.exe "
                   "fails to start, copy it beside the exe by hand")
  endif()
endif()

# .kdr recording format: round trip, truncation, refusals, and that the replay
# source rebuilds a camera from the file's own intrinsics.
add_executable(depth_record_check
  test/depth_record_check.cpp
  depth_record.cpp
  frame_source.cpp
  depth_camera.cpp
  voxel_world.cpp)
target_include_directories(depth_record_check PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(depth_record_check PRIVATE ${OpenCV_LIBS})
if(BUILD_TESTS)
  add_test(NAME depth_record_check COMMAND depth_record_check)
  # Python writes / C++ reads, and back. Needs the C++ test to have run first,
  # so it reads the file that one wrote.
  find_package(Python3 COMPONENTS Interpreter QUIET)
  if(Python3_FOUND)
    add_test(NAME kdr_crosscheck
             COMMAND ${Python3_EXECUTABLE}
                     ${CMAKE_CURRENT_SOURCE_DIR}/test/kdr_crosscheck.py
                     $<TARGET_FILE:voxel_live>)
    set_tests_properties(kdr_crosscheck PROPERTIES DEPENDS depth_record_check)
  endif()
endif()

# Stereo occlusion shadow: asserts the SIDE (a mirrored shadow would look
# plausible and bias every obstacle boundary) and the WIDTH against
# f*B*(1/Zn - 1/Zf).
add_executable(occlusion_check
  test/occlusion_check.cpp
  depth_camera.cpp
  voxel_world.cpp)
target_include_directories(occlusion_check PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(occlusion_check PRIVATE ${OpenCV_LIBS})
if(BUILD_TESTS)
  add_test(NAME occlusion_check COMMAND occlusion_check)
endif()

if(BUILD_TESTS)
  # Search-path test for cmake/find_realsense.cmake. Needs no librealsense: it
  # builds synthetic SDK trees and asserts the module locates them. The bug it
  # exists for was an SDK that WAS installed and simply not looked for in the
  # right place, which only a positive-path test can catch.
  add_test(NAME find_realsense_check
           COMMAND ${CMAKE_COMMAND} -P
                   ${CMAKE_CURRENT_SOURCE_DIR}/test/find_realsense_check.cmake)
endif()

# Overlay alignment: the voxel render must land on the same pixels as the depth
# image it was built from. Explicitly rejects a left/right and an up/down flip,
# because on a forest a mirrored render looks entirely plausible.
add_executable(overlay_align_check
  test/overlay_align_check.cpp
  voxel_map.cpp
  depth_camera.cpp
  voxel_world.cpp)
target_include_directories(overlay_align_check PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(overlay_align_check PRIVATE ${OpenCV_LIBS})
if(BUILD_TESTS)
  add_test(NAME overlay_align_check COMMAND overlay_align_check)
endif()
