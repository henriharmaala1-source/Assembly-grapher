cmake_minimum_required(VERSION 3.16)
project(nav_sim CXX)

# The simulator, the planner baselines, the trainer's environment and the
# `kestrel` desk tool. NOT self-contained any more, on purpose: the navigation
# core it simulates is ../navcore, the one copy the aircraft also flies, and the
# tracker is ../onboard/src. A second copy of either is how the two trees broke
# each other before.

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)
find_package(OpenCV QUIET COMPONENTS highgui)
find_package(OpenCV QUIET OPTIONAL_COMPONENTS videoio)

# THE NAVIGATION CORE lives in ../navcore and is shared with onboard, which is
# the whole point of it: the pipeline voxel_live runs is the pipeline the
# aircraft runs. Every target below links it instead of listing its sources.
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../navcore
                 ${CMAKE_CURRENT_BINARY_DIR}/navcore)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../navcore)
# OPTIONAL, AND THE DEMO DEGRADES RATHER THAN FAILING. objdetect carries the
# trained HOG people detector, so it is the one that needs no model file; dnn
# runs an exported policy and an ONNX person detector when either is supplied.
# Both ship with almost every OpenCV build, and neither is worth making the
# whole tree un-buildable over -- `demo` names on screen what it could not load.
find_package(OpenCV QUIET OPTIONAL_COMPONENTS objdetect)
find_package(OpenCV QUIET OPTIONAL_COMPONENTS dnn)

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
# Headless renderer check. No window needed, so it runs in CI and over ssh --
# which matters, because the defect it exists to catch (one-pixel cubes) is
# invisible in every other test and obvious in its output.
add_executable(iso_render_check
  test/iso_render_check.cpp

)
target_link_libraries(iso_render_check PRIVATE navcore)
target_include_directories(iso_render_check PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(iso_render_check PRIVATE ${OpenCV_LIBS})

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

# THE DEMO WORLDS ARE ON THE LATTICE, asserted rather than eyeballed. gallery
# and hall exist to make the first-person voxel view consistent across the
# 2.0/1.0/0.25 m ladder, and that rests on one geometric claim: every solid
# voxel fills a whole 2 m lattice cell. A cell that is part solid and part air
# is a wall that MOVES as you fly at it, and at 2 m voxels the defect is
# invisible until you are close -- which is when a demo is being watched. This
# found two off-by-one errors that the render did not.
add_executable(gallery_lattice_check
  test/gallery_lattice_check.cpp
)
target_link_libraries(gallery_lattice_check PRIVATE navcore)
target_include_directories(gallery_lattice_check PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(gallery_lattice_check PRIVATE ${OpenCV_LIBS})
if(BUILD_TESTS)
  add_test(NAME gallery_lattice_check COMMAND gallery_lattice_check)
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

# Debian and Ubuntu install omplConfig.cmake to /usr/share/ompl/cmake, which is
# NOT on CMake's default search path -- so `find_package(ompl QUIET)` reported
# "no OMPL" on a machine where libompl-dev was installed, and the forward
# planner silently fell back to a straight-line probe.
#
# That failure is quiet in exactly the wrong way. The run still completes and
# still prints a forward-planner cost, so a sweep reads as "the global planner
# did not help" when the global planner never ran: measured here at 0.02 ms per
# replan against RRTConnect's 1.0 ms, which is the tell if anyone looks.
#
# HINTS rather than a hard path, so a source build or a different prefix still
# wins. The status line now names which planner is actually compiled in.
find_package(ompl QUIET HINTS /usr/share/ompl/cmake /usr/local/share/ompl/cmake)
if(ompl_FOUND)
  message(STATUS "nav_sim: OMPL found - forward planner uses RRTConnect")
else()
  message(STATUS "nav_sim: no OMPL - forward planner falls back to a straight-line probe"
                 " (Debian/Ubuntu: apt install libompl-dev libboost-serialization-dev)")
endif()

add_executable(voxel_sim
  voxel_sim.cpp
  ompl_planner.cpp

  depth_improve.cpp

)
target_link_libraries(voxel_sim PRIVATE navcore)
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


)
target_link_libraries(gui_preview PRIVATE navcore)
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



    ompl_planner.cpp)
  target_link_libraries(voxel_gui PRIVATE navcore)
  target_include_directories(voxel_gui PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(voxel_gui PRIVATE ${OpenCV_LIBS})
  if(ompl_FOUND)
    target_compile_definitions(voxel_gui PRIVATE NAVSIM_HAVE_OMPL=1)
    target_include_directories(voxel_gui PRIVATE ${OMPL_INCLUDE_DIRS})
    target_link_libraries(voxel_gui PRIVATE ${OMPL_LIBRARIES})
  endif()
  message(STATUS "nav_sim: voxel_gui enabled")
endif()

# --- the live sim -----------------------------------------------------------
# voxel_live.cpp is compiled into kestrel only (`kestrel sim`). It used to be
# built a second time as a standalone voxel_live executable -- the same source,
# the same flags, two binaries -- which is a thing to keep in sync and nothing
# else. librealsense is loaded at RUN time (realsense_dyn.hpp), so no target
# here has a build-time SDK dependency.

# .kdr recording format: round trip, truncation, refusals, and that the replay
# source rebuilds a camera from the file's own intrinsics.
add_executable(depth_record_check
  test/depth_record_check.cpp


)
target_link_libraries(depth_record_check PRIVATE navcore)
target_include_directories(depth_record_check PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(depth_record_check PRIVATE ${OpenCV_LIBS})
if(NOT WIN32)
  target_link_libraries(depth_record_check PRIVATE ${CMAKE_DL_LIBS})
endif()
if(BUILD_TESTS)
  add_test(NAME depth_record_check COMMAND depth_record_check)
  # Python writes / C++ reads, and back. Needs the C++ test to have run first,
  # so it reads the file that one wrote.
  find_package(Python3 COMPONENTS Interpreter QUIET)
  if(Python3_FOUND)
    add_test(NAME kdr_crosscheck
             COMMAND ${Python3_EXECUTABLE}
                     ${CMAKE_CURRENT_SOURCE_DIR}/test/kdr_crosscheck.py
                     $<TARGET_FILE:kestrel> sim)
    set_tests_properties(kdr_crosscheck PROPERTIES DEPENDS depth_record_check)
  endif()
endif()

# Stereo occlusion shadow: asserts the SIDE (a mirrored shadow would look
# plausible and bias every obstacle boundary) and the WIDTH against
# f*B*(1/Zn - 1/Zf).
add_executable(occlusion_check
  test/occlusion_check.cpp
)
target_link_libraries(occlusion_check PRIVATE navcore)
target_include_directories(occlusion_check PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(occlusion_check PRIVATE ${OpenCV_LIBS})
if(BUILD_TESTS)
  add_test(NAME occlusion_check COMMAND occlusion_check)
endif()


# Overlay alignment: the voxel render must land on the same pixels as the depth
# image it was built from. Explicitly rejects a left/right and an up/down flip,
# because on a forest a mirrored render looks entirely plausible.
add_executable(overlay_align_check
  test/overlay_align_check.cpp

)
target_link_libraries(overlay_align_check PRIVATE navcore)
target_include_directories(overlay_align_check PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(overlay_align_check PRIVATE ${OpenCV_LIBS})
if(BUILD_TESTS)
  add_test(NAME overlay_align_check COMMAND overlay_align_check)
endif()

# The far-field bearing field: confirmation, the running-minimum trap, the
# sample floor (test/bearing_field_check.cpp says why each one matters).
add_executable(bearing_field_check
  test/bearing_field_check.cpp

)
target_link_libraries(bearing_field_check PRIVATE navcore)
target_include_directories(bearing_field_check PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(bearing_field_check PRIVATE ${OpenCV_LIBS})
add_test(NAME bearing_field_check COMMAND bearing_field_check)

# Velocity from optical flow + depth, against known motion.
add_executable(flow_velocity_check
  test/flow_velocity_check.cpp
  flow_velocity.cpp
)
target_link_libraries(flow_velocity_check PRIVATE navcore)
target_include_directories(flow_velocity_check PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(flow_velocity_check PRIVATE ${OpenCV_LIBS})
add_test(NAME flow_velocity_check COMMAND flow_velocity_check)

# Python bindings, optional: the C++ tree builds and tests without them.
# pybind11's newer CMake needs Python found with Development.Module first,
# otherwise pybind11_add_module fails on an unknown python3_add_library.
find_package(Python3 COMPONENTS Interpreter Development.Module QUIET)
find_package(pybind11 QUIET)
if(pybind11_FOUND)
  pybind11_add_module(voxelenv
    python/voxelenv_py.cpp
    kestrel_report.cpp
    rl_env.cpp
)
  target_link_libraries(voxelenv PRIVATE navcore)
  target_include_directories(voxelenv PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(voxelenv PRIVATE ${OpenCV_LIBS})
  message(STATUS "nav_sim: pybind11 found - building the voxelenv python module")
else()
  message(STATUS "nav_sim: no pybind11 - skipping the voxelenv python module"
                 " (pip install pybind11, then -Dpybind11_DIR=$(python3 -m pybind11 --cmakedir))")
endif()

# ---------------------------------------------------------------------------
# CUDA depth render. OFF by default and it must stay that way: the .cu kernel
# has never been compiled by its author, and cuda_depth_check is the gate that
# decides whether it may be trusted on a given machine.
#
# Only the RENDERER may leave the CPU. The map, the planner and the veto are
# the code that flies (see frame_source.hpp) and a second GPU implementation of
# them would drift from what is deployed.
option(NAVSIM_WITH_CUDA "Build the CUDA depth renderer (needs a toolkit)" OFF)
set(NAVSIM_DEPTH_CUDA_SRC depth_cuda.cpp)
set(NAVSIM_CUDA_LIBS "")
if(NAVSIM_WITH_CUDA)
  include(CheckLanguage)
  check_language(CUDA)
  if(CMAKE_CUDA_COMPILER)
    enable_language(CUDA)
    set(NAVSIM_DEPTH_CUDA_SRC depth_cuda.cu)
    add_compile_definitions(NAVSIM_HAVE_CUDA=1)
    message(STATUS "nav_sim: CUDA depth renderer ENABLED - run cuda_depth_check before trusting it")
  else()
    message(WARNING "nav_sim: NAVSIM_WITH_CUDA=ON but no CUDA compiler found; using the CPU renderer")
  endif()
endif()

add_executable(cuda_depth_check
  test/cuda_depth_check.cpp
  ${NAVSIM_DEPTH_CUDA_SRC}
)
target_link_libraries(cuda_depth_check PRIVATE navcore)
target_include_directories(cuda_depth_check PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(cuda_depth_check PRIVATE ${OpenCV_LIBS} ${NAVSIM_CUDA_LIBS})
add_test(NAME cuda_depth_check COMMAND cuda_depth_check)

# ---------------------------------------------------------------------------
# kestrel -- one binary over the three things this project can demonstrate.
# It reaches into ../onboard for the tracker, which is the only place these two
# trees are linked together; everything else stays separate on purpose.
#
# videoio is OPTIONAL, exactly as highgui already is. Without it `track` still
# runs over an IMAGE SEQUENCE, which is deterministic and diffable where a video
# decode is neither -- the better thing to regression-test against anyway.
# NO second find_package here. Re-running it with a COMPONENTS list that the
# install does not have FAILS HARD and clears OpenCV_LIBS for everything below,
# so a missing videoio took the whole configure down with it. The find at the
# top of this file already ran; just ask whether the target came out of it.
# What the aircraft decides with, from onboard/src, verbatim. perception_module.hpp
# keeps OpenCV contrib out of it, so the Windows build (prebuilt OpenCV) compiles
# it too.
set(KESTREL_AUTONOMY_SRC
  ${CMAKE_CURRENT_SOURCE_DIR}/../onboard/src/voxel_nav.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../onboard/src/mission.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../onboard/src/nav_map.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../onboard/src/world_model.cpp)
add_executable(kestrel
  kestrel.cpp
  kestrel_gui.cpp
  kestrel_report.cpp
  kestrel_python.cpp
  # The live sim, compiled IN rather than spawned: `kestrel sim`.
  voxel_live.cpp
  # The demo: four panes, four threads, no second implementation of anything.
  kestrel_demo.cpp
  # People in the camera image: built-in YOLOX-nano, HOG, or an .onnx file.
  person_detector.cpp
  # The showcase flight: the AIRCRAFT'S OWN voxel module and mission, compiled
  # from onboard/src -- not a copy -- flown against a simulated D435i.
  flight_show.cpp
  # ... and its pictures, drawn from snapshots of it; read-only.
  flight_view.cpp
  ${KESTREL_AUTONOMY_SRC}

  rl_env.cpp
  ../onboard/src/lock_tracker_fused.cpp
  ../onboard/src/crop_filters.cpp
  ../onboard/src/gray_frame.cpp
  ../onboard/src/optical_flow.cpp)
target_link_libraries(kestrel PRIVATE navcore)
target_include_directories(kestrel PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/../onboard/include
  ${OpenCV_INCLUDE_DIRS})
target_link_libraries(kestrel PRIVATE ${OpenCV_LIBS} ${CMAKE_DL_LIBS})
# The showcase maps flown by the real stack: no collision, and it gets somewhere.
if(BUILD_TESTS)
  add_executable(flight_show_check test/flight_show_check.cpp flight_show.cpp
                 ${KESTREL_AUTONOMY_SRC})
  target_include_directories(flight_show_check PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/../onboard/include
    ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(flight_show_check PRIVATE navcore ${OpenCV_LIBS})
  add_test(NAME flight_show_check COMMAND flight_show_check)
endif()
if(OpenCV_objdetect_FOUND OR TARGET opencv_objdetect)
  target_compile_definitions(kestrel PRIVATE KESTREL_HAVE_OBJDETECT=1)
  message(STATUS "kestrel: objdetect found - demo can detect people")
else()
  message(STATUS "kestrel: no objdetect - demo's people pane says so on screen")
endif()
if(OpenCV_dnn_FOUND OR TARGET opencv_dnn)
  target_compile_definitions(kestrel PRIVATE KESTREL_HAVE_DNN=1)
  # YOLOX-nano (Megvii, Apache-2.0 -- models/LICENSE-YOLOX) COMPILED IN, so the
  # demo finds people with no model file beside the exe. The .onnx is turned
  # into a source file at build time; 3.7 MB on the binary.
  set(YOLOX_MODEL ${CMAKE_CURRENT_SOURCE_DIR}/models/yolox_nano.onnx)
  set(YOLOX_CPP ${CMAKE_CURRENT_BINARY_DIR}/embedded_yolox.cpp)
  add_custom_command(OUTPUT ${YOLOX_CPP}
    COMMAND ${CMAKE_COMMAND} -DIN=${YOLOX_MODEL} -DOUT=${YOLOX_CPP} -DNS=kdemo
            -DSYM=yoloxNano -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/embed_file.cmake
    DEPENDS ${YOLOX_MODEL} ${CMAKE_CURRENT_SOURCE_DIR}/cmake/embed_file.cmake
    COMMENT "Embedding YOLOX-nano into kestrel")
  target_sources(kestrel PRIVATE ${YOLOX_CPP})
  target_compile_definitions(kestrel PRIVATE KESTREL_EMBED_YOLOX=1)
  # The detector's decoders against known answers, and the embedded model.
  if(BUILD_TESTS)
    add_executable(person_detector_check test/person_detector_check.cpp
                   person_detector.cpp ${YOLOX_CPP})
    target_include_directories(person_detector_check PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR} ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(person_detector_check PRIVATE ${OpenCV_LIBS})
    target_compile_definitions(person_detector_check PRIVATE
      KESTREL_HAVE_DNN=1 KESTREL_EMBED_YOLOX=1)
    if(OpenCV_objdetect_FOUND OR TARGET opencv_objdetect)
      target_compile_definitions(person_detector_check PRIVATE KESTREL_HAVE_OBJDETECT=1)
    endif()
    add_test(NAME person_detector_check COMMAND person_detector_check)
  endif()
  message(STATUS "kestrel: dnn found - demo can fly an exported .onnx policy")
else()
  message(STATUS "kestrel: no dnn - demo flies a classical planner, and says so")
endif()
# TRAINING IS THE ONE COMMAND THAT LEAVES THE PROCESS, and embedding CPython
# would not change that. The trainer is PyTorch and stable-baselines3: linking
# libpython in still requires that exact interpreter, its ABI and its
# site-packages to be present, so the dependency is identical and the failure
# modes are worse -- a version mismatch becomes a link error or a crash instead
# of a message naming the pip command. So kestrel calls python3 for `train` and
# checks the imports first (see cmdTrain), and does everything else itself.
# highgui gates the WINDOW only. Without it kestrel still runs every command;
# `gui` and a bare double-click fall back to the text menu and say why.
if(OpenCV_highgui_FOUND OR TARGET opencv_highgui)
  target_compile_definitions(kestrel PRIVATE SIM_HAVE_HIGHGUI=1)
  message(STATUS "nav_sim: kestrel built WITH highgui - the window is available")
else()
  message(STATUS "nav_sim: kestrel built without highgui - text menu only")
endif()
# The window is the one thing in this binary that cannot be inspected over ssh,
# so it gets a headless layout check the way the sim's flight view already has
# gui_preview. `kestrel gui --shot PREFIX` writes the panels to PNG; --check
# asserts nothing overlaps, runs off the canvas, or is squeezed unreadable.
if(BUILD_TESTS AND (OpenCV_highgui_FOUND OR TARGET opencv_highgui))
  add_test(NAME kestrel_gui_check COMMAND kestrel gui --check)
endif()

# The failure report has the same problem the window has -- it is a picture, so
# nobody reviews it over ssh -- and gets the same answer. `report --check` lays
# out every panel from synthetic records covering all ten outcome classes and
# asserts the things that go wrong: text off the canvas, text over text, a class
# with no colour of its own, a class no input can reach, and a panel that came
# out uniformly flat because nothing was drawn. No python, no checkpoint, no
# flying, so it runs in CI beside the GUI check.
if(BUILD_TESTS)
  add_test(NAME kestrel_report_check COMMAND kestrel report --check)
endif()

# The demo's layout, asserted with no camera and no display -- four panes that
# do not overlap, fit the canvas, clear the status strip, and captions that
# survive elision with their first word intact. That last rule is what stops a
# pane captioned "classical: cover" from rendering as "..." and claiming to be
# the learned policy by omission.
if(BUILD_TESTS)
  add_test(NAME kestrel_demo_check COMMAND kestrel demo --check)
endif()

if(TARGET opencv_videoio)
  target_compile_definitions(kestrel PRIVATE KESTREL_HAVE_VIDEOIO=1)
  message(STATUS "nav_sim: kestrel built WITH videoio - `track` accepts video files")
else()
  message(STATUS "nav_sim: kestrel built without videoio - `track` takes image sequences")
endif()

add_executable(loop_closure_check
  test/loop_closure_check.cpp
  place_descriptor.cpp)
target_link_libraries(loop_closure_check PRIVATE navcore)
target_include_directories(loop_closure_check PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(loop_closure_check PRIVATE ${OpenCV_LIBS})
add_test(NAME loop_closure_check COMMAND loop_closure_check)

add_executable(rl_env_check
  test/rl_env_check.cpp
  rl_env.cpp


)
target_link_libraries(rl_env_check PRIVATE navcore)
target_include_directories(rl_env_check PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(rl_env_check PRIVATE ${OpenCV_LIBS})
add_test(NAME rl_env_check COMMAND rl_env_check)

add_executable(lookaround_check
  test/lookaround_check.cpp

)
target_link_libraries(lookaround_check PRIVATE navcore)
target_include_directories(lookaround_check PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(lookaround_check PRIVATE ${OpenCV_LIBS})
add_test(NAME lookaround_check COMMAND lookaround_check)

add_executable(attitude_check test/attitude_check.cpp)
target_include_directories(attitude_check PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
add_test(NAME attitude_check COMMAND attitude_check)

add_executable(scan_match_check
  test/scan_match_check.cpp

)
target_link_libraries(scan_match_check PRIVATE navcore)
target_include_directories(scan_match_check PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(scan_match_check PRIVATE ${OpenCV_LIBS})
add_test(NAME scan_match_check COMMAND scan_match_check)

add_executable(flow_odometry_check
  test/flow_odometry_check.cpp
  flow_velocity.cpp
)
target_link_libraries(flow_odometry_check PRIVATE navcore)
target_include_directories(flow_odometry_check PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(flow_odometry_check PRIVATE ${OpenCV_LIBS})
add_test(NAME flow_odometry_check COMMAND flow_odometry_check)

add_executable(imu_odometry_check test/imu_odometry_check.cpp)
target_include_directories(imu_odometry_check PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
add_test(NAME imu_odometry_check COMMAND imu_odometry_check)

# "The planner only wants to go up now, because it's the most open." Unknown
# bearings were scoring as maximally open, so the aircraft steered toward
# whatever it could not see -- which outdoors is the sky.
add_executable(sky_open_check
  test/sky_open_check.cpp


)
target_link_libraries(sky_open_check PRIVATE navcore)
target_include_directories(sky_open_check PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(sky_open_check PRIVATE ${OpenCV_LIBS})
add_test(NAME sky_open_check COMMAND sky_open_check)

# Chase view: a path with fixed BODY shape must land on the same pixels at every
# heading. The two panes that show the plan use different references -- the
# top-down one is world-referenced and does not rotate -- so they cannot be
# checked against each other by eye, and each of the three yaw conventions in
# the chain could invert without any single view looking wrong.
add_executable(chase_turn_check
  test/chase_turn_check.cpp


)
target_link_libraries(chase_turn_check PRIVATE navcore)
target_include_directories(chase_turn_check PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(chase_turn_check PRIVATE ${OpenCV_LIBS})
if(BUILD_TESTS)
  add_test(NAME chase_turn_check COMMAND chase_turn_check)
endif()

# ---------------------------------------------------------------------------
# ONE EXECUTABLE. Everything above this line is a target, and a plain
# `cmake --build build` used to drop a dozen binaries next to each other --
# voxel_live, voxel_sim, rl_bench, gui_preview and ten *_check programs. That is
# a directory of executables, not a deliverable, and it is impossible to tell
# from the outside which one a reviewer is supposed to run.
#
# So the default build produces exactly ONE: kestrel. It has the live sim, the
# tracker and the planner baselines compiled into it, so nothing that was
# reachable before became unreachable -- the entry point moved from a filename
# to a subcommand.
#
# The rest are not deleted, they are taken off the `all` target. Each is still
# buildable by name (`cmake --build build --target scan_match_check`), and the
# test suite still builds all of them, because ctest depends on the binaries:
#   -DBUILD_TESTS=ON   builds and registers everything
#   -DNAV_SIM_ALL_TARGETS=ON  builds everything without turning on ctest
option(NAV_SIM_ALL_TARGETS "Build every tool and check, not just kestrel" OFF)
if(NOT NAV_SIM_ALL_TARGETS AND NOT BUILD_TESTS)
  get_property(_nav_sim_targets DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
               PROPERTY BUILDSYSTEM_TARGETS)
  foreach(_t IN LISTS _nav_sim_targets)
    # voxelenv is the python extension `kestrel train` imports; it is part of
    # the deliverable even though it is not an executable.
    if(NOT _t STREQUAL "kestrel" AND NOT _t STREQUAL "voxelenv")
      set_target_properties(${_t} PROPERTIES EXCLUDE_FROM_ALL TRUE)
    endif()
  endforeach()
  message(STATUS "nav_sim: default build produces ONE executable (kestrel);"
                 " -DNAV_SIM_ALL_TARGETS=ON or -DBUILD_TESTS=ON for the rest")
endif()
