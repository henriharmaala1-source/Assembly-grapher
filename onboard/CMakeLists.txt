cmake_minimum_required(VERSION 3.16)
project(kestrel CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Threads REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS
  core imgproc video videoio highgui tracking dnn)
# calib3d carries StereoBM/StereoSGBM; only the stereo bench needs it, so it is
# looked up separately and the tool is skipped rather than failing the build.
find_package(OpenCV QUIET COMPONENTS calib3d)

add_executable(kestrel
  src/main.cpp
  src/world_model.cpp
  src/perception.cpp
  src/scheduler.cpp
  src/road_follow.cpp
  src/controller.cpp
  src/state_estimator.cpp
  src/serial_port.cpp
  src/msp_backend.cpp
  src/mavlink_v2.cpp
  src/mavlink_backend.cpp
  src/lock_tracker.cpp
  src/kalman_center.cpp
  src/depth_nav.cpp
  src/i2c_hal.cpp
  src/vl53_tof_source.cpp
  src/mcu_tof_source.cpp
  src/sim_fc_backend.cpp
  src/deliberator.cpp
  src/mission.cpp
  src/nav_map.cpp
  src/control_mode.cpp
  src/config.cpp
  src/rc_command.cpp
  src/fc_link.cpp
  src/black_box.cpp
  src/realtime.cpp)

target_include_directories(kestrel PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${OpenCV_INCLUDE_DIRS})

target_link_libraries(kestrel PRIVATE ${OpenCV_LIBS} Threads::Threads)

# Offline black-box decoder — a ground tool, not flight code. Links only the
# format code + world_model (for monoNowS / WorldState), no threads/no camera.
add_executable(blackbox_decode
  tools/blackbox_decode.cpp
  src/black_box.cpp
  src/world_model.cpp)
target_include_directories(blackbox_decode PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(blackbox_decode PRIVATE ${OpenCV_LIBS})

# Pi bring-up harness, stage one: do RC/AUX signals reach the Pi at all?
# READ ONLY on the FC -- it links MavlinkBackend for telemetry and never calls
# sendControl. Keep it that way: this is the tool you run before you trust the
# link, so it must not be able to move anything.
add_executable(rc_probe
  tools/rc_probe.cpp
  src/mavlink_backend.cpp
  src/mavlink_v2.cpp
  src/serial_port.cpp)
target_include_directories(rc_probe PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(rc_probe PRIVATE Threads::Threads)

# A fake ArduPilot on a PTY, so the harness can be developed without hardware.
# Emits a KNOWN pattern (ch1 sweeps, ch7 flicks) so rc_probe has a right answer
# to be checked against, not merely a signal to display.
add_executable(fake_fc tools/fake_fc.cpp src/mavlink_v2.cpp)
target_include_directories(fake_fc PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(fake_fc PRIVATE util)

# Field-test recorder for the lock-on tracker: AUX switch on the FC arms the
# lock, everything is recorded with the overlay burned in plus a CSV.
# READ ONLY on the FC -- it links MspBackend for telemetry and never calls
# sendControl. Keep it that way; control belongs in kestrel behind the arbiter.
add_executable(lockon_rec
  tools/lockon_rec.cpp
  src/lock_tracker_fused.cpp
  src/gray_frame.cpp
  src/crop_filters.cpp
  src/optical_flow.cpp
  src/msp_backend.cpp
  src/serial_port.cpp)
target_include_directories(lockon_rec PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(lockon_rec PRIVATE ${OpenCV_LIBS} Threads::Threads)

# Stereo matching cost on the Pi 5 CPU -- the number the IMX296 bet rests on.
# Needs no cameras and nothing else from the tree; run it on the Pi directly.
include(CheckCXXCompilerFlag)
if(OpenCV_calib3d_FOUND OR TARGET opencv_calib3d)
  add_executable(stereo_bench tools/stereo_bench.cpp)
  target_include_directories(stereo_bench PRIVATE ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(stereo_bench PRIVATE ${OpenCV_LIBS})
  check_cxx_compiler_flag("-mcpu=cortex-a76" HAS_A76_SB)
  if(HAS_A76_SB AND CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    target_compile_options(stereo_bench PRIVATE -mcpu=cortex-a76)
  endif()
endif()

# Cortex-A76 tuning when building natively on the Pi 5.
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-mcpu=cortex-a76" HAS_A76)
if(HAS_A76 AND CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  target_compile_options(kestrel PRIVATE -mcpu=cortex-a76)
endif()

# ---------------------------------------------------------------------------
# Headless software-in-the-loop validation of AUTONOMY (pick-a-direction, GO).
# Compiles the REAL sim FC + estimator + mission against a synthetic obstacle
# field and asserts the drone stays safe (never breaches its standoff) and
# reaches the goal in the scenarios the reactive layer is designed for.
#   cmake -B build -DBUILD_TESTS=ON && cmake --build build && ctest --test-dir build
# ---------------------------------------------------------------------------
option(BUILD_TESTS "Build the SITL validation harness + unit tests" OFF)
if(BUILD_TESTS)
  enable_testing()

  function(kestrel_test name)                 # kestrel_test(<name> <extra srcs...>)
    add_executable(${name} test/${name}.cpp ${ARGN})
    target_include_directories(${name} PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(${name} PRIVATE ${OpenCV_LIBS} Threads::Threads)
    add_test(NAME ${name} COMMAND ${name})
  endfunction()

  add_executable(sim_autonomy
    test/sim_autonomy.cpp
    src/sim_fc_backend.cpp
    src/state_estimator.cpp
    src/mission.cpp
    src/nav_map.cpp
    src/world_model.cpp)
  target_include_directories(sim_autonomy PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(sim_autonomy PRIVATE ${OpenCV_LIBS} Threads::Threads)
  add_test(NAME autonomy_sitl COMMAND sim_autonomy)

  kestrel_test(test_estimator src/state_estimator.cpp)
  kestrel_test(test_modes src/control_mode.cpp src/controller.cpp
               src/mission.cpp src/nav_map.cpp src/world_model.cpp)
  kestrel_test(test_msp src/msp_backend.cpp src/serial_port.cpp)
  target_link_libraries(test_msp PRIVATE util)   # openpty
  # MAVLink codec against golden frames from pymavlink. No serial, no hardware:
  # the frames ARE the specification, so this runs anywhere.
  kestrel_test(test_mavlink src/mavlink_v2.cpp)
  # The backend over a PTY: proves it USES the codec correctly, which is a
  # different failure mode from the codec being right.
  kestrel_test(test_mavlink_backend src/mavlink_backend.cpp src/mavlink_v2.cpp
               src/serial_port.cpp)
  target_link_libraries(test_mavlink_backend PRIVATE util)   # openpty
  kestrel_test(test_config src/config.cpp src/controller.cpp
               src/mission.cpp src/nav_map.cpp src/control_mode.cpp src/world_model.cpp
               src/rc_command.cpp)
  kestrel_test(test_rc src/rc_command.cpp src/control_mode.cpp
               src/controller.cpp src/mission.cpp src/nav_map.cpp src/world_model.cpp)
  kestrel_test(test_nav_map src/nav_map.cpp)
  kestrel_test(test_fc_link src/fc_link.cpp src/sim_fc_backend.cpp
               src/world_model.cpp src/realtime.cpp)
  kestrel_test(test_realtime src/realtime.cpp)
  kestrel_test(test_black_box src/black_box.cpp src/world_model.cpp)

  # The fused tracker is deliberately OpenCV-free, so its tests link nothing but
  # the tracker itself -- they run anywhere a compiler does, including a machine
  # with no OpenCV dev packages. Kept out of kestrel_test() for that reason.
  add_executable(test_lock_tracker_fused
    test/test_lock_tracker_fused.cpp
    src/lock_tracker_fused.cpp
    src/gray_frame.cpp
    src/crop_filters.cpp
    src/optical_flow.cpp)
  target_include_directories(test_lock_tracker_fused PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
  add_test(NAME lock_tracker_fused COMMAND test_lock_tracker_fused)

  # The OLD tracker -- the one perception.hpp still instantiates, and so the
  # one that actually flies. It had no test at all until a review found the
  # template scale-convention bug this pins.
  kestrel_test(test_lock_tracker_reacquire src/lock_tracker.cpp src/kalman_center.cpp)
endif()
