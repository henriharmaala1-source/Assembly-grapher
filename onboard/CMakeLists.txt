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
  src/lock_tracker.cpp
  src/kalman_center.cpp
  src/depth_nav.cpp
  src/i2c_hal.cpp
  src/vl53_tof_source.cpp
  src/mcu_tof_source.cpp
  src/sim_fc_backend.cpp
  src/deliberator.cpp
  src/mission.cpp
  src/control_mode.cpp
  src/config.cpp
  src/rc_command.cpp
  src/fc_link.cpp
  src/realtime.cpp)

target_include_directories(kestrel PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${OpenCV_INCLUDE_DIRS})

target_link_libraries(kestrel PRIVATE ${OpenCV_LIBS} Threads::Threads)

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
    src/world_model.cpp)
  target_include_directories(sim_autonomy PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(sim_autonomy PRIVATE ${OpenCV_LIBS} Threads::Threads)
  add_test(NAME autonomy_sitl COMMAND sim_autonomy)

  kestrel_test(test_estimator src/state_estimator.cpp)
  kestrel_test(test_modes src/control_mode.cpp src/controller.cpp
               src/mission.cpp src/world_model.cpp)
  kestrel_test(test_msp src/msp_backend.cpp src/serial_port.cpp)
  target_link_libraries(test_msp PRIVATE util)   # openpty
  kestrel_test(test_config src/config.cpp src/controller.cpp
               src/mission.cpp src/control_mode.cpp src/world_model.cpp
               src/rc_command.cpp)
  kestrel_test(test_rc src/rc_command.cpp src/control_mode.cpp
               src/controller.cpp src/mission.cpp src/world_model.cpp)
  kestrel_test(test_fc_link src/fc_link.cpp src/sim_fc_backend.cpp
               src/world_model.cpp src/realtime.cpp)
  kestrel_test(test_realtime src/realtime.cpp)
endif()
