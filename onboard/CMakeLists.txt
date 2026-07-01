cmake_minimum_required(VERSION 3.16)
project(kestrel CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(OpenCV REQUIRED COMPONENTS
  core imgproc video videoio highgui tracking dnn)

add_executable(kestrel
  src/main.cpp
  src/world_model.cpp
  src/perception.cpp
  src/scheduler.cpp
  src/road_follow.cpp
  src/fsm.cpp
  src/controller.cpp
  src/state_estimator.cpp
  src/serial_port.cpp
  src/msp_backend.cpp
  src/lock_tracker.cpp
  src/kalman_center.cpp
  src/depth_nav.cpp
  src/i2c_hal.cpp
  src/vl53_tof_source.cpp
  src/mcu_tof_source.cpp)

target_include_directories(kestrel PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${OpenCV_INCLUDE_DIRS})

target_link_libraries(kestrel PRIVATE ${OpenCV_LIBS})

# Cortex-A76 tuning when building natively on the Pi 5.
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-mcpu=cortex-a76" HAS_A76)
if(HAS_A76 AND CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  target_compile_options(kestrel PRIVATE -mcpu=cortex-a76)
endif()
