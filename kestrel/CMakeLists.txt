cmake_minimum_required(VERSION 3.16)
project(kestrel CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(OpenCV REQUIRED COMPONENTS
  core imgproc video videoio highgui tracking dnn)

# Reuse the rpi5_tracker perception sources directly.
set(RPI5 ${CMAKE_CURRENT_SOURCE_DIR}/../rpi5_tracker/src)

add_executable(kestrel
  src/main.cpp
  src/world_model.cpp
  src/perception.cpp
  src/scheduler.cpp
  src/road_follow.cpp
  src/fsm.cpp
  src/controller.cpp
  src/serial_port.cpp
  src/msp_backend.cpp
  ${RPI5}/lock_tracker.cpp
  ${RPI5}/kalman_center.cpp
  ${RPI5}/depth_nav.cpp)

target_include_directories(kestrel PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${RPI5}
  ${OpenCV_INCLUDE_DIRS})

target_link_libraries(kestrel PRIVATE ${OpenCV_LIBS})

# Cortex-A76 tuning when building natively on the Pi 5.
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-mcpu=cortex-a76" HAS_A76)
if(HAS_A76 AND CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  target_compile_options(kestrel PRIVATE -mcpu=cortex-a76)
endif()
