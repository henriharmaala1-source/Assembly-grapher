cmake_minimum_required(VERSION 3.16)
project(rpi5_lockon CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(OpenCV REQUIRED COMPONENTS
  core imgproc video videoio highgui tracking dnn)

add_executable(lockon
  src/main.cpp
  src/lock_tracker.cpp
  src/kalman_center.cpp
  src/depth_nav.cpp)

target_link_libraries(lockon PRIVATE ${OpenCV_LIBS})
target_include_directories(lockon PRIVATE ${OpenCV_INCLUDE_DIRS})

# Cortex-A76 tuning when building natively on the Pi 5.
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-mcpu=cortex-a76" HAS_A76)
if(HAS_A76 AND CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  target_compile_options(lockon PRIVATE -mcpu=cortex-a76)
endif()
