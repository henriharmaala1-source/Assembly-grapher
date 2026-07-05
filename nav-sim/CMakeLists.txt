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
  planners.cpp)

target_include_directories(nav_sim PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(nav_sim PRIVATE ${OpenCV_LIBS})

if(OpenCV_highgui_FOUND OR TARGET opencv_highgui)
  target_compile_definitions(nav_sim PRIVATE SIM_HAVE_HIGHGUI=1)
  message(STATUS "nav_sim: highgui found — --display enabled")
else()
  message(STATUS "nav_sim: no highgui — headless only (--save= PNG dumps still work)")
endif()
