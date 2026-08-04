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
  voxel_map.cpp
  voxel_planner.cpp)
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

# --- point-and-click front end (needs highgui) --------------------------------
if(OpenCV_highgui_FOUND OR TARGET opencv_highgui)
  add_executable(voxel_gui
    voxel_gui.cpp
    voxel_world.cpp
    depth_camera.cpp
    voxel_map.cpp
    voxel_planner.cpp
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
