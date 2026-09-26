# ---------------------------------------------------------------------------
# navcore -- THE NAVIGATION CORE, and the one copy of it.
#
# Depth in, a flyable direction out: the camera model, the frame sources
# (sim / replay / live RealSense), the three-rung voxel map, the far-field
# bearing field, the primitive library and the swept-volume veto. This is the
# pipeline voxel_live runs, and it is the pipeline that FLIES -- onboard links
# this library rather than carrying a second navigation stack beside it.
#
# WHY A LIBRARY. These files used to be listed by name in every nav-sim target
# that needed them: voxel_world.cpp was compiled separately into 24 of them,
# depth_camera.cpp into 23. Each target was free to pick a different subset,
# and onboard could not use any of it without reaching across into nav-sim's
# source tree -- which is exactly how a header change in one tree broke the
# other twice without either tree's own tests noticing.
#
# WHAT IS DELIBERATELY NOT HERE: rendering windows (highgui), the OMPL global
# planner, the CUDA depth renderer, the RL environment and the tracker. The
# core needs OpenCV core + imgproc and nothing else, so it builds on a Pi 5 as
# it stands. voxel_world IS here, because depth_camera and frame_source take a
# VoxelWorld for their simulated sources; it is inert on the aircraft.
#
# The only conditional compilation inside is realsense_dyn's dlopen/LoadLibrary
# -- a one-line platform call, the exception CLAUDE.md allows. No per-target
# switches, so compiling once is behaviour-identical to compiling per target.
# ---------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.16)

if(NOT TARGET navcore)
  # video (pyramidal LK) is for vio.cpp. It ships in the Pi's apt OpenCV and
  # in the Windows prebuilt, so the core still builds everywhere it did.
  find_package(OpenCV REQUIRED COMPONENTS core imgproc video)

  add_library(navcore STATIC
    voxel_world.cpp
    depth_camera.cpp
    depth_record.cpp
    realsense_dyn.cpp
    frame_source.cpp
    voxel_map.cpp
    bearing_field.cpp
    voxel_planner.cpp
    voxel_traj.cpp
    scan_match.cpp
    depth_vis.cpp
    nav_pipeline.cpp
    footage.cpp
    vio.cpp
    # External SLAM over a local socket (kestrel-orbslam), and the one pose
    # layer onboard and the desktop tools share.
    slam_link.cpp
    slam_client.cpp
    visual_pose.cpp)

  target_compile_features(navcore PUBLIC cxx_std_17)
  # PUBLIC, so anything that links navcore finds its headers by basename --
  # the way every file in both trees already includes them.
  target_include_directories(navcore PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR} ${OpenCV_INCLUDE_DIRS})
  find_package(Threads REQUIRED)
  target_link_libraries(navcore PUBLIC ${OpenCV_LIBS} ${CMAKE_DL_LIBS} Threads::Threads)
  if(WIN32)
    target_link_libraries(navcore PUBLIC ws2_32)   # slam_link.cpp's sockets
  endif()
  # The python extension module is a shared object that links this, so the
  # objects must be position-independent. It costs nothing measurable.
  set_target_properties(navcore PROPERTIES POSITION_INDEPENDENT_CODE ON)
endif()
