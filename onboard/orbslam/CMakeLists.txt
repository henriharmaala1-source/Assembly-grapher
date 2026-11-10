# kestrel-orbslam: ORB-SLAM3 in its own process (GPLv3), see README.md.
#
#   ./fetch_orbslam3.sh                       # pinned, headless ORB-SLAM3
#   cmake -S . -B build -DORB_SLAM3_DIR=third_party/ORB_SLAM3
#   cmake --build build
#
# Deliberately NOT part of onboard's own build: onboard must build, test and
# fly without it, and linking it would put onboard under the GPL.
cmake_minimum_required(VERSION 3.16)
project(kestrel_orbslam CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(ORB_SLAM3_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/ORB_SLAM3"
    CACHE PATH "ORB-SLAM3 source tree, built by fetch_orbslam3.sh")
if(NOT EXISTS "${ORB_SLAM3_DIR}/lib/libORB_SLAM3.so")
  message(FATAL_ERROR "No ORB-SLAM3 at ${ORB_SLAM3_DIR}: run fetch_orbslam3.sh first")
endif()

find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc features2d calib3d)
find_package(Eigen3 3.1 REQUIRED NO_MODULE)

add_executable(kestrel-orbslam
  kestrel_orbslam.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../../navcore/slam_link.cpp)
target_include_directories(kestrel-orbslam PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../../navcore
  ${CMAKE_CURRENT_SOURCE_DIR}/headless
  ${ORB_SLAM3_DIR}
  ${ORB_SLAM3_DIR}/include
  ${ORB_SLAM3_DIR}/include/CameraModels
  ${ORB_SLAM3_DIR}/Thirdparty/Sophus)
target_link_libraries(kestrel-orbslam PRIVATE
  ${ORB_SLAM3_DIR}/lib/libORB_SLAM3.so
  ${ORB_SLAM3_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
  ${ORB_SLAM3_DIR}/Thirdparty/g2o/lib/libg2o.so
  ${OpenCV_LIBS} Eigen3::Eigen boost_serialization crypto pthread)
set_target_properties(kestrel-orbslam PROPERTIES
  BUILD_RPATH "${ORB_SLAM3_DIR}/lib;${ORB_SLAM3_DIR}/Thirdparty/DBoW2/lib;${ORB_SLAM3_DIR}/Thirdparty/g2o/lib"
  INSTALL_RPATH "${ORB_SLAM3_DIR}/lib;${ORB_SLAM3_DIR}/Thirdparty/DBoW2/lib;${ORB_SLAM3_DIR}/Thirdparty/g2o/lib")
