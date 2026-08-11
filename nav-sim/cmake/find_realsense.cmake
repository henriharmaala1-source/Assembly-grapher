# Locate librealsense, including where the Windows installer actually puts it.
#
# A bare find_package(realsense2) FAILS on a machine where the SDK is correctly
# installed, and that is not a mistake anyone makes only once. CMake searches
# <prefix>/lib/cmake/realsense2 for prefixes such as "C:/Program Files (x86)",
# while the installer puts the package at
#
#     C:/Program Files (x86)/Intel RealSense SDK 2.0/lib/cmake/realsense2
#
# — one directory deeper than anything CMake looks in. So the SDK is present,
# the Viewer works, and the build still reports "no RealSense SDK", which sends
# people off reinstalling something they already have.
#
# Two ways in: hint the real install locations for the config package, then fall
# back to locating the header and the library by hand. Sets realsense2_FOUND and,
# on success, the imported target realsense2::realsense2.
#
# Kept in its own file so test/find_realsense_check.cmake can exercise THIS
# logic against a synthetic SDK tree rather than a copy of it — the search paths
# are the part that is wrong today, so they are the part that needs a test.

set(_rs_hints
    "$ENV{REALSENSE2_DIR}"
    "$ENV{ProgramW6432}/Intel RealSense SDK 2.0"
    "$ENV{ProgramFiles}/Intel RealSense SDK 2.0"
    "C:/Program Files (x86)/Intel RealSense SDK 2.0"
    "C:/Program Files/Intel RealSense SDK 2.0"
    "/usr/local"
    "/usr"
    "/opt/librealsense")

# Preferred: the config package the SDK ships, which brings the right compile
# definitions and transitive dependencies with it.
find_package(realsense2 QUIET CONFIG
             HINTS ${_rs_hints}
             PATHS ${_rs_hints}
             PATH_SUFFIXES lib/cmake/realsense2 cmake/realsense2)

# Fallback: a header and a library is all this project actually needs. This
# covers a source build, an unpacked archive, and any install whose config
# package is missing or unreadable.
if(NOT realsense2_FOUND)
  find_path(RS2_INCLUDE_DIR librealsense2/rs.hpp
            HINTS ${_rs_hints}
            PATH_SUFFIXES include)
  find_library(RS2_LIBRARY
               NAMES realsense2 realsense2.lib librealsense2
               HINTS ${_rs_hints}
               PATH_SUFFIXES lib lib/x64 lib64 bin/x64)
  if(RS2_INCLUDE_DIR AND RS2_LIBRARY)
    # add_library is not scriptable, and this module is include()d by
    # test/find_realsense_check.cmake under `cmake -P` to exercise the search
    # paths without configuring the whole project. In script mode the variables
    # ARE the result; the imported target is only needed for a real build.
    if(NOT CMAKE_SCRIPT_MODE_FILE AND NOT TARGET realsense2::realsense2)
      add_library(realsense2::realsense2 UNKNOWN IMPORTED)
      set_target_properties(realsense2::realsense2 PROPERTIES
        IMPORTED_LOCATION "${RS2_LIBRARY}"
        INTERFACE_INCLUDE_DIRECTORIES "${RS2_INCLUDE_DIR}")
    endif()
    set(realsense2_FOUND TRUE)
    message(STATUS "nav_sim: librealsense found by hand -- ${RS2_LIBRARY}")
  endif()
endif()
