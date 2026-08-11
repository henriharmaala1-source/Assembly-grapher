# Locate librealsense, including where the installer actually puts it.
#
# A bare find_package(realsense2) FAILS on a machine where the SDK is correctly
# installed. CMake searches <prefix>/lib/cmake/realsense2 for prefixes such as
# "C:/Program Files (x86)", while the installer puts the package one directory
# deeper, inside its own product folder. So the SDK is present, the Viewer
# works, and the build reports it missing.
#
# WHY THIS GLOBS RATHER THAN HARD-CODING THE PATH. The product folder name is
# not something to guess at: the project moved from Intel to RealSense AI, the
# Windows installer asset was renamed from Intel.RealSense.SDK-WIN10-<ver>.exe
# to RealSense.SDK.exe, and the install directory has every reason to have
# changed with it. A hard-coded "Intel RealSense SDK 2.0" is a guess that fails
# silently and looks exactly like a missing SDK. So: glob every Program Files
# directory whose name mentions RealSense, whatever it is called this year.
#
# AND IT SAYS WHERE IT LOOKED. A find module that reports only "not found" makes
# every failure a round trip. This one prints its candidate roots, so the next
# person can see immediately whether their install is simply somewhere else.
#
# Sets realsense2_FOUND and, on success, the imported target
# realsense2::realsense2. Exercised without librealsense by
# test/find_realsense_check.cmake.

set(_rs_hints "")

# A CMake variable first, so -DREALSENSE2_DIR=... works. This is the form a
# person reaches for, and reading only the ENVIRONMENT variable made the
# obvious command line silently do nothing -- the exact failure this module
# exists to stop.
if(REALSENSE2_DIR)
  file(TO_CMAKE_PATH "${REALSENSE2_DIR}" _p)
  list(APPEND _rs_hints "${_p}")
endif()

# Environment next, and through TO_CMAKE_PATH: env vars come back with
# backslashes on Windows, and "C:\Program Files/Intel..." is a string CMake is
# entitled to mangle.
foreach(_v REALSENSE2_DIR ProgramW6432 ProgramFiles "ProgramFiles(x86)")
  if(DEFINED ENV{${_v}})
    file(TO_CMAKE_PATH "$ENV{${_v}}" _p)
    if(_v STREQUAL "REALSENSE2_DIR")
      list(APPEND _rs_hints "${_p}")
    else()
      # Any product folder mentioning RealSense, whatever it is called.
      file(GLOB _found LIST_DIRECTORIES true "${_p}/*RealSense*" "${_p}/*realsense*")
      list(APPEND _rs_hints ${_found})
    endif()
  endif()
endforeach()

# The usual literal locations, in case the environment is not set.
foreach(_root "C:/Program Files" "C:/Program Files (x86)")
  file(GLOB _found LIST_DIRECTORIES true "${_root}/*RealSense*" "${_root}/*realsense*")
  list(APPEND _rs_hints ${_found})
endforeach()

list(APPEND _rs_hints "/usr/local" "/usr" "/opt/librealsense")
list(REMOVE_DUPLICATES _rs_hints)

# Preferred: the config package the SDK ships, which brings the right compile
# definitions and transitive dependencies with it.
find_package(realsense2 QUIET CONFIG
             HINTS ${_rs_hints}
             PATHS ${_rs_hints}
             PATH_SUFFIXES lib/cmake/realsense2 cmake/realsense2)

# Fallback: a header and a library is all this project actually needs. Covers a
# source build, an unpacked archive, and any install whose config package is
# missing, unreadable, or built for another compiler.
if(NOT realsense2_FOUND)
  find_path(RS2_INCLUDE_DIR librealsense2/rs.hpp
            HINTS ${_rs_hints}
            PATH_SUFFIXES include)
  find_library(RS2_LIBRARY
               NAMES realsense2 realsense2.lib librealsense2
               HINTS ${_rs_hints}
               PATH_SUFFIXES lib lib/x64 lib/x86 lib64 bin/x64)
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

# Say where we looked, always. This is three lines of output on success and the
# difference between a fix and a round trip on failure.
if(realsense2_FOUND)
  message(STATUS "nav_sim: librealsense OK")
else()
  message(STATUS "nav_sim: librealsense NOT found. Searched these roots:")
  if(NOT _rs_hints)
    message(STATUS "    (none -- no RealSense directory under Program Files, "
                   "and REALSENSE2_DIR is not set)")
  endif()
  foreach(_h ${_rs_hints})
    if(EXISTS "${_h}/include/librealsense2/rs.hpp")
      message(STATUS "    ${_h}   <- has the header but no library was found")
    elseif(EXISTS "${_h}")
      message(STATUS "    ${_h}   (exists, no include/librealsense2/rs.hpp)")
    else()
      message(STATUS "    ${_h}   (does not exist)")
    endif()
  endforeach()
endif()
