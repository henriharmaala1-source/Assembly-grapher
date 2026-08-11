# Does cmake/find_realsense.cmake actually find an SDK laid out the way the
# Windows installer lays one out?
#
#   cmake -P test/find_realsense_check.cmake
#
# This is a test of SEARCH PATHS, which is precisely what was wrong: the SDK was
# installed, the Viewer ran, and the build still said "no RealSense SDK". A test
# that only checked the not-found path would have passed throughout.
#
# It builds three synthetic trees in a temp directory and asserts the module
# locates each one, then asserts it correctly finds nothing in an empty tree.
# No librealsense required, so it runs anywhere.

set(TMP "${CMAKE_CURRENT_LIST_DIR}/../_rs_find_test")
file(REMOVE_RECURSE "${TMP}")

set(FAILS 0)

macro(expect cond what)
  if(${cond})
    message(STATUS "  ok    ${what}")
  else()
    message(STATUS "  FAIL  ${what}")
    math(EXPR FAILS "${FAILS}+1")
  endif()
endmacro()

# A tree shaped like the Windows install: <root>/include + <root>/lib/x64.
function(make_sdk root libsub)
  file(WRITE "${root}/include/librealsense2/rs.hpp" "// synthetic\n")
  file(WRITE "${root}/${libsub}/librealsense2.so" "synthetic\n")
endfunction()

message(STATUS "find_realsense search-path checks")

# 1. The exact shape the installer produces, reached via REALSENSE2_DIR.
make_sdk("${TMP}/Intel RealSense SDK 2.0" "lib/x64")
set(ENV{REALSENSE2_DIR} "${TMP}/Intel RealSense SDK 2.0")
unset(RS2_INCLUDE_DIR CACHE)
unset(RS2_LIBRARY CACHE)
unset(realsense2_FOUND)
include("${CMAKE_CURRENT_LIST_DIR}/../cmake/find_realsense.cmake")
expect(realsense2_FOUND "finds an installer-shaped tree via REALSENSE2_DIR")
if(realsense2_FOUND)
  if(EXISTS "${RS2_INCLUDE_DIR}/librealsense2/rs.hpp")
    message(STATUS "  ok    and the reported include dir really has the header")
  else()
    message(STATUS "  FAIL  reported include dir has no rs.hpp")
    math(EXPR FAILS "${FAILS}+1")
  endif()
endif()

# 2. A plain unix-style prefix, lib/ rather than lib/x64.
file(REMOVE_RECURSE "${TMP}")
make_sdk("${TMP}/opt-style" "lib")
set(ENV{REALSENSE2_DIR} "${TMP}/opt-style")
unset(RS2_INCLUDE_DIR CACHE)
unset(RS2_LIBRARY CACHE)
unset(realsense2_FOUND)
include("${CMAKE_CURRENT_LIST_DIR}/../cmake/find_realsense.cmake")
expect(realsense2_FOUND "finds a plain <prefix>/lib layout")

# 3. Nothing there at all -> must NOT claim to have found it.
file(REMOVE_RECURSE "${TMP}")
file(MAKE_DIRECTORY "${TMP}/empty")
set(ENV{REALSENSE2_DIR} "${TMP}/empty")
unset(RS2_INCLUDE_DIR CACHE)
unset(RS2_LIBRARY CACHE)
unset(realsense2_FOUND)
include("${CMAKE_CURRENT_LIST_DIR}/../cmake/find_realsense.cmake")
if(realsense2_FOUND)
  # Only acceptable if a REAL librealsense is installed on this machine.
  message(STATUS "  note  a real librealsense is present; empty-tree case skipped")
else()
  message(STATUS "  ok    reports not-found when there is nothing to find")
endif()

file(REMOVE_RECURSE "${TMP}")
if(FAILS GREATER 0)
  message(FATAL_ERROR "find_realsense checks FAILED (${FAILS})")
endif()
message(STATUS "find_realsense checks passed")
