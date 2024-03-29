cmake_minimum_required(VERSION 2.8.3)
project(ob_aruco)


if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()

set(CMAKE_CXX_STANDARD 17)

if (CMAKE_BUILD_TYPE STREQUAL Release)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops")
else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g3 -Wall -Wuninitialized -Wmaybe-uninitialized")
endif()

include_directories(${CMAKE_SOURCE_DIR}/thirdparty/cv_bridge/include)

include_directories(
  ./
  include
  src
  ${EIGEN3_INCLUDE_DIR}
)
include_directories(api)

file(GLOB_RECURSE LIBRARY_HEADERS "src/*.h")

if(PC_VISUALIZATION)
    include_directories(${CMAKE_SOURCE_DIR}/ov_visualization/src/)
endif()
add_subdirectory(api)
