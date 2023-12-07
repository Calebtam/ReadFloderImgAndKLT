cmake_minimum_required(VERSION 3.3)
project(a)

# set(CMAKE_BUILD_TYPE "Release")
# add_definitions("-DENABLE_SSE")
# set(CMAKE_CXX_FLAGS "-std=c++11 ${SSE_FLAGS} -g -O3 -march=native")

# Include libraries (if we don't have opencv 4, then fallback to opencv 3)
# The OpenCV version needs to match the one used by cv_bridge otherwise you will get a segmentation fault!
find_package(Eigen3 REQUIRED)
find_package(OpenCV 4 QUIET)
if (NOT OpenCV_FOUND)
    find_package(OpenCV 3 REQUIRED)
endif ()
find_package(Boost REQUIRED COMPONENTS system filesystem thread date_time)
find_package(Ceres REQUIRED)

find_package(Sophus REQUIRED)
find_package(Pangolin REQUIRED)

include_directories(
        ${OpenCV_INCLUDE_DIRS}
        ${G2O_INCLUDE_DIRS}
        ${Sophus_INCLUDE_DIRS}
        "/usr/include/eigen3/"
        ${Pangolin_INCLUDE_DIRS}
)

# We need c++14 for ROS2, thus just require it for everybody, enable c++17 to avoid eigen alignment issues
# NOTE: To future self, hope this isn't an issue...
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include_directories(
    ${OpenCV_INCLUDE_DIRS}
    # ${G2O_INCLUDE_DIRS}
    # ${Sophus_INCLUDE_DIRS}
    "/usr/include/eigen3/"
    # ${Pangolin_INCLUDE_DIRS}
)

# Build ROS nodes
set(LIBS
   ${EIGEN3_LIBS}
   ${catkin_LIBRARIES}
   ${Pangolin_LIBRARIES}
   ${OpenCV_LIBS}
   ${Sophus_LIBS}
)

add_executable(a a.cpp)
target_link_libraries(a ${LIBS})

add_executable(b b.cpp)
target_link_libraries(b ${LIBS})

add_executable(c c.cpp)
target_link_libraries(c ${LIBS})

cmake_minimum_required(VERSION 2.8)
project(ch8)

add_executable(optical_flow optical_flow.cpp)
target_link_libraries(optical_flow ${OpenCV_LIBS})
