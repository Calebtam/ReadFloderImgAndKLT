cmake_minimum_required(VERSION 3.3)
project(a)

# Compile as C++11, supported in ROS Kinetic and newer, enable c++17 to avoid eigen alignment issues
#add_compile_options(-std=c++17 -g) # for std::make_unique
set(CMAKE_CXX_STANDARD 17)
#add_definitions(-w)

add_compile_options(-fPIC -shared -rdynamic ) # -Werror
add_definitions(-DHPC_SETAFFINITY_OPT)#set CPU affinity, only available in x3m

# set(CMAKE_BUILD_TYPE "Release")
# add_definitions("-DENABLE_SSE")
if (CMAKE_BUILD_TYPE STREQUAL Release)
    # Enable compile optimizations
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops")
else()
    # Enable debug flags (use if you want to debug in gdb)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g3 -Wall -Wuninitialized -Wmaybe-uninitialized")
endif()
# set(CMAKE_CXX_FLAGS "-std=c++11 ${SSE_FLAGS} -g -O3 -march=native")

# Include libraries (if we don't have opencv 4, then fallback to opencv 3)
# The OpenCV version needs to match the one used by cv_bridge otherwise you will get a segmentation fault!
find_package(Eigen3 REQUIRED)
find_package(OpenCV 4 QUIET)
if (NOT OpenCV_FOUND)
    find_package(OpenCV 3 REQUIRED)
endif ()
# find_package(Boost REQUIRED COMPONENTS system filesystem thread date_time regex)
find_package(Boost REQUIRED)
find_package(Ceres REQUIRED)

find_package(Sophus REQUIRED)
find_package(Pangolin REQUIRED)


# find_package(catkin REQUIRED COMPONENTS cv_bridge)
# # find_package(catkin REQUIRED COMPONENTS regex)
# find_package(catkin REQUIRED COMPONENTS
#   roscpp
#   image_transport
#   rospy
# #   cv_bridge
# #   regex
# #   opencv2
#   rospy
#   sensor_msgs
#   std_msgs
# )
MESSAGE(STATUS "#### catkin_  ${catkin_INCLUDE_DIRS}" )

##################### include ###########################
include_directories(
        ${OpenCV_INCLUDE_DIRS}
        ${G2O_INCLUDE_DIRS}
        ${Sophus_INCLUDE_DIRS}
        "/usr/include/eigen3/"
        # "/opt/ros/noetic/include/"
        ${Pangolin_INCLUDE_DIRS}
        # ${catkin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        geometrics
)

set(ROSBAG_DIR ${CMAKE_SOURCE_DIR}/third_3_party/rosbag)
set(CvBridge_DIR ${CMAKE_SOURCE_DIR}/third_3_party/CvBridgeSimple)
MESSAGE(STATUS "#### ROSBAG_DIR  ${ROSBAG_DIR}" )
MESSAGE(STATUS "#### CvBridge_DIR  ${CvBridge_DIR}" )
include_directories(
    include
    ${ROSBAG_DIR}/include
    ${ROSBAG_DIR}/include/MessageType
    ${ROSBAG_DIR}/include/RosbagStorage
    ${CvBridge_DIR}/include
)
################################# LIB ############################
# Build ROS nodes
set(LIBS
   ${EIGEN3_LIBS}
  #  ${catkin_LIBRARIES}
   ${Pangolin_LIBRARIES}
   ${OpenCV_LIBS}
   ${Sophus_LIBS}
   ${Boost_LIBRARIES}
   fmt
   ${CERES_LIBRARIES}
)

add_subdirectory(third_3_party)

############################# EXCUTABLE ####################################

add_executable(a Km/a.cpp)
target_link_libraries(a ${LIBS})

add_executable(d RandomSelectTest/d.cpp)
target_link_libraries(d ${LIBS})

############################# my ####################################

# add_executable(sliding_window my/sliding_window.cpp)
# target_link_libraries(sliding_window ${LIBS})
# add_executable(operater_test my/operater_test.cpp)
# target_link_libraries(operater_test ${LIBS})



###################################################################################

MESSAGE(STATUS "####  ${CMAKE_SOURCE_DIR}" )


##################################################################################
# cmake_minimum_required(VERSION 3.14)
# project(to_rosbag)
# set(CMAKE_BUILD_TYPE Debug)
# set(CMAKE_CXX_STANDARD 14)

# ros pkg dep
# set(ROS_DEP roscpp sensor_msgs rosbag)
# when create new message, build with message_generation
# find_package(catkin REQUIRED COMPONENTS message_generation ${ROS_DEP})
# find_package(catkin REQUIRED COMPONENTS ${ROS_DEP})

# catkin_package() 是catkin提供的CMake的宏，这个指令是用来提供一些catkin的信息来创建pkg-config和CMake文件。
# 这个指令必须在 add_library() 或 add_executable()前执行， 这个指令有 5 个可选参数：
# INCLUDE_DIRS - 被这个package导出的include路径 
# LIBRARIES - 被这个package导出的library
# CATKIN_DEPENDS - 这个工程依赖的其他catkin projects 
# DEPENDS - 这个工程依赖的Non-catkin CMake projects 
# CFG_EXTRAS - 其他的构建选项。
# catkin_package(CATKIN_DEPENDS ${ROS_DEP} message_runtime)
# catkin_package(CATKIN_DEPENDS ${ROS_DEP})


# # file() 
# aux_source_directory(convert_to_rosbag ALL_SRC)
# add_executable(${PROJECT_NAME} ${ALL_SRC})
# target_link_libraries(${PROJECT_NAME} PUBLIC ${catkin_LIBRARIES})
# target_include_directories(${PROJECT_NAME} PUBLIC ./convert_to_rosbag ${catkin_INCLUDE_DIRS})

# # 
# set_target_properties(${PROJECT_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ./)
###########################################################################################

add_subdirectory(opencv)
add_subdirectory(ImageQualityTest)
add_subdirectory(RosBagReadTest)
# add_subdirectory(Sophus)
add_subdirectory(EigenTest)
add_subdirectory(STL)
add_subdirectory(Camera)
add_subdirectory(Ceres)
add_subdirectory(geometrics)
add_subdirectory(glim)
add_subdirectory(Extrinsics)
add_subdirectory(message)
add_subdirectory(Cov)