cmake_minimum_required(VERSION 3.3)
project(openvins)

message("BUILD_FOR: ${BUILD_FOR}")

if(NOT DEFINED BUILD_FOR)
    list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
    find_package(Eigen3 REQUIRED)
    find_package(OpenCV 4)
    if(OpenCV_FOUND)
        include_directories(${OpenCV_INCLUDE_DIRS})
        # include_directories(${OpenCV_INCLUDE_DIRS}/include)
        # link_libraries(${OpenCV_LIBS})
    endif()
    find_package(Boost REQUIRED COMPONENTS system filesystem thread date_time)
    if(Boost_FOUND)
        include_directories(${Boost_INCLUDE_DIRS})
    endif()
    set(PC_VISUALIZATION ON)

elseif (${BUILD_FOR} STREQUAL X3)
    add_definitions(-DBUILD_FOR_X3)
    set(CROSS_COMPILER_TOOLCHAIN /opt/gcc-ubuntu-9.3.0-2020.03-x86_64-aarch64-linux-gnu)
    list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
else()

endif()


if (${CMAKE_SYSTEM_PROCESSOR} STREQUAL x86_64)
    message("====${CMAKE_SYSTEM_PROCESSOR}====")
    # set(PC_VISUALIZATION OFF)
    set(DISPLAY_IMAGE_TRACK OFF)
    message("this is x86_64 cpu")
else ()
    message("====${CMAKE_SYSTEM_PROCESSOR}====")
    set(DISPLAY_IMAGE_TRACK OFF)
    set(PC_VISUALIZATION OFF)
    message("this is not x86_64 cpu")
endif()

set(CODE_OPTIMIZATION ON)
set(NarrowLockField ON)
add_compile_options(-fPIC -shared -rdynamic ) # -Werror
if(CODE_OPTIMIZATION)
    MESSAGE(STATUS "Enable CODE_OPTIMIZATION!!")
    add_definitions(-DCODE_OPTIMIZATION)
endif()
if(DISPLAY_IMAGE_TRACK)
    MESSAGE(STATUS " ## DISPLAY_IMAGE_TRACK !!")
    add_definitions(-DDISPLAY_IMAGE_TRACK)
endif()
if(NarrowLockField)
    MESSAGE(STATUS "NarrowLockField !!")
    add_definitions(-DNarrowLockField)
endif()
add_definitions(-DHPC_SETAFFINITY_OPT)#set CPU affinity, only available in x3m

IF (NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release) #Release or Debug
ENDIF ()

MESSAGE(STATUS "BUILD TYPE:" ${CMAKE_BUILD_TYPE})

# if(USE_ROS)
#     add_definitions(-DUSE_ROS)
# else()
set(USE_ROS OFF)
include_directories(thirdparty/rosbag/include)
include_directories(thirdparty/rosbag/include/MessageType)
include_directories(thirdparty/rosbag/include/RosbagStorage)
include_directories(thirdparty/cv_bridge/include)
include_directories(thirdparty)
# endif()

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

if(PC_VISUALIZATION)
    MESSAGE(STATUS "Enable PC_VISUALIZATION!!")
    add_definitions(-DPC_VISUALIZATION)
endif()

add_subdirectory(ob)

