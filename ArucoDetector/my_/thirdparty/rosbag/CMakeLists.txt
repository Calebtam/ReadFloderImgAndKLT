cmake_minimum_required(VERSION 3.0.2)

project(rosbag)

# RosTime
add_library(RosTime SHARED
        src/rostime/rate.cpp
        src/rostime/time.cpp
        src/rostime/duration.cpp
        )

IF (NOT ANDROID)
    target_include_directories(RosTime
            PRIVATE
            ${Boost_INCLUDE_DIRS}
            PUBLIC
            ${CMAKE_CURRENT_SOURCE_DIR}/include
            )

    # RosbagStorage
    FILE(GLOB_RECURSE RosbagStorage_SOURCES
            src/RosbagStorage/*.c
            src/RosbagStorage/*.cpp
            )

    add_library(RosbagStorage SHARED
            ${RosbagStorage_SOURCES}
            )

    target_include_directories(RosbagStorage
            PUBLIC
            $<BUILD_INTERFACE:${Boost_INCLUDE_DIRS}>
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>

            INTERFACE
            $<INSTALL_INTERFACE:include>
            )

    target_link_libraries(RosbagStorage
            PRIVATE
            RosTime
            PUBLIC
            glog
            Eigen3::Eigen
            #        message_flow
            #        Utility
#            CvBridgeSimple
            cv_bridge_simple
            )


    if (ENABLE_TESTING)
        add_subdirectory(test)
    endif ()

ELSE ()
    target_include_directories(RosTime
            PRIVATE
            ${CMAKE_SOURCE_DIR}/lib-includes
            PUBLIC
            ${CMAKE_CURRENT_SOURCE_DIR}/include
            )

    # RosbagStorage
    FILE(GLOB_RECURSE RosbagStorage_SOURCES
            src/RosbagStorage/*.c
            src/RosbagStorage/*.cpp
            )

    add_library(RosbagStorage SHARED
            ${RosbagStorage_SOURCES}
            )

    target_include_directories(RosbagStorage
            PUBLIC
            $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/lib-includes/include>
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>

            INTERFACE
            $<INSTALL_INTERFACE:include>
            )

    target_link_libraries(RosbagStorage
            PRIVATE
            RosTime
            PUBLIC
            ${jniLibs_DIR}/libglog.so
#            CvBridgeSimple
            cv_bridge_simple
            )
ENDIF ()