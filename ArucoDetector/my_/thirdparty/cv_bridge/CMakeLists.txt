cmake_minimum_required(VERSION 3.0.2)

project(cv_bridge)
find_package(Boost REQUIRED COMPONENTS system filesystem thread date_time regex)

add_library(cv_bridge_simple SHARED
        src/cv_bridge.cpp
        src/rgb_colors.cpp
        )

IF (NOT ANDROID)
    target_include_directories(cv_bridge_simple
            PUBLIC
            $<BUILD_INTERFACE:${Boost_INCLUDE_DIRS}>
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>

            INTERFACE
            $<INSTALL_INTERFACE:include>
            )

    target_link_libraries(cv_bridge_simple
            PRIVATE
            ${OpenCV_LIBS}
            ${Boost_LIBRARIES}

            PUBLIC
            )

ELSE ()
    target_include_directories(cv_bridge_simple
            PUBLIC
            $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/lib-includes/include>
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>

            INTERFACE
            $<INSTALL_INTERFACE:include>
            )

    target_link_libraries(cv_bridge_simple
            PRIVATE
            ${jniLibs_DIR}/libopencv_java4.so

            PUBLIC
            )
ENDIF ()