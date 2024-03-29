cmake_minimum_required(VERSION 3.3)
project(ob_aruco_api)

include_directories(
        ./
        include
)
include_directories(${CROSS_COMPILER_TOOLCHAIN}/aarch64-linux-gnu/usr/local/include/eigen3)
include_directories(${CROSS_COMPILER_TOOLCHAIN}//aarch64-linux-gnu/include/opencv4)

if (CMAKE_BUILD_TYPE STREQUAL Release)
    # Enable compile optimizations
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops")
else()
    # Enable debug flags (use if you want to debug in gdb)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g3 -Wall -Wuninitialized -Wmaybe-uninitialized")
endif()
MESSAGE(STATUS "#### ${PROJECT_SOURCE_DIR}")


add_library(${PROJECT_NAME}  SHARED obaruco_api.cpp obsystem.cpp cvAruco/aruco_detector.cpp 
                                    utils/draw_cube.cpp utils/print.cpp utils/VioManagerOptions.cpp)

target_link_libraries(${PROJECT_NAME}
        -lopencv_highgui
        -lopencv_core
        -lopencv_imgproc
        -lopencv_imgcodecs
        # -lopencv_shape
        # -lopencv_videoio
        -lopencv_calib3d
        # -lopencv_features2d
        # -lopencv_flann
        # -lopencv_ml
        -lopencv_objdetect
        # -lopencv_photo
        # -lopencv_stitching
        # -lopencv_superres
        # -lopencv_video
        # -lopencv_videostab
        -lopencv_aruco
        -lboost_filesystem
)

add_executable(o0test test/o0_test.cpp)
target_link_libraries(o0test ${PROJECT_NAME})

# add_executable(o1test test/o1_test.cpp)
# target_link_libraries(o1test ${PROJECT_NAME} opencv_imgcodecs opencv_highgui ${Boost_LIBRARIES})

if(NOT DEFINED BUILD_FOR_X3)
    include_directories(./include/option)

    # obsensor sdk
    set(OrbbecSDK_ROOT_DIR ${CMAKE_SOURCE_DIR}/SDK)
    message(STATUS " =============================================== configuration: \"${CMAKE_SOURCE_DIR}\"")
    set(OrbbecSDK_LIBRARY_DIRS ${OrbbecSDK_ROOT_DIR}/lib)
    set(OrbbecSDK_INCLUDE_DIR ${OrbbecSDK_ROOT_DIR}/include)
    include_directories(${OrbbecSDK_INCLUDE_DIR})
    link_directories(${OrbbecSDK_LIBRARY_DIRS})


    add_executable(o2test test/o2_test.cpp)
    target_link_libraries(o2test ${PROJECT_NAME} ${OpenCV_LIBS} OrbbecSDK ${Boost_LIBRARIES} pthread)
    
    # G2O
    find_package(G2O REQUIRED)
    include_directories(${G2O_INCLUDE_DIRS})
    # csparse
    find_package(CSparse REQUIRED)
    include_directories(${CSPARSE_INCLUDE_DIR})
    # Sophus
    find_package(Sophus REQUIRED)
    include_directories(${Sophus_INCLUDE_DIRS})

    set(O3_LIBS
            ${Sophus_LIBRARIES}
            g2o_core g2o_stuff g2o_types_sba g2o_solver_csparse g2o_csparse_extension
            ${CSPARSE_LIBRARY}
        #     boost_thread
            ${Boost_LIBRARIES}
            fmt
            )

    add_executable(o3test test/o3_test.cpp MyModel/Aruco.cpp)
    target_link_libraries(o3test ${PROJECT_NAME} ${O3_LIBS} ${OpenCV_LIBS} OrbbecSDK ${Boost_LIBRARIES})

    install(FILES ${PROJECT_SOURCE_DIR}/obfusion_api.h
            DESTINATION /orbbec/slam/include
            )

    add_executable(o4test test/o4.cpp)
    target_link_libraries(o4test ${OpenCV_LIBS} ${Boost_LIBRARIES})
endif()