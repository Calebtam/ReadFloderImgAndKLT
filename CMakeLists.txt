cmake_minimum_required(VERSION 3.15)
project(Test)

set(CMAKE_CXX_STANDARD 14)

find_package(OpenCV 4 QUIET)
if (NOT OpenCV_FOUND)
    find_package(OpenCV 3 REQUIRED)
    if (NOT OpenCV_FOUND)
        message(FATAL_ERROR "OpenCV > 3.0 not found.")
    endif ()
endif ()

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED QUIET COMPONENTS system filesystem thread date_time program_options chrono)
if (NOT OpenCV_FOUND)
    find_package(OpenCV 3)
    if (NOT OpenCV_FOUND)
        message(FATAL_ERROR "OpenCV > 3.0 not found.")
    endif ()
endif ()
if (CMAKE_BUILD_TYPE STREQUAL Release)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops")
else()
    # Enable debug flags (use if you want to debug in gdb)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g3 -Wall -Wuninitialized -Wmaybe-uninitialized -fno-omit-frame-pointer")
endif()
include_directories(${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
MESSAGE("OPENCV VERSION:")
MESSAGE(${OpenCV_VERSION})

add_executable(Test main2.cpp MutexTest.cpp )
add_executable(NotifyTest ThreadNotifyTest/NotifyTest.cpp )
target_link_libraries(Test -lpthread )
target_link_libraries(NotifyTest -lpthread )
include_directories(CallbackTest)
add_executable(CallbackTest CallbackTest/callbackTest.cpp CallbackTest/ClassTest1.cpp
        CallbackTest/ClassTest2.cpp CallbackTest/LockTest.cpp )

add_executable(opencvMatTest OpencvMatTest/matTest.cpp)
target_link_libraries(opencvMatTest ${OpenCV_LIBS} -lpthread)

add_executable(popBackTest vector_pop_back/popBack.cpp)

add_executable(WriteReadTest WriteReadTheadTest/WriteReadMain.cpp)
target_link_libraries(WriteReadTest -lpthread)

add_executable(MultiStateClass MultiStateClass/ClassMultiStates.cpp)

add_executable(LockTest LockTest/LockTest.cpp)
target_link_libraries(LockTest -lpthread)

add_executable(PointerTest PointerTest/PointTest.cpp)
target_link_libraries(PointerTest -lpthread)

add_executable(MemoryTest MemoryTest/memory.cpp)
target_link_libraries(MemoryTest -lpthread)

add_executable(switchTest SwitchTest/switchTest.cc)


add_executable(functionBindTest FuncitonBind/main.cc)
# add_subdirectory(RbsReceiveTest)

add_executable(systemCmdTest SystemCommand/systemcmd.cc)


add_executable(RotationTest RotationVector/main.cc)

add_executable(TypeTest typeTest/typeTest.cpp)

add_executable(SharedPointerTest SharedPointer/shared_point.cpp)

add_executable(data_structure DataStructure/main.cpp)

add_executable(virtual_class_test virtual_class_test/virtual_class_main.cpp)

add_executable(eigenMatrixTest EigenSpeedTest/eigenMatrixTest.cpp)
target_link_libraries(eigenMatrixTest ${Boost_LIBRARIES})