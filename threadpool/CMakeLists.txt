
message(STATUS ${CMAKE_CURRENT_SOURCE_DIR} "111")
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

add_executable(threadpool 
                    detect_img.cpp
                    workerpool.c
                    zarray.c
                    )

target_link_libraries(threadpool ${LIBS})
