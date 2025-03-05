include_directories(
    ./
)

add_executable(glim_callbak_test 
    glim_callbak_test.cpp
    callbacks.cpp)
target_link_libraries(glim_callbak_test ${LIBS})
