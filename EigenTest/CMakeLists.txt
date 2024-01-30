add_executable(b b.cpp)
target_link_libraries(b ${LIBS})

add_executable(c c.cpp)
target_link_libraries(c ${LIBS})


add_executable(matrix_init Matrix_init.cpp)
target_link_libraries(matrix_init ${LIBS})

add_executable(linearDecomposition linearDecomposition.cpp)
target_link_libraries(linearDecomposition ${LIBS})

add_executable(sliceAndIndex sliceAndIndex.cpp)
target_link_libraries(sliceAndIndex ${LIBS})