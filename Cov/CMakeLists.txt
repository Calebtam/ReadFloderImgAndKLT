############################# cov ####################################
add_executable(cov_test_a a.cpp)
target_link_libraries(cov_test_a ${LIBS})
add_executable(cov_test_b b.cpp)
target_link_libraries(cov_test_b ${LIBS})
add_executable(cov_test_c c.cpp)
target_link_libraries(cov_test_c ${LIBS})