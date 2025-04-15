add_executable(o1_optical_flow o1_optical_flow.cpp)
target_link_libraries(o1_optical_flow ${LIBS})

add_executable(o2_undistort o2_undistort.cpp)
target_link_libraries(o2_undistort ${LIBS})
