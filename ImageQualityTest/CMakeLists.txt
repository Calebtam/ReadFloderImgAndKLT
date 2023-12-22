add_executable(o2_log_test o2_log_test.cpp)
add_executable(o2_rosbagread o2_RosBagRead.cpp)
add_executable(o3_dcw3 o3_DCW3.cpp)

target_link_libraries(o2_log_test ${LIBS}  ${Boost_LIBRARIES} RosTime RosbagStorage cv_bridge_simple)
target_link_libraries(o2_rosbagread ${LIBS} ${Boost_LIBRARIES} RosTime RosbagStorage cv_bridge_simple)
target_link_libraries(o3_dcw3 ${LIBS} ${Boost_LIBRARIES} RosTime RosbagStorage cv_bridge_simple)