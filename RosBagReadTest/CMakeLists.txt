add_executable(rosbagread RosBagRead.cpp)
target_link_libraries(rosbagread ${LIBS} ${Boost_LIBRARIES} RosTime RosbagStorage cv_bridge_simple)
