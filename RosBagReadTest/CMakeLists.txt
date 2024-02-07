add_executable(rosbagread RosBagRead.cpp)
target_link_libraries(rosbagread ${LIBS} ${Boost_LIBRARIES} RosTime RosbagStorage cv_bridge_simple)

add_executable(bagreadRtk RosBagRead2.cpp)
target_link_libraries(bagreadRtk ${LIBS} ${Boost_LIBRARIES} RosTime RosbagStorage cv_bridge_simple)
