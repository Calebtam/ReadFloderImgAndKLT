add_executable(ArucoDetector src/main.cpp src/ArucoDetector.cpp)


target_link_libraries(ArucoDetector ${LIBS}  ${Boost_LIBRARIES} RosTime RosbagStorage cv_bridge_simple)
