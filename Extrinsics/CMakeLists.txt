###################################################################################
# IMU 相关的验证

add_executable(calExtrinsics_imu calExtrinsics_imu.cpp)
target_link_libraries(calExtrinsics_imu ${LIBS})

add_executable(calExtrinsics_topcam calExtrinsics_topcam.cpp)
target_link_libraries(calExtrinsics_topcam ${LIBS})

add_executable(CheckExtrinsics Check.cpp)
target_link_libraries(CheckExtrinsics ${LIBS})