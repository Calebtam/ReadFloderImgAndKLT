target_include_directories(obslam PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/utils/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/mapping/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/solver/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/vo/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/sp_detector/include>
    $<INSTALL_INTERFACE:include>
    ${TORCH_INCLUDE_DIRS}
)

target_include_directories(obslam PRIVATE
    $<INSTALL_INTERFACE:include/obslam>
    ${OpenCV_INCLUDE_DIRS}
    ${GLOG_INCLUDE_DIRS}
    ${GTSAM_INCLUDE_DIR}
)

target_link_libraries(obslam PUBLIC
    Eigen3::Eigen
    Sophus::Sophus
    gtsam 
    gtsam_unstable
    glog::glog
    ${OpenCV_LIBS}
    ${TORCH_LIBRARIES}
)

install(TARGETS obslam
    EXPORT obslamTargets            # 安装 obslamTargets.cmake
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)

# 安装头文件
# install(DIRECTORY utils/include/ DESTINATION include/utils)
# install(DIRECTORY mapping/include/ DESTINATION include/mapping)
# install(DIRECTORY solver/include/ DESTINATION include/solver)
# install(DIRECTORY vo/include/ DESTINATION include/vo)
# install(DIRECTORY sp_detector/include/ DESTINATION include/sp_detector)
install(DIRECTORY utils/include/ mapping/include/ solver/include/ vo/include/ sp_detector/include/
        DESTINATION include)


# 安装 obslamTargets.cmake 文件
install(
    EXPORT obslamTargets
    FILE obslamTargets.cmake
    NAMESPACE obslam::              # ✅ 其他包使用时需要 obslam::obslam
    DESTINATION lib/cmake/obslam
)

# 安装 obslamConfig.cmake
include(CMakePackageConfigHelpers)

configure_package_config_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/obslam-config.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/obslamConfig.cmake
    INSTALL_DESTINATION lib/cmake/obslam
    PATH_VARS TORCH_INCLUDE_DIRS TORCH_LIBRARIES  # 添加 Torch 配置
)

write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/obslamConfigVersion.cmake
    COMPATIBILITY SameMajorVersion
)

install(
  FILES 
    ${CMAKE_CURRENT_BINARY_DIR}/obslamConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/obslamConfigVersion.cmake
  DESTINATION lib/cmake/obslam
)