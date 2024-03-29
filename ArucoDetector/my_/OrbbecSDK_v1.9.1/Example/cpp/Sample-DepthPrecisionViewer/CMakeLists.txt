#  minimum required cmake version: 3.1.15 support vs2019

cmake_minimum_required(VERSION 3.1.15)
project(Sample-DepthPrecisionViewer)

add_executable(DepthPrecisionViewer DepthPrecisionViewer.cpp)

set_property(TARGET DepthPrecisionViewer PROPERTY CXX_STANDARD 11)
target_link_libraries(DepthPrecisionViewer OrbbecSDK ${OpenCV_LIBS})

