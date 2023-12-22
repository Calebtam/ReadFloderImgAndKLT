
CMAKE_MINIMUM_REQUIRED(VERSION 3.10)

set(RBS_DIR /orbbec)
include_directories(${RBS_DIR}/rbs/include)
link_directories(${RBS_DIR}/rbs/lib/)
add_executable(RbsReceiveTest RbsReceive.cc)

target_link_libraries(RbsReceiveTest rbs -lpthread)

add_executable(RbsSendTest RbsSend.cc)

target_link_libraries(RbsSendTest rbs)