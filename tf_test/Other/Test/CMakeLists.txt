cmake_minimum_required(VERSION 2.8.3)
project(testcfg)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
## 查找catkin宏和库
## 如果组件列表类似于 find_package(catkin REQUIRED COMPONENTS xyz)
##用过，还找其他catkin包
find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  rospy
  tf2
  tf2_ros
  roslint
)

## System dependencies are found with CMake's conventions
## 通过 CMake 的约定找到系统依赖项
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
## 如果包中有 setup.py，请取消注释。该宏确保
## 安装其中声明的模块和全局脚本
## 请参阅http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
##          声明 ROS 消息、服务和操作             ##
################################################

## To declare and build messages, services or actions from within this package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## 要从此包中声明和构建消息、服务或操作，请执行以下步骤：
## *令 MSG_DEP_SET 为您使用其消息类型的包集
## 您的messages/services/actions（例如 std_msgs、actionlib_msgs，...）。
  # * 在文件 package.xml 中：
  # * 为“message_ Generation”添加 build_depend 标签
  # * 为 MSG_DEP_SET 中的每个包添加一个 build_depend 和一个 exec_depend 标签
  # * 如果 MSG_DEP_SET 不为空，则已引入以下依赖项
## 但仍然可以肯定地声明：
  ## *为“message_runtime”添加 exec_depend 标签
  ## *在此文件 (CMakeLists.txt) 中：
  ## *将“message_ Generation”和MSG_DEP_SET中的每个包添加到

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
## 使用此处列出的任何依赖项生成添加的消息和服务
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
## 声明ROS动态重新配置参数 ##
################################################

## To declare and build dynamic reconfigure parameters within this package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below and list every .cfg file to be processed
## 要在此包中声明和构建动态重新配置参数，请执行以下步骤：
  ## *在文件 package.xml 中：
  ## *为“dynamic_reconfigure”添加 build_depend 和 exec_depend 标签
  ## *在此文件 (CMakeLists.txt) 中：
  ## *添加“dynamic_reconfigure”
## find_package(catkin 所需组件 ...)
  ## *取消注释下面的“generate_dynamic_reconfigure_options”部分并列出要处理的每个 .cfg 文件

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
   cfg/Tf.cfg
   cfg/tutorials.cfg  
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
## catkin_package 宏为你的包生成 cmake 配置文件
## 声明要传递给依赖项目的内容
## INCLUDE_DIRS：如果你的包包含头文件，请取消注释
## LIBRARIES：您在此项目中创建的库，依赖项目也需要
## CATKIN_DEPENDS：catkin_packages依赖的项目还需要
## DEPENDS：该项目的系统依赖项，依赖项目也需要

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES tunable_static_tf_broadcaster
#  CATKIN_DEPENDS dynamic_reconfigure rospy tf2 tf2_ros
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
## 指定头文件的附加位置
## 您的包裹位置应列在其他位置之前
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
message(" ==================== ${PROJECT_NAME} include_directories: ${catkin_INCLUDE_DIRS}   ${include}")
## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/tunable_static_tf_broadcaster.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(
  ${PROJECT_NAME}_node
  src/tunable_static_tf_broadcaster_node.cpp
  src/tunable_static_tf_broadcaster.cpp
)

roslint_cpp(
  src/tunable_static_tf_broadcaster_node.cpp
  src/tunable_static_tf_broadcaster.cpp
)

add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_gencfg)

#add dynamic reconfigure api
find_package(catkin REQUIRED dynamic_reconfigure)

add_executable(
  SetDynamicReconfigureCpp
  src/SetDynamicReconfigureCpp.cpp
)
# make sure configure headers are built before any node using them
add_dependencies(SetDynamicReconfigureCpp ${PROJECT_NAME}_gencfg)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

catkin_install_python(PROGRAMS scripts/tunable_static_tf_broadcaster_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
# 库的头文件还没有被复制到devel/include的文件夹下面来，因此，我们还是无法在另一个库中include hello的头文件。
# 复制库头文件到devel文件夹下的方法
# 这个利用cmake的install功能就可以实现了，我们在CMakeLists.txt的最后添加如下语句：
# install(FILES 
# 		${header_file}
# 		DESTINATION ${CATKIN_DEVEL_PREFIX}/include
# )
# 这里，install的意思就是把header_file安装到${CATKIN_DEVEL_PREFIX}/include文件夹下。其中，CATKIN_DEVEL_PREFIX就是表示devel文件夹的变量。
# 另外，直接安装文件夹也是可以的：
# install(DIRECTORY
# 		${header_directory}
# 		DESTINATION ${CATKIN_DEVEL_PREFIX}/include
# )
# 然后，在编译的时候，我们需要采用如下命令：
# catkin_make install
# 这时，头文件就被安装到devel目录下了，这个时候我们就可以在其他package中，直接引用这些头文件啦。
install(
  TARGETS ${PROJECT_NAME}_node 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY
    cfg
    launch
    params
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
