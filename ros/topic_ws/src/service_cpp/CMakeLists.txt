cmake_minimum_required(VERSION 3.8)
project(service_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 替换为你的工作空间实际路径
set(CUSTOM_SRV_INSTALL_DIR "/home/nifei/project/github/ros/topic_ws/install/custom_srv_interfaces")

# 将包的配置文件目录添加到 CMake 搜索路径
list(APPEND CMAKE_PREFIX_PATH ${CUSTOM_SRV_INSTALL_DIR})

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(custom_srv_interfaces REQUIRED)

#编译服务端节点
add_executable(move_robot_server src/service_server.cpp)
ament_target_dependencies(move_robot_server rclcpp custom_srv_interfaces)

#编译客户端节点
add_executable(move_robot_client src/service_client.cpp)
ament_target_dependencies(move_robot_client rclcpp custom_srv_interfaces)

#安装服务端和客户端可执行文件
install(TARGETS
  move_robot_server
  move_robot_client
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
