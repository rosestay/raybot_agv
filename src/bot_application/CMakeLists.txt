cmake_minimum_required(VERSION 3.8)
project(bot_application)

# 默认C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找基本依赖项
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# 查找消息和接口生成所需的依赖项
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(action_msgs REQUIRED)

# 查找其他依赖项
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(diff_drive_msgs REQUIRED)

# 安装Python包
ament_python_install_package(${PROJECT_NAME})

# 安装Python脚本
install(PROGRAMS
  scripts/nav_to_point_action_node.py
  # 添加主要的Python脚本
  scripts/init_robot_pose.py
  scripts/get_robot_pose.py
  scripts/nav_to_pose.py
  DESTINATION lib/${PROJECT_NAME}
)

# 安装launch文件
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# 安装action文件
install(DIRECTORY
  action
  DESTINATION share/${PROJECT_NAME}
)

# 创建数据目录
install(DIRECTORY
  DESTINATION share/${PROJECT_NAME}/data
)

# 导出依赖项
ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(builtin_interfaces)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(action_msgs)
ament_export_dependencies(tf2_geometry_msgs)
ament_export_dependencies(nav2_msgs)

ament_package()