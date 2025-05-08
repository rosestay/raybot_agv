cmake_minimum_required(VERSION 3.8)
project(diff_drive_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)  # 添加geometry_msgs依赖
find_package(action_msgs REQUIRED)    # 添加action_msgs依赖
find_package(rosidl_default_generators REQUIRED)

# 声明消息文件
set(msg_files
  "msg/WheelCommand.msg"
  "msg/Feedback.msg"
)

# 声明action文件
set(action_files
  "action/NavigateToPoint.action"
)

# 生成接口
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${action_files}  # 添加action文件到生成接口
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs
)

# 确保导出依赖项
ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(geometry_msgs)  # 导出新依赖
ament_export_dependencies(action_msgs)    # 导出新依赖

# 标记为接口包
ament_package()