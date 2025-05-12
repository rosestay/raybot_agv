# ROS2 实时地图更新导航系统

本项目实现了一个基于ROS2的实时地图更新导航系统，支持SLAM建图、动态地图更新和自主导航功能。

## 系统概述

该系统包含以下核心功能：
- 实时SLAM建图
- 动态地图更新和保存
- 地图状态监控
- 支持静态和动态地图模式的导航

## 系统架构

```
other_navigation/
├── package.xml                     # 功能包描述文件
├── CMakeLists.txt                  # 构建配置文件
├── launch/                         # 启动文件目录
│   ├── navigation2.launch.py       # 导航系统启动文件
│   ├── online_slam_launch.py       # SLAM系统启动文件
│   └── realtime_mapping_system.launch.py  # 实时建图系统启动文件
├── scripts/                        # Python节点脚本
│   ├── map_saver_node.py          # 地图保存节点
│   ├── slam_monitor.py            # SLAM状态监控节点
│   ├── realtime_map_updater.py    # 实时地图更新节点
│   └── dynamic_map_loader.py      # 动态地图加载节点
├── config/                         # 配置文件目录
│   ├── slam_toolbox_config.yaml   # SLAM工具配置
│   └── nav2_params.yaml           # 导航参数配置
└── maps/                          # 地图文件目录
    ├── map.yaml                   # 静态地图描述文件
    └── map.pgm                    # 静态地图图像文件
```

## 依赖项

### 系统依赖
- ROS2 Foxy/Humble (推荐Humble)
- Ubuntu 20.04/22.04

### ROS2包依赖
- `slam_toolbox` - SLAM工具
- `nav2_bringup` - Navigation2核心功能
- `nav2_map_server` - 地图服务
- `rviz2` - 可视化工具
- `diff_drive_robot` - 差分驱动控制（根据实际机器人调整）

### Python依赖
```bash
pip3 install numpy
```

## 安装步骤

1. 创建ROS2工作空间（如果还没有）：
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

2. 克隆或复制项目到工作空间：
```bash
# 假设项目文件已经在 other_navigation 目录中
cp -r /path/to/other_navigation ~/ros2_ws/src/
```

3. 安装依赖：
```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

4. 构建功能包：
```bash
colcon build --packages-select other_navigation
source install/setup.bash
```

5. 设置脚本执行权限：
```bash
cd ~/ros2_ws/src/other_navigation/scripts
chmod +x *.py
```

## 使用方法

### 1. 实时建图模式

启动SLAM建图系统：
```bash
ros2 launch other_navigation online_slam_launch.py
```

该命令会启动：
- SLAM Toolbox进行实时建图
- 地图保存服务（每60秒自动保存）
- SLAM状态监控
- 地图更新监控

### 2. 导航模式

#### 动态地图导航（推荐）
```bash
# 先启动SLAM系统
ros2 launch other_navigation online_slam_launch.py

# 在另一个终端启动导航
ros2 launch other_navigation navigation2.launch.py use_dynamic_map:=true
```

#### 静态地图导航
```bash
ros2 launch other_navigation navigation2.launch.py use_dynamic_map:=false map:=/path/to/your/map.yaml
```

### 3. 完整系统启动

一键启动实时建图和导航系统：
```bash
# 启动实时建图系统
ros2 launch other_navigation realtime_mapping_system.launch.py

# 在另一个终端启动导航
ros2 launch other_navigation navigation2.launch.py use_dynamic_map:=true
```

### 4. 仿真环境使用

在Gazebo仿真环境中使用：
```bash
ros2 launch other_navigation navigation2.launch.py use_sim_time:=true use_dynamic_map:=true
```

## 参数配置

### SLAM参数（slam_toolbox_config.yaml）
- `mode`: mapping - 建图模式
- `map_update_interval`: 1.0 - 地图更新频率（秒）
- `do_loop_closing`: true - 启用回环检测

### 导航参数（nav2_params.yaml）
- `controller_frequency`: 20.0 - 控制器频率
- `max_vel_x`: 0.26 - 最大线速度
- `max_vel_theta`: 1.0 - 最大角速度

### 地图保存参数
- `save_interval`: 60.0 - 地图保存间隔（秒）
- `map_save_path`: 地图保存路径
- `map_name_prefix`: 地图文件名前缀

## 主要节点说明

### 1. map_saver_node.py
- 功能：定期保存SLAM生成的地图
- 话题：订阅 `/map`
- 服务：使用 `/map_saver/save_map`

### 2. slam_monitor.py
- 功能：监控SLAM状态和定位质量
- 话题：订阅 `/map` 和 `/pose`
- 发布：`/slam_status`

### 3. realtime_map_updater.py
- 功能：检测地图变化并发布更新
- 话题：订阅 `/map`，发布 `/updated_map`
- 参数：`update_threshold` - 更新阈值

### 4. dynamic_map_loader.py
- 功能：提供动态地图服务
- 话题：订阅 `/map`，发布 `/dynamic_map`
- 服务：`/dynamic_map_service`

## 常见问题

### 1. 地图无法正常保存
- 检查maps目录权限
- 确认map_saver服务正常运行
- 查看日志确认保存路径正确

### 2. 导航无法启动
- 确认已启动SLAM系统
- 检查地图话题是否正常发布
- 验证TF变换是否完整

### 3. 动态地图不更新
- 检查dynamic_map_loader节点状态
- 确认SLAM正在发布地图
- 调整update_threshold参数

## 调试命令

查看系统状态：
```bash
# 查看运行的节点
ros2 node list

# 查看话题
ros2 topic list

# 监控地图话题
ros2 topic echo /map --no-arr

# 查看TF树
ros2 run tf2_tools view_frames.py

# 查看参数
ros2 param list
```

## 性能优化

1. **降低地图更新频率**：
   修改 `slam_toolbox_config.yaml` 中的 `map_update_interval`

2. **调整地图分辨率**：
   修改 `nav2_params.yaml` 中的 `resolution` 参数

3. **优化SLAM参数**：
   调整回环检测和扫描匹配参数

## 扩展功能

1. **多机器人支持**：
   添加命名空间支持，修改话题名称

2. **地图编辑**：
   集成地图编辑工具，支持手动修正

3. **云端备份**：
   添加地图云端同步功能

