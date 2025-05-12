# realtime_mapping_system.launch.py
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # 获取package路径
    other_navigation_dir = get_package_share_directory('other_navigation')
    
    # SLAM节点
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            os.path.join(other_navigation_dir, 'config', 'slam_toolbox_config.yaml'),
            {'use_sim_time': use_sim_time}
        ]
    )
    
    # 实时地图更新节点
    realtime_map_updater = Node(
        package='other_navigation',
        executable='realtime_map_updater.py',
        name='realtime_map_updater',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'update_threshold': 0.05},
            {'min_update_interval': 1.0}
        ]
    )
    
    # 地图保存节点
    map_saver_node = Node(
        package='other_navigation',
        executable='map_saver_node.py',
        name='map_saver_node',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'save_interval': 30.0},
            {'map_save_path': os.path.join(other_navigation_dir, 'maps')}
        ]
    )
    
    # SLAM监控节点
    slam_monitor_node = Node(
        package='other_navigation',
        executable='slam_monitor.py',
        name='slam_monitor',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time}
        ]
    )
    
    # 动态地图加载器
    dynamic_map_loader = Node(
        package='other_navigation',
        executable='dynamic_map_loader.py',
        name='dynamic_map_loader',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'map_save_path': os.path.join(other_navigation_dir, 'maps')}
        ]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        slam_toolbox_node,
        realtime_map_updater,
        map_saver_node,
        slam_monitor_node,
        dynamic_map_loader
    ])