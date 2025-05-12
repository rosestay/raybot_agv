import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 声明参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    slam_params_file = LaunchConfiguration('slam_params_file')
    
    # 获取配置文件路径
    slam_toolbox_dir = FindPackageShare('slam_toolbox')
    nav2_bringup_dir = FindPackageShare('nav2_bringup')
    other_navigation_dir = get_package_share_directory('other_navigation')
    
    # 声明launch参数
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
        
    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(other_navigation_dir, 
                                  'config', 'slam_toolbox_config.yaml'),
        description='Full path to the ROS2 parameters file for slam_toolbox')
    
    # 启动slam_toolbox在线建图模式
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('/scan', '/scan'),
            ('/map', '/map'),
            ('/map_metadata', '/map_metadata'),
            ('/tf', '/tf'),
            ('/tf_static', '/tf_static')
        ]
    )
    
    # 启动地图保存服务
    map_saver_server_node = Node(
        package='nav2_map_server',
        executable='map_saver_server',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'save_map_timeout': 2000},
            {'free_thresh_default': 0.25},
            {'occupied_thresh_default': 0.65}
        ]
    )
    
    # 启动自定义的地图保存节点
    map_saver_node = Node(
        package='other_navigation',
        executable='map_saver_node.py',
        name='map_saver_node',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'save_interval': 60.0},  # 每60秒保存一次
            {'map_save_path': os.path.join(other_navigation_dir, 'maps')},
            {'map_name_prefix': 'dynamic_map'}
        ]
    )
    
    # 启动SLAM监控节点
    slam_monitor_node = Node(
        package='other_navigation',
        executable='slam_monitor.py',
        name='slam_monitor',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'map_change_threshold': 0.1},
            {'pose_uncertainty_threshold': 0.5}
        ]
    )
    
    # 创建launch描述
    ld = LaunchDescription()
    
    # 添加launch参数
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_slam_params_file_cmd)
    
    # 添加节点
    ld.add_action(slam_toolbox_node)
    ld.add_action(map_saver_server_node)
    ld.add_action(map_saver_node)
    ld.add_action(slam_monitor_node)
    
    return ld