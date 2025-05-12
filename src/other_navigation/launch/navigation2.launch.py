import os 
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Get package share directories
    bot_navigation_dir = get_package_share_directory('other_navigation')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    rviz_config_dir = os.path.join(
         nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz'
    )
    
    # Launch configurations
    use_sim_time = launch.substitutions.LaunchConfiguration(
        'use_sim_time', default='false'
    )
    
    # 修改：默认使用静态地图模式，直到动态地图系统稳定
    use_dynamic_map = launch.substitutions.LaunchConfiguration(
        'use_dynamic_map', default='false'
    )
    
    # 静态地图路径
    map_yaml_path = launch.substitutions.LaunchConfiguration(
        'map', default=os.path.join(bot_navigation_dir, 'maps', 'map.yaml')
    )
    
    nav2_param_path = launch.substitutions.LaunchConfiguration(
        'params_file', default=os.path.join(bot_navigation_dir, 'config', 'nav2_params.yaml')
    )
    
    # Create launch description
    return launch.LaunchDescription([
        # Declare launch arguments
        launch.actions.DeclareLaunchArgument(
            'use_sim_time', 
            default_value=use_sim_time,
            description='Use simulation (Gazebo) clock if true'
        ),
        launch.actions.DeclareLaunchArgument(
            'use_dynamic_map', 
            default_value=use_dynamic_map,
            description='Use dynamic map from SLAM if true'
        ),
        launch.actions.DeclareLaunchArgument(
            'map', 
            default_value=map_yaml_path,
            description='Path to map yaml file'
        ),
        launch.actions.DeclareLaunchArgument(
            'params_file', 
            default_value=nav2_param_path,
            description='Path to navigation parameters yaml file'
        ),
        
        # 首先启动地图服务器（静态地图模式）
        launch.actions.GroupAction(
            condition=launch.conditions.UnlessCondition(use_dynamic_map),
            actions=[
                launch.actions.IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
                    ),
                    launch_arguments={
                        'map': map_yaml_path,
                        'use_sim_time': use_sim_time,
                        'params_file': nav2_param_path
                    }.items(),
                )
            ]
        ),
        
        # 动态地图模式（需要先启动SLAM）
        launch.actions.GroupAction(
            condition=launch.conditions.IfCondition(use_dynamic_map),
            actions=[
                # 先启动动态地图加载器
                launch_ros.actions.Node(
                    package='other_navigation',
                    executable='dynamic_map_loader.py',
                    name='dynamic_map_loader',
                    output='screen',
                    parameters=[{
                        'use_sim_time': use_sim_time,
                        'map_save_path': os.path.join(bot_navigation_dir, 'maps')
                    }]
                ),
                
                # 延迟启动Nav2，等待地图服务准备就绪
                launch.actions.TimerAction(
                    period=2.0,
                    actions=[
                        launch.actions.IncludeLaunchDescription(
                            PythonLaunchDescriptionSource(
                                os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
                            ),
                            launch_arguments={
                                'map': '',  # 动态地图模式不需要地图文件
                                'use_sim_time': use_sim_time,
                                'params_file': nav2_param_path,
                                'use_respawn': 'true',
                                'autostart': 'true'
                            }.items(),
                        )
                    ]
                )
            ]
        ),
        
        # Differential wheel controller
        launch_ros.actions.Node(
            package='diff_drive_robot', 
            executable='diff_wheel_control',
            name='diff_wheel_control',
            output='screen'
        ),
        
        # Launch RViz2
        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),
    ])