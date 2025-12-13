from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    wheel_radius  = LaunchConfiguration('wheel_radius')
    wheel_base    = LaunchConfiguration('wheel_base')
    ticks_per_rev = LaunchConfiguration('ticks_per_rev')
    loop_rate_hz  = LaunchConfiguration('loop_rate_hz')

    return LaunchDescription([
        DeclareLaunchArgument('wheel_radius',  default_value='0.035'),
        DeclareLaunchArgument('wheel_base',    default_value='0.220'),
        DeclareLaunchArgument('ticks_per_rev', default_value='1000'),
        DeclareLaunchArgument('loop_rate_hz',  default_value='50.0'),

        # 1) 키보드 → /cmd_vel
       # Node(
       #     package='teleop_twist_keyboard',
       #     executable='teleop_twist_keyboard',
       #     name='teleop_twist_keyboard',
       #     output='screen',
       #     remappings=[('cmd_vel', '/cmd_vel')],
       # ),

        # 2) 가상 엔코더 (/cmd_vel → /ecd)
        Node(
            package='jarabot_sim',
            executable='jara_sim_encoder',
            name='jara_sim_encoder',
            output='screen',
            parameters=[{
                'wheel_radius': wheel_radius,
                'wheel_base': wheel_base,
                'ticks_per_rev': ticks_per_rev,
                'loop_rate_hz': loop_rate_hz,
                'encoder_noise_std': 0.5,
            }]
        ),

        # 3) 오도메트리 (/ecd → /odom + TF)
        Node(
    		package='jarabot_sim',
    		executable='jara_sim_odometry',
    		name='jara_sim_odometry',
    		output='screen',
            parameters=[{
                'wheel_radius': wheel_radius,
                'wheel_base': wheel_base,
                'ticks_per_rev': ticks_per_rev,
                'frame_id': 'odom',
                'child_frame_id': 'base_link',
            }]
        ),

        # 4) 가상 라이다 (/odom → /scan)
        Node(
            package='jarabot_sim',
            executable='jara_sim_lidar',
            name='jara_sim_lidar',
            output='screen',
            parameters=[{
                'range_noise_std': 0.01,
            }]
        ),

        # 5) 로봇 몸체 + 진행 방향 마커
        Node(
            package='jarabot_sim',
            executable='jara_sim_marker',
            name='jara_sim_marker',
            output='screen',
            parameters=[{
                'frame_id': 'base_link',
            }]
        ),

        # 6) 오도메트리 궤적 Path
        Node(
            package='jarabot_sim',
            executable='jara_sim_path',
            name='jara_sim_path',
            output='screen',
            parameters=[{
                'max_points': 5000,
            }]
        ),
    ])
