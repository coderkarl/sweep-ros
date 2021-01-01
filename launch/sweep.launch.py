from launch import LaunchDescription
from launch_ros.actions import Node
from numpy import number

from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

# https://answers.ros.org/question/306935/ros2-include-a-launch-file-from-a-launch-file/
# Also needed to add rviz to the CMakeLists.txt install(DIRECTORY
from ament_index_python.packages import get_package_share_directory
import os
from math import pi

def generate_launch_description():
    ld = LaunchDescription()
    
    # NOTE, in riz, check your topic Reliability Policy and Durability Policy
    config_rviz = os.path.join(
        get_package_share_directory('sweep_ros'),
        'rviz', 'sweep.rviz'
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d', config_rviz],
        #remappings=[("goal_pose", "wp_goal")]
    )
    
    sweep_hz = 5
    
    sweep_node = Node(
        package="sweep_ros",
        executable="sweep_node",
        output="screen",
        emulate_tty=True,
        parameters=[{"serial_port": "/dev/sweep"},
                    {"serial_baud_rate": 115200},
                    {"frame_id": "laser"},
                    {"rotation_speed": sweep_hz},
                    {"sample_rate": 500}
        ]
    )
    
    static_laser_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_laser_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser']
    )
    
    pc_to_scan_node = Node(
        package='pointcloud_to_laserscan',
        executable = 'pointcloud_to_laserscan_node',
        output="screen",
        emulate_tty=True,
        remappings=[('cloud_in', 'pc2')],
        parameters=[                    {'min_height': -10.0},
                    {'max_height': 10.0},
                    {'angle_min': -135.0*pi/180.0},
                    {'angle_max': 135.0*pi/180.0},
                    {'angle_increment': pi/180.0},
                    {'scan_time': 1.0/sweep_hz},
                    {'range_min': 0.15},
                    {'range_max': 40.0},
                    {'use_inf': True},
                    {'queue_size': 1}
                    #{'target_frame': 'laser'},
                    #{'transform_tolerance': 0.001},
        ]
    )
    
    ld.add_action(sweep_node)
    ld.add_action(static_laser_tf_node)
    ld.add_action(pc_to_scan_node)
    ld.add_action(rviz_node)
    
    return ld
