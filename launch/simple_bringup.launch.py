from http.server import executable
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    ekf_config_path = PathJoinSubstitution(
        [FindPackageShare('robonav'),'config','ekf.yaml']
    )

    lidar_launch_path = PathJoinSubstitution(
        [FindPackageShare('ydlidar_ros2_driver'),'launch','ydlidar_launch.py']
    )

    return LaunchDescription([
        Node(
            package="micro_ros_agent",
            executable="micro_ros_agent",
            name="micro_ros_agent_teensy",
            arguments=['serial', '--dev','/dev/ttyACM0'],
            output='screen',
        ),

        Node(
            package='robonav',
            executable='robot_core_node',
            name='robot_core_node',
            output='screen',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lidar_launch_path)

        ),

      

    ])
