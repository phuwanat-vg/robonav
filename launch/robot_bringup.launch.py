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
        Node(
            package='robonav',
            executable='imu_publisher',
            name='imu_converter_node',
            output='screen',
        ),

        Node(
            package = 'tf2_ros',
            executable = 'static_transform_publisher',
            name = 'imu_tf_publisher',
            arguments = ['0.1', '0.0', '0.05', '0.0', '0.0', '0.0', '1.0', 'base_link', 'imu_link'],
        ),

        #Node(package='tf2_ros',
        #    executable='static_transform_publisher',
        #    name='static_tf_pub_laser',
        #    arguments=['0.05', '0', '0.2','3.14159', '0', '0' ,'base_link','laser_frame'],
        #),

        Node(
            package = 'robot_localization',
            executable = 'ekf_node',
            name = 'ekf_filter_node',
            output = 'screen',
            parameters = [
                 ekf_config_path
            ],
            remappings = [("odometry/filtered","odom")]
        ),

    ])