from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, TextSubstitution
import os

def generate_launch_description():
    launch_file_dir = os.path.dirname(os.path.realpath(__file__))
    params_file_path = os.path.join(launch_file_dir, 'params.yaml')
    # print(f"\nparams_file_path: {params_file_path}\n")
    return LaunchDescription([
        # Declare the launch argument for the YAML file to load
        DeclareLaunchArgument(
            'param_file',
            default_value=params_file_path,
            description='Path to the ROS2 parameters file to load realative to the inertial-sense-sdk/ROS/ros2/launch directory'
        ),

        Node(
            package='inertial_sense_ros2',
            executable='inertial_sense_ros2_node',
            name='inertial_sense_ros2',
            namespace='inertial_sense',
            output='screen',
            arguments=[LaunchConfiguration('param_file')] # Pass as argv[1]
        ),

        LogInfo(
            msg = [
                TextSubstitution(text="Parameter file: "),
                LaunchConfiguration('param_file')
            ]
        ),
    ])