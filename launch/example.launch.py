#!/usr/bin/env python3
"""
Launch file for ROS 2 Control GUI with optional YAML configuration.

Usage examples:
  # Launch without config file (uses joint_states fallback)
  ros2 launch ros2_control_gui joint_controller_gui.launch.py
  
  # Launch with specific config file
  ros2 launch ros2_control_gui joint_controller_gui.launch.py config_file:=/path/to/config.yaml
  
  # Launch with relative path (expands from current working directory)
  ros2 launch ros2_control_gui joint_controller_gui.launch.py config_file:=./config/robot_config.yaml
  
  # Launch with environment variable
  ros2 launch ros2_control_gui joint_controller_gui.launch.py config_file:=$HOME/robot_configs/my_robot.yaml
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    """Generate launch description for ROS 2 Control GUI"""

    ros2_control_demo_package = get_package_share_directory('ros2_control_demo_example_1')
    rrbot_launch_path = os.path.join(
      ros2_control_demo_package,
      'launch',
      'rrbot.launch.py'
    )

    rrbot_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(rrbot_launch_path)
    )
    
    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=ros2_control_demo_package + '/config/rrbot_controllers.yaml',
        description='Path to YAML configuration file for joint controllers. '
                   'If not specified, the GUI will use joint_states as fallback '
                   'or allow manual YAML loading through the GUI.'
    )
    
    # Node configuration
    joint_controller_gui_node = Node(
        package='ros2_control_gui',
        executable='joint_controller_gui',
        name='joint_controller_gui',
        parameters=[{
            'config_file': LaunchConfiguration('config_file')
        }],
        output='screen',
        emulate_tty=True,  # Ensures proper console output formatting
    )
    
    return LaunchDescription([
        config_file_arg,
        joint_controller_gui_node,
        rrbot_launch
    ])
