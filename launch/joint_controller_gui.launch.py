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


def generate_launch_description():
    """Generate launch description for ROS 2 Control GUI"""
    
    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='',
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
    ])
