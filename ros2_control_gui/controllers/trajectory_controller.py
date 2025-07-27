"""
Trajectory controller implementation.
"""

from typing import List
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from .base_controller import BaseController


class TrajectoryController(BaseController):
    """Controller for trajectory-based commands using JointTrajectory"""
    
    def create_publisher(self):
        """Create publisher for JointTrajectory messages"""
        topic_name = self.get_topic_name()
        return self.ros_node.create_publisher(JointTrajectory, topic_name, 10)
    
    def send_command(self, publisher, joint_positions: List[float]):
        """Send JointTrajectory command"""
        msg = JointTrajectory()
        msg.header.stamp = self.ros_node.get_clock().now().to_msg()
        msg.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.time_from_start.sec = 1
        msg.points = [point]
        
        publisher.publish(msg)
        
        self.ros_node.get_logger().info(
            f"JointTrajectory command sent to {self.controller_name} with joints {self.joint_names}: {joint_positions}"
        )
    
    def get_topic_name(self) -> str:
        """Get topic name for trajectory controller"""
        command_topics = self.controller_info.get('command_topics', [])
        
        if command_topics:
            return command_topics[0][0]
        
        # Fallback naming conventions for trajectory controllers
        possible_topics = [
            f"/{self.controller_name}/joint_trajectory",
            f"/{self.controller_name}/command",
            f"/{self.controller_name}/commands"
        ]
        
        return possible_topics[0]
