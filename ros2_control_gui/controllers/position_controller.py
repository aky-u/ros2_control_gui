"""
Position controller implementation.
"""

from typing import List
from std_msgs.msg import Float64MultiArray
from .base_controller import BaseController


class PositionController(BaseController):
    """Controller for position-based commands using Float64MultiArray"""
    
    def create_publisher(self):
        """Create publisher for Float64MultiArray messages"""
        topic_name = self.get_topic_name()
        return self.ros_node.create_publisher(Float64MultiArray, topic_name, 10)
    
    def send_command(self, publisher, joint_positions: List[float]):
        """Send Float64MultiArray command"""
        msg = Float64MultiArray()
        msg.data = joint_positions
        publisher.publish(msg)
        
        self.ros_node.get_logger().info(
            f"Float64MultiArray command sent to {self.controller_name}: {joint_positions}"
        )
    
    def get_topic_name(self) -> str:
        """Get topic name for position controller"""
        command_topics = self.controller_info.get('command_topics', [])
        
        if command_topics:
            return command_topics[0][0]
        
        # Fallback naming conventions
        possible_topics = [
            f"/{self.controller_name}/commands",
            f"/{self.controller_name}/command",
            f"/{self.controller_name}/reference"
        ]
        
        return possible_topics[0]
