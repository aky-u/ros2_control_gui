"""
Base controller class for different controller types.
"""

from abc import ABC, abstractmethod
from typing import List, Any


class BaseController(ABC):
    """Abstract base class for all controller types"""
    
    def __init__(self, ros_node, controller_info: dict):
        self.ros_node = ros_node
        self.controller_info = controller_info
        self.controller_name = controller_info['name']
        self.controller_type = controller_info['type']
        self.joint_names = controller_info.get('joint_names', [])
    
    @abstractmethod
    def create_publisher(self):
        """Create and return the appropriate publisher for this controller type"""
        pass
    
    @abstractmethod
    def send_command(self, publisher, joint_positions: List[float]):
        """Send command using the publisher"""
        pass
    
    @abstractmethod
    def get_topic_name(self) -> str:
        """Get the topic name for this controller"""
        pass
    
    def get_fallback_topic_name(self) -> str:
        """Get fallback topic name if not found in command_topics"""
        return f"/{self.controller_name}/commands"
