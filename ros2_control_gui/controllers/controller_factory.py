"""
Factory for creating controller instances based on controller type.
"""

from .base_controller import BaseController
from .position_controller import PositionController
from .trajectory_controller import TrajectoryController


class ControllerFactory:
    """Factory class for creating appropriate controller instances"""
    
    def __init__(self, ros_node):
        self.ros_node = ros_node
        
        # Mapping of controller types to controller classes
        self.controller_mapping = {
            'position': PositionController,
            'effort': PositionController,  # Use same as position for now
            'velocity': PositionController,  # Use same as position for now
            'trajectory': TrajectoryController,
        }
    
    def create_controller(self, controller_type: str, controller_info: dict) -> BaseController:
        """Create appropriate controller instance based on type"""
        message_type = self._determine_message_type(controller_type)
        
        controller_class = self.controller_mapping.get(message_type, PositionController)
        return controller_class(self.ros_node, controller_info)
    
    def _determine_message_type(self, controller_type: str) -> str:
        """Determine message type based on controller type"""
        controller_type_lower = controller_type.lower()
        
        # Check for trajectory controllers
        if ('joint_trajectory_controller' in controller_type_lower or
            'jointtrajectorycontroller' in controller_type_lower or
            'trajectory' in controller_type_lower):
            return 'trajectory'
        
        # Check for effort controllers
        if 'effort' in controller_type_lower:
            return 'effort'
        
        # Check for velocity controllers
        if 'velocity' in controller_type_lower:
            return 'velocity'
        
        # Default to position
        return 'position'
    
    def register_controller_type(self, message_type: str, controller_class):
        """Register a new controller type"""
        self.controller_mapping[message_type] = controller_class
    
    def get_supported_types(self) -> list:
        """Get list of supported controller types"""
        return list(self.controller_mapping.keys())
