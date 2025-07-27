"""
ROS 2 node for joint controller communication.
"""

import rclpy
from rclpy.node import Node
import threading

from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from controller_manager_msgs.srv import ListControllers
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from PySide6.QtCore import QObject, Signal

from ..config.yaml_config import YamlConfigManager
from ..controllers.controller_factory import ControllerFactory


class GuiSignals(QObject):
    """Qt signals for thread-safe GUI updates"""
    joint_state_updated = Signal(dict)  # {joint_name: position}
    controllers_received = Signal(list)  # [(name, type, state)]
    controller_selected = Signal(str)  # controller_name
    controller_info_received = Signal(dict)  # controller configuration
    service_error = Signal(str)  # Error message


class JointControllerNode(Node):
    """Main ROS 2 node for controller communication"""
    
    def __init__(self):
        super().__init__('joint_controller_node')
        
        # ROS subscriptions and clients
        self.joint_state_sub = self.create_subscription(
            JointState, "/joint_states", self.joint_state_callback, 10
        )
        self.list_controllers_client = self.create_client(
            ListControllers, '/controller_manager/list_controllers'
        )
        
        # Core components
        self.gui_signals = GuiSignals()
        self.yaml_config = YamlConfigManager()
        self.controller_factory = ControllerFactory(self)
        
        # State management
        self.command_publishers = {}
        self.current_controller = None
        self.current_controller_info = None
        self.joint_names = []  # Joint names from joint_states
        self.service_in_progress = False
        self.controller_types = {}  # Store controller types from service
        self._publisher_lock = threading.Lock()
        self._shutdown_requested = False

    def joint_state_callback(self, msg: JointState):
        """Handle incoming joint state messages"""
        if self._shutdown_requested:
            return
            
        joint_data = dict(zip(msg.name, msg.position))
        self.gui_signals.joint_state_updated.emit(joint_data)
        
        # Store joint names for reference
        if not self.joint_names:
            self.joint_names = list(msg.name)
    
    def load_yaml_config(self, yaml_file_path: str) -> bool:
        """Load YAML configuration file"""
        return self.yaml_config.load_config(yaml_file_path)
    
    def request_controllers(self) -> bool:
        """Request list of active controllers"""
        if self._shutdown_requested:
            return False
            
        if self.service_in_progress:
            self.get_logger().warn("Service call already in progress, please wait...")
            return False

        if not self.list_controllers_client.service_is_ready():
            self.get_logger().info("Waiting for list_controllers service...")
            self.gui_signals.service_error.emit("Controller manager service not ready. Please wait...")
            return False

        try:
            self.service_in_progress = True
            request = ListControllers.Request()
            future = self.list_controllers_client.call_async(request)
            future.add_done_callback(self.handle_controller_response)
            self.get_logger().info("Requesting controllers...")
            return True
        except Exception as e:
            self.service_in_progress = False
            self.get_logger().error(f"Failed to send service request: {e}")
            self.gui_signals.service_error.emit(f"Failed to request controllers: {e}")
            return False

    def handle_controller_response(self, future):
        """Handle controller list response"""
        if self._shutdown_requested:
            return
            
        try:
            self.service_in_progress = False
            response = future.result()
            
            if not response:
                self.get_logger().error("Received empty response from controller service")
                self.gui_signals.service_error.emit("Empty response from controller service")
                return

            controller_info = []
            active_count = 0
            
            for ctrl in response.controller:
                if ctrl.state == 'active':
                    controller_info.append((ctrl.name, ctrl.type, ctrl.state))
                    self.controller_types[ctrl.name] = ctrl.type
                    active_count += 1
            
            self.get_logger().info(f"Found {active_count} active controllers out of {len(response.controller)} total")
            
            if not controller_info:
                self.gui_signals.service_error.emit("No active controllers found")
            else:
                self.gui_signals.controllers_received.emit(controller_info)
                
        except Exception as e:
            self.service_in_progress = False
            self.get_logger().error(f"Service call failed: {e}")
            self.gui_signals.service_error.emit(f"Service call failed: {e}")

    def select_controller(self, controller_name: str):
        """Select and configure a controller"""
        if self._shutdown_requested:
            return
            
        try:
            self.current_controller = controller_name
            self.get_logger().info(f"Selecting controller: {controller_name}")
            
            # Discover controller info
            self.discover_controller_info(controller_name)
            
        except Exception as e:
            self.get_logger().error(f"Failed to select controller {controller_name}: {e}")
            self.gui_signals.service_error.emit(f"Failed to select controller: {e}")

    def discover_controller_info(self, controller_name: str):
        """Discover controller configuration information"""
        if self._shutdown_requested:
            return
            
        try:
            controller_type = self.controller_types.get(controller_name, "unknown")
            
            # Get joint names from YAML config or fallback to joint_states
            controller_joint_names, config_source = self._get_controller_joint_names(controller_name)
            
            # Find command topics
            command_topics = self._find_command_topics(controller_name)
            
            # Create controller info structure
            controller_info = {
                'name': controller_name,
                'type': controller_type,
                'command_topics': command_topics,
                'joint_names': controller_joint_names,
                'config_source': config_source
            }
            
            self.current_controller_info = controller_info
            self.gui_signals.controller_info_received.emit(controller_info)
            
        except Exception as e:
            self.get_logger().error(f"Failed to discover controller info for {controller_name}: {e}")
            self.gui_signals.service_error.emit(f"Failed to discover controller info: {e}")

    def _get_controller_joint_names(self, controller_name: str):
        """Get joint names for controller from YAML config or fallback"""
        if self.yaml_config.is_loaded():
            joint_names = self.yaml_config.get_controller_joint_names(controller_name)
            if joint_names:
                self.get_logger().info(f"Using joint names from YAML config for {controller_name}: {joint_names}")
                return joint_names, f"YAML: {self.yaml_config.get_config_filename()}"
            else:
                self.get_logger().info(f"Controller {controller_name} not found in YAML config, using joint_states as fallback: {self.joint_names}")
                return self.joint_names.copy(), "joint_states (controller not in YAML)"
        else:
            self.get_logger().info(f"No YAML config loaded, using joint_states for {controller_name}: {self.joint_names}")
            return self.joint_names.copy(), "joint_states (no YAML loaded)"

    def _find_command_topics(self, controller_name: str):
        """Find command topics for the controller"""
        topic_names_and_types = self.get_topic_names_and_types()
        command_topics = []
        
        for topic_name, topic_types in topic_names_and_types:
            if controller_name in topic_name and ('command' in topic_name or 'trajectory' in topic_name):
                command_topics.append((topic_name, topic_types[0]))
        
        return command_topics

    def setup_publisher(self, controller_info: dict):
        """Setup publisher for the selected controller"""
        if self._shutdown_requested:
            return
            
        controller_name = controller_info['name']
        controller_type = controller_info['type']
        
        try:
            # Use factory to create appropriate controller
            controller = self.controller_factory.create_controller(controller_type, controller_info)
            
            # Setup publisher using the controller
            with self._publisher_lock:
                # Cleanup existing publisher
                if controller_name in self.command_publishers:
                    del self.command_publishers[controller_name]
                
                # Create new publisher
                self.command_publishers[controller_name] = controller.create_publisher()
            
            self.get_logger().info(f"Successfully setup publisher for controller: {controller_name} (type: {controller_type})")
            self.gui_signals.controller_selected.emit(controller_name)
            
        except Exception as e:
            self.get_logger().error(f"Failed to setup publisher for {controller_name}: {e}")
            self.gui_signals.service_error.emit(f"Failed to setup publisher: {e}")

    def send_joint_command(self, joint_positions: list):
        """Send command to the selected controller"""
        if self._shutdown_requested:
            return
            
        if not self.current_controller or self.current_controller not in self.command_publishers:
            self.get_logger().warn("No controller selected or publisher not ready")
            return

        try:
            with self._publisher_lock:
                if self.current_controller not in self.command_publishers:
                    return
                    
                publisher = self.command_publishers[self.current_controller]
                controller_type = self.current_controller_info.get('type', '')
                
                # Use factory to create and send message
                controller = self.controller_factory.create_controller(controller_type, self.current_controller_info)
                controller.send_command(publisher, joint_positions)
            
        except Exception as e:
            self.get_logger().error(f"Failed to send command: {e}")

    def shutdown(self):
        """Safely shutdown the node"""
        self._shutdown_requested = True
        
        with self._publisher_lock:
            self.command_publishers.clear()
