import sys
import rclpy
from rclpy.node import Node
import threading

from std_msgs.msg import String, Float64MultiArray
from sensor_msgs.msg import JointState
from controller_manager_msgs.srv import ListControllers
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from PySide6.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QPushButton,
    QHBoxLayout, QLabel, QSlider, QDialog, QListWidget,
    QListWidgetItem, QSpinBox, QDoubleSpinBox, QGroupBox,
    QCheckBox, QMessageBox
)
from PySide6.QtCore import QThread, Qt, QObject, Signal, QTimer


# Signal class for Qt-safe updates
class GuiSignals(QObject):
    joint_state_updated = Signal(dict)  # {joint_name: position}
    controllers_received = Signal(list)  # [(name, type, state)]
    controller_selected = Signal(str)  # controller_name
    controller_info_received = Signal(dict)  # controller configuration
    service_error = Signal(str)  # Error message


# ROS 2 Node
class JointControllerNode(Node):
    def __init__(self):
        super().__init__('joint_controller_node')

        self.joint_state_sub = self.create_subscription(
            JointState, "/joint_states", self.joint_state_callback, 10
        )

        self.list_controllers_client = self.create_client(
            ListControllers, '/controller_manager/list_controllers'
        )

        self.gui_signals = GuiSignals()
        self.command_publishers = {}
        self.current_controller = None
        self.current_controller_info = None
        self.joint_names = []
        self.service_in_progress = False
        self.controller_types = {}  # Store controller types from service
        self._publisher_lock = threading.Lock()  # Thread safety for publishers
        self._shutdown_requested = False

    def joint_state_callback(self, msg: JointState):
        if self._shutdown_requested:
            return
            
        joint_data = dict(zip(msg.name, msg.position))
        self.gui_signals.joint_state_updated.emit(joint_data)
        
        # Store joint names for reference
        if not self.joint_names:
            self.joint_names = list(msg.name)

    def request_controllers(self):
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
                    # Store the controller type from the service
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

    def discover_controller_info(self, controller_name: str):
        """Discover controller information using service data and topic introspection"""
        if self._shutdown_requested:
            return
            
        try:
            # Get controller type from service response
            controller_type = self.controller_types.get(controller_name, "unknown")
            
            # Get all topics and their types
            topic_names_and_types = self.get_topic_names_and_types()
            
            # Find command topics for this controller
            command_topics = []
            for topic_name, topic_types in topic_names_and_types:
                if controller_name in topic_name and ('command' in topic_name or 'trajectory' in topic_name):
                    command_topics.append((topic_name, topic_types[0]))
            
            # Create controller info structure using actual controller type from service
            controller_info = {
                'name': controller_name,
                'type': controller_type,  # Use type from service response
                'command_topics': command_topics,
                'joint_names': self.joint_names  # Use joint names from joint_states
            }
            
            self.current_controller_info = controller_info
            self.gui_signals.controller_info_received.emit(controller_info)
            
        except Exception as e:
            self.get_logger().error(f"Failed to discover controller info for {controller_name}: {e}")
            self.gui_signals.service_error.emit(f"Failed to discover controller info: {e}")

    def select_controller(self, controller_name: str):
        """Select and setup publisher for the chosen controller"""
        if self._shutdown_requested:
            return
            
        try:
            self.current_controller = controller_name
            self.get_logger().info(f"Selecting controller: {controller_name}")
            
            # Discover controller info using service data and topic introspection
            self.discover_controller_info(controller_name)
            
        except Exception as e:
            self.get_logger().error(f"Failed to select controller {controller_name}: {e}")
            self.gui_signals.service_error.emit(f"Failed to select controller: {e}")

    def determine_message_type_from_controller_type(self, controller_type: str) -> str:
        """Determine message type based on actual controller type from service"""
        controller_type_lower = controller_type.lower()
        
        # Check for trajectory controllers
        if ('joint_trajectory_controller' in controller_type_lower or
            'jointtrajectorycontroller' in controller_type_lower or
            'trajectory' in controller_type_lower):
            return 'trajectory'
        
        # For all other controller types, use Float64MultiArray
        return 'position'

    def cleanup_publisher(self, controller_name: str):
        """Safely cleanup existing publisher with thread safety"""
        with self._publisher_lock:
            if controller_name in self.command_publishers:
                try:
                    publisher = self.command_publishers[controller_name]
                    # Don't call destroy() as it can cause issues during shutdown
                    del self.command_publishers[controller_name]
                    self.get_logger().info(f"Cleaned up publisher for {controller_name}")
                except Exception as e:
                    self.get_logger().warn(f"Error cleaning up publisher for {controller_name}: {e}")

    def setup_publisher(self, controller_info: dict):
        """Setup publisher based on controller configuration"""
        if self._shutdown_requested:
            return
            
        controller_name = controller_info['name']
        controller_type = controller_info['type']
        
        try:
            # Clean up existing publisher
            self.cleanup_publisher(controller_name)
            
            # Determine message type based on actual controller type
            message_type = self.determine_message_type_from_controller_type(controller_type)
            
            # Determine the appropriate topic
            command_topics = controller_info.get('command_topics', [])
            
            if command_topics:
                topic_name, _ = command_topics[0]
            else:
                # Fallback to naming convention based on message type
                if message_type == 'trajectory':
                    topic_name = f"/{controller_name}/joint_trajectory"
                else:
                    topic_name = f"/{controller_name}/commands"
            
            # Create appropriate publisher based on determined message type
            with self._publisher_lock:
                if message_type == 'trajectory':
                    self.command_publishers[controller_name] = self.create_publisher(
                        JointTrajectory, topic_name, 10
                    )
                else:
                    self.command_publishers[controller_name] = self.create_publisher(
                        Float64MultiArray, topic_name, 10
                    )
            
            self.get_logger().info(f"Successfully setup publisher for controller: {controller_name} (type: {controller_type}, message: {message_type})")
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
                
                # Determine message type based on actual controller type
                controller_type = self.current_controller_info.get('type', '')
                message_type = self.determine_message_type_from_controller_type(controller_type)
                
                if message_type == 'trajectory':
                    # Send JointTrajectory message
                    msg = JointTrajectory()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    
                    # Use joint names from joint_states
                    msg.joint_names = self.joint_names[:len(joint_positions)]
                    
                    point = JointTrajectoryPoint()
                    point.positions = joint_positions
                    point.time_from_start.sec = 1
                    msg.points = [point]
                    
                    publisher.publish(msg)
                    self.get_logger().info(f"JointTrajectory command sent to {self.current_controller}: {joint_positions}")
                else:
                    # Send Float64MultiArray message
                    msg = Float64MultiArray()
                    msg.data = joint_positions
                    publisher.publish(msg)
                    self.get_logger().info(f"Float64MultiArray command sent to {self.current_controller}: {joint_positions}")
            
        except Exception as e:
            self.get_logger().error(f"Failed to send command: {e}")

    def shutdown(self):
        """Safely shutdown the node"""
        self._shutdown_requested = True
        
        # Clean up all publishers
        with self._publisher_lock:
            controller_names = list(self.command_publishers.keys())
            for controller_name in controller_names:
                try:
                    del self.command_publishers[controller_name]
                except Exception as e:
                    self.get_logger().warn(f"Error during shutdown cleanup: {e}")
            self.command_publishers.clear()


# ROS spinning thread with improved error handling
class RosSpinThread(QThread):
    def __init__(self, node: Node):
        super().__init__()
        self.node = node
        self._shutdown_requested = False

    def run(self):
        try:
            while not self._shutdown_requested and rclpy.ok():
                try:
                    rclpy.spin_once(self.node, timeout_sec=0.1)
                except Exception as e:
                    if not self._shutdown_requested:
                        print(f"ROS spin error: {e}")
                    break
        except Exception as e:
            if not self._shutdown_requested:
                print(f"ROS thread error: {e}")

    def shutdown(self):
        """Request shutdown of the thread"""
        self._shutdown_requested = True


# Dialog to show and select controllers
class ControllersDialog(QDialog):
    def __init__(self, controllers: list, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Select Active Controller")
        self.setModal(True)
        self.selected_controller = None
        self.resize(500, 300)
        
        layout = QVBoxLayout(self)
        
        # Info label
        info_label = QLabel("Select a controller to command:")
        layout.addWidget(info_label)
        
        # Controller list with improved display
        self.list_widget = QListWidget()
        self.list_widget.setAlternatingRowColors(True)
        
        for name, ctrl_type, state in controllers:
            # Create a more detailed display format
            display_text = f"{name}\n  Type: {ctrl_type}\n  State: {state}"
            item = QListWidgetItem(display_text)
            item.setData(Qt.UserRole, name)
            
            # Add tooltip with full information
            tooltip = f"Controller: {name}\nType: {ctrl_type}\nState: {state}"
            item.setToolTip(tooltip)
            
            self.list_widget.addItem(item)
        
        self.list_widget.itemDoubleClicked.connect(self.on_item_selected)
        layout.addWidget(self.list_widget)
        
        # Status label
        status_label = QLabel(f"Found {len(controllers)} active controller(s)")
        status_label.setStyleSheet("color: gray; font-style: italic;")
        layout.addWidget(status_label)
        
        # Buttons
        button_layout = QHBoxLayout()
        select_button = QPushButton("Select")
        select_button.clicked.connect(self.on_select_clicked)
        select_button.setDefault(True)
        
        cancel_button = QPushButton("Cancel")
        cancel_button.clicked.connect(self.reject)
        
        button_layout.addWidget(select_button)
        button_layout.addWidget(cancel_button)
        layout.addLayout(button_layout)

    def on_item_selected(self, item):
        self.selected_controller = item.data(Qt.UserRole)
        self.accept()

    def on_select_clicked(self):
        current_item = self.list_widget.currentItem()
        if current_item:
            self.selected_controller = current_item.data(Qt.UserRole)
            self.accept()


# Main GUI Window
class MainWindow(QWidget):
    def __init__(self, ros_node: JointControllerNode):
        super().__init__()
        self.setWindowTitle("Joint Controller GUI")
        self.ros_node = ros_node
        self.layout = QVBoxLayout(self)
        self.sliders = {}
        self.current_controller = None
        self.current_controller_info = None
        
        # Continuous command timer
        self.continuous_timer = QTimer()
        self.continuous_timer.timeout.connect(self.send_continuous_command)

        # Controller selection section
        controller_group = QGroupBox("Controller Selection")
        controller_layout = QVBoxLayout()
        
        self.controller_label = QLabel("No controller selected")
        controller_layout.addWidget(self.controller_label)
        
        self.select_button = QPushButton("Select Controller")
        self.select_button.clicked.connect(self.show_controller_dialog)
        controller_layout.addWidget(self.select_button)
        
        # Controller info display
        self.controller_info_label = QLabel("No controller info")
        controller_layout.addWidget(self.controller_info_label)
        
        controller_group.setLayout(controller_layout)
        self.layout.addWidget(controller_group)

        # Continuous command section
        continuous_group = QGroupBox("Continuous Command Settings")
        continuous_layout = QHBoxLayout()
        
        # Continuous checkbox
        self.continuous_checkbox = QCheckBox("Send Continuous")
        self.continuous_checkbox.toggled.connect(self.on_continuous_toggled)
        continuous_layout.addWidget(self.continuous_checkbox)
        
        # Frequency setting
        freq_label = QLabel("Frequency (Hz):")
        continuous_layout.addWidget(freq_label)
        
        self.frequency_spinbox = QDoubleSpinBox()
        self.frequency_spinbox.setRange(0.1, 100.0)
        self.frequency_spinbox.setValue(10.0)
        self.frequency_spinbox.setSingleStep(0.5)
        self.frequency_spinbox.setDecimals(1)
        self.frequency_spinbox.valueChanged.connect(self.on_frequency_changed)
        continuous_layout.addWidget(self.frequency_spinbox)
        
        continuous_group.setLayout(continuous_layout)
        self.layout.addWidget(continuous_group)

        # Joint control section
        self.joint_group = QGroupBox("Joint Control")
        self.joint_layout = QVBoxLayout()
        self.joint_group.setLayout(self.joint_layout)
        self.joint_group.setEnabled(False)
        self.layout.addWidget(self.joint_group)

        # Command button
        self.command_button = QPushButton("Send Command")
        self.command_button.clicked.connect(self.send_command)
        self.command_button.setEnabled(False)
        self.layout.addWidget(self.command_button)

        # Connect ROS-to-GUI signals
        self.ros_node.gui_signals.joint_state_updated.connect(
            self.update_joint_sliders
        )
        self.ros_node.gui_signals.controllers_received.connect(
            self.handle_controllers_received
        )
        self.ros_node.gui_signals.controller_selected.connect(
            self.handle_controller_selected
        )
        self.ros_node.gui_signals.controller_info_received.connect(
            self.handle_controller_info_received
        )
        self.ros_node.gui_signals.service_error.connect(
            self.handle_service_error
        )

        # Auto-request controllers on startup
        self.ros_node.request_controllers()

    def on_continuous_toggled(self, checked: bool):
        """Handle continuous mode toggle"""
        if checked:
            frequency = self.frequency_spinbox.value()
            interval_ms = int(1000 / frequency)
            self.continuous_timer.start(interval_ms)
            self.command_button.setText("Stop Continuous")
            self.get_logger().info(f"Continuous mode started at {frequency} Hz")
        else:
            self.continuous_timer.stop()
            self.command_button.setText("Send Command")
            self.get_logger().info("Continuous mode stopped")

    def on_frequency_changed(self, frequency: float):
        """Handle frequency change"""
        if self.continuous_timer.isActive():
            interval_ms = int(1000 / frequency)
            self.continuous_timer.start(interval_ms)  # Restart with new interval
            self.get_logger().info(f"Frequency changed to {frequency} Hz")

    def send_continuous_command(self):
        """Send command in continuous mode"""
        if self.current_controller and self.current_controller_info:
            self.send_command()

    def show_controller_dialog(self):
        """Show controller selection dialog"""
        self.select_button.setEnabled(False)
        self.select_button.setText("Loading...")
        
        # Request controllers with improved error handling
        success = self.ros_node.request_controllers()
        if not success:
            self.select_button.setEnabled(True)
            self.select_button.setText("Select Controller")

    def handle_controllers_received(self, controllers: list):
        """Handle successful controller list reception"""
        self.select_button.setEnabled(True)
        self.select_button.setText("Select Controller")
        
        if not controllers:
            self.controller_label.setText("No active controllers found")
            return
            
        dialog = ControllersDialog(controllers, self)
        if dialog.exec() == QDialog.Accepted and dialog.selected_controller:
            self.ros_node.select_controller(dialog.selected_controller)

    def handle_service_error(self, error_message: str):
        """Handle service errors"""
        self.select_button.setEnabled(True)
        self.select_button.setText("Select Controller")
        
        QMessageBox.warning(self, "Service Error", error_message)

    def handle_controller_selected(self, controller_name: str):
        self.current_controller = controller_name
        self.controller_label.setText(f"Selected: {controller_name}")

    def handle_controller_info_received(self, controller_info: dict):
        """Handle controller configuration and setup UI accordingly"""
        self.current_controller_info = controller_info
        
        # Display controller info with actual type from service
        info_text = f"Name: {controller_info['name']}\n"
        info_text += f"Type: {controller_info['type']}\n"
        info_text += f"Joint Names: {len(controller_info['joint_names'])}\n"
        info_text += f"Command Topics: {controller_info.get('command_topics', [])}"
        self.controller_info_label.setText(info_text)
        
        # Setup publisher
        self.ros_node.setup_publisher(controller_info)
        
        # Clear existing sliders
        self.clear_sliders()
        
        # Create sliders based on joint names
        joint_names = controller_info['joint_names']
        for i, joint_name in enumerate(joint_names):
            self.create_slider(joint_name, i)
        
        self.joint_group.setEnabled(True)
        self.command_button.setEnabled(True)

    def clear_sliders(self):
        """Clear all existing sliders"""
        for widgets in self.sliders.values():
            widgets['slider'].deleteLater()
            widgets['label'].deleteLater()
        self.sliders.clear()
        
        # Clear layout
        while self.joint_layout.count():
            child = self.joint_layout.takeAt(0)
            if child.widget():
                child.widget().deleteLater()

    def create_slider(self, joint_name: str, index: int):
        """Create a slider for a joint"""
        row = QHBoxLayout()
        
        # Joint name label
        label = QLabel(f"{joint_name}:")
        label.setMinimumWidth(150)
        row.addWidget(label)
        
        # Command slider
        slider = QSlider(Qt.Horizontal)
        slider.setRange(-314, 314)  # -3.14 to 3.14 radians * 100
        slider.setValue(0)
        row.addWidget(slider)
        
        # Value display
        value_label = QLabel("0.000")
        value_label.setMinimumWidth(60)
        row.addWidget(value_label)
        
        # Connect slider to value update
        slider.valueChanged.connect(
            lambda v, lbl=value_label: lbl.setText(f"{v/100:.3f}")
        )
        
        # Connect mouse release event for auto-reset in continuous mode
        slider.sliderReleased.connect(
            lambda s=slider: self.on_slider_released(s)
        )
        
        self.joint_layout.addLayout(row)
        self.sliders[joint_name] = {'slider': slider, 'label': value_label, 'index': index}

    def on_slider_released(self, slider: QSlider):
        """Handle slider release - reset to zero if in continuous mode"""
        if self.continuous_checkbox.isChecked():
            slider.setValue(0)

    def update_joint_sliders(self, joint_data: dict):
        """Update slider background colors to show current joint states"""
        # Optional: Update slider tooltips or background to show current position
        pass

    def send_command(self):
        if not self.current_controller or not self.current_controller_info:
            return
            
        # Get current slider values in the correct order
        joint_names = self.current_controller_info['joint_names']
        joint_positions = [0.0] * len(joint_names)
        
        for joint_name, widgets in self.sliders.items():
            if joint_name in joint_names:
                position = widgets['slider'].value() / 100.0
                index = joint_names.index(joint_name)
                joint_positions[index] = position
        
        # Send command to ROS node
        self.ros_node.send_joint_command(joint_positions)

    def get_logger(self):
        """Helper method to access ROS logger"""
        return self.ros_node.get_logger()

    def closeEvent(self, event):
        """Handle window close event"""
        if self.continuous_timer.isActive():
            self.continuous_timer.stop()
        event.accept()


def main():
    rclpy.init()

    ros_node = JointControllerNode()
    ros_thread = RosSpinThread(ros_node)
    ros_thread.start()

    app = QApplication(sys.argv)
    window = MainWindow(ros_node)
    window.resize(800, 700)
    window.show()

    try:
        sys.exit(app.exec())
    finally:
        # Proper shutdown sequence
        ros_thread.shutdown()
        ros_node.shutdown()
        ros_thread.wait(timeout=2000)  # Wait up to 2 seconds
        ros_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()