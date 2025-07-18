import sys
import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Float64MultiArray
from sensor_msgs.msg import JointState
from controller_manager_msgs.srv import ListControllers
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from PySide6.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QPushButton,
    QHBoxLayout, QLabel, QSlider, QDialog, QListWidget,
    QListWidgetItem, QSpinBox, QDoubleSpinBox, QGroupBox,
    QCheckBox
)
from PySide6.QtCore import QThread, Qt, QObject, Signal, QTimer


# Signal class for Qt-safe updates
class GuiSignals(QObject):
    joint_state_updated = Signal(dict)  # {joint_name: position}
    controllers_received = Signal(list)  # [(name, type, state)]
    controller_selected = Signal(str)  # controller_name
    controller_info_received = Signal(dict)  # controller configuration


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

    def joint_state_callback(self, msg: JointState):
        joint_data = dict(zip(msg.name, msg.position))
        self.gui_signals.joint_state_updated.emit(joint_data)
        
        # Store joint names for reference
        if not self.joint_names:
            self.joint_names = list(msg.name)

    def request_controllers(self):
        if not self.list_controllers_client.service_is_ready():
            self.get_logger().info("Waiting for list_controllers service...")
            return

        request = ListControllers.Request()
        future = self.list_controllers_client.call_async(request)
        future.add_done_callback(self.handle_controller_response)

    def handle_controller_response(self, future):
        try:
            response = future.result()
            controller_info = [
                (ctrl.name, ctrl.type, ctrl.state) for ctrl in response.controller
                if ctrl.state == 'active'  # Only show active controllers
            ]
            self.gui_signals.controllers_received.emit(controller_info)
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    def discover_controller_info(self, controller_name: str):
        """Discover controller information using topic introspection"""
        # Get all topics and their types
        topic_names_and_types = self.get_topic_names_and_types()
        
        # Find command topics for this controller
        command_topics = []
        for topic_name, topic_types in topic_names_and_types:
            if controller_name in topic_name and ('command' in topic_name or 'trajectory' in topic_name):
                command_topics.append((topic_name, topic_types[0]))
        
        # Determine controller type based on topic names and types
        controller_type = "unknown"
        if any('trajectory' in topic for topic, _ in command_topics):
            controller_type = "joint_trajectory_controller"
        elif any('command' in topic for topic, _ in command_topics):
            controller_type = "position_controller"
        
        # Create controller info structure
        controller_info = {
            'name': controller_name,
            'type': controller_type,
            'command_topics': command_topics,
            'joint_names': self.joint_names  # Use joint names from joint_states
        }
        
        self.current_controller_info = controller_info
        self.gui_signals.controller_info_received.emit(controller_info)

    def select_controller(self, controller_name: str):
        """Select and setup publisher for the chosen controller"""
        self.current_controller = controller_name
        
        # Discover controller info using topic introspection
        self.discover_controller_info(controller_name)

    def setup_publisher(self, controller_info: dict):
        """Setup publisher based on controller configuration"""
        controller_name = controller_info['name']
        
        # Destroy existing publisher if any
        if controller_name in self.command_publishers:
            self.command_publishers[controller_name].destroy()
        
        # Determine the appropriate topic and message type
        command_topics = controller_info.get('command_topics', [])
        
        if command_topics:
            topic_name, message_type = command_topics[0]
            
            if 'trajectory_msgs/msg/JointTrajectory' in message_type:
                self.command_publishers[controller_name] = self.create_publisher(
                    JointTrajectory, topic_name, 10
                )
            elif 'std_msgs/msg/Float64MultiArray' in message_type:
                self.command_publishers[controller_name] = self.create_publisher(
                    Float64MultiArray, topic_name, 10
                )
        else:
            # Fallback to naming convention
            if 'joint_trajectory_controller' in controller_name or 'trajectory' in controller_name:
                topic = f"/{controller_name}/joint_trajectory"
                self.command_publishers[controller_name] = self.create_publisher(
                    JointTrajectory, topic, 10
                )
            else:
                topic = f"/{controller_name}/commands"
                self.command_publishers[controller_name] = self.create_publisher(
                    Float64MultiArray, topic, 10
                )
        
        self.get_logger().info(f"Selected controller: {controller_name}")
        self.gui_signals.controller_selected.emit(controller_name)

    def send_joint_command(self, joint_positions: list):
        """Send command to the selected controller"""
        if not self.current_controller or self.current_controller not in self.command_publishers:
            self.get_logger().warn("No controller selected or publisher not ready")
            return

        publisher = self.command_publishers[self.current_controller]
        
        # Determine message type based on controller info
        is_trajectory = False
        if self.current_controller_info:
            command_topics = self.current_controller_info.get('command_topics', [])
            if command_topics:
                _, message_type = command_topics[0]
                is_trajectory = 'trajectory_msgs/msg/JointTrajectory' in message_type
            else:
                is_trajectory = 'joint_trajectory_controller' in self.current_controller or 'trajectory' in self.current_controller
        
        if is_trajectory:
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
        else:
            # Send Float64MultiArray message
            msg = Float64MultiArray()
            msg.data = joint_positions
            publisher.publish(msg)

        self.get_logger().info(f"Command sent to {self.current_controller}: {joint_positions}")


# ROS spinning thread
class RosSpinThread(QThread):
    def __init__(self, node: Node):
        super().__init__()
        self.node = node

    def run(self):
        rclpy.spin(self.node)


# Dialog to show and select controllers
class ControllersDialog(QDialog):
    def __init__(self, controllers: list, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Select Active Controller")
        self.setModal(True)
        self.selected_controller = None
        
        layout = QVBoxLayout(self)
        
        # Info label
        info_label = QLabel("Select a controller to command:")
        layout.addWidget(info_label)
        
        # Controller list
        self.list_widget = QListWidget()
        for name, ctrl_type, state in controllers:
            item = QListWidgetItem(f"{name}  [{ctrl_type}, {state}]")
            item.setData(Qt.UserRole, name)
            self.list_widget.addItem(item)
        
        self.list_widget.itemDoubleClicked.connect(self.on_item_selected)
        layout.addWidget(self.list_widget)
        
        # Buttons
        button_layout = QHBoxLayout()
        select_button = QPushButton("Select")
        select_button.clicked.connect(self.on_select_clicked)
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
        
        select_button = QPushButton("Select Controller")
        select_button.clicked.connect(self.show_controller_dialog)
        controller_layout.addWidget(select_button)
        
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
        self.ros_node.request_controllers()

    def handle_controllers_received(self, controllers: list):
        if not controllers:
            self.controller_label.setText("No active controllers found")
            return
            
        dialog = ControllersDialog(controllers, self)
        if dialog.exec() == QDialog.Accepted and dialog.selected_controller:
            self.ros_node.select_controller(dialog.selected_controller)

    def handle_controller_selected(self, controller_name: str):
        self.current_controller = controller_name
        self.controller_label.setText(f"Selected: {controller_name}")

    def handle_controller_info_received(self, controller_info: dict):
        """Handle controller configuration and setup UI accordingly"""
        self.current_controller_info = controller_info
        
        # Display controller info
        info_text = f"Type: {controller_info['type']}\n"
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
        ros_node.destroy_node()
        rclpy.shutdown()
        ros_thread.quit()
        ros_thread.wait()


if __name__ == "__main__":
    main()