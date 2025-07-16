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
    QListWidgetItem, QSpinBox, QDoubleSpinBox, QGroupBox
)
from PySide6.QtCore import QThread, Qt, QObject, Signal


# Signal class for Qt-safe updates
class GuiSignals(QObject):
    joint_state_updated = Signal(dict)  # {joint_name: position}
    controllers_received = Signal(list)  # [(name, state)]
    controller_selected = Signal(str)  # controller_name


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
        self.joint_names = []

    def joint_state_callback(self, msg: JointState):
        joint_data = dict(zip(msg.name, msg.position))
        self.gui_signals.joint_state_updated.emit(joint_data)
        
        # Store joint names for commanding
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
                (ctrl.name, ctrl.state) for ctrl in response.controller
                if ctrl.state == 'active'  # Only show active controllers
            ]
            self.gui_signals.controllers_received.emit(controller_info)
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    def select_controller(self, controller_name: str):
        """Select and setup publisher for the chosen controller"""
        self.current_controller = controller_name
        
        # Destroy existing publisher if any
        if controller_name in self.command_publishers:
            self.command_publishers[controller_name].destroy()
        
        # Create appropriate publisher based on controller type
        if 'joint_trajectory_controller' in controller_name or 'trajectory' in controller_name:
            topic = f"/{controller_name}/joint_trajectory"
            self.command_publishers[controller_name] = self.create_publisher(
                JointTrajectory, topic, 10
            )
        else:
            # For position/velocity/effort controllers
            topic = f"/{controller_name}/commands"
            self.command_publishers[controller_name] = self.create_publisher(
                Float64MultiArray, topic, 10
            )
        
        self.get_logger().info(f"Selected controller: {controller_name}")
        self.gui_signals.controller_selected.emit(controller_name)

    def send_joint_command(self, joint_positions: dict):
        """Send command to the selected controller"""
        if not self.current_controller or self.current_controller not in self.command_publishers:
            self.get_logger().warn("No controller selected or publisher not ready")
            return

        publisher = self.command_publishers[self.current_controller]
        
        if 'joint_trajectory_controller' in self.current_controller or 'trajectory' in self.current_controller:
            # Send JointTrajectory message
            msg = JointTrajectory()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.joint_names = list(joint_positions.keys())
            
            point = JointTrajectoryPoint()
            point.positions = list(joint_positions.values())
            point.time_from_start.sec = 1  # 1 second to reach target
            msg.points = [point]
            
            publisher.publish(msg)
        else:
            # Send Float64MultiArray message
            msg = Float64MultiArray()
            # Ensure positions are in the same order as joint_names
            msg.data = [joint_positions.get(name, 0.0) for name in self.joint_names]
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
        for name, state in controllers:
            item = QListWidgetItem(f"{name}  [{state}]")
            item.setData(Qt.UserRole, name)  # Store controller name
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

        # Controller selection section
        controller_group = QGroupBox("Controller Selection")
        controller_layout = QVBoxLayout()
        
        self.controller_label = QLabel("No controller selected")
        controller_layout.addWidget(self.controller_label)
        
        select_button = QPushButton("Select Controller")
        select_button.clicked.connect(self.show_controller_dialog)
        controller_layout.addWidget(select_button)
        
        controller_group.setLayout(controller_layout)
        self.layout.addWidget(controller_group)

        # Joint control section
        self.joint_group = QGroupBox("Joint Control")
        self.joint_layout = QVBoxLayout()
        self.joint_group.setLayout(self.joint_layout)
        self.joint_group.setEnabled(False)  # Disabled until controller selected
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

        # Auto-request controllers on startup
        self.ros_node.request_controllers()

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
        self.joint_group.setEnabled(True)
        self.command_button.setEnabled(True)

    def update_joint_sliders(self, joint_data: dict):
        for name, position in joint_data.items():
            if name not in self.sliders:
                row = QHBoxLayout()
                
                # Joint name label
                label = QLabel(f"{name}:")
                label.setMinimumWidth(100)
                row.addWidget(label)
                
                # Position slider
                slider = QSlider(Qt.Horizontal)
                slider.setRange(-314, 314)  # approx. -3.14 to 3.14 radians * 100
                slider.setValue(int(position * 100))
                row.addWidget(slider)
                
                # Position value display
                value_label = QLabel(f"{position:.3f}")
                value_label.setMinimumWidth(60)
                row.addWidget(value_label)
                
                # Connect slider to value update
                slider.valueChanged.connect(
                    lambda v, lbl=value_label: lbl.setText(f"{v/100:.3f}")
                )
                
                self.joint_layout.addLayout(row)
                self.sliders[name] = {'slider': slider, 'label': value_label}
            else:
                # Update existing slider
                self.sliders[name]['slider'].setValue(int(position * 100))
                self.sliders[name]['label'].setText(f"{position:.3f}")

    def send_command(self):
        if not self.current_controller:
            return
            
        # Get current slider values
        joint_positions = {}
        for name, widgets in self.sliders.items():
            position = widgets['slider'].value() / 100.0
            joint_positions[name] = position
        
        # Send command to ROS node
        self.ros_node.send_joint_command(joint_positions)


# Main function
def main():
    rclpy.init()

    ros_node = JointControllerNode()
    ros_thread = RosSpinThread(ros_node)
    ros_thread.start()

    app = QApplication(sys.argv)
    window = MainWindow(ros_node)
    window.resize(800, 600)
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