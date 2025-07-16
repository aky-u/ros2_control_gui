import sys
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import JointState
from controller_manager_msgs.srv import ListControllers

from PySide6.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QPushButton,
    QHBoxLayout, QLabel, QSlider
)
from PySide6.QtCore import QThread, Qt, QObject, Signal


# Signal class to safely send data from ROS thread to Qt
class JointStateSignal(QObject):
    joint_state_updated = Signal(dict)  # {joint_name: position}


# ROS 2 Node
class JointControllerNode(Node):
    def __init__(self):
        super().__init__('joint_controller_node')

        # Subscriber to joint_states
        self.joint_state_sub = self.create_subscription(
            JointState, "/joint_states", self.joint_state_callback, 10
        )

        # Client to list controllers
        self.list_controllers_client = self.create_client(
            ListControllers, '/controller_manager/list_controllers'
        )

        self.joint_state_signal = JointStateSignal()

    def joint_state_callback(self, msg: JointState):
        joint_data = dict(zip(msg.name, msg.position))
        self.joint_state_signal.joint_state_updated.emit(joint_data)

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
            for controller in response.controller:
                self.get_logger().info(
                    f"Controller: {controller.name}, State: {controller.state}"
                )
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")


# ROS spinning thread
class RosSpinThread(QThread):
    def __init__(self, node: Node):
        super().__init__()
        self.node = node

    def run(self):
        rclpy.spin(self.node)


# Main GUI Window
class MainWindow(QWidget):
    def __init__(self, ros_node: JointControllerNode):
        super().__init__()
        self.setWindowTitle("Joint State Viewer")
        self.ros_node = ros_node
        self.layout = QVBoxLayout(self)
        self.sliders = {}

        # Button to call controller manager
        button = QPushButton("List Controllers")
        button.clicked.connect(self.ros_node.request_controllers)
        self.layout.addWidget(button)

        # Connect signal to GUI update
        self.ros_node.joint_state_signal.joint_state_updated.connect(
            self.update_joint_sliders
        )

    def update_joint_sliders(self, joint_data: dict):
        for name, position in joint_data.items():
            if name not in self.sliders:
                # New joint, create row
                row = QHBoxLayout()
                label = QLabel(name)
                slider = QSlider(Qt.Horizontal)
                slider.setRange(-314, 314)  # -3.14 to 3.14 rad * 100
                row.addWidget(label)
                row.addWidget(slider)
                self.layout.addLayout(row)
                self.sliders[name] = slider
            # Update slider
            self.sliders[name].setValue(int(position * 100))


# Main function
def main():
    rclpy.init()

    ros_node = JointControllerNode()
    ros_thread = RosSpinThread(ros_node)
    ros_thread.start()

    app = QApplication(sys.argv)
    window = MainWindow(ros_node)
    window.resize(600, 400)
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
