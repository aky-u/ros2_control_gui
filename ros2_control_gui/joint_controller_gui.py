import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from PySide6.QtWidgets import QApplication, QWidget, QPushButton, QVBoxLayout
from PySide6.QtCore import QThread

from controller_manager_msgs.srv import ListControllers
from controller_manager_msgs.srv import SwitchController

# ROS 2 Node for joint controller GUI
class JointControllerGui(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)

        # Create Client to get ros2 controllers state
        self.list_controllers_client = \
            self.create_client(ListControllers, '/controller_manager/list_controllers')
        
        self.controllers_list = []

    def publish_message(self, msg: str):
        msg_obj = String()
        msg_obj.data = msg
        self.publisher_.publish(msg_obj)

    def request_controllers(self):
        if not self.list_controllers_client.service_is_ready():
            self.get_logger().info('Waiting for /list_controllers service...')
            return

        request = ListControllers.Request()
        future = self.list_controllers_client.call_async(request)
        future.add_done_callback(self.on_controllers_response)

    def on_controllers_response(self, future):
        try:
            response = future.result()
            self.controllers_list.clear()
            for controller in response.controller:
                self.controllers_list.append(controller)
                self.get_logger().info(f"Controller: {controller.name}, State: {controller.state}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

# Thread to run ROS spin in the background
class RosSpinThread(QThread):
    def __init__(self, node: Node):
        super().__init__()
        self.node = node

    def run(self):
        rclpy.spin(self.node)

    def stop(self):
        self.node.destroy_node()
        rclpy.shutdown()
        self.quit()
        self.wait()


# Main GUI
class MainWindow(QWidget):
    def __init__(self, ros_node: JointControllerGui):
        super().__init__()
        self.setWindowTitle("This is an app made with PySide6.")
        self.ros_node = ros_node

        self.layout = QVBoxLayout(self)

        button = QPushButton("Try pressing me!!", self)
        button.pressed.connect(self.on_button_pressed)
        button.released.connect(lambda: self.on_button_released(90))

        self.layout.addWidget(button)

    def on_button_pressed(self):
        print("Button Pressed!")
        self.ros_node.publish_message("Button Pressed!")

    def on_button_released(self, radian):
        print(f"Released with radian: {radian}")
        # self.ros_node.publish_message(f"Released with radian {radian}")
        self.ros_node.request_controllers()


# Main function to run the application
def main():
    rclpy.init()

    # Create ROS node
    ros_node = JointControllerGui()

    # Start ROS spin in a separate thread
    ros_thread = RosSpinThread(ros_node)
    ros_thread.start()

    # Create and show the GUI
    app = QApplication(sys.argv)
    window = MainWindow(ros_node)
    window.resize(800, 600)
    window.show()

    try:
        sys.exit(app.exec())
    finally:
        # Stop the ROS thread and clean up
        ros_thread.stop()

if __name__ == '__main__':
    main()
