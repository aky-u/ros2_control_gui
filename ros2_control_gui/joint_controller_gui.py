import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from PySide6.QtWidgets import QApplication, QWidget, QPushButton, QVBoxLayout
from PySide6.QtCore import QThread


# ROS 2 Node for joint controller GUI
class JointControllerGui(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)

    def publish_message(self, msg: str):
        msg_obj = String()
        msg_obj.data = msg
        self.publisher_.publish(msg_obj)


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
        self.ros_node.publish_message(f"Released with radian {radian}")


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
