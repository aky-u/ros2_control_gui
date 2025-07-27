"""
ROS thread for spinning the ROS node.
"""

from PySide6.QtCore import QThread
import rclpy
from rclpy.node import Node


class RosSpinThread(QThread):
    """Thread for spinning ROS node"""
    
    def __init__(self, node: Node):
        super().__init__()
        self.node = node
        self._shutdown_requested = False

    def run(self):
        """Main thread execution"""
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
