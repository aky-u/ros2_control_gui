#!/usr/bin/env python3
"""
Main entry point for the ROS 2 Control GUI application.
"""

import sys
import signal
import rclpy
from PySide6.QtWidgets import QApplication
from PySide6.QtCore import QTimer

from .core.ros_node import JointControllerNode
from .gui.ros_thread import RosSpinThread
from .gui.main_window import MainWindow


def main():
    """Main application entry point"""
    # Handle Ctrl+C gracefully
    def signal_handler(sig, frame):
        print("\nShutdown requested via Ctrl+C...")
        QApplication.quit()
    
    signal.signal(signal.SIGINT, signal_handler)
    
    # Initialize ROS
    rclpy.init()
    
    # Create ROS node and thread
    ros_node = JointControllerNode()
    ros_thread = RosSpinThread(ros_node)
    ros_thread.start()
    
    # Create Qt application
    app = QApplication(sys.argv)
    
    # Allow Ctrl+C to work by processing events periodically
    timer = QTimer()
    timer.timeout.connect(lambda: None)  # Just process events
    timer.start(100)  # Process every 100ms
    
    # Create and show main window
    window = MainWindow(ros_node)
    window.resize(800, 700)
    window.show()
    
    try:
        exit_code = app.exec()
        print("GUI closed normally")
        return exit_code
    except KeyboardInterrupt:
        print("Caught KeyboardInterrupt, shutting down...")
        return 0
    finally:
        # Proper shutdown sequence
        print("Shutting down ROS node and thread...")
        ros_thread.shutdown()
        ros_node.shutdown()
        ros_thread.wait(timeout=2000)  # Wait up to 2 seconds
        ros_node.destroy_node()
        rclpy.shutdown()
        print("Shutdown complete")


if __name__ == "__main__":
    sys.exit(main())
