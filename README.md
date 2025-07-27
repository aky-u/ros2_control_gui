# ros2_control_gui

This package provides the GUI to deal with the ros2 controllers.

## Features

- **Modular Architecture**: Well-organized codebase for easy extension with new controller types
- **YAML Configuration Loading**: Load controller configurations from YAML files through GUI file dialog
- **Automatic Controller Discovery**: Automatically discovers active controllers
- **Plugin-based Controller Support**: Easy to add new controller types via the factory pattern
- **Proper Joint Configuration**: Gets joint configuration from YAML files or falls back to joint_states
- **Multiple Controller Types**: Supports trajectory, position, effort, and velocity controllers
- **Continuous Command Mode**: Real-time control with configurable frequency

## Requirements

- ROS 2 (tested with Humble/Iron)
- PySide6
- PyYAML
- ros2_control framework
- controller_manager

## Build

```bash
mkdir exp_ws/src -p
cd exp_ws/src
git clone https://github.com/aky-u/ros2_control_gui.git
cd ..
colcon build --symlink-install
```

## Project Structure

The project has been refactored into a modular architecture:

```text
ros2_control_gui/
├── core/
│   ├── __init__.py
│   └── ros_node.py              # Main ROS 2 node
├── config/
│   ├── __init__.py
│   └── yaml_config.py           # YAML configuration manager
├── controllers/
│   ├── __init__.py
│   ├── base_controller.py       # Abstract base controller
│   ├── position_controller.py   # Position controller implementation
│   ├── trajectory_controller.py # Trajectory controller implementation
│   └── controller_factory.py    # Factory for creating controllers
├── gui/
│   ├── __init__.py
│   ├── main_window.py          # Main GUI window
│   ├── controllers_dialog.py   # Controller selection dialog
│   └── ros_thread.py           # ROS spinning thread
├── main.py                     # Main entry point
└── joint_controller_gui.py     # Legacy wrapper for backward compatibility
```

## Adding New Controller Types

To add support for a new controller type:

1. **Create a new controller class** in `controllers/` that inherits from `BaseController`
2. **Implement the required methods**: `create_publisher()`, `send_command()`, `get_topic_name()`
3. **Register the controller** in `ControllerFactory`

Example:

```python
# controllers/my_new_controller.py
from .base_controller import BaseController
from my_msgs.msg import MyMessage

class MyNewController(BaseController):
    def create_publisher(self):
        return self.ros_node.create_publisher(MyMessage, self.get_topic_name(), 10)
    
    def send_command(self, publisher, joint_positions):
        # Implement your message creation and publishing logic
        pass
    
    def get_topic_name(self):
        return f"/{self.controller_name}/my_command"

# Then register in controller_factory.py
self.controller_mapping['my_new_type'] = MyNewController
```

## Usage

1. **Run the GUI**:

   ```bash
    ros2 run ros2_control_gui joint_controller_gui
   ```

2. **Load YAML Configuration**:
   - Click "Load YAML Config" button
   - Select your controller configuration YAML file
   - The GUI will use this configuration to determine joint names and order

3. **Select Controller**:
   - Click "Select Controller" button
   - Choose from the list of active controllers

4. **Control Joints**:
   - Use the sliders to set joint positions
   - Click "Send Command" to send once, or enable "Send Continuous" for real-time control

## Known bugs

- Joint trajectory command does not work.
- The order of sliding bar depends on the order of the joint names in `joint_states` topic, not following the controllers definition.
- The size of the commanded vector depends on the number of the joints in `joint_states` topic, which is not always correct.

## Memo

<https://zenn.dev/m10k1/articles/fbb33e79661050>

```bash
pip install PySide6
pip install qt6-tools
```
