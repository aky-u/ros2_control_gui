# ros2_control_gui

This package provides the GUI to deal with the ros2 controllers.

## Features

- GUI for commanding ROS 2 controllers
- **YAML Configuration Loading**: Load controller configurations from YAML files through GUI file dialog
- Automatically discovers active controllers
- Gets joint configuration from YAML files or falls back to joint_states
- Supports both trajectory and position/effort/velocity controllers
- Continuous command mode with configurable frequency

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
