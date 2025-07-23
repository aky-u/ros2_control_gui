# ros2_control_gui

This package provides the GUI to deal with the ros2 controllers.

## Features

TODO

## Requirements

TODO

## Build

```bash
mkdir exp_ws/src -p
cd exp_ws/src
git clone https://github.com/aky-u/ros2_control_gui.git
cd ..
colcon build --symlink-install
```

## Known bags

- Joint trajectory command does not work.
- The order of sliding bar depends on the order of the joint names in `joint_states` topic, not following the controllers definition.
- The size of the commanded vector depends on the number of the joints in `joint_states` topic, which is not always correct.

## Memo

<https://zenn.dev/m10k1/articles/fbb33e79661050>

```bash
pip install PySide6
pip install qt6-tools
```