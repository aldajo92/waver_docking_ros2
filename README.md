# WAVER Docking (WIP)

This is a ROS2 package made in C++ as a proof of concept to implement a modified version of the PID Controller to perform a docking operation based on localization.

# Execution
```bash
ros2 run waver_docking docking_node --ros-args --params-file ./src/waver_docking/params/docking_params.yaml

# or

ros2 launch waver_docking docking.launch.py
```

## Author
Alejandro Daniel Jose Gomez Florez (aldajo92)
