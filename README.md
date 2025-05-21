# WAVER Docking (WIP)

This is a ROS2 package made in C++ and Python that implements:
- A modified version of the PID Controller to perform a docking operation based on localization (C++)
- A 2D point visualization system to display and monitor docking points (Python)

## Features
- PID-based docking control
- Real-time visualization of docking points (P and Q) in RViz2
- Parameter-based point configuration

## Installation
```bash
# Clone the repository into your ROS2 workspace
cd ~/your_ws/src
git clone <repository_url>

# Build the package
cd ..
colcon build --packages-select waver_docking
source install/setup.bash
```

## Execution

### Docking Node
```bash
# Run the docking node with parameters
ros2 run waver_docking docking_node --ros-args --params-file ./src/waver_docking/params/docking_params.yaml

# Or use the launch file
ros2 launch waver_docking docking.launch.py
```

### Point Visualization
```bash
# Run the graph_2d node with parameters
ros2 run waver_docking graph_2d_pub_node.py --ros-args --params-file ./src/waver_docking/params/points_params.yaml

# Or launch the point visualization with RViz2
ros2 launch waver_docking graph_2d_pub.launch.py
```

## Parameters
The points visualization uses parameters defined in `params/points_params.yaml`:
```yaml
points_node:
  ros__parameters:
    P.x: 1.0  # X coordinate of point P
    P.y: 1.0  # Y coordinate of point P
    Q.x: 0.0  # X coordinate of point Q
    Q.y: 0.0  # Y coordinate of point Q
    robot_frame: "base_footprint"  # Robot's reference frame for visualization
```

## Visualization
The point visualization:
- Shows points P and Q in RViz2
- Uses the robot's reference frame (base_footprint by default)
- Includes a pre-configured RViz2 setup

## Author
Alejandro Daniel Jose Gomez Florez (aldajo92)
