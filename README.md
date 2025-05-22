# WAVER Docking

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

### Interactive Points Visualization
```bash
# Run the graph_2d node with parameters
ros2 run waver_docking interactive_points_node.py --ros-args --params-file ./src/waver_docking/params/points_params.yaml

# Or launch the point interactive with RViz2
ros2 launch waver_docking interactive_points.launch.py
```

## Parameters
The interactive points visualization uses parameters defined in the node:
```yaml
interactive_points_node:
  ros__parameters:
    P.x: 1.0  # X coordinate of point P
    P.y: 1.0  # Y coordinate of point P
    Q.x: 0.0  # X coordinate of point Q
    Q.y: 0.0  # Y coordinate of point Q
    fixed_frame: "map"  # Reference frame for visualization
```

## Visualization Features
The interactive point visualization provides:
- Interactive markers for points P and Q that can be moved in real-time
- Perpendicular line markers for alignment reference
- Real-time position updates in the terminal
- Pre-configured RViz2 setup

## Topics
- `/visualization_marker_array`: MarkerArray messages for point and line visualization
- `/graph_points/update`: Interactive marker updates for point manipulation

## Author
Alejandro Daniel Jose Gomez Florez (aldajo92)
