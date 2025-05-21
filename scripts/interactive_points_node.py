#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, InteractiveMarker, InteractiveMarkerControl, MarkerArray
from geometry_msgs.msg import Point, Quaternion
from std_msgs.msg import ColorRGBA
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
import numpy as np

class InteractiveGraphNode(Node):
    def __init__(self):
        super().__init__('interactive_graph_node')
        
        # Create marker publisher for the graph visualization
        self.marker_array_pub = self.create_publisher(MarkerArray, 'visualization_marker_array', 10)
        
        # Create interactive marker server
        self.server = InteractiveMarkerServer(self, "graph_points")
        
        # Declare and get parameters
        self.declare_parameter('P.x', 0.0)
        self.declare_parameter('P.y', 0.0)
        self.declare_parameter('Q.x', 0.0)
        self.declare_parameter('Q.y', 0.0)
        self.declare_parameter('fixed_frame', 'map')

        # Get parameters for fixed frame
        self.fixed_frame = self.get_parameter('fixed_frame').value
        
        # Initialize coordinates
        self.p_point = Point(
            x=float(self.get_parameter('P.x').value),
            y=float(self.get_parameter('P.y').value),
            z=0.0
        )
        self.q_point = Point(
            x=float(self.get_parameter('Q.x').value),
            y=float(self.get_parameter('Q.y').value),
            z=0.0
        )
        
        # Initialize color state (True for red, False for blue)
        self.is_red = True
        
        # Define colors
        self.red_color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
        self.blue_color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0)
        self.p_color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0)  # For P
        self.q_color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)  # For Q
        self.green_color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)  # For perpendicular markers
        
        # Create interactive markers for P and Q
        self.create_point_marker("P", self.p_point, self.p_color)
        self.create_point_marker("Q", self.q_point, self.q_color)
        
        # Create timer to periodically publish markers
        self.timer = self.create_timer(1.0, self.timer_callback)  # 1Hz update rate

        # Print node information
        self.get_logger().info(f"Fixed frame: {self.fixed_frame}")
        self.get_logger().info(f"Point P: ({self.p_point.x:.2f}, {self.p_point.y:.2f})")
        self.get_logger().info(f"Point Q: ({self.q_point.x:.2f}, {self.q_point.y:.2f})")

    def create_point_marker(self, name: str, position: Point, color: ColorRGBA):
        # Create an interactive marker
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = self.fixed_frame
        int_marker.name = name
        int_marker.description = f"Point {name}"
        int_marker.pose.position = position
        int_marker.scale = 0.2

        # Create a sphere marker
        marker = Marker()
        marker.type = Marker.SPHERE
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color = color

        # Create a non-interactive control for the sphere
        visual_control = InteractiveMarkerControl()
        visual_control.always_visible = True
        visual_control.markers.append(marker)
        int_marker.controls.append(visual_control)

        # Create a planar control
        control = InteractiveMarkerControl()
        control.orientation = Quaternion(x=0.0, y=1.0, z=0.0, w=1.0)
        control.name = f"move_plane_{name}"
        control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        int_marker.controls.append(control)

        # Add controls for moving in the plane
        # X axis control
        control = InteractiveMarkerControl()
        control.orientation = Quaternion(x=1.0, y=0.0, z=0.0, w=1.0)
        control.name = "move_x"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control)

        # Y axis control - rotate 90 degrees around Z axis from X
        control = InteractiveMarkerControl()
        control.orientation = Quaternion(x=0.0, y=0.0, z=1.0, w=1.0)
        control.name = "move_y"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control)

        # Set up the callback and add to server
        self.server.setCallback(int_marker.name, self.marker_feedback)
        self.server.insert(int_marker)
        self.server.applyChanges()

    def create_perpendicular_markers(self) -> Marker:
        """Create markers perpendicular to P-Q line."""
        marker = Marker()
        marker.header.frame_id = self.fixed_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "perpendicular_points"
        marker.id = 1
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        
        # Set the scale
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        
        # Set the color
        marker.color = self.green_color
        
        # Calculate midpoint
        x_mid = (self.p_point.x + self.q_point.x) / 2
        y_mid = (self.p_point.y + self.q_point.y) / 2
        
        # Calculate direction vector P->Q
        x_dir = self.q_point.x - self.p_point.x
        y_dir = self.q_point.y - self.p_point.y
        
        # Normalize direction vector
        dir_norm = np.sqrt(np.square(x_dir) + np.square(y_dir))
        if dir_norm > 0:  # Avoid division by zero
            # Rotate 90 degrees to get perpendicular direction
            angle = np.radians(-90)
            x_dir_new = (x_dir*np.cos(angle) - y_dir*np.sin(angle)) / dir_norm
            y_dir_new = (x_dir*np.sin(angle) + y_dir*np.cos(angle)) / dir_norm
            
            # Create points along perpendicular line
            marker.points = []
            for t in range(0, 3):
                point = Point()
                point.x = x_mid + (t * x_dir_new)
                point.y = y_mid + (t * y_dir_new)
                point.z = 0.0
                marker.points.append(point)
        
        return marker

    def marker_feedback(self, feedback):
        # Update point position based on feedback
        if feedback.marker_name == "P":
            self.p_point = feedback.pose.position
        elif feedback.marker_name == "Q":
            self.q_point = feedback.pose.position

        self.get_logger().info(
            f"Point {feedback.marker_name} moved to ({feedback.pose.position.x:.2f}, {feedback.pose.position.y:.2f})"
        )

    def create_graph_marker(self) -> Marker:
        """Create a marker message for the graph points."""
        marker = Marker()
        marker.header.frame_id = self.fixed_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "graph_points"
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        
        # Set the scale
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        
        # Set the color
        marker.color = self.red_color if self.is_red else self.blue_color
        
        # Add points
        marker.points = [self.p_point, self.q_point]
        
        # Set the pose orientation (identity)
        marker.pose.orientation.w = 1.0
        
        return marker
        
    def timer_callback(self):
        # Toggle color
        self.is_red = not self.is_red
        
        # Create marker array
        marker_array = MarkerArray()
        
        # Add graph points marker
        marker_array.markers.append(self.create_graph_marker())
        
        # Add perpendicular markers
        marker_array.markers.append(self.create_perpendicular_markers())
        
        # Publish marker array
        self.marker_array_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = InteractiveGraphNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 