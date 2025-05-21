#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
import numpy as np

class Graph2DPublisherNode(Node):
    def __init__(self):
        super().__init__('graph_2d_pub_node')
        
        # Create marker publisher
        self.marker_pub = self.create_publisher(Marker, 'visualization_marker', 10)
        
        # Declare and get parameters
        self.declare_parameter('P.x', 0.0)
        self.declare_parameter('P.y', 0.0)
        self.declare_parameter('Q.x', 0.0)
        self.declare_parameter('Q.y', 0.0)
        self.declare_parameter('source_frame', 'map')
        
        # Initialize coordinates as a single numpy array with shape (2, 2)
        # First dimension: points (P, Q)
        # Second dimension: coordinates (x, y)
        self.coordinates = np.array([
            [self.get_parameter('P.x').value, self.get_parameter('P.y').value],  # P coordinates
            [self.get_parameter('Q.x').value, self.get_parameter('Q.y').value]   # Q coordinates
        ])
        
        # Initialize color state (True for red, False for blue)
        self.is_red = True
        
        # Define colors
        self.red_color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
        self.blue_color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0)
        
        # Create timer to periodically publish markers
        self.timer = self.create_timer(1.0, self.timer_callback)  # 1Hz update rate
        
    def create_point(self, x: float, y: float) -> Point:
        """Create a ROS Point message."""
        return Point(x=float(x), y=float(y), z=0.0)
        
    def create_marker(self, coordinates: np.ndarray, color: ColorRGBA) -> Marker:
        """
        Create a marker message from numpy coordinates.
        Args:
            coordinates: numpy array of shape (N, 2) where N is number of points
                       and 2 is for (x, y) coordinates
            color: ColorRGBA message for marker color
        """
        marker = Marker()
        marker.header.frame_id = "map"
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
        marker.color = color
        
        # Convert numpy coordinates to Point messages
        marker.points = [self.create_point(x, y) for x, y in coordinates]
        
        # Set the pose orientation (identity)
        marker.pose.orientation.w = 1.0
        
        return marker
        
    def timer_callback(self):
        # Update coordinates with current parameter values
        self.coordinates = np.array([
            [self.get_parameter('P.x').value, self.get_parameter('P.y').value],  # P coordinates
            [self.get_parameter('Q.x').value, self.get_parameter('Q.y').value]   # Q coordinates
        ])
        
        # Select color and toggle for next time
        current_color = self.red_color if self.is_red else self.blue_color
        self.is_red = not self.is_red
        
        # Create and publish marker with all points
        marker = self.create_marker(
            self.coordinates,
            current_color
        )
        self.marker_pub.publish(marker)
        
        # Log points and color (optional)
        color_name = "RED" if self.is_red else "BLUE"
        self.get_logger().debug(
            f'Published points with {color_name} color:\n'
            f'P: ({self.coordinates[0,0]}, {self.coordinates[0,1]})\n'
            f'Q: ({self.coordinates[1,0]}, {self.coordinates[1,1]})'
        )

def main(args=None):
    rclpy.init(args=args)
    node = Graph2DPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 