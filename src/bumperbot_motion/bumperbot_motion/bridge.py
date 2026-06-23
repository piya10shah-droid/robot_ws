#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray

class BridgeNode(Node):
    def __init__(self):
        super().__init__("bridge_node")
        
        # Subscriptions
        self.create_subscription(String, "/jetson_write", self.parse_write_data, 10)
        self.create_subscription(String, "/jetson_read", self.parse_read_data, 10)
        
        # Publishers
        self.pub_write = self.create_publisher(Float64MultiArray, "/robot_speeds_write", 10)
        self.pub_read = self.create_publisher(Float64MultiArray, "/robot_speeds_read", 10)

    def parse_and_publish(self, data, publisher):
        """Helper method to parse string and publish to given publisher."""
        try:
            # Split and remove empty strings from trailing commas
            parts = [p for p in data.split(",") if p]
            
            # Extract numerical values (assuming rp##.## and lp##.##)
            rp_val = float(parts[0][2:])
            lp_val = float(parts[1][2:])
            
            num_msg = Float64MultiArray()
            num_msg.data = [rp_val, lp_val]
            
            publisher.publish(num_msg)
        except (ValueError, IndexError) as e:
            self.get_logger().error(f"Failed to parse data '{data}': {e}")

    def parse_write_data(self, msg):
        self.parse_and_publish(msg.data, self.pub_write)

    def parse_read_data(self, msg):
        self.parse_and_publish(msg.data, self.pub_read)

def main(args=None):
    rclpy.init(args=args)
    bridge = BridgeNode()
    rclpy.spin(bridge)
    bridge.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()