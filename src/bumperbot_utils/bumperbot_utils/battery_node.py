import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool

class BatteryNode(Node):

    def __init__(self):
        super().__init__('battery_node')
        self.battery_level_sub_ = self.create_subscription(
            Float32,
            'battery_level',
            self.battery_level_callback)
        
        self.safety_stop_pub_ = self.create_publisher(
            Bool,
            'safety_stop',
            10)
        
    def battery_level_callback(self, msg):
        battery_level = msg.data
        lock_msg = Bool()

        if battery_level < 30:
            lock_msg.data = True
            self.get_logger().info("BATTERY LEVEL IS LESS THAN 30%, SENDING LOCK COMMAND")
        else:
            lock_msg.data = False
            self.get_logger().info("BATTERY LEVEL IS MORE THAN 30%")

        
        self.safety_stop_pub_.publish(lock_msg)