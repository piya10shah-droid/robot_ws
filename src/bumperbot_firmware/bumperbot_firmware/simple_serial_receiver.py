#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class SimpleSerialTransmitter(Node):
    def __init__(self):
        super().__init__("simple_serial_transmitter")

        self.declare_parameter("port", "/dev/arduino")
        self.declare_parameter("baudrate", 115200)

        port = self.get_parameter("port").value
        baudrate = self.get_parameter("baudrate").value

        self.arduino = serial.Serial(port=port, baudrate=baudrate, timeout=0.1)
        
        # Subscribe to the topic
        self.subscription = self.create_subscription(
            String,
            '/serial_transmitter',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        # Send the received string data directly to serial
        self.arduino.write(msg.data.encode("utf-8"))
        self.get_logger().info(f"Forwarded: {msg.data}")

def main():
    rclpy.init()
    node = SimpleSerialTransmitter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()