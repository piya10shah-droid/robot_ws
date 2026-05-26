#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial


class SimpleSerialTransmitter(Node):
    def __init__(self):
        super().__init__("simple_serial_transmitter")

        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("frequency", 5.0) #hz

        self.port_ = self.get_parameter("port").value
        self.baudrate_ = self.get_parameter("baudrate").value
        self.frequency_ = self.get_parameter("frequency").get_parameter_value().double_value

        self.arduino_ = serial.Serial(port=self.port_, baudrate=self.baudrate_, timeout=0.1)

        self.interval_ = 1/self.frequency_
        self.timer_ = self.create_timer(self.interval_, self.msgCallback)

    def msgCallback(self):
        self.get_logger().info(f"Timer triggered. Current internal count: {counter_}")
        serial_packet = f"{counter_}\n"
        self.arduino_.write(serial_packet.encode("utf-8"))
        counter_ += 1


def main():
    rclpy.init()

    simple_serial_transmitter = SimpleSerialTransmitter()
    rclpy.spin(simple_serial_transmitter)
    
    simple_serial_transmitter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()