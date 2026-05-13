#!/usr/bin/env python3
import time
import smbus

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32


# Config Register (R/W)
_REG_CONFIG = 0x00

# SHUNT VOLTAGE REGISTER (R)
_REG_SHUNTVOLTAGE = 0x01

# BUS VOLTAGE REGISTER (R)
_REG_BUSVOLTAGE = 0x02

# POWER REGISTER (R)
_REG_POWER = 0x03

# CURRENT REGISTER (R)
_REG_CURRENT = 0x04

# CALIBRATION REGISTER (R/W)
_REG_CALIBRATION = 0x05


class BusVoltageRange:
    RANGE_16V = 0x00
    RANGE_32V = 0x01


class Gain:
    DIV_1_40MV = 0x00
    DIV_2_80MV = 0x01
    DIV_4_160MV = 0x02
    DIV_8_320MV = 0x03


class ADCResolution:
    ADCRES_9BIT_1S = 0x00
    ADCRES_12BIT_32S = 0x0D


class Mode:
    SANDBVOLT_CONTINUOUS = 0x07


class INA219:
    def __init__(self, i2c_bus=7, addr=0x41):
        self.bus = smbus.SMBus(i2c_bus)
        self.addr = addr

        self._cal_value = 0
        self._current_lsb = 0
        self._power_lsb = 0

        self.set_calibration_16V_5A()

    def read(self, address):
        data = self.bus.read_i2c_block_data(self.addr, address, 2)
        return (data[0] << 8) + data[1]

    def write(self, address, data):
        self.bus.write_i2c_block_data(
            self.addr,
            address,
            [(data >> 8) & 0xFF, data & 0xFF]
        )

    def set_calibration_16V_5A(self):

        self._current_lsb = 0.1524
        self._cal_value = 26868
        self._power_lsb = 0.003048

        self.write(_REG_CALIBRATION, self._cal_value)

        config = (
            (BusVoltageRange.RANGE_16V << 13)
            | (Gain.DIV_2_80MV << 11)
            | (ADCResolution.ADCRES_12BIT_32S << 7)
            | (ADCResolution.ADCRES_12BIT_32S << 3)
            | Mode.SANDBVOLT_CONTINUOUS
        )

        self.write(_REG_CONFIG, config)

    def getShuntVoltage_mV(self):
        value = self.read(_REG_SHUNTVOLTAGE)

        if value > 32767:
            value -= 65536

        return value * 0.01

    def getBusVoltage_V(self):
        value = self.read(_REG_BUSVOLTAGE)
        return (value >> 3) * 0.004

    def getCurrent_mA(self):
        value = self.read(_REG_CURRENT)

        if value > 32767:
            value -= 65536

        return value * self._current_lsb

    def getPower_W(self):
        value = self.read(_REG_POWER)

        if value > 32767:
            value -= 65536

        return value * self._power_lsb


class INA219Publisher(Node):

    def __init__(self):
        super().__init__("ina219_publisher")

        # Parameters
        self.declare_parameter("i2c_bus", 7)
        self.declare_parameter("i2c_addr", 0x41)
        self.declare_parameter("publish_rate", 2.0)

        i2c_bus = self.get_parameter("i2c_bus").value
        i2c_addr = self.get_parameter("i2c_addr").value
        publish_rate = self.get_parameter("publish_rate").value

        # INA219
        self.ina219 = INA219(
            i2c_bus=i2c_bus,
            addr=i2c_addr
        )

        # Publishers
        self.current_pub = self.create_publisher(
            Float32,
            "/current",
            10
        )

        self.percentage_pub = self.create_publisher(
            Float32,
            "/percentage",
            10
        )

        self.load_voltage_pub = self.create_publisher(
            Float32,
            "/load_voltage",
            10
        )

        self.power_pub = self.create_publisher(
            Float32,
            "/power",
            10
        )

        # Timer
        self.timer = self.create_timer(
            1.0 / publish_rate,
            self.publish_data
        )

        self.get_logger().info("INA219 ROS2 Publisher Started")

    def publish_data(self):

        try:
            bus_voltage = self.ina219.getBusVoltage_V()
            shunt_voltage = self.ina219.getShuntVoltage_mV() / 1000.0
            current = self.ina219.getCurrent_mA() / 1000.0
            power = self.ina219.getPower_W()

            percentage = (bus_voltage - 9) / 3.6 * 100
            percentage = max(0.0, min(percentage, 100.0))

            # Create messages
            current_msg = Float32()
            current_msg.data = float(current)

            percentage_msg = Float32()
            percentage_msg.data = float(percentage)

            load_voltage_msg = Float32()
            load_voltage_msg.data = float(bus_voltage)

            power_msg = Float32()
            power_msg.data = float(power)


            # Publish
            self.current_pub.publish(current_msg)
            self.percentage_pub.publish(percentage_msg)
            self.load_voltage_pub.publish(load_voltage_msg)
            self.power_pub.publish(power_msg)
            self.get_logger().info(
                f"Voltage: {bus_voltage:.2f} V | "
                f"Current: {current:.2f} A | "
                f"Power: {power:.2f} W | "
                f"Battery: {percentage:.1f}%"
            )

        except Exception as e:
            self.get_logger().error(f"INA219 Read Error: {e}")


def main(args=None):

    rclpy.init(args=args)

    node = INA219Publisher()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()