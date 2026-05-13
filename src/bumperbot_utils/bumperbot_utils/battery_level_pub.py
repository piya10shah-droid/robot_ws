#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import smbus
import random


# INA219 Register Constants
_REG_CONFIG         = 0x00
_REG_SHUNTVOLTAGE   = 0x01
_REG_BUSVOLTAGE     = 0x02
_REG_POWER          = 0x03
_REG_CURRENT        = 0x04
_REG_CALIBRATION    = 0x05


class BusVoltageRange:
    RANGE_16V = 0x00


class Gain:
    DIV_2_80MV = 0x01


class ADCResolution:
    ADCRES_12BIT_32S = 0x0D


class Mode:
    SANDBVOLT_CONTINUOUS = 0x07


class BatteryPublisher(Node):

    def __init__(self):
        super().__init__('battery_monitor_node')

        # ROS2 Parameters
        self.declare_parameter('random_data', False)
        self.declare_parameter('pub_low', False)

        # Publishers
        self.voltage_pub = self.create_publisher(
            Float32,
            'battery/load_voltage',
            10
        )

        self.current_pub = self.create_publisher(
            Float32,
            'battery/current',
            10
        )

        self.power_pub = self.create_publisher(
            Float32,
            'battery/power',
            10
        )

        self.percent_pub = self.create_publisher(
            Float32,
            'battery/percentage',
            10
        )

        # Hardware Initialization
        self.i2c_bus_id = 7
        self.addr = 0x41
        self.initialized = False

        try:
            self.bus = smbus.SMBus(self.i2c_bus_id)

            self.set_calibration_16V_5A()

            self.initialized = True

            self.get_logger().info(
                "INA219 Initialized on Bus 7, Addr 0x41"
            )

        except Exception as e:
            self.get_logger().warn(
                f"Hardware init failed "
                f"(Check connections): {e}"
            )

        # Publish at 1 Hz
        self.timer = self.create_timer(
            1.0,
            self.publish_battery_data
        )

    def set_calibration_16V_5A(self):

        self._current_lsb = 0.1524
        self._cal_value = 26868
        self._power_lsb = 0.003048

        self.write(
            _REG_CALIBRATION,
            self._cal_value
        )

        config = (
            (BusVoltageRange.RANGE_16V << 13) |
            (Gain.DIV_2_80MV << 11) |
            (ADCResolution.ADCRES_12BIT_32S << 7) |
            (ADCResolution.ADCRES_12BIT_32S << 3) |
            Mode.SANDBVOLT_CONTINUOUS
        )

        self.write(_REG_CONFIG, config)

    def write(self, address, data):

        self.bus.write_i2c_block_data(
            self.addr,
            address,
            [
                (data >> 8) & 0xFF,
                data & 0xFF
            ]
        )

    def read(self, address):

        data = self.bus.read_i2c_block_data(
            self.addr,
            address,
            2
        )

        return (data[0] << 8) + data[1]

    def get_bus_voltage_v(self):

        value = self.read(_REG_BUSVOLTAGE)

        return (value >> 3) * 0.004

    def get_current_ma(self):

        value = self.read(_REG_CURRENT)

        if value > 32767:
            value -= 65536

        return value * self._current_lsb

    def get_power_w(self):

        value = self.read(_REG_POWER)

        if value > 32767:
            value -= 65536

        return value * self._power_lsb

    def publish_battery_data(self):

        # Read Parameters
        use_random = self.get_parameter(
            'random_data'
        ).get_parameter_value().bool_value

        pub_low = self.get_parameter(
            'pub_low'
        ).get_parameter_value().bool_value

        # RANDOM DATA MODE
        if use_random:

            # Percentage logic
            if pub_low:
                percentage = random.uniform(0.0, 20.0)
            else:
                percentage = random.uniform(0.0, 100.0)

            # Simulated battery voltage
            bus_voltage = (
                9.0 +
                (percentage / 100.0) * 3.6
            )

            # Simulated current
            current_a = random.uniform(0.1, 2.0)

            # Simulated power
            power_w = bus_voltage * current_a

        # REAL INA219 MODE
        else:

            if not self.initialized:

                self.get_logger().error(
                    "Hardware not initialized. "
                    "Set 'random_data' to true "
                    "for simulation."
                )

                return

            # Real sensor readings
            bus_voltage = self.get_bus_voltage_v()

            current_a = (
                self.get_current_ma() / 1000.0
            )

            power_w = self.get_power_w()

            percentage = (
                (bus_voltage - 9.0) / 3.6
            ) * 100.0

        # Clamp percentage
        percentage = max(
            0.0,
            min(percentage, 100.0)
        )

        # Publish voltage
        self.voltage_pub.publish(
            Float32(
                data=float(bus_voltage)
            )
        )

        # Publish current
        self.current_pub.publish(
            Float32(
                data=float(current_a)
            )
        )

        # Publish power
        self.power_pub.publish(
            Float32(
                data=float(power_w)
            )
        )

        # Publish percentage
        self.percent_pub.publish(
            Float32(
                data=float(percentage)
            )
        )

        # Console output
        self.get_logger().info(
            f"Voltage: {bus_voltage:.2f} V | "
            f"Current: {current_a:.2f} A | "
            f"Power: {power_w:.2f} W | "
            f"Battery: {percentage:.2f}%"
        )


def main(args=None):

    rclpy.init(args=args)

    node = BatteryPublisher()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()