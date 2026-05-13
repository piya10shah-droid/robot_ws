#include <chrono>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <linux/i2c-dev.h>
#include <memory>
#include <stdexcept>
#include <string>
#include <sys/ioctl.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

using namespace std::chrono_literals;

// Register Addresses
#define REG_CONFIG         0x00
#define REG_SHUNTVOLTAGE   0x01
#define REG_BUSVOLTAGE     0x02
#define REG_POWER          0x03
#define REG_CURRENT        0x04
#define REG_CALIBRATION    0x05


class INA219
{
public:
    INA219(int bus = 7, int addr = 0x41)
        : address_(addr)
    {
        std::string device = "/dev/i2c-" + std::to_string(bus);

        file_ = open(device.c_str(), O_RDWR);

        if (file_ < 0)
        {
            throw std::runtime_error("Failed to open I2C bus");
        }

        if (ioctl(file_, I2C_SLAVE, address_) < 0)
        {
            throw std::runtime_error("Failed to connect to INA219");
        }

        setCalibration16V5A();
    }

    ~INA219()
    {
        if (file_ >= 0)
        {
            close(file_);
        }
    }

    float getShuntVoltage_mV()
    {
        int16_t value = readRegister(REG_SHUNTVOLTAGE);
        return value * 0.01f;
    }

    float getBusVoltage_V()
    {
        uint16_t value = readRegister(REG_BUSVOLTAGE);
        return ((value >> 3) * 0.004f);
    }

    float getCurrent_mA()
    {
        int16_t value = readRegister(REG_CURRENT);
        return value * current_lsb_;
    }

    float getPower_W()
    {
        int16_t value = readRegister(REG_POWER);
        return value * power_lsb_;
    }

private:
    int file_;
    int address_;

    float current_lsb_;
    float power_lsb_;
    uint16_t calibration_value_;

    void writeRegister(uint8_t reg, uint16_t value)
    {
        uint8_t buffer[3];

        buffer[0] = reg;
        buffer[1] = (value >> 8) & 0xFF;
        buffer[2] = value & 0xFF;

        if (write(file_, buffer, 3) != 3)
        {
            throw std::runtime_error("I2C write failed");
        }
    }

    uint16_t readRegister(uint8_t reg)
    {
        uint8_t buffer[2];

        if (write(file_, &reg, 1) != 1)
        {
            throw std::runtime_error("I2C register select failed");
        }

        if (read(file_, buffer, 2) != 2)
        {
            throw std::runtime_error("I2C read failed");
        }

        return (buffer[0] << 8) | buffer[1];
    }

    void setCalibration16V5A()
    {
        current_lsb_ = 0.1524f;
        power_lsb_ = 0.003048f;
        calibration_value_ = 26868;

        writeRegister(REG_CALIBRATION, calibration_value_);

        uint16_t config =
            (0x00 << 13) |   // RANGE_16V
            (0x01 << 11) |   // DIV_2_80MV
            (0x0D << 7)  |   // ADCRES_12BIT_32S
            (0x0D << 3)  |   // ADCRES_12BIT_32S
            0x07;            // SANDBVOLT_CONTINUOUS

        writeRegister(REG_CONFIG, config);
    }
};


class INA219Publisher : public rclcpp::Node
{
public:
    INA219Publisher()
        : Node("ina219_publisher")
    {
        this->declare_parameter("i2c_bus", 7);
        this->declare_parameter("i2c_addr", 0x41);
        this->declare_parameter("publish_rate", 2.0);

        int i2c_bus = this->get_parameter("i2c_bus").as_int();
        int i2c_addr = this->get_parameter("i2c_addr").as_int();
        double publish_rate = this->get_parameter("publish_rate").as_double();

        ina219_ = std::make_shared<INA219>(i2c_bus, i2c_addr);

        current_pub_ =
            this->create_publisher<std_msgs::msg::Float32>(
                "/current", 10);

        percentage_pub_ =
            this->create_publisher<std_msgs::msg::Float32>(
                "/percentage", 10);

        load_voltage_pub_ =
            this->create_publisher<std_msgs::msg::Float32>(
                "/load_voltage", 10);

        power_pub_ =
            this->create_publisher<std_msgs::msg::Float32>(
                "/power", 10);

        auto timer_period =
            std::chrono::milliseconds(
                static_cast<int>(1000.0 / publish_rate));

        timer_ = this->create_wall_timer(
            timer_period,
            std::bind(&INA219Publisher::publishData, this));

        RCLCPP_INFO(
            this->get_logger(),
            "INA219 ROS2 Publisher Started");
    }

private:
    std::shared_ptr<INA219> ina219_;

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr current_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr percentage_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr load_voltage_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr power_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

    void publishData()
    {
        try
        {
            float bus_voltage = ina219_->getBusVoltage_V();

            float shunt_voltage =
                ina219_->getShuntVoltage_mV() / 1000.0f;

            float current =
                ina219_->getCurrent_mA() / 1000.0f;

            float power =
                ina219_->getPower_W();

            float percentage =
                ((bus_voltage - 9.0f) / 3.6f) * 100.0f;

            percentage =
                std::max(0.0f, std::min(percentage, 100.0f));

            std_msgs::msg::Float32 current_msg;
            current_msg.data = current;

            std_msgs::msg::Float32 percentage_msg;
            percentage_msg.data = percentage;

            std_msgs::msg::Float32 voltage_msg;
            voltage_msg.data = bus_voltage;

            std_msgs::msg::Float32 power_msg;
            power_msg.data = power;

            current_pub_->publish(current_msg);
            percentage_pub_->publish(percentage_msg);
            load_voltage_pub_->publish(voltage_msg);
            power_pub_->publish(power_msg);

            RCLCPP_INFO(
                this->get_logger(),
                "Voltage: %.2f V | Current: %.2f A | Power: %.2f W | Battery: %.1f%%",
                bus_voltage,
                current,
                power,
                percentage);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "INA219 Read Error: %s",
                e.what());
        }
    }
};


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<INA219Publisher>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}