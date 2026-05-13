#include <chrono>
#include <memory>
#include <random>
#include <algorithm>
#include <iomanip>
#include <sstream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

using namespace std::chrono_literals;

// INA219 Register Constants
constexpr uint8_t REG_CONFIG       = 0x00;
constexpr uint8_t REG_SHUNTVOLTAGE = 0x01;
constexpr uint8_t REG_BUSVOLTAGE   = 0x02;
constexpr uint8_t REG_POWER        = 0x03;
constexpr uint8_t REG_CURRENT      = 0x04;
constexpr uint8_t REG_CALIBRATION  = 0x05;

class BusVoltageRange
{
public:
    static constexpr uint16_t RANGE_16V = 0x00;
};

class Gain
{
public:
    static constexpr uint16_t DIV_2_80MV = 0x01;
};

class ADCResolution
{
public:
    static constexpr uint16_t ADCRES_12BIT_32S = 0x0D;
};

class Mode
{
public:
    static constexpr uint16_t SANDBVOLT_CONTINUOUS = 0x07;
};

class BatteryPublisher : public rclcpp::Node
{
public:
    BatteryPublisher()
        : Node("battery_monitor_node"),
          generator_(std::random_device{}())
    {
        // ROS2 Parameters
        this->declare_parameter<bool>("pub_low", false);

        // Publishers
        voltage_pub_ = this->create_publisher<std_msgs::msg::Float32>(
            "battery/load_voltage", 10);

        current_pub_ = this->create_publisher<std_msgs::msg::Float32>(
            "battery/current", 10);

        power_pub_ = this->create_publisher<std_msgs::msg::Float32>(
            "battery/power", 10);

        percent_pub_ = this->create_publisher<std_msgs::msg::Float32>(
            "battery/percentage", 10);

        // Publish timer
        timer_ = this->create_wall_timer(
            1s,
            std::bind(&BatteryPublisher::publish_battery_data, this));

        RCLCPP_INFO(
            this->get_logger(),
            "Battery monitor node started in RANDOM DATA mode");
    }

private:
    void publish_battery_data()
    {
        bool pub_low = this->get_parameter("pub_low").as_bool();

        float percentage;

        // Percentage logic
        if (pub_low)
        {
            std::uniform_real_distribution<float> percent_dist(0.0f, 20.0f);
            percentage = percent_dist(generator_);
        }
        else
        {
            std::uniform_real_distribution<float> percent_dist(0.0f, 100.0f);
            percentage = percent_dist(generator_);
        }

        // Simulated battery voltage
        float bus_voltage =
            9.0f + (percentage / 100.0f) * 3.6f;

        // Simulated current
        std::uniform_real_distribution<float> current_dist(0.1f, 2.0f);
        float current_a = current_dist(generator_);

        // Simulated power
        float power_w = bus_voltage * current_a;

        // Clamp percentage
        percentage = std::clamp(percentage, 0.0f, 100.0f);

        // Publish voltage
        std_msgs::msg::Float32 voltage_msg;
        voltage_msg.data = bus_voltage;
        voltage_pub_->publish(voltage_msg);

        // Publish current
        std_msgs::msg::Float32 current_msg;
        current_msg.data = current_a;
        current_pub_->publish(current_msg);

        // Publish power
        std_msgs::msg::Float32 power_msg;
        power_msg.data = power_w;
        power_pub_->publish(power_msg);

        // Publish percentage
        std_msgs::msg::Float32 percent_msg;
        percent_msg.data = percentage;
        percent_pub_->publish(percent_msg);

        // Console output
        std::ostringstream oss;

        oss << std::fixed << std::setprecision(2)
            << "Voltage: " << bus_voltage << " V | "
            << "Current: " << current_a << " A | "
            << "Power: " << power_w << " W | "
            << "Battery: " << percentage << "%";

        RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());
    }

private:
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr voltage_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr current_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr power_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr percent_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

    std::mt19937 generator_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<BatteryPublisher>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}