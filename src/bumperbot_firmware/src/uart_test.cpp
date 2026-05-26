#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <libserial/SerialStream.h>
#include <string>
#include <memory>

class UartTestNode : public rclcpp::Node
{
public:
  UartTestNode() : Node("uart_test_node"), write_counter_(0)
  {
    this->declare_parameter<std::string>("port", "/dev/ttyTHS1");
    this->declare_parameter<double>("freq", 10.0);

    this->get_parameter("port", port_);
    this->get_parameter("freq", freq_);

    jetson_write_pub_ = this->create_publisher<std_msgs::msg::String>("jetson_write", 10);
    jetson_read_pub_ = this->create_publisher<std_msgs::msg::String>("jetson_read", 10);

    try
    {
      arduino_.Open(port_);
      arduino_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
      arduino_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
      arduino_.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
      arduino_.SetParity(LibSerial::Parity::PARITY_NONE);
      arduino_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
      
      RCLCPP_INFO(this->get_logger(), "UART Port %s successfully initialized.", port_.c_str());
    }
    catch (...)
    {
      RCLCPP_FATAL(this->get_logger(), "Failed to open serial port: %s", port_.c_str());
      return;
    }

    auto timer_duration = std::chrono::duration<double>(1.0 / freq_);

    timer_ = this->create_wall_timer(
      timer_duration, 
      std::bind(&UartTestNode::controlLoop, this)
    );
  }

  ~UartTestNode()
  {
    if (arduino_.IsOpen())
    {
      arduino_.Close();
    }
  }

private:
  void controlLoop()
  {
    read_step();
    write_step();
  }

  void read_step()
  {
    // Check if data is available to avoid blocking the real-time loop
    if (arduino_.IsDataAvailable())
    {
      std::string message;
      // SOLUTION: Use standard std::getline to read a line from the stream
      std::getline(arduino_, message);

      if (!message.empty() && jetson_read_pub_)
      {
        auto msg = std_msgs::msg::String();
        msg.data = message;
        jetson_read_pub_->publish(msg);
      }
    }
  }

  void write_step()
  {
    std::string counter_string = std::to_string(write_counter_) + "\n";

    try
    {
      // SOLUTION: Use the standard stream insertion operator (<<) to transmit data
      arduino_ << counter_string;
      
      // Flush the stream buffer immediately to clear transmission latency
      arduino_ << std::flush;

      if (jetson_write_pub_)
      {
        auto msg = std_msgs::msg::String();
        msg.data = counter_string;
        jetson_write_pub_->publish(msg);
      }

      write_counter_++;
    }
    catch (...)
    {
      RCLCPP_ERROR(this->get_logger(), "Error writing data to port %s", port_.c_str());
    }
  }

  LibSerial::SerialStream arduino_;
  std::string port_;
  double freq_;
  unsigned long write_counter_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr jetson_write_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr jetson_read_pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UartTestNode>());
  rclcpp::shutdown();
  return 0;
}