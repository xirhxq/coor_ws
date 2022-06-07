#include <sstream>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
//      timer_ = this->create_wall_timer(500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

    void Pubber()
    {
      auto message = std_msgs::msg::String();
      message.data = "Hello, world! " + std::to_string(count_++);
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_->publish(message);
    }
  private:
//    void timer_callback()
//    {
//      auto message = std_msgs::msg::String();
//      message.data = "Hello, world! " + std::to_string(count_++);
//      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
//      publisher_->publish(message);
//    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MinimalPublisher>();
//  auto chatter_pub = node->create_publisher<std_msgs::msg::String>("chatter", 1000);
  rclcpp::Rate loop_rate(10);
//  int count = 0;
//  
//  std_msgs::msg::String msg;
  
  while (rclcpp::ok())
  {
//    std::stringstream ss;
//    ss << "hello world " << count++;
//    msg.data = ss.str();
//    
//    RCLCPP_INFO(node->get_logger(), "%s\n", msg.data.c_str());
//    chatter_pub->publish(msg);
    node->Pubber();
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }
//  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}


