#include <sstream>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16.hpp"

#include "MyMathFun.h"
#include "MyDataFun.h"

using namespace geometry_msgs::msg;
using namespace std::chrono_literals;
using std::placeholders::_1;

class commander : public rclcpp::Node{ 
public:
    commander() : Node("commander"){
        cmd_pub = this->create_publisher<std_msgs::msg::Int16>(
            "/commander_cmd", 10
        );
        timer_ = this->create_wall_timer(50ms, std::bind(&commander::timer_callback, this));
    }

void timer_callback(){
    std_msgs::msg::Int16 data;
    std::cout << "Type 233 to start sUAVs" << std::endl;
    std::cin >> data.data;
    cmd_pub->publish(data);
}

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr cmd_pub;

};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<commander>());
    rclcpp::shutdown();
    return 0;
}