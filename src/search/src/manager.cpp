#include <sstream>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <fstream>
#include <vector>
#include <ctime>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "geometry_msgs/msg/pose.hpp"

#define sUAV_NUM 10
#define VESSEL_NUM 7

using namespace geometry_msgs::msg;
using namespace std::chrono_literals;
using std::placeholders::_1;

class Manager : public rclcpp::Node {

public:

    // Log fstream
    std::ofstream log_file; 

    // Variable names in log file
    std::vector<std::string> name_vec; 

    // Sim time clock
    double clock; 

    // [Invalid] Groundtruth position of sUAV itself
    Point real_suav_pos[sUAV_NUM];

    // [Invalid] Groundtruth position of target vessel A-G
    Point real_vsl_pos[VESSEL_NUM];

    // Time duration from takeoff
    double task_time;

    // Task begin time
    double task_begin_time;

    Manager() : Node("Manager") {
        time_t tt = time(NULL);
        tm* t = localtime(&tt);
        char iden_path[256];
        sprintf(iden_path, "/home/ps/coor_ws/src/search/data/%02d-%02d_%02d-%02d_Manager.txt",
        t->tm_mon + 1, t->tm_mday, t->tm_hour, t->tm_min);
        log_file.open(iden_path, std::ios::out);
        while(!log_file) std::cout << "Error: Could not write data!" << std::endl;
        
        name_vec = {"time", "day", "hour", "min", "sec"};
        for (int i = 0; i < 7; i++){
            for (int j = 0; j < 3; j++){
                // "vessel_('a' + i)_('x' + j)"
                std::string s = "vessel_";
                s = s + char('a' + i) + '_' + char('x' + j);
                name_vec.push_back(s);
            }
        }
        for (int i = 1; i <= 10; i++){
            for (int j = 0; j < 3; j++){
                // 'suav_i_('x'+j)
                std::string s = "suav_";
                s = s + char('0' + i) + '_' + char('x' + j);
                name_vec.push_back(s);
            }
        }

        for (auto t: name_vec){
            log_file << t << "\t";
        } log_file << std::endl;

        // [Valid] Clock
        clock_sub = this->create_subscription<rosgraph_msgs::msg::Clock>(
            "/clock", 10, 
            [this](const rosgraph_msgs::msg::Clock & msg){
                this->clock = 1.0 * msg.clock.sec + 1.0 * msg.clock.nanosec / 1e9;
            }
        );

        // [Invalid] Groundtruth Pose of Target Vessels
        for (int i = 0; i < VESSEL_NUM; i++){
            std::string chara_str;
            chara_str = chara_str + char('A' + i);
            vsl_sub[i] = this->create_subscription<geometry_msgs::msg::Pose>(
                    "/model/Vessel_" + chara_str + "/world_pose", 10,
                    [i, this](const geometry_msgs::msg::Pose & msg) -> void{     
                        this->real_vsl_pos[i] = msg.position;
                    }
            );
        }

        // [Invalid] Groundtruth Pose of UAVs
        for (int i = 1; i <= VESSEL_NUM; i++){
            std::string chara_str;
            chara_str = chara_str + char('0' + i);
            vsl_sub[i] = this->create_subscription<geometry_msgs::msg::Pose>(
                    "/model/suav_" + chara_str + "/world_pose", 10,
                    [i, this](const geometry_msgs::msg::Pose & msg) -> void{     
                        this->real_suav_pos[i] = msg.position;
                    }
            );
        }
        
        timer_ = this->create_wall_timer(50ms, std::bind(&Manager::timer_callback, this));

    }

    double get_time_now(){
        return clock;
        // return this->get_clock()->now().seconds();
    }

    void update_time(){
        task_time = get_time_now() - task_begin_time;
    }

    void log_once(){
        time_t tt = time(NULL);
        tm* t = localtime(&tt);
        log_file.precision(7);
        log_file << get_time_now() - task_begin_time << "\t"
                 << t->tm_mday << "\t"
                 << t->tm_hour << "\t"
                 << t->tm_min << "\t"
                 << t->tm_sec << "\t";
        for (int i = 0; i < VESSEL_NUM; i++){
            log_file << real_vsl_pos[i].x << "\t"
                     << real_vsl_pos[i].y << "\t"
                     << real_vsl_pos[i].z << "\t";
        }
        for (int i = 1; i <= sUAV_NUM; i++){
            log_file << real_suav_pos[i].x << "\t"
                     << real_suav_pos[i].y << "\t"
                     << real_suav_pos[i].z << "\t";
        }
        log_file << std::endl;
    }

    void timer_callback(){
        log_once();
        printf("Log!\n");
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_sub;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr suav_sub[sUAV_NUM + 1];
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr vsl_sub[VESSEL_NUM];
};


int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Manager>());
    rclcpp::shutdown();
    return 0;
}