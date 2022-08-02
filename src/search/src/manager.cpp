#include <sstream>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <fstream>
#include <vector>
#include <ctime>
#include <cmath>

#include "Utils.h"

#include "rclcpp/rclcpp.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"

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

    // Detection status feedback
    std::string status;

    // Score
    double score;

    // Competition phase
    std::string phase;

    // [Invalid] Groundtruth position of sUAV itself
    Point real_suav_pos[sUAV_NUM + 1];

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
        
        name_vec = {"time", "day", "hour", "min", "sec", "run_time", "phase", "status", "score"};
        for (int i = 0; i < VESSEL_NUM; i++){
            for (int j = 0; j < 3; j++){
                // "vessel_('a' + i)_('x' + j)"
                std::string s = "vessel_";
                s = s + char('a' + i) + '_' + char('x' + j);
                name_vec.push_back(s);
            }
        }
        for (int i = 1; i <= sUAV_NUM; i++){
            for (int j = 0; j < 3; j++){
                // 'suav_i_('x'+j)
                std::string s = "suav_";
                s = s + std::to_string(i) + '_' + char('x' + j);
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
            std::cout << "Subscribe to /model/Vessel_" + chara_str + "/world_pose" << std::endl;
            vsl_sub[i] = this->create_subscription<geometry_msgs::msg::Pose>(
                    "/model/Vessel_" + chara_str + "/world_pose", 10,
                    [i, this](const geometry_msgs::msg::Pose & msg) -> void{     
                        this->real_vsl_pos[i] = msg.position;
                    }
            );
        }

        // [Invalid] Groundtruth Pose of UAVs
        for (int i = 1; i <= sUAV_NUM; i++){
            std::string chara_str;
            chara_str = std::to_string(i);
            std::cout << "Subscribe to /model/suav_" + chara_str + "/world_pose" << std::endl;
            suav_sub[i] = this->create_subscription<geometry_msgs::msg::Pose>(
                    "/model/suav_" + chara_str + "/world_pose", 10,
                    [i, this](const geometry_msgs::msg::Pose & msg) -> void{     
                        this->real_suav_pos[i] = msg.position;
                    }
            );
        }

        // [Valid] Score
        score_sub = this->create_subscription<std_msgs::msg::Float32>(
            "/mbzirc/score", 10, 
            [this](const std_msgs::msg::Float32 & msg){
                this->score = msg.data;
            }
        );

        // [Valid] Phase
        phase_sub = this->create_subscription<std_msgs::msg::String>(
            "/mbzirc/phase", 10,
            [this](const std_msgs::msg::String & msg){
                this->phase = msg.data;
            }
        );

        // [Valid] Received status
        report_status_sub = this->create_subscription<std_msgs::msg::String>(
            "/mbzirc/target/stream/status", 10,
            [this](const std_msgs::msg::String & msg){
                this->status = msg.data;
            }
        );
        
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
        log_file << clock << "\t"
                 << phase << "\t"
                 << status << "\t"
                 << score << "\t";
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
        std::cout << "\033c" << std::flush;
        log_once();
        output_map();
        // printf("Log!\n");
    }

    void output_map(){
        const int l = 45, w = 101;
        char mp[l][w];
        for (int i = 0; i < l; i++){
            for (int j = 0; j < w; j++){
                mp[i][j] = ' ';
            }
        }
        const double max_l = 3162.28;
        auto f = [max_l](double d, int a)->int{
            int res = int((d + max_l / 2) / max_l * a);
            assert(res >= 0 && res < a);
            return res;
        };
        for (int i = 1; i < sUAV_NUM; i++){
            for (int k = 0; k < l; k++){
                mp[k][int(1.0 * i / sUAV_NUM * w)] = '|';
            }
        }
        for (int i = 0; i < VESSEL_NUM; i++){
            mp[l - 1 - f(real_vsl_pos[i].x, l)][w - 1 - f(real_vsl_pos[i].y, w)] = 'A' + i;
        }
        for (int i = 1; i <= sUAV_NUM; i++){
            char c = i == 10? '0' : '0' + i;
            if (real_suav_pos[i].z <= 1) continue;
            mp[l - 1 - f(real_suav_pos[i].x, l)][w - 1 - f(real_suav_pos[i].y, w)] = c;
        }
        // std::cout << "Now Map:" << std::endl;
        std::cout << "+";
        for (int j = 0; j < w; j++) std::cout << "-";
        std::cout << "+" << std::endl;
        for (int i = 0; i < l; i++){
            std::cout << "|";
            for (int j = 0; j < w; j++){
                if (mp[i][j] >= 'A' && mp[i][j] <= 'G'){
                    std::cout << BOLDGREEN << mp[i][j] << RESET;
                }
                else if (mp[i][j] >= '0' && mp[i][j] <= '9'){
                    std::cout << BOLDYELLOW << mp[i][j] << RESET;
                }
                else std::cout << mp[i][j];
            }
            std::cout << "|" << std::endl;
        }
        std::cout << "+";
        for (int j = 0; j < w; j++) std::cout << "-";
        std::cout << "+" << std::endl;
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_sub;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr suav_sub[sUAV_NUM + 1];
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr vsl_sub[VESSEL_NUM];
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr score_sub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr phase_sub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr report_status_sub;
};


int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Manager>());
    rclcpp::shutdown();
    return 0;
}