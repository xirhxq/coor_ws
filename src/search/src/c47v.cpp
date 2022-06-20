/*---------------------------------------------------
This is a code for testing!!!!!!!!!!!!!!!!!!

To record the video of circling target vessels, I have
to cut off the detection nodes (because of the multi
subscription would slow down the frequency of image),
hence must re-write the whole control code.
---------------------------------------------------*/

#include <sstream>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "std_msgs/msg/int16.hpp"
#include "target_bbox_msgs/msg/bounding_boxes.hpp"

#include "MyMathFun.h"
#include "MyDataFun.h"
typedef geometry_msgs::msg::Point Point;

#define UAV_NUM 10
#define VESSEL_NUM 7

using namespace geometry_msgs::msg;
using namespace std::chrono_literals;
using std::placeholders::_1;

int sUAV_id;

typedef enum TASK_CODE{
    INIT,
    TAKEOFF,
    SEARCH,
    MAP,
    HOLD,
    LAND
} STATE_CODE;

class sUAV : public rclcpp::Node {
public:

    // Position & Attitude & (Linear) Velocity & Air Pressure Height of sUAV itself
    Point UAV_pos;
    Quaternion UAV_att_imu, UAV_att_pos;
    Point UAV_vel;
    double air_pressure;

    // Euler Angles & Transform Matrix Computed
    double UAV_Euler[3];
    double R_e2b[3][3];

    // Position of Target Vessel A-G
    Point vsl_pos[VESSEL_NUM];
    Point real_vsl_pos[VESSEL_NUM];
    MyMathFun::DATA_STAT vsl_pos_stat[VESSEL_NUM];

    // Information Target Vessel Information of Vision
    double vis_vsl_pix[2];
    int vis_vsl_flag, vis_vsl_num;
    std::string vis_vsl_id;
    double q_LOS_v[3];
    double pos_err_v[3];
    Point int_map_pos;
    Point vis_vsl_pos;

    // Position of USV
    Point USV_pos;

    // Velocity & yaw rate saturation of UAV control 
    Point sat_vel;
    double sat_yaw_rate;

    // Some task points in world frame
    Point birth_point, takeoff_point;

    // Time duration from takeoff
    double task_time;

    // Task begin time
    double task_begin_time;
    
    // [StepHold] Time from start & Preset time duration
    double hold_time, hold_duration = 10.0;

    // UAV task state
    STATE_CODE  task_state;

    // [StepSearch] Preset Search Trajectory & Preset Loop # & Trajectory points finished
    std::vector<Point> search_tra;
    int loop, search_tra_finish;

    // [StepSearch] Detecing Results of Other UAVs
    int det_res[UAV_NUM + 1];
    
    // [StepMap] Vessel ID to map
    int vsl_id;

    // [StepMap] Map trajectory radius & Map Lateral velocity & Map Height
    const double MAP_Y_VEL = 3;
    const double CAMERA_ANGLE = 30;
    const double MAP_TRA_HEIGHT = 18;
    const double MAP_TRA_RADIUS = MAP_TRA_HEIGHT / tan(CAMERA_ANGLE * DEG2RAD);

    // [StepMap] Initial relative yaw
    double map_init_theta;

    // [StepMap] Map Start Time
    double map_start_time;

    sUAV(char *name) : Node("suav_" + std::string(name)) {
        sUAV_id = std::atoi(name);

        // [Valid] IMU Data: 1. Pose 2. Orientation
        imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
                "/suav_" + std::to_string(sUAV_id) + "/imu/data", 10,
                std::bind(&sUAV::imu_callback, this, _1));


        // [Valid] Air Pressure Height
        alt_sub = this->create_subscription<sensor_msgs::msg::FluidPressure>(
                "/suav_" + std::to_string(sUAV_id) + "/air_pressure", 10,
                std::bind(&sUAV::alt_callback, this, _1));

        // [Invalid] Groundtruth Pose of Quadrotors
        nav_sub = this->create_subscription<geometry_msgs::msg::Pose>(
                "/model/suav_" + std::to_string(sUAV_id) + "/world_pose", 10,
                std::bind(&sUAV::nav_callback, this, _1));

        // [Invalid] Groundtruth Pose of USV
        usv_sub = this->create_subscription<geometry_msgs::msg::Pose>(
                "/model/usv/world_pose", 10,
                std::bind(&sUAV::usv_pose_callback, this, _1));

        // [Invalid] Groundtruth Pose of Target Vessels
        for (int i = 0; i < VESSEL_NUM; i++){
            std::string chara_str;
            chara_str = chara_str + char('A' + i);
            auto fnc = [this](int i_){
                return [i_, this](const geometry_msgs::msg::Pose & msg) -> void{     
                    this->real_vsl_pos[i_] = msg.position;
                };
            };
            vsl_sub[i] = this->create_subscription<geometry_msgs::msg::Pose>(
                    "/model/Vessel_" + chara_str + "/world_pose", 10,
                    fnc(i));
        }

        // [Valid] Publish Quadrotor Velocity Command
        vel_cmd_pub = this->create_publisher<geometry_msgs::msg::Twist>(
                "/suav_" + std::to_string(sUAV_id) + "/cmd_vel", 10);


        timer_ = this->create_wall_timer(50ms, std::bind(&sUAV::timer_callback, this));
        sat_vel.x = 10;
        sat_vel.y = 10;
        sat_vel.z = 5;
        sat_yaw_rate = 30 * DEG2RAD;

    }

private:
    
    void imu_callback(const sensor_msgs::msg::Imu & msg){
        MyDataFun::set_value_quaternion(this->UAV_att_imu, msg.orientation);
        double q[4];
        q[0] = msg.orientation.w;
        q[1] = msg.orientation.x;
        q[2] = msg.orientation.y;
        q[3] = msg.orientation.z;
        MyMathFun::quaternion_2_euler(q, this->UAV_Euler); 		// ENU
        MyMathFun::Euler_2_Dcm(this->UAV_Euler, this->R_e2b);		// Rotation Matrix: ENU to Body
    }

    void alt_callback(const sensor_msgs::msg::FluidPressure & msg){
        air_pressure = msg.fluid_pressure;
    }

    void nav_callback(const geometry_msgs::msg::Pose & msg){
        MyDataFun::set_value(this->UAV_pos, msg.position);
        MyDataFun::set_value_quaternion(this->UAV_att_pos, msg.orientation);
    }

    void usv_pose_callback(const geometry_msgs::msg::Pose & msg){
        MyDataFun::set_value(this->USV_pos, msg.position);
    }

    void update_time(){
        rclcpp::Time time_now = this->get_clock()->now();
        task_time = time_now.seconds() - task_begin_time;
    }

    template<typename T>
    bool is_near(T a, double r){
        return MyDataFun::dis(a, UAV_pos) <= r;
    }

    template<typename T>
    bool is_near_2d(T a, double r){
        return MyDataFun::dis_2d(a, UAV_pos) <= r;
    }

    template<typename T>
    bool pos_valid(T a){
        return a.x >= -1300.0;
    }

    template<typename T>
    void saturate_vel(T &a){
        a.x = MyMathFun::LimitValue(a.x, sat_vel.x);
        a.y = MyMathFun::LimitValue(a.y, sat_vel.y);
        a.z = MyMathFun::LimitValue(a.z, sat_vel.z);
    }

    void saturate_yaw_rate(double &a){
        if (a <= -PI) a += 2 * PI;
        if (a >= PI) a -= 2 * PI;
        a = MyMathFun::LimitValue(a, sat_yaw_rate);
    }
    
    template<typename T>
    void e2b(T &a){
        double xx = a.x, yy = a.y, zz = a.z;
        a.x = R_e2b[0][0] * xx + R_e2b[0][1] * yy + R_e2b[0][2] * zz;
        a.y = R_e2b[1][0] * xx + R_e2b[1][1] * yy + R_e2b[1][2] * zz;
        a.z = R_e2b[2][0] * xx + R_e2b[2][1] * yy + R_e2b[2][2] * zz;
    }

    template<typename T>
    void UAV_Control_earth(T ctrl_cmd, double yaw_rate){
        geometry_msgs::msg::Twist cmd;
        cmd.angular.z = yaw_rate;
        MyDataFun::set_value(cmd.linear, ctrl_cmd);
        printf("UAV vel cmd in earth frame: %.6lf %.6lf %.6lf\n", cmd.linear.x, cmd.linear.y, cmd.linear.z);
        e2b(cmd.linear);
        printf("UAV vel cmd in body frame: %.6lf %.6lf %.6lf\n", cmd.linear.x, cmd.linear.y, cmd.linear.z);
        saturate_vel(cmd.linear);
        saturate_yaw_rate(cmd.angular.z);
        printf("Saturated UAV vel cmd in body frame: %.6lf %.6lf %.6lf\n", cmd.linear.x, cmd.linear.y, cmd.linear.z);
        printf("Saturated UAV yaw cmd in body frame: %.6lf\n", cmd.angular.z);
        vel_cmd_pub->publish(cmd);
    }

    void UAV_Control_earth(double x, double y, double z, double yaw_rate){
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = x;
        cmd.linear.y = y;
        cmd.linear.z = z;
        UAV_Control_earth(cmd.linear, yaw_rate);
    }

    template<typename T>
    void UAV_Control_body(T ctrl_cmd, double yaw_rate){
        geometry_msgs::msg::Twist cmd;
        cmd.angular.z = yaw_rate;
        MyDataFun::set_value(cmd.linear, ctrl_cmd);
        printf("UAV vel cmd in body frame: %.6lf %.6lf %.6lf\n", cmd.linear.x, cmd.linear.y, cmd.linear.z);
        saturate_vel(cmd.linear);
        vel_cmd_pub->publish(cmd);
    }

    void UAV_Control_body(double x, double y, double z, double yaw_rate){
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = x;
        cmd.linear.y = y;
        cmd.linear.z = z;
        UAV_Control_body(cmd.linear, yaw_rate);
    }

    template<typename T>
    void UAV_Control_circle_while_facing(T a){
        printf("Control in circle while facing (%.2lf, %.2lf, %.2lf)\n", a.x, a.y, a.z);
        geometry_msgs::msg::Twist cmd;
        double dis2vsl = sqrt(pow(a.y - UAV_pos.y, 2) + pow(a.x - UAV_pos.x, 2));
        printf("Distance 2 Vessel: %.2lf\n", dis2vsl);
        cmd.linear.x = (dis2vsl - MAP_TRA_RADIUS) * 0.2;
        cmd.linear.y = MAP_Y_VEL;
        cmd.linear.z = (MAP_TRA_HEIGHT - UAV_pos.z) * 0.2;
        double des_yaw = atan2(a.y - UAV_pos.y, a.x - UAV_pos.x);
        printf("Now yaw is %.2lf while desired yaw is %.2lf\n", UAV_Euler[2] * RAD2DEG, des_yaw * RAD2DEG);
        cmd.angular.z = des_yaw - UAV_Euler[2];
        printf("UAV yaw cmd: %.2lf\n", cmd.angular.z);
        printf("UAV vel cmd in body frame: %.6lf %.6lf %.6lf\n", cmd.linear.x, cmd.linear.y, cmd.linear.z);
        saturate_vel(cmd.linear);
        saturate_yaw_rate(cmd.angular.z);
        printf("Saturated UAV vel cmd in body frame: %.6lf %.6lf %.6lf\n", cmd.linear.x, cmd.linear.y, cmd.linear.z);
        printf("Saturated UAV yaw cmd in body frame: %.6lf\n", cmd.angular.z);
        vel_cmd_pub->publish(cmd);

    }
    
    template<typename T>
    void UAV_Control_to_Point_earth(T ctrl_cmd){
        printf("Control to Point (%.2lf, %.2lf, %.2lf)\n", ctrl_cmd.x, ctrl_cmd.y, ctrl_cmd.z);
        UAV_Control_earth(MyDataFun::minus(ctrl_cmd, UAV_pos), -0.2 * UAV_Euler[2]);
    }

    void UAV_Control_to_Point_earth(double x, double y, double z){
        Point p;
        p.x = x;
        p.y = y;
        p.z = z;
        UAV_Control_to_Point_earth(p);
    }

    void StepInit(){
        UAV_Control_earth(0, 0, 0, 0);
        MyDataFun::set_value(birth_point, UAV_pos);
        MyDataFun::set_value(takeoff_point, birth_point);
        takeoff_point.z += 20;
        printf("Takeoff Point @ (%.2lf, %.2lf, %.2lf) !!!!!!!!!!!!!!!!!!!!!!!!!!!!\n", takeoff_point.x, takeoff_point.y, takeoff_point.z);
        task_begin_time = this->get_clock()->now().seconds();
        if (UAV_pos.x != 0.0 || UAV_pos.y != 0.0 || UAV_pos.z != 0.0){
            printf("Get Ground Truth Position!!!!!!!!!!!!!!!!!!!!!!\n");
            task_state = TAKEOFF;
        }
    }

    void StepTakeoff(){
        UAV_Control_to_Point_earth(takeoff_point);
     	printf("Takeoff!!!\n");
        if (is_near(takeoff_point, 1)){
            printf("Takeoff Completed !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
            task_state = SEARCH;
            vsl_id = 4;
        }
    }

    void StepSearch(){
        printf("To Vessel %c !!!!!!!!!!!\n", 'A' + vsl_id);
        UAV_Control_to_Point_earth(real_vsl_pos[vsl_id]);
        if (is_near_2d(real_vsl_pos[vsl_id], 80)){
            printf("Near Vessel %c !!!!!!!!!!!!!!!!!!!!!!!!\n", 'A' + vsl_id);
            task_state = MAP;
            StepMapInit();
        }
    }
	
    void StepMapInit(){
        map_init_theta = atan2(UAV_pos.y - real_vsl_pos[vsl_id].y, UAV_pos.x - real_vsl_pos[vsl_id].x);
        printf("theta_init = %.2lf !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n", map_init_theta * RAD2DEG);
        map_start_time = this->get_clock()->now().seconds();
    }

    void StepMap(){
     	printf("MAP!!!\n");
        UAV_Control_circle_while_facing(real_vsl_pos[vsl_id]);
        if (this->get_clock()->now().seconds() - map_start_time >= 120){
            if (vsl_id == 6){
                task_state = HOLD;
                hold_time = this->get_clock()->now().seconds();
            }
            else {
                task_state = SEARCH;
                vsl_id++;
            }
        }
    }
    
    void StepHold(){
        printf("Holding: %.2lf!!!\n", task_time - hold_time);
        UAV_Control_earth(0, 0, 0, 0);
        if (task_time - hold_time >= hold_duration){
            printf("Hold %.2lf seconds completed !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n", hold_duration);
            task_state = LAND;
        }
    }

    void StepLand(){
        UAV_Control_to_Point_earth(birth_point);
    }

    void timer_callback() {
        update_time();
        printf("---------------------------------\n-----------A New Frame-----------\n---------------------------------\n");
        printf("Time: %.2lf\n", task_time);
        printf("Me @ (%.2lf, %.2lf, %.2lf)\n", UAV_pos.x, UAV_pos.y, UAV_pos.z);

        switch (task_state){
            case INIT:{
                StepInit();
                break;
            }
            case TAKEOFF:{
                StepTakeoff();
                break;
            }
            case SEARCH:{
                StepSearch();
                break;
            }
            case MAP:{
                StepMap();
                break;
            }
            case HOLD:{
                StepHold();
                break;
            }
            case LAND:{
                StepLand();
                break;
            }
            default:{
            	StepHold();
            	break;
            }
        }
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
    rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr alt_sub;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr nav_sub, usv_sub, vsl_sub[VESSEL_NUM];
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_cmd_pub;
};


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<sUAV>(argv[1]));
    rclcpp::shutdown();
    return 0;
}
