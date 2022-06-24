#include <sstream>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>
#include <vector>
#include <fstream>
#include <ctime>

#include "rclcpp/rclcpp.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
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
#define KP 0.2
#define X_KP KP
#define Y_KP KP
#define Z_KP KP
#define YAW_KP 1.0

using namespace geometry_msgs::msg;
using namespace std::chrono_literals;
using std::placeholders::_1;

int testUAV_id;

typedef enum TASK_CODE{
    INIT,
    TAKEOFF,
    PREPARE,
    SEARCH,
    HOLD,
    BACK,
    LAND
} STATE_CODE;

class testUAV : public rclcpp::Node {
public:
    // Log fstream
    std::ofstream log_file; 
    
    // Variable names in log file
    std::vector<std::string> name_vec; 
    
    // Sim time clock
    double clock; 

    // [Invalid] Groundtruth position of testUAV itself
    Point UAV_pos;

    // Attitude of testUAV itself by imu
    Quaternion UAV_att_imu;
    
    // [Invalid] Groundtruth attitude of testUAV itself
    Quaternion UAV_att_pos;

    // [Invalid] Groundtruth velocity in world frame of testUAV itself
    Point UAV_vel;

    // Air pressure height of testUAV itself
    double air_pressure;

    // Euler angles of testUAV itself
    double UAV_Euler[3];

    // Transform matrix of testUAV itself
    double R_e2b[3][3];



    // Target LOS in body frame
    double q_LOS_v[3];

    // Target relative position in world frame
    double pos_err_v[3];

    // Target vessel position by vision
    Point vis_vsl_pos;

    // Position of USV
    Point USV_pos;

    // Velocity saturation of UAV control 
    Point sat_vel;

    // Yaw rate saturation of UAV control
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

    // [StepSearch] Preset Search Trajectory
    std::vector<Point> search_tra;

    // [StepSearch] Preset Loop #
    int loop;
    
    // [StepSearch] Trajectory points finished
    int search_tra_finish;


    // Camera angle
    const double CAMERA_ANGLE = 30;



    testUAV(char *name) : Node("testuav_" + std::string(name)) {
        testUAV_id = std::atoi(name);

        time_t tt = time(NULL);
        tm* t = localtime(&tt);
        char iden_path[256];
        sprintf(iden_path, "/home/ps/coor_ws/src/search/data/%02d-%02d_%02d-%02d_%d.txt",
        t->tm_mon + 1, t->tm_mday, t->tm_hour, t->tm_min, testUAV_id);
        log_file.open(iden_path, std::ios::out);
        while(!log_file) std::cout << "Error: Could not write data!" << std::endl;
        
        name_vec = {"time", "day", "hour", "min", "sec", "control_state", 
                    "uav_pos_x", "uav_pos_y", "uav_pos_z",
                    "uav_roll", "uav_pitch", "uav_yaw",
                    "uav_vel_x", "uav_vel_y", "uav_vel_z"};

        for (auto t: name_vec){
            log_file << t << "\t";
        } log_file << std::endl;


        // [Valid] Clock
        clock_sub = this->create_subscription<rosgraph_msgs::msg::Clock>(
            "/clock", 10, std::bind(&testUAV::clock_callback, this, _1)
        );

        // [Valid] IMU Data: 1. Pose 2. Orientation
        imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
                "/suav_" + std::to_string(testUAV_id) + "/imu/data", 10,
                std::bind(&testUAV::imu_callback, this, _1));


        // [Valid] Air Pressure Height
        alt_sub = this->create_subscription<sensor_msgs::msg::FluidPressure>(
                "/suav_" + std::to_string(testUAV_id) + "/air_pressure", 10,
                std::bind(&testUAV::alt_callback, this, _1));

        // [Invalid] Groundtruth Pose of Quadrotors
        nav_sub = this->create_subscription<geometry_msgs::msg::Pose>(
                "/model/suav_" + std::to_string(testUAV_id) + "/world_pose", 10,
                std::bind(&testUAV::nav_callback, this, _1));




        // [Valid] Publish Quadrotor Velocity Command
        vel_cmd_pub = this->create_publisher<geometry_msgs::msg::Twist>(
                "/suav_" + std::to_string(testUAV_id) + "/cmd_vel", 10);


        timer_ = this->create_wall_timer(50ms, std::bind(&testUAV::timer_callback, this));
        sat_vel.x = 15;
        sat_vel.y = 15;
        sat_vel.z = 5;
        sat_yaw_rate = 90 * DEG2RAD;
        loop = 1;
        double search_height = 50;
        for (int i = 1; i <= loop; i++){
            search_tra.push_back(MyDataFun::new_point(-1500, 1500, search_height));
            search_tra.push_back(MyDataFun::new_point(1800, 1500, search_height));
            search_tra.push_back(MyDataFun::new_point(1800, -1500, search_height));
            search_tra.push_back(MyDataFun::new_point(-1500, -1500, search_height));
        }

    }

private:

    void clock_callback(const rosgraph_msgs::msg::Clock & msg){
        clock = 1.0 * msg.clock.sec + 1.0 * msg.clock.nanosec / 1e9;
    }
    
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

    double get_time_now(){
        return clock;
        // return this->get_clock()->now().seconds();
    }

    void update_time(){
        task_time = get_time_now() - task_begin_time;
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
        cmd.angular.z = yaw_rate * YAW_KP;
        MyDataFun::set_value(cmd.linear, ctrl_cmd);
        cmd.linear.x *= X_KP;
        cmd.linear.y *= Y_KP;
        cmd.linear.z *= Z_KP;
        printf("UAV vel cmd in earth frame: %.6lf %.6lf %.6lf\n", cmd.linear.x, cmd.linear.y, cmd.linear.z);
        saturate_vel(cmd.linear);
        printf("Saturated UAV vel cmd in earth frame: %.6lf %.6lf %.6lf\n", cmd.linear.x, cmd.linear.y, cmd.linear.z);
        e2b(cmd.linear);
        printf("Saturated UAV vel cmd in body frame: %.6lf %.6lf %.6lf\n", cmd.linear.x, cmd.linear.y, cmd.linear.z);
        saturate_yaw_rate(cmd.angular.z);
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
    void UAV_Control_to_Point_earth(T ctrl_cmd){
        printf("Control to Point (%.2lf, %.2lf, %.2lf)\n", ctrl_cmd.x, ctrl_cmd.y, ctrl_cmd.z);
        double yaw_diff = MyDataFun::angle_2d(UAV_pos, ctrl_cmd) - UAV_Euler[2];
        if (MyDataFun::dis_2d(ctrl_cmd, UAV_pos) <= 5) yaw_diff = 0;
        UAV_Control_earth(MyDataFun::minus(ctrl_cmd, UAV_pos), yaw_diff);
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
        MyDataFun::set_value(takeoff_point, UAV_pos);
        takeoff_point.z += 50;
        printf("Takeoff Point @ (%.2lf, %.2lf, %.2lf) !!!!!!!!!!!!!!!!!!!!!!!!!!!!\n", takeoff_point.x, takeoff_point.y, takeoff_point.z);
        task_begin_time = get_time_now();
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
            task_state = PREPARE;
        }
    }

    void StepPrepare(){
        UAV_Control_to_Point_earth(search_tra[0]);
     	printf("Prepare!!!\n");
        if (is_near(search_tra[0], 1)){
            printf("Prepare Completed !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
            task_state = SEARCH;
            search_tra_finish = 0;
        }
    }

    void StepSearch(){
     	printf("Search!!!\n");
        UAV_Control_to_Point_earth(search_tra[search_tra_finish]);
        if (is_near(search_tra[search_tra_finish], 1)){
            search_tra_finish++;
            printf("#%d Trajectory Point Arrived !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n", search_tra_finish);
            if (search_tra_finish == int(search_tra.size())){
                printf("Search Failed !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
                search_tra_finish = 0;
                task_state = LAND;
            }
        }
        
    }
	

    
    void StepHold(){
        printf("Holding: %.2lf!!!\n", task_time - hold_time);
        UAV_Control_earth(0, 0, 0, 0);
        if (task_time - hold_time >= hold_duration){
            printf("Hold %.2lf seconds completed !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n", hold_duration);
            task_state = BACK;
        }
    }

    void StepBack(){
        UAV_Control_to_Point_earth(takeoff_point);
     	printf("Back!!!\n");
        if (is_near(takeoff_point, 1)){
            printf("Back Completed !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
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
        // printf("Quaternion by imu: (%.2lf, %.2lf, %.2lf, %.2lf)\n", UAV_att_imu.w, UAV_att_imu.x, UAV_att_imu.y, UAV_att_imu.z);
        // printf("Quaternion by pos: (%.2lf, %.2lf, %.2lf, %.2lf)\n", UAV_att_pos.w, UAV_att_pos.x, UAV_att_pos.y, UAV_att_pos.z);
        // printf("Euler angle: (Phi %.2lf, Theta %.2lf, Psi %.2lf)\n", UAV_Euler[0] * RAD2DEG, UAV_Euler[1] * RAD2DEG, UAV_Euler[2] * RAD2DEG);
        // printf("Transform Matrix: ------\n");
        // for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) printf("%.2lf%c", R_e2b[i][j], (j==2)?'\n':'\t');
        printf("\n");
        switch (task_state){
            case INIT:{
                StepInit();
                break;
            }
            case TAKEOFF:{
                StepTakeoff();
                break;
            }
            case PREPARE:{
                StepPrepare();
                break;
            }
            case SEARCH:{
                StepSearch();
                break;
            }
            case HOLD:{
                StepHold();
                break;
            }
            case BACK:{
                StepBack();
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
    rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_sub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
    rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr alt_sub;
    rclcpp::Subscription<target_bbox_msgs::msg::BoundingBoxes>::SharedPtr det_box_sub;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr nav_sub, usv_sub;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr det_pub;
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_cmd_pub;
};


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<testUAV>(argv[1]));
    rclcpp::shutdown();
    return 0;
}
