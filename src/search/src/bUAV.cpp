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

using namespace geometry_msgs::msg;
using namespace std::chrono_literals;
using std::placeholders::_1;

int bUAV_id;
double buav_hold_points[10][3] = {{-1500, 0, 100},
                                  {-1500, 50, 50}, {-2000, 0, 140},
                                  {-2500, 0, 100}, {-2500, -1000, 100},
                                  {-2500, 1000, 100}, {-2500, 0, 50}};

typedef enum TASK_CODE{
    INIT,
    TAKEOFF,
    PREPARE,
    HOLD,
    BACK,
    LAND
} STATE_CODE;

class bUAV : public rclcpp::Node {
public:

    // Position & Attitude & (Linear) Velocity & Air Pressure Height of bUAV itself
    Point UAV_pos;
    Quaternion UAV_att_imu, UAV_att_pos;
    Point UAV_vel;
    double air_pressure;

    // Euler Angles & Transform Matrix Computed
    double UAV_Euler[3];
    double R_e2b[3][3];

    // Velocity & yaw rate saturation of UAV control 
    Point sat_vel;
    double sat_yaw_rate;

    // Some task points in world frame
    Point birth_point, takeoff_point;

    // Time duration from takeoff
    double task_time;

    // Task begin time
    double task_begin_time;
    
    // UAV task state
    STATE_CODE  task_state;

    // [StepHold] Time from start & Preset time duration
    double hold_time, hold_duration = 10.0;

    // [StepHold] Hold Point
    Point hold_point;


    bUAV(char *name) : Node("buav_" + std::string(name)) {
        bUAV_id = std::atoi(name);

        // [Valid] IMU Data: 1. Pose 2. Orientation
        imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
                "/buav_" + std::to_string(bUAV_id) + "/imu/data", 10,
                std::bind(&bUAV::imu_callback, this, _1));


        // [Valid] Air Pressure Height
        alt_sub = this->create_subscription<sensor_msgs::msg::FluidPressure>(
                "/buav_" + std::to_string(bUAV_id) + "/air_pressure", 10,
                std::bind(&bUAV::alt_callback, this, _1));

        // [Invalid] Groundtruth Pose of Quadrotor
        nav_sub = this->create_subscription<geometry_msgs::msg::Pose>(
                "/model/buav_" + std::to_string(bUAV_id) + "/world_pose", 10,
                std::bind(&bUAV::nav_callback, this, _1));

        // [Valid] Publish Quadrotor Velocity Command
        vel_cmd_pub = this->create_publisher<geometry_msgs::msg::Twist>(
                "/buav_" + std::to_string(bUAV_id) + "/cmd_vel", 10);


        timer_ = this->create_wall_timer(50ms, std::bind(&bUAV::timer_callback, this));
        sat_vel.x = 5;
        sat_vel.y = 5;
        sat_vel.z = 2;
        sat_yaw_rate = 30 * DEG2RAD;

        hold_point = MyDataFun::new_point(buav_hold_points[bUAV_id]);

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
        takeoff_point.z += 20 + 5 * bUAV_id;
        printf("Takeoff Point @ (%.2lf, %.2lf, %.2lf) !!!!!!!!!!!!!!!!!!!!!!!!!!!!\n", birth_point.x, birth_point.y, birth_point.z);
        rclcpp::Time time_now = this->get_clock()->now();
        task_begin_time = time_now.seconds();
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
        UAV_Control_to_Point_earth(hold_point);
        printf("To Hold Point!!!!!\n");
        if (is_near(hold_point, 1)){
            printf("Prepare Completed!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
            task_state = HOLD;
            hold_time = this->get_clock()->now().seconds() - task_begin_time;
        }
    }
    
    void StepHold(){
        printf("Holding: %.2lf!!!\n", task_time - hold_time);
        UAV_Control_to_Point_earth(hold_point);
        if (task_time - hold_time >= hold_duration){
            printf("Hold %.2lf seconds completed !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n", hold_duration);
            task_state = BACK;
        }
    }

    void StepBack(){
        UAV_Control_to_Point_earth(takeoff_point);
     	printf("Back!!!\n");
        if (is_near(takeoff_point, 1)){
            printf("Back to Takeoff Point !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
            task_state = LAND;
        }
    }

    void StepLand(){
        UAV_Control_to_Point_earth(birth_point);
        printf("Land!!!\n");
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
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
    rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr alt_sub;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr nav_sub;
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_cmd_pub;
};


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<bUAV>(argv[1]));
    rclcpp::shutdown();
    return 0;
}
