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
typedef geometry_msgs::msg::Point Point;

#define VESSEL_NUM 7
#define PI std::acos(-1)
#define DEG2RAD PI/180
#define RAD2DEG 180/PI

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

    // Position of USV
    Point USV_pos;

    // Velocity & yaw rate saturation of UAV control 
    Point sat_vel;
    double sat_yaw_rate;

    // Some task points in world frame
    Point takeoff_point;

    // Time duration from takeoff
    double task_time;
    
    // [StepHold] Time from start & Preset time duration
    double hold_time, hold_duration = 10.0;

    // UAV task state
    STATE_CODE  task_state;

    // [StepSearch] Preset Search Trajectory & Preset Loop # & Trajectory points finished
    std::vector<Point> search_tra;
    int loop, search_tra_finish;
    
    // [StepMap] Vessel ID to map
    int vsl_id;

    // [StepMap] Relative trajectory of map & Trajectory points finished & Initial relative yaw
    std::vector<Point> map_tra;
    int map_tra_finish;
    double map_init_theta;
    

    sUAV(char *name) : Node("sUAV_" + std::string(name)) {
        sUAV_id = std::atoi(name);
        imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
                "/quadrotor_" + std::to_string(sUAV_id) + "/imu/data", 10,
                std::bind(&sUAV::imu_callback, this, _1));
        alt_sub = this->create_subscription<sensor_msgs::msg::FluidPressure>(
                "/quadrotor_" + std::to_string(sUAV_id) + "/air_pressure", 10,
                std::bind(&sUAV::alt_callback, this, _1));
        nav_sub = this->create_subscription<geometry_msgs::msg::Pose>(
                "/model/quadrotor_" + std::to_string(sUAV_id) + "/world_pose", 10,
                std::bind(&sUAV::nav_callback, this, _1));
        usv_sub = this->create_subscription<geometry_msgs::msg::Pose>(
                "/model/usv/world_pose", 10,
                std::bind(&sUAV::usv_pose_callback, this, _1));
        for (int i = 0; i < VESSEL_NUM; i++){
            std::string chara_str;
            chara_str = chara_str + char('A' + i);
            auto fnc = [this](int i_){
                return [i_, this](const geometry_msgs::msg::Pose & msg) -> void{     
                    this->vsl_pos[i_] = msg.position;
                };
            };
            vsl_sub[i] = this->create_subscription<geometry_msgs::msg::Pose>(
                    "/model/Vessel_" + chara_str + "/world_pose", 10,
                    fnc(i));
        }
        vel_cmd_pub = this->create_publisher<geometry_msgs::msg::Twist>(
                "/quadrotor_" + std::to_string(sUAV_id) + "/cmd_vel", 10);
        timer_ = this->create_wall_timer(500ms, std::bind(&sUAV::timer_callback, this));
        sat_vel.x = 5;
        sat_vel.y = 5;
        sat_vel.z = 2;
        sat_yaw_rate = 30 * DEG2RAD;
        loop = 1;
        double search_single_width = 100, search_signle_depth = 150;
        double search_forward_y = (std::abs (sUAV_id - 5.5) - 0.25) * search_single_width  * ((sUAV_id > 5) * 2 - 1);
        double search_backward_y = (std::abs (sUAV_id - 5.5) + 0.25) * search_single_width * ((sUAV_id > 5) * 2 - 1);
        double search_backward_x = -1450;
        double search_forward_x = search_backward_x + search_signle_depth;
        double search_height = 30;
        for (int i = 1; i <= loop; i++){
            search_tra.push_back(new_point(search_backward_x, search_forward_y, search_height));
            search_tra.push_back(new_point(search_forward_x, search_forward_y, search_height));
            search_tra.push_back(new_point(search_forward_x, search_backward_y, search_height));
            search_tra.push_back(new_point(search_backward_x, search_backward_y, search_height));
        }
    }

private:
    
    Point new_point(double x, double y, double z){
        Point res;
        double p[3] = {x, y, z};
        set_value(res, p);
        return res;
    }
	
	void quaternion_2_euler(double quat[4], double angle[3]){
		angle[0] = atan2(2.0 * (quat[3] * quat[2] + quat[0] * quat[1]), 1.0 - 2.0 * (quat[1] * quat[1] + quat[2] * quat[2]));
		angle[1] = asin(2.0 * (quat[2] * quat[0] - quat[3] * quat[1]));
		angle[2] = atan2(2.0 * (quat[3] * quat[0] + quat[1] * quat[2]), -1.0 + 2.0 * (quat[0] * quat[0] + quat[1] * quat[1]));
	}
	
	void Euler_2_Dcm(double euler[3], double dcm[3][3]){
		double phi = euler[0], theta = euler[1], psi = euler[2];
		double sinPhi = sin(phi), cosPhi = cos(phi);
		double sinThe = sin(theta), cosThe = cos(theta);
		double sinPsi = sin(psi), cosPsi = cos(psi);
		dcm[0][0] = cosThe * cosPsi;
		dcm[0][1] = cosThe * sinPsi; 
		dcm[0][2] = -sinThe; 

		dcm[1][0] = -cosPhi * sinPsi + sinPhi * sinThe * cosPsi;
		dcm[1][1] = cosPhi * cosPsi + sinPhi * sinThe * sinPsi;
		dcm[1][2] = sinPhi * cosThe;

		dcm[2][0] = sinPhi * sinPsi + cosPhi * sinThe * cosPsi;
		dcm[2][1] = -sinPhi * cosPsi + cosPhi * sinThe * sinPsi;
		dcm[2][2] = cosPhi * cosThe;
	}

    void imu_callback(const sensor_msgs::msg::Imu & msg){
        set_value_quaternion(this->UAV_att_imu, msg.orientation);
        double q[4];
        q[0] = msg.orientation.w;
        q[1] = msg.orientation.x;
        q[2] = msg.orientation.y;
        q[3] = msg.orientation.z;
        quaternion_2_euler(q, this->UAV_Euler); 		// ENU
        Euler_2_Dcm(this->UAV_Euler, this->R_e2b);		// Rotation Matrix: ENU to Body
    }

    void alt_callback(const sensor_msgs::msg::FluidPressure & msg){
        air_pressure = msg.fluid_pressure;
    }

    void nav_callback(const geometry_msgs::msg::Pose & msg){
        this->UAV_pos = msg.position;
        set_value_quaternion(this->UAV_att_pos, msg.orientation);
    }

    void usv_pose_callback(const geometry_msgs::msg::Pose & msg){
        this->USV_pos = msg.position;
    }

    void update_time(){
        rclcpp::Time time_now = this->get_clock()->now();
        double currTimeSec = time_now.seconds() - task_begin_time.seconds();
        task_time =  currTimeSec;
    }

    double LimitValue(double x, double sat){
        return std::min(std::abs(sat), std::max(-std::abs(sat), x));
    }

    template<typename T1, typename T2>
    double dis(T1 a, T2 b){
        return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2) + std::pow(a.z - b.z, 2));
    }

    template<typename T1, typename T2>
    double dis_2d(T1 a, T2 b){
        return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
    }

    template<typename T>
    bool is_near(T a, double r){
        return dis(a, UAV_pos) <= r;
    }

    template<typename T>
    bool is_near_2d(T a, double r){
        return dis_2d(a, UAV_pos) <= r;
    }

    template<typename T1, typename T2>
    void set_value(T1 &a, T2 b){
        a.x = b.x;
        a.y = b.y;
        a.z = b.z;
    }

    template<typename T1, typename T2>
    void set_value_quaternion(T1 &a, T2 b){
        a.x = b.x;
        a.y = b.y;
        a.z = b.z;
        a.w = b.w;
    }
    
    template<typename T>
    void set_value(T &a, double b[]){
        a.x = b[0];
        a.y = b[1];
        a.z = b[2];
    }

    template<typename T>
    void saturate_vel(T &a){
        a.x = LimitValue(a.x, sat_vel.x);
        a.y = LimitValue(a.y, sat_vel.y);
        a.z = LimitValue(a.z, sat_vel.z);
    }

    void saturate_yaw_rate(double &a){
        if (a <= -PI) a += 2 * PI;
        if (a >= PI) a -= 2 * PI;
        a = LimitValue(a, sat_yaw_rate);
    }
    
    template<typename T>
    void e2b(T &a){
        double xx = a.x, yy = a.y, zz = a.z;
        a.x = R_e2b[0][0] * xx + R_e2b[0][1] * yy + R_e2b[0][2] * zz;
        a.y = R_e2b[1][0] * xx + R_e2b[1][1] * yy + R_e2b[1][2] * zz;
        a.z = R_e2b[2][0] * xx + R_e2b[2][1] * yy + R_e2b[2][2] * zz;
    }

    template<typename T>
    void UAV_Control(T ctrl_cmd, double yaw_rate){
        geometry_msgs::msg::Twist cmd;
        cmd.angular.z = yaw_rate;
        set_value(cmd.linear, ctrl_cmd);
        saturate_vel(cmd.linear);
        printf("UAV vel cmd in earth frame: %.6lf %.6lf %.6lf\n", cmd.linear.x, cmd.linear.y, cmd.linear.z);
        e2b(cmd.linear);
        printf("UAV vel cmd in body frame: %.6lf %.6lf %.6lf\n", cmd.linear.x, cmd.linear.y, cmd.linear.z);
        vel_cmd_pub->publish(cmd);
    }

    void UAV_Control(double x, double y, double z, double yaw_rate){
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = x;
        cmd.linear.y = y;
        cmd.linear.z = z;
        UAV_Control(cmd.linear, yaw_rate);
    }

    template<typename T1, typename T2>
    void UAV_Control_to_point_while_facing(T1 a, T2 b){
        printf("Control to Point (%.2lf, %.2lf, %.2lf) while facing (%.2lf, %.2lf, %.2lf)\n", a.x, a.y, a.z, b.x, b.y, b.z);
        geometry_msgs::msg::Twist cmd;
        set_value(cmd.linear, point_minus(a, UAV_pos));
        double des_yaw = atan2(b.y - UAV_pos.y, b.x - UAV_pos.x);
        printf("Now yaw is %.2lf while desired yaw is %.2lf\n", UAV_Euler[2] * RAD2DEG, des_yaw * RAD2DEG);
        cmd.angular.z = des_yaw - UAV_Euler[2];
        saturate_vel(cmd.linear);
        saturate_yaw_rate(cmd.angular.z);
        printf("UAV yaw cmd: %.2lf\n", cmd.angular.z);
        printf("UAV vel cmd in earth frame: %.6lf %.6lf %.6lf\n", cmd.linear.x, cmd.linear.y, cmd.linear.z);
        e2b(cmd.linear);
        printf("UAV vel cmd in body frame: %.6lf %.6lf %.6lf\n", cmd.linear.x, cmd.linear.y, cmd.linear.z);
        vel_cmd_pub->publish(cmd);
    }
    
    template<typename T1, typename T2>
    T1 point_minus(T1 a, T2 b){
        T1 res = a;
        res.x = a.x - b.x;
        res.y = a.y - b.y;
        res.z = a.z - b.z;
        return res;
    }    

    template<typename T1, typename T2>
    T1 point_plus(T1 a, T2 b){
        T1 res = a;
        res.x = a.x + b.x;
        res.y = a.y + b.y;
        res.z = a.z + b.z;
        return res;
    }

    template<typename T>
    void UAV_Control_to_Point(T ctrl_cmd){
        printf("Control to Point (%.2lf, %.2lf, %.2lf)\n", ctrl_cmd.x, ctrl_cmd.y, ctrl_cmd.z);
        UAV_Control(point_minus(ctrl_cmd, UAV_pos), 0);
    }

    void StepInit(){
        set_value(takeoff_point, UAV_pos);
        printf("Takeoff Point @ (%.2lf, %.2lf, %.2lf) !!!!!!!!!!!!!!!!!!!!!!!!!!!!\n", takeoff_point.x, takeoff_point.y, takeoff_point.z);
        if (UAV_pos.x != 0.0 || UAV_pos.y != 0.0 || UAV_pos.z != 0.0){
            printf("Get Ground Truth Position!!!!!!!!!!!!!!!!!!!!!!\n");
            task_state = TAKEOFF;
        }
    }


    void StepTakeoff(){
        UAV_Control_to_Point(search_tra[0]);
     	printf("Takeoff!!!\n");
        if (is_near(search_tra[0], 1)){
            printf("Takeoff Completed !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
            task_state = SEARCH;
            search_tra_finish = 0;
        }
    }

    void StepSearch(){
     	printf("Search!!!\n");
        UAV_Control_to_Point(search_tra[search_tra_finish]);
        if (is_near(search_tra[search_tra_finish], 1)){
            search_tra_finish++;
            printf("#%d Trajectory Point Arrived !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n", search_tra_finish);
            if (search_tra_finish == int(search_tra.size())){
                printf("Search Completed !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
                search_tra_finish = 0;
                task_state = MAP;
                StepMapInit();
            }
        }
        for (int i = 0; i < VESSEL_NUM; i++){
            if (is_near_2d(vsl_pos[i], 30)){
                printf("Got Vessel %d !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n", i);
                vsl_id = i;
                search_tra_finish = 0;
                task_state = MAP;
                StepMapInit();
            }
        }
    }

    void StepMapInit(){
        map_tra_finish = 0;
        map_init_theta = atan2(UAV_pos.y - vsl_pos[vsl_id].y, UAV_pos.x - vsl_pos[vsl_id].x);
        printf("theta_init = %.2lf !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n", map_init_theta * RAD2DEG);
        const int MAP_POINT = 10;
        for (int i = 0; i < MAP_POINT; i++){
            map_tra.push_back(new_point(std::cos(1.0 * i / MAP_POINT * 2 * PI + map_init_theta) * 30,
                                        std::sin(1.0 * i / MAP_POINT * 2 * PI + map_init_theta) * 30,
                                        30));
            printf("Map Tra #%d: %.2lf, %.2lf, %.2lf\n", i, map_tra[i].x, map_tra[i].y, map_tra[i].z);
        }
    }

    void StepMap(){
     	printf("MAP!!!\n");
        UAV_Control_to_point_while_facing(point_plus(map_tra[map_tra_finish], vsl_pos[vsl_id]), vsl_pos[vsl_id]);
        if (is_near(point_plus(map_tra[map_tra_finish], vsl_pos[vsl_id]), 1)){
            map_tra_finish++;
            printf("#%d Map Point Arrived !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n", map_tra_finish);
            if (map_tra_finish == int(map_tra.size())){
                printf("Map Completed !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
                map_tra_finish = 0;
                hold_time = task_time;
                task_state = HOLD;
            }
        }
    }
    
    void StepHold(){
        printf("Hold!!!\n");
        UAV_Control(0, 0, 0, 0);
        if (task_time - hold_time >= hold_duration){
            printf("Hold %.2lf seconds completed !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n", hold_duration);
            task_state = LAND;
        }
    }

    void StepLand(){
        UAV_Control_to_Point(takeoff_point);
    }

    void timer_callback() {
        update_time();
        printf("---------------------------------\n-----------A New Frame-----------\n---------------------------------\n");
        printf("Time: %.2lf\n", task_time - task_begin_time);
        printf("Me @ (%.2lf, %.2lf, %.2lf)\n", UAV_pos.x, UAV_pos.y, UAV_pos.z);
        printf("Quaternion by imu: (%.2lf, %.2lf, %.2lf, %.2lf)\n", UAV_att_imu.w, UAV_att_imu.x, UAV_att_imu.y, UAV_att_imu.z);
        printf("Quaternion by pos: (%.2lf, %.2lf, %.2lf, %.2lf)\n", UAV_att_pos.w, UAV_att_pos.x, UAV_att_pos.y, UAV_att_pos.z);
        printf("Euler angle: (Phi %.2lf, Theta %.2lf, Psi %.2lf)\n", UAV_Euler[0] * RAD2DEG, UAV_Euler[1] * RAD2DEG, UAV_Euler[2] * RAD2DEG);
        printf("Transform Matrix: ------\n");
        for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) printf("%.2lf%c", R_e2b[i][j], (j==2)?'\n':'\t');
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

    rclcpp::Time task_begin_time;
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
