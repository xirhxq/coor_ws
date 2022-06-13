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
    Point takeoff_point;

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
    const double MAP_TRA_HEIGHT = 40;
    const double MAP_TRA_RADIUS = MAP_TRA_HEIGHT / tan(CAMERA_ANGLE * DEG2RAD);

    // [StepMap] Initial relative yaw
    double map_init_theta;

    sUAV(char *name) : Node("sUAV_" + std::string(name)) {
        sUAV_id = std::atoi(name);

        // [Valid] IMU Data: 1. Pose 2. Orientation
        imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
                "/quadrotor_" + std::to_string(sUAV_id) + "/imu/data", 10,
                std::bind(&sUAV::imu_callback, this, _1));


        // [Valid] Air Pressure Height
        alt_sub = this->create_subscription<sensor_msgs::msg::FluidPressure>(
                "/quadrotor_" + std::to_string(sUAV_id) + "/air_pressure", 10,
                std::bind(&sUAV::alt_callback, this, _1));

        // [Invalid] Groundtruth Pose of Quadrotors
        nav_sub = this->create_subscription<geometry_msgs::msg::Pose>(
                "/model/quadrotor_" + std::to_string(sUAV_id) + "/world_pose", 10,
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
        
        // [Valid] Detecting Results of Others
        for (int i = 1; i <= UAV_NUM; i++){
            if (i == sUAV_id) {
                det_pub = this->create_publisher<std_msgs::msg::Int16>(
                    "/quadrotor_" + std::to_string(sUAV_id) + "/det_res", 10
                );
                continue;
            }
            auto fnc = [this](int i_){
                return [i_, this](const std_msgs::msg::Int16 & msg) -> void{
                    this->det_res[i_] = msg.data;
                };
            };
            det_sub[i] = this->create_subscription<std_msgs::msg::Int16>(
                "/quadrotor_" + std::to_string(i) + "/det_res", 10,
                fnc(i)
            );
        }


        // [Valid] Result of Detecting
		det_box_sub = this->create_subscription<target_bbox_msgs::msg::BoundingBoxes>(
    	"/quadrotor_" + std::to_string(sUAV_id) + "/targets/bboxs", 10, std::bind(&sUAV::det_callback, this, _1));

        // [Valid] Publish Quadrotor Velocity Command
        vel_cmd_pub = this->create_publisher<geometry_msgs::msg::Twist>(
                "/quadrotor_" + std::to_string(sUAV_id) + "/cmd_vel", 10);


        timer_ = this->create_wall_timer(50ms, std::bind(&sUAV::timer_callback, this));
        sat_vel.x = 5;
        sat_vel.y = 5;
        sat_vel.z = 2;
        sat_yaw_rate = 30 * DEG2RAD;
        loop = 1;
        double search_single_width = 50, search_signle_depth = 400;
        double search_forward_y = (std::abs (sUAV_id - 5.5) - 0.25) * search_single_width  * ((sUAV_id > 5) * 2 - 1);
        double search_backward_y = (std::abs (sUAV_id - 5.5) + 0.25) * search_single_width * ((sUAV_id > 5) * 2 - 1);
        double search_backward_x = -1400;
        double search_forward_x = search_backward_x + search_signle_depth;
        double search_height = 50;
        for (int i = 1; i <= loop; i++){
            search_tra.push_back(MyDataFun::new_point(search_backward_x, search_forward_y, search_height));
            search_tra.push_back(MyDataFun::new_point(search_forward_x, search_forward_y, search_height));
            search_tra.push_back(MyDataFun::new_point(search_forward_x, search_backward_y, search_height));
            search_tra.push_back(MyDataFun::new_point(search_backward_x, search_backward_y, search_height));
        }

        for (int i = 0; i < VESSEL_NUM; i++){
            double far_away[3] = {5000.0, 5000.0, 5000.0};
            MyDataFun::set_value(vsl_pos[i], far_away);
        }
        
        for (int i = 1; i <= UAV_NUM; i++){
            det_res[i] = 0;
        }

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

	void det_callback(const target_bbox_msgs::msg::BoundingBoxes & msg){
        for (int i = 0; i < int(msg.bounding_boxes.size()); i++){
            vis_vsl_pix[0] = msg.bounding_boxes[i].x_center;
            vis_vsl_pix[1] = msg.bounding_boxes[i].y_center;
			vis_vsl_flag = msg.bounding_boxes[i].flag;
			vis_vsl_id = msg.bounding_boxes[i].class_id;
            // printf("Got Target: %s\n", vis_vsl_id.c_str());
            vis_vsl_num = vis_vsl_id[vis_vsl_id.length() - 1] - 'a';
            MyMathFun::angle_transf(UAV_Euler, CAMERA_ANGLE * DEG2RAD, vis_vsl_pix, q_LOS_v);
            // printf("qlos: (%.2lf, %.2lf)\n", q_LOS_v[1] * RAD2DEG, q_LOS_v[2] * RAD2DEG);
            pos_err_v[2] = -UAV_pos.z;	
            pos_err_v[0] = (-pos_err_v[2]/tan(q_LOS_v[1]))*cos(q_LOS_v[2]);
            pos_err_v[1] = (-pos_err_v[2]/tan(q_LOS_v[1]))*sin(q_LOS_v[2]);
            // printf("Rel Pos of Target: (%.2lf, %.2lf, %.2lf)\n", pos_err_v[0], pos_err_v[1], pos_err_v[2]);
            vis_vsl_pos.x = UAV_pos.x + pos_err_v[0];
            vis_vsl_pos.y = UAV_pos.y + pos_err_v[1];
            vis_vsl_pos.z = UAV_pos.z + pos_err_v[2];
            if (!pos_valid(vis_vsl_pos)) continue;
            MyDataFun::set_value(vsl_pos[vis_vsl_num], vis_vsl_pos);
            vsl_pos_stat[vis_vsl_num].new_data(MyDataFun::dis(real_vsl_pos[vis_vsl_num], vsl_pos[vis_vsl_num]));
            
        }
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
        return a.x >= -1400.0;
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

    template<typename T1, typename T2>
    void UAV_Control_to_point_while_facing(T1 a, T2 b){
        printf("Control to Point (%.2lf, %.2lf, %.2lf) while facing (%.2lf, %.2lf, %.2lf)\n", a.x, a.y, a.z, b.x, b.y, b.z);
        geometry_msgs::msg::Twist cmd;
        MyDataFun::set_value(cmd.linear, MyDataFun::minus(a, UAV_pos));
        double des_yaw = atan2(b.y - UAV_pos.y, b.x - UAV_pos.x);
        printf("Now yaw is %.2lf while desired yaw is %.2lf\n", UAV_Euler[2] * RAD2DEG, des_yaw * RAD2DEG);
        cmd.angular.z = des_yaw - UAV_Euler[2];
        printf("UAV yaw cmd: %.2lf\n", cmd.angular.z);
        printf("UAV vel cmd in earth frame: %.6lf %.6lf %.6lf\n", cmd.linear.x, cmd.linear.y, cmd.linear.z);
        e2b(cmd.linear);
        printf("UAV vel cmd in body frame: %.6lf %.6lf %.6lf\n", cmd.linear.x, cmd.linear.y, cmd.linear.z);
        saturate_vel(cmd.linear);
        saturate_yaw_rate(cmd.angular.z);
        printf("Saturated UAV vel cmd in body frame: %.6lf %.6lf %.6lf\n", cmd.linear.x, cmd.linear.y, cmd.linear.z);
        printf("Saturated UAV yaw cmd in body frame: %.6lf\n", cmd.angular.z);
        vel_cmd_pub->publish(cmd);
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
        UAV_Control_earth(MyDataFun::minus(ctrl_cmd, UAV_pos), 0);
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
        MyDataFun::set_value(takeoff_point, UAV_pos);
        printf("Takeoff Point @ (%.2lf, %.2lf, %.2lf) !!!!!!!!!!!!!!!!!!!!!!!!!!!!\n", takeoff_point.x, takeoff_point.y, takeoff_point.z);
        rclcpp::Time time_now = this->get_clock()->now();
        task_begin_time = time_now.seconds();
        if (UAV_pos.x != 0.0 || UAV_pos.y != 0.0 || UAV_pos.z != 0.0){
            printf("Get Ground Truth Position!!!!!!!!!!!!!!!!!!!!!!\n");
            task_state = TAKEOFF;
        }
    }

    void StepTakeoff(){
        UAV_Control_to_Point_earth(search_tra[0]);
     	printf("Takeoff!!!\n");
        if (is_near(search_tra[0], 1)){
            printf("Takeoff Completed !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
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
                StepMapInit();
            }
        }
        for (int i = 0; i < VESSEL_NUM; i++){
            if (is_near_2d(vsl_pos[i], 3000)){
                printf("Got Vessel %d !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n", i);
                for (int j = 1; j <= UAV_NUM; j++){
                    if (j == sUAV_id) continue;
                    if ((det_res[j] >> i) & 1){
                        printf("But UAV %d has got it!!!!!!\n", j);
                        return;
                    }
                }
                det_res[sUAV_id] += 1 << i;
                vsl_id = i;
                search_tra_finish = 0;
                task_state = MAP;
                StepMapInit();
            }
        }
        
    }
	
    void StepMapInit(){
        map_init_theta = atan2(UAV_pos.y - vsl_pos[vsl_id].y, UAV_pos.x - vsl_pos[vsl_id].x);
        printf("theta_init = %.2lf !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n", map_init_theta * RAD2DEG);
    }

    void StepMap(){
     	printf("MAP!!!\n");
        // UAV_Control_to_point_while_facing(MyDataFun::plus(map_tra[map_tra_finish], vsl_pos[vsl_id]), vsl_pos[vsl_id]);
        UAV_Control_circle_while_facing(vis_vsl_pos);
        if (0){
            task_state = HOLD;
        }
    }
    
    void StepHold(){
        printf("Hold!!!\n");
        UAV_Control_earth(0, 0, 0, 0);
        if (task_time - hold_time >= hold_duration){
            printf("Hold %.2lf seconds completed !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n", hold_duration);
            task_state = LAND;
        }
    }

    void StepLand(){
        UAV_Control_to_Point_earth(takeoff_point);
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
        for (int i = 0; i < VESSEL_NUM; i++){
            printf("Vessel %c: %s by vision, %s by cheat, mean=%.4lf, std=%.4lf, rms=%.4lf cnt=%d\n", 'A' + i, MyDataFun::output_str(vsl_pos[i]).c_str(), MyDataFun::output_str(real_vsl_pos[i]).c_str(),
             vsl_pos_stat[i].mean, vsl_pos_stat[i].std, vsl_pos_stat[i].rms, vsl_pos_stat[i].cnt);
        }
        printf("Detection Status: %d\n", det_res[sUAV_id]);

        std_msgs::msg::Int16 tmp;
        tmp.data = det_res[sUAV_id];
        det_pub->publish(tmp); 
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
    rclcpp::Subscription<target_bbox_msgs::msg::BoundingBoxes>::SharedPtr det_box_sub;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr det_sub[UAV_NUM + 1];
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr det_pub;
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_cmd_pub;
};


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<sUAV>(argv[1]));
    rclcpp::shutdown();
    return 0;
}
