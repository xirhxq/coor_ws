#include <sstream>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>
#include <vector>
#include <fstream>
#include <ctime>
#include <queue>

#include "rclcpp/rclcpp.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point.hpp"
typedef geometry_msgs::msg::Point Point;
#include "geometry_msgs/msg/pose.hpp"
#include "std_msgs/msg/int16.hpp"
#include "target_bbox_msgs/msg/bounding_boxes.hpp"
#include "ros_ign_interfaces/msg/dataframe.hpp"


#include "MyMathFun.h"
#include "MyDataFun.h"

#define UAV_NUM 10
#define VESSEL_NUM 7
#define KP 0.2
#define X_KP KP
#define Y_KP KP
#define Z_KP KP
#define YAW_KP 1

using namespace geometry_msgs::msg;
using namespace std::chrono_literals;
using std::placeholders::_1;

int sUAV_id;

typedef enum TASK_CODE{
    INIT,
    TAKEOFF,
    PREPARE,
    SEARCH,
    MAP,
    HOLD,
    BACK,
    LAND
} STATE_CODE;

class sUAV : public rclcpp::Node {
public:
    // Log fstream
    std::ofstream log_file; 
    
    // Variable names in log file
    std::vector<std::string> name_vec; 
    
    // Sim time clock
    double clock;

    // [Invalid] Groundtruth position of sUAV itself
    Point UAV_pos;

    // Attitude of sUAV itself by imu
    Quaternion UAV_att_imu;
    
    // [Invalid] Groundtruth attitude of sUAV itself
    Quaternion UAV_att_pos;

    // [Invalid] Groundtruth velocity in world frame of sUAV itself
    Point UAV_vel;

    // Air pressure height of sUAV itself
    double air_pressure;

    // Euler angles of sUAV itself
    double UAV_Euler[3];

    // Transform matrix of sUAV itself
    double R_e2b[3][3];

    // Position of target vessel A-G by vision
    Point vsl_pos[VESSEL_NUM];

    // [Invalid] Groundtruth position of target vessel A-G
    Point real_vsl_pos[VESSEL_NUM];

    // [Invalid] Statistic values of target vessel position
    MyMathFun::DATA_STAT vsl_pos_stat[VESSEL_NUM];

    // [Valid] MidFilter of target vessel position
    MyMathFun::XYZ_Filter<Point> vsl_pos_fil[VESSEL_NUM];

    // Pixel x & y values of target
    double vis_vsl_pix[2];

    // Vision detection flag
    int vis_vsl_flag;
    
    // Target vessel number
    int vis_vsl_num;

    // Target vessel id by vision
    std::string vis_vsl_id;

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

    // [StepInit] Takeoff Command
    int16_t cmd;

    // [StepSearch] Preset Search Trajectory
    std::vector<Point> search_tra;

    // [StepSearch] Preset Loop #
    int loop;
    
    // [StepSearch] Trajectory points finished
    int search_tra_finish;

    // [StepSearch] Detecing Results of Other UAVs
    int det_res[UAV_NUM + 1];
    
    // [StepMap] Vessel ID to map
    int vsl_id;

    // [StepMap] Map Lateral velocity
    const double MAP_Y_VEL = 3;

    // Camera angle
    const double CAMERA_ANGLE = 30;

    // [StepMap] Map trajectory height
    const double MAP_TRA_HEIGHT = 30;

    // [StepMap] Map trajectory radius
    const double MAP_TRA_RADIUS = MAP_TRA_HEIGHT / tan(CAMERA_ANGLE * DEG2RAD);

    // [StepMap] Initial relative yaw
    double map_init_theta;

    // [StepMap] Initial time
    double map_init_time;

    sUAV(char *name) : Node("suav_" + std::string(name)) {
        sUAV_id = std::atoi(name);

        time_t tt = time(NULL);
        tm* t = localtime(&tt);
        char iden_path[256];
        sprintf(iden_path, "/home/ps/coor_ws/src/search/data/%02d-%02d_%02d-%02d_%02d.txt",
        t->tm_mon + 1, t->tm_mday, t->tm_hour, t->tm_min, sUAV_id);
        log_file.open(iden_path, std::ios::out);
        while(!log_file) std::cout << "Error: Could not write data!" << std::endl;
        
        name_vec = {"time", "day", "hour", "min", "sec", "control_state", 
                    "uav_pos_x", "uav_pos_y", "uav_pos_z",
                    "uav_roll", "uav_pitch", "uav_yaw",
                    "uav_vel_x", "uav_vel_y", "uav_vel_z"};

        for (int i = 0; i < VESSEL_NUM; i++){
            std::string s = "vessel_";
            s = s + char('a' + i) + "_error";
            name_vec.push_back(s);
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

        // [Valid] IMU Data: 1. Pose 2. Orientation
        imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
                "/suav_" + std::to_string(sUAV_id) + "/imu/data", 10,
                [this](const sensor_msgs::msg::Imu & msg){
                    MyDataFun::set_value_quaternion(this->UAV_att_imu, msg.orientation);
                    double q[4];
                    q[0] = msg.orientation.w;
                    q[1] = msg.orientation.x;
                    q[2] = msg.orientation.y;
                    q[3] = msg.orientation.z;
                    MyMathFun::quaternion_2_euler(q, this->UAV_Euler); 		// ENU
                    MyMathFun::Euler_2_Dcm(this->UAV_Euler, this->R_e2b);		// Rotation Matrix: ENU to Body
                }
        );

        // [Valid] Air Pressure Height
        alt_sub = this->create_subscription<sensor_msgs::msg::FluidPressure>(
                "/suav_" + std::to_string(sUAV_id) + "/air_pressure", 10,
                [this](const sensor_msgs::msg::FluidPressure & msg){
                    this->air_pressure = msg.fluid_pressure;
                }
        );

        // [Invalid] Groundtruth Pose of Quadrotors
        nav_sub = this->create_subscription<geometry_msgs::msg::Pose>(
                "/model/suav_" + std::to_string(sUAV_id) + "/world_pose", 10,
                [this](const geometry_msgs::msg::Pose & msg){
                    MyDataFun::set_value(this->UAV_pos, msg.position);
                    MyDataFun::set_value_quaternion(this->UAV_att_pos, msg.orientation);
                }
        );

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
                    "/suav_" + std::to_string(sUAV_id) + "/det_res", 10
                );
                continue;
            }
            det_sub[i] = this->create_subscription<std_msgs::msg::Int16>(
                "/suav_" + std::to_string(i) + "/det_res", 10,
                [i, this](const std_msgs::msg::Int16 & msg) -> void{
                    this->det_res[i] = msg.data;
                }
            );
        }


        // [Valid] Publish Quadrotor Velocity Command
        vel_cmd_pub = this->create_publisher<geometry_msgs::msg::Twist>(
                "/suav_" + std::to_string(sUAV_id) + "/cmd_vel", 10);

        // [Valid] Received data
        com_sub = this->create_subscription<ros_ign_interfaces::msg::Dataframe>(
            "/buav_" + std::to_string(sUAV_id) + "/rx", 10, 
            [this](const ros_ign_interfaces::msg::Dataframe & msg){
                // printf("I heard msg with strength %lf\n", msg.rssi);
            }
        );

        // [Valid] Published data
        com_pub = this->create_publisher<ros_ign_interfaces::msg::Dataframe>(
            "/buav_" + std::to_string(sUAV_id) + "/tx", 10
        );

        // [Valid] Commander 
        cmd_sub = this->create_subscription<std_msgs::msg::Int16>(
            "/commander_cmd", 10, 
            [this](const std_msgs::msg::Int16 & msg){
                this->cmd = msg.data;
            }
        );
        

        timer_ = this->create_wall_timer(50ms, std::bind(&sUAV::timer_callback, this));
        sat_vel.x = 5;
        sat_vel.y = 5;
        sat_vel.z = 5;
        sat_yaw_rate = 90 * DEG2RAD;
        loop = 1;
        double search_single_width = 70, search_signle_depth = 1000;
        double search_forward_y = (std::abs (sUAV_id - 5.5) - 0.25) * search_single_width  * ((sUAV_id > 5) * 2 - 1);
        double search_backward_y = (std::abs (sUAV_id - 5.5) + 0.25) * search_single_width * ((sUAV_id > 5) * 2 - 1);
        double search_backward_x = -1500;
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

	void det_callback(const target_bbox_msgs::msg::BoundingBoxes & msg){
        for (int i = 0; i < int(msg.bounding_boxes.size()); i++){
            vis_vsl_pix[0] = msg.bounding_boxes[i].x_center;
            vis_vsl_pix[1] = msg.bounding_boxes[i].y_center;
			vis_vsl_flag = msg.bounding_boxes[i].flag;
			vis_vsl_id = msg.bounding_boxes[i].class_id;
            // printf("Got Target: %s\n", vis_vsl_id.c_str());
            if (vis_vsl_id.length() != 8) continue;
            vis_vsl_num = vis_vsl_id[vis_vsl_id.length() - 1] - 'a';
            if (vis_vsl_num < 0 || vis_vsl_num > 6) continue;
            // printf("visvslnum = %d\n", vis_vsl_num);
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
            // filter_mid(vis_vsl_pos);
            vsl_pos_fil[vis_vsl_num].new_data(vis_vsl_pos);
            vis_vsl_pos = vsl_pos_fil[vis_vsl_num].result();
            MyDataFun::set_value(vsl_pos[vis_vsl_num], vis_vsl_pos);
            vsl_pos_stat[vis_vsl_num].new_data(MyDataFun::dis(real_vsl_pos[vis_vsl_num], vsl_pos[vis_vsl_num]));
            
        }
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
        return a.x >= -1500.0 && a.x <= 1500.0 && a.y >= -1500.0 && a.y <= 1500.0;
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
    void UAV_Control_circle_while_facing(T a){
        printf("Control in circle while facing (%.2lf, %.2lf, %.2lf)\n", a.x, a.y, a.z);
        geometry_msgs::msg::Twist cmd;
        double dis2vsl = sqrt(pow(a.y - UAV_pos.y, 2) + pow(a.x - UAV_pos.x, 2));
        printf("Distance 2 Vessel: %.2lf\n", dis2vsl);
        cmd.linear.x = (dis2vsl - MAP_TRA_RADIUS) * X_KP;
        cmd.linear.y = MAP_Y_VEL;
        cmd.linear.z = (dis2vsl * tan(CAMERA_ANGLE * DEG2RAD) - UAV_pos.z) * Z_KP;
        double des_yaw = MyDataFun::angle_2d(UAV_pos, a);
        printf("Now yaw is %.2lf while desired yaw is %.2lf\n", UAV_Euler[2] * RAD2DEG, des_yaw * RAD2DEG);
        cmd.angular.z = YAW_KP * (des_yaw - UAV_Euler[2]);
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

    template<typename T1, typename T2>
    void UAV_Control_to_Point_with_facing(T1 ctrl_cmd, T2 p){
        double yaw_diff = MyDataFun::angle_2d(UAV_pos, p) - UAV_Euler[2];
        UAV_Control_earth(MyDataFun::minus(ctrl_cmd, UAV_pos), yaw_diff);
    }

    // template<typename T1, typename T2>
    // void Relay_Common(){
    //     ros_ign_interfaces::msg::Dataframe Data_Send;
    //     ros_ign_interfaces::msg::Dataframe Data_Reciv;
        
    //     Data_send.src_address="/suav_" + std::to_string(sUAV_id)
    //     Data_send.src_address="/suav_" + std::to_string(sUAV_id)
    //     Data_send.data.push_back(233);
    // }

    void StepInit(){
        printf("Takeoff Point @ (%.2lf, %.2lf, %.2lf) !!!!!!!!!!!!!!!!!!!!!!!!!!!!\n", takeoff_point.x, takeoff_point.y, takeoff_point.z);
        MyDataFun::set_value(birth_point, UAV_pos);
        MyDataFun::set_value(takeoff_point, UAV_pos);
        takeoff_point.z += 10 + sUAV_id * 2;
        search_tra[0].z = takeoff_point.z;
        if (cmd != 233){
            return;
        }
        UAV_Control_earth(0, 0, 0, 0);
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

            // [Valid] Result of Detecting
            det_box_sub = this->create_subscription<target_bbox_msgs::msg::BoundingBoxes>(
            "/suav_" + std::to_string(sUAV_id) + "/slot0/targets/bboxs", 10, std::bind(&sUAV::det_callback, this, _1));
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
                task_state = BACK;
                // StepMapInit();
            }
        }
        for (int i = 0; i < VESSEL_NUM; i++){
            if (is_near_2d(vsl_pos[i], 3000) && vsl_pos_stat[i].cnt >= 50){
                printf("Got Vessel %c !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n", 'A' + i);
                bool other_flag = false;
                for (int j = 1; j <= UAV_NUM; j++){
                    if (j == sUAV_id) continue;
                    if ((det_res[j] >> i) & 1){
                        printf("But UAV %d has got it!!!!!!\n", j);
                        other_flag = true;
                    }
                }
                if (other_flag) continue;
                det_res[sUAV_id] += 1 << i;
                vsl_id = i;
                search_tra_finish = 0;
                task_state = MAP;
                StepMapInit();
            }
        }
        
    }
	
    void StepMapInit(){
        map_init_theta = MyDataFun::angle_2d(vsl_pos[vsl_id], UAV_pos);
        printf("theta_init = %.2lf !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n", map_init_theta * RAD2DEG);
        map_init_time = get_time_now();
    }

    void StepMap(){
     	printf("MAP around Vessel %c!!!\n", 'A' + vsl_id);

        double now_theta = MyDataFun::angle_2d(vsl_pos[vsl_id], UAV_pos);
        double map_theta = now_theta + 20 / MAP_TRA_RADIUS;

        // double map_ang_vel = 1 / MAP_TRA_RADIUS;
        // double map_time = get_time_now() - map_init_time;
        // double map_theta = map_time * map_ang_vel + map_init_theta;
        Point map_point;

        double dis2vsl = sqrt(pow(vsl_pos[vsl_id].y - UAV_pos.y, 2) + pow(vsl_pos[vsl_id].x - UAV_pos.x, 2));

        map_point.x = vsl_pos[vsl_id].x + MAP_TRA_RADIUS * cos(map_theta);
        map_point.y = vsl_pos[vsl_id].y + MAP_TRA_RADIUS * sin(map_theta);
        map_point.z = std::max(UAV_pos.z, dis2vsl * tan(CAMERA_ANGLE * DEG2RAD));
        printf("Next Point: (%.2lf, %.2lf, %.2lf)\n", map_point.x, map_point.y, map_point.z);
        UAV_Control_to_Point_with_facing(map_point, vsl_pos[vsl_id]);
        // UAV_Control_circle_while_facing(vsl_pos[vsl_id]);
        if (0){
            task_state = HOLD;
            hold_time = get_time_now();
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

    void log_once(){
        time_t tt = time(NULL);
        tm* t = localtime(&tt);
        log_file.precision(7);
        log_file << get_time_now() - task_begin_time << "\t"
                 << t->tm_mday << "\t"
                 << t->tm_hour << "\t"
                 << t->tm_min << "\t"
                 << t->tm_sec << "\t"
                 << task_state << "\t"
                 << UAV_pos.x << "\t"
                 << UAV_pos.y << "\t"
                 << UAV_pos.z << "\t"
                 << UAV_Euler[0] << "\t"
                 << UAV_Euler[1] << "\t"
                 << UAV_Euler[2] << "\t"
                 << UAV_vel.x << "\t"
                 << UAV_vel.y << "\t"
                 << UAV_vel.z << "\t";
        for (int i = 0; i < VESSEL_NUM; i++){
            log_file << MyDataFun::dis(real_vsl_pos[i], vsl_pos[i]) << "\t";
        }
        log_file << std::endl;
    }

    void timer_callback() {
        update_time();
        printf("---------------------------------\n-----------A New Frame-----------\n---------------------------------\n");
        printf("Time: %.2lf\n", task_time);
        printf("Me @ (%.2lf, %.2lf, %.2lf)\n", UAV_pos.x, UAV_pos.y, UAV_pos.z);
        vsl_pos_fil[0].output();
        // printf("Quaternion by imu: (%.2lf, %.2lf, %.2lf, %.2lf)\n", UAV_att_imu.w, UAV_att_imu.x, UAV_att_imu.y, UAV_att_imu.z);
        // printf("Quaternion by pos: (%.2lf, %.2lf, %.2lf, %.2lf)\n", UAV_att_pos.w, UAV_att_pos.x, UAV_att_pos.y, UAV_att_pos.z);
        // printf("Euler angle: (Phi %.2lf, Theta %.2lf, Psi %.2lf)\n", UAV_Euler[0] * RAD2DEG, UAV_Euler[1] * RAD2DEG, UAV_Euler[2] * RAD2DEG);
        // printf("Transform Matrix: ------\n");
        // for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) printf("%.2lf%c", R_e2b[i][j], (j==2)?'\n':'\t');
        // for (int i = 0; i < VESSEL_NUM; i++){
        //     printf("Vessel %c: %s by vision, %s by cheat, mean=%.4lf, std=%.4lf, rms=%.4lf cnt=%d\n", 'A' + i, MyDataFun::output_str(vsl_pos[i]).c_str(), MyDataFun::output_str(real_vsl_pos[i]).c_str(),
        //      vsl_pos_stat[i].mean, vsl_pos_stat[i].std, vsl_pos_stat[i].rms, vsl_pos_stat[i].cnt);
        // }
        printf("Detection Status: ");
        for (int i = 0; i < VESSEL_NUM; i++){
            if (det_res[sUAV_id] >> i & 1) printf("%c", 'A' + i);
        }
        printf("\n");

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
            case PREPARE:{
                StepPrepare();
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
        log_once();
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_sub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
    rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr alt_sub;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr nav_sub, vsl_sub[VESSEL_NUM];
    rclcpp::Subscription<target_bbox_msgs::msg::BoundingBoxes>::SharedPtr det_box_sub;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr det_sub[UAV_NUM + 1];
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr det_pub;
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_cmd_pub;
    rclcpp::Subscription<ros_ign_interfaces::msg::Dataframe>::SharedPtr com_sub;
    rclcpp::Publisher<ros_ign_interfaces::msg::Dataframe>::SharedPtr com_pub;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr cmd_sub;
};


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<sUAV>(argv[1]));
    rclcpp::shutdown();
    return 0;
}
