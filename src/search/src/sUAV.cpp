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
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point.hpp"
typedef geometry_msgs::msg::Point Point;
#include "geometry_msgs/msg/pose.hpp"
#include "std_msgs/msg/int16.hpp"
#include "target_bbox_msgs/msg/bounding_boxes.hpp"
#include "ros_ign_interfaces/msg/dataframe.hpp"
#include "ros_ign_interfaces/msg/string_vec.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"


#include "MyMathFun.h"
#include "MyDataFun.h"

#define UAV_NUM 10
#define VESSEL_NUM 7
#define KP 0.2
#define X_KP KP
#define Y_KP KP
#define Z_KP KP
#define YAW_KP 1

#define TARGET_VESSEL 'c'
#define COMM_RANGE 800

#define DOUBLE_ENCODE_SIZE 4
#define VSL_DET_ENCODE_SIZE (3 * DOUBLE_ENCODE_SIZE + 1)

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
    BRIDGE,
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

    // Run clock
    double run_clock;

    // Score
    double score;

    // Competition phase
    std::string phase;

    // Detection status
    std::string status;

    // [Invalid] Groundtruth position of sUAV itself
    Point UAV_pos;

    // Attitude of sUAV itself by imu
    Quaternion UAV_att_imu;
    
    // [Invalid] Groundtruth attitude of sUAV itself
    Quaternion UAV_att_pos;

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

    // [StepSearch] Search Time
    double search_time;

    // [StepSearch] Preset Search Trajectory
    std::vector<Point> search_tra;

    // [StepSearch] Preset Loop #
    int loop;
    
    // [StepSearch] Trajectory points finished
    int search_tra_finish;

    // [StepSearch] Detecing Results of Other UAVs
    int det_res[UAV_NUM + 1];

    // [StepSearch] Detection success counter
    int det_cnt[VESSEL_NUM];

    // [StepSearch] Vessel position result altogether
    Point vsl_det_pos[VESSEL_NUM];
    
    // [StepMap] Vessel ID to map
    int vsl_id;

    // [StepMap] Map Lateral velocity
    const double MAP_Y_VEL = 3;

    // Camera angle
    const double CAMERA_ANGLE = 30;

    // [StepMap] Map vessel id list
    int map_res;

    // [StepMap] Map trajectory height
    const double MAP_TRA_HEIGHT = 30;

    // [StepMap] Map trajectory radius
    const double MAP_TRA_RADIUS = MAP_TRA_HEIGHT / tan(CAMERA_ANGLE * DEG2RAD);

    // [StepMap] Initial relative yaw
    double map_init_theta;

    // [StepMap] Initial time
    double map_init_time;

    // [StepMap] Half loop flag
    bool half_loop_flag;

    // [StepBridge] Bridge point
    Point bridge_point;

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
                    "uav_roll", "uav_pitch", "uav_yaw"};

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

        // [Valid] Publish Quadrotor Velocity Command
        vel_cmd_pub = this->create_publisher<geometry_msgs::msg::Twist>(
                "/suav_" + std::to_string(sUAV_id) + "/cmd_vel", 10);

        // [Valid] Received data
        com_sub = this->create_subscription<ros_ign_interfaces::msg::Dataframe>(
            "/suav_" + std::to_string(sUAV_id) + "/rx", 10, 
            [this](const ros_ign_interfaces::msg::Dataframe & msg){
                assert(msg.data.size() == VSL_DET_ENCODE_SIZE * VESSEL_NUM);
                // printf("msg time: %.2lf\n", msg.header.stamp.sec + 1.0 * msg.header.stamp.nanosec / 1e9);
                for (int i = 0; i < VESSEL_NUM; i++){
                    if (msg.data[i * VSL_DET_ENCODE_SIZE] != 0 && msg.data[i * VSL_DET_ENCODE_SIZE] != sUAV_id){
                        int other_id = msg.data[i * VSL_DET_ENCODE_SIZE];
                        new_det_res(other_id, i);
                        double tmp[3];
                        for (int j = 0; j < 3; j++){
                            tmp[j] = 1.0 * double(msg.data[i * VSL_DET_ENCODE_SIZE + j * DOUBLE_ENCODE_SIZE + 2] << 16) 
                                    + double(msg.data[i * VSL_DET_ENCODE_SIZE + j * DOUBLE_ENCODE_SIZE + 3] << 8) 
                                    + double(msg.data[i * VSL_DET_ENCODE_SIZE + j * DOUBLE_ENCODE_SIZE + 4]);
                            tmp[j] /= 100.0;
                            if (msg.data[i * VSL_DET_ENCODE_SIZE + j * DOUBLE_ENCODE_SIZE + 1] == 1) tmp[j] = -tmp[j]; 
                        }
                        MyDataFun::set_value(vsl_det_pos[i], tmp);
                    }
                }
                // printf("I heard msg with strength %lf\n", msg.rssi);
            }
        );

        // [Valid] Published data
        com_pub = this->create_publisher<ros_ign_interfaces::msg::Dataframe>(
            "/suav_" + std::to_string(sUAV_id) + "/tx", 10
        );

        // [Invalid] Commander 
        cmd_sub = this->create_subscription<std_msgs::msg::Int16>(
            "/commander_cmd", 10, 
            [this](const std_msgs::msg::Int16 & msg){
                this->cmd = msg.data;
            }
        );

        // [Valid] Video report
        report_stream_pub = this->create_publisher<sensor_msgs::msg::Image>(
            "/suav_" + std::to_string(sUAV_id) + "/mbzirc/target/stream/start", 10
        );

        // [Valid] Target report
        report_target_pub = this->create_publisher<ros_ign_interfaces::msg::StringVec>(
            "/suav_" + std::to_string(sUAV_id) + "/mbzirc/target/stream/report", 10
        );

        // [Valid] Received status
        report_status_sub = this->create_subscription<std_msgs::msg::String>(
            "/mbzirc/target/stream/status", 10,
            [this](const std_msgs::msg::String & msg){
                this->status = msg.data;
                if (msg.data == "vessel_id_success"){
                    this->vsl_det_cmd_pub->publish(msg);
                }
            }
        );
        
        // [Valid] Run clock
        run_clock_sub = this->create_subscription<rosgraph_msgs::msg::Clock>(
            "/mbzirc/run_clock", 10,
            [this](const rosgraph_msgs::msg::Clock & msg){
                this->run_clock = double(msg.clock.sec) + double(msg.clock.nanosec) / 1e9;
            }
        );

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

        // [Valid] Command for vessel detection
        vsl_det_cmd_pub = this->create_publisher<std_msgs::msg::String> (
            "/suav_" + std::to_string(sUAV_id) + "/vessel_det_status", 10
        );

        timer_ = this->create_wall_timer(50ms, std::bind(&sUAV::timer_callback, this));
        sat_vel.x = 5;
        sat_vel.y = 5;
        sat_vel.z = 5;
        sat_yaw_rate = 90 * DEG2RAD;
        loop = 1;
        double side_length = 3062.28;
        double search_single_width = side_length / 10, search_signle_depth = side_length;
        // double search_forward_y = (std::abs (sUAV_id - 5.5) - 0.25) * search_single_width  * ((sUAV_id > 5) * 2 - 1);
        // double search_backward_y = (std::abs (sUAV_id - 5.5) + 0.25) * search_single_width * ((sUAV_id > 5) * 2 - 1);
        double search_backward_x = -side_length / 2.0;
        double search_forward_x = search_backward_x + search_signle_depth;
        double search_height = 50;
        // for (int i = 1; i <= loop; i++){
        //     search_tra.push_back(MyDataFun::new_point(search_backward_x, search_forward_y, search_height));
        //     search_tra.push_back(MyDataFun::new_point(search_forward_x, search_forward_y, search_height));
        //     search_tra.push_back(MyDataFun::new_point(search_forward_x, search_backward_y, search_height));
        //     search_tra.push_back(MyDataFun::new_point(search_backward_x, search_backward_y, search_height));
        // }
        double search_y = search_single_width * (sUAV_id - 5.5);
        search_tra.push_back(MyDataFun::new_point(search_backward_x, search_y, search_height));
        for (int i = 1; i <= loop; i++){
            MyDataFun::put_discrete_points(search_tra, MyDataFun::new_point(search_forward_x, search_y, search_height), 10);
            // search_tra.push_back(MyDataFun::new_point(search_forward_x, search_y, search_height));
            MyDataFun::put_discrete_points(search_tra, MyDataFun::new_point(search_backward_x, search_y, search_height), 10);
            // search_tra.push_back(MyDataFun::new_point(search_backward_x, search_y, search_height));
        }
        // MyDataFun::output_vector(search_tra);
        map_res = 0;

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
            det_cnt[vis_vsl_num]++;
            vsl_pos_stat[vis_vsl_num].new_data(MyDataFun::dis(real_vsl_pos[vis_vsl_num], vsl_pos[vis_vsl_num]));
            // if (vis_vsl_num == 1){
            //     ros_ign_interfaces::msg::StringVec data;
            //     data.data.push_back("vessel");
            //     data.data.push_back(std::to_string(int(msg.bounding_boxes[i].x_center)));
            //     data.data.push_back(std::to_string(int(msg.bounding_boxes[i].y_center)));
            //     report_target_pub->publish(data);
            // }
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
        return a.x >= -1470.0 && a.x <= 1500.0 && a.y >= -1500.0 && a.y <= 1500.0;
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

    bool has_det(int id, int vsl_id){
        return (det_res[id] >> vsl_id) & 1;
    }

    void new_det_res(int id, int vsl_id){
        if (!has_det(id, vsl_id)){
            det_res[id] += (1 << vsl_id);
        }
    }

    int tgt_vsl_det_id(){
        int vsl_id = TARGET_VESSEL - 'a';
        for (int i = 1; i <= UAV_NUM; i++){
            if (has_det(i, vsl_id)) return i;
        }
        return 0;
    }

    template<typename T>
    int bridge_num(T target_pos){
        return std::ceil(MyDataFun::dis(target_pos, MyDataFun::new_point(-1500, 0, 0)) / COMM_RANGE) - 1;
    }

    bool if_i_am_bridge(int id){
        int det_id = tgt_vsl_det_id();
        if (det_id == 0) return false;
        int bdg_num = bridge_num(vsl_det_pos[TARGET_VESSEL - 'a']);
        if (bdg_num == 0) return false;
        else if (bdg_num == 1){
            if (det_id <= 5) return id == det_id + 1;
            else return id == det_id - 1;
        }
        else if (bdg_num == 2){
            if (det_id == 1) return id > det_id && id <= det_id + 2;
            else if (det_id == 10) return id < det_id && id >= det_id - 2;
            else return id == det_id - 1 || id == det_id + 1;
        }
        else if (bdg_num == 3){
            if (det_id == 1) return id > det_id && id <= det_id + 3;
            else if (det_id == 10) return id < det_id && id >= det_id - 3;
            else if (det_id >= 2 && det_id <= 5) return id >= det_id - 1 && id <= det_id + 2 && id != det_id;
            else return id <= det_id + 1 && id >= det_id - 2 && id != det_id;
        }
        else if (bdg_num == 4){
            if (det_id <= 3) return id <= 5 && id != det_id;
            else if (det_id >= 8) return id >= 6 && id != det_id;
            else if (det_id >= 4 && det_id <= 5) return id >= 3 && id <= 7 && id != det_id;
            else return id >= 4 && id <= 8 && id != det_id;
        }
        else {
            printf("Invalid bridge num!\n");
        }
        return false;
    }

    int which_bridge(int id){
        std::vector<int> bdg;
        for (int i = 1; i <= UAV_NUM; i++){
            if (if_i_am_bridge(i)){
                bdg.push_back(i);
            }
        }
        std::sort(bdg.begin(), bdg.end(), [](int a, int b){return abs(1.0 * a - 5.5) > abs(1.0 * b - 5.5);});
        auto it = std::find(bdg.begin(), bdg.end(), id);
        return std::distance(bdg.begin(), it) + 1;
    }

    template<typename T>
    T bridge_pos(int id){
        if (!if_i_am_bridge(id)) return MyDataFun::new_point(0, 0, 0);
        Point distance = MyDataFun::minus(vsl_det_pos[TARGET_VESSEL - 'a'], MyDataFun::new_point(-1500, 0, 0));
        distance =  MyDataFun::scale(distance, 1.0 * which_bridge(id) / (bridge_num(vsl_det_pos[TARGET_VESSEL - 'a']) + 1));
        distance = MyDataFun::plus(distance, MyDataFun::new_point(-1500, 0, 0));
        distance.z = 10 + 2 * sUAV_id;
        return distance;
    }

    template<typename T>
    void UAV_Control_earth(T ctrl_cmd, double yaw_rate){
        geometry_msgs::msg::Twist cmd;
        cmd.angular.z = yaw_rate * YAW_KP;
        MyDataFun::set_value(cmd.linear, ctrl_cmd);
        cmd.linear.x *= X_KP;
        cmd.linear.y *= Y_KP;
        cmd.linear.z *= Z_KP;
        printf("Vel cmd (earth): %.2lf %.2lf %.2lf\n", cmd.linear.x, cmd.linear.y, cmd.linear.z);
        saturate_vel(cmd.linear);
        printf("Sat vel cmd (earth): %.2lf %.2lf %.2lf\n", cmd.linear.x, cmd.linear.y, cmd.linear.z);
        e2b(cmd.linear);
        printf("Sat vel cmd (body): %.2lf %.2lf %.2lf\n", cmd.linear.x, cmd.linear.y, cmd.linear.z);
        saturate_yaw_rate(cmd.angular.z);
        printf("Sat yaw cmd (body): %.2lf\n", cmd.angular.z);
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
        printf("To Point (%.2lf, %.2lf, %.2lf)\n", ctrl_cmd.x, ctrl_cmd.y, ctrl_cmd.z);
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

    void StepInit(){
        printf("Takeoff Point @ (%.2lf, %.2lf, %.2lf)\n", takeoff_point.x, takeoff_point.y, takeoff_point.z);
        MyDataFun::set_value(birth_point, UAV_pos);
        MyDataFun::set_value(takeoff_point, UAV_pos);
        birth_point.z -= 10;
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
     	printf("Takeoff!!!\n");
        UAV_Control_to_Point_earth(takeoff_point);
        if (is_near(takeoff_point, 1)){
            task_state = PREPARE;
        }
    }

    void StepPrepare(){
     	printf("Prepare!!!\n");
        UAV_Control_to_Point_earth(search_tra[0]);
        if (is_near(search_tra[0], 1)){
            task_state = SEARCH;
            search_tra_finish = 0;
            search_time = get_time_now();

            // [Valid] Result of Detecting
            det_box_sub = this->create_subscription<target_bbox_msgs::msg::BoundingBoxes>(
            "/suav_" + std::to_string(sUAV_id) + "/slot0/targets/bboxs", 10, std::bind(&sUAV::det_callback, this, _1));
            memset(det_cnt, 0, sizeof(det_cnt));

            std_msgs::msg::String data;
            data.data = "vessel_det";
            vsl_det_cmd_pub->publish(data);
        }
    }

    void StepSearch(){
     	printf("Search!!!\n");
        printf("To Point %s(%d/%ld)\n", MyDataFun::output_str(search_tra[search_tra_finish]).c_str(), search_tra_finish, search_tra.size());
        double tra_yaw = MyDataFun::angle_2d(UAV_pos, search_tra[search_tra_finish]);
        double shaking_angle = std::sin((get_time_now() - search_time) / 8) * PI / 2;
        double yaw_diff = tra_yaw + shaking_angle - UAV_Euler[2];
        printf("Desired angle %.2lf while now %.2lf\n", tra_yaw + shaking_angle, UAV_Euler[2]);
        if (MyDataFun::dis_2d(search_tra[search_tra_finish], UAV_pos) <= 5) yaw_diff = 0;
        UAV_Control_earth(MyDataFun::minus(search_tra[search_tra_finish], UAV_pos), yaw_diff);
        if (is_near(search_tra[search_tra_finish], 1)){
            search_tra_finish++;
            if (search_tra_finish == int(search_tra.size())){
                search_tra_finish = 0;
                task_state = BACK;
                // StepMapInit();
            }
        }    
        bool finish_flag = true;
        for (int i = 0; i < VESSEL_NUM; i++){
            bool tmp_flag = false;
            for (int j = 1; j <= UAV_NUM; j++){
                if (has_det(j, i)){
                    tmp_flag = true;
                }
            }
            if (!tmp_flag) finish_flag = false;
        }
        if (finish_flag){
            task_state = BACK;
        }
        for (int i = 0; i < VESSEL_NUM; i++){
            if (is_near_2d(vsl_pos[i], 3000) && det_cnt[i] >= 50){
                printf("Got Vessel %c!\n", 'A' + i);
                bool other_flag = false;
                for (int j = 1; j <= UAV_NUM; j++){
                    if (j == sUAV_id) continue;
                    if (has_det(j, i)){
                        printf("But UAV %d has got it!\n", j);
                        other_flag = true;
                    }
                }
                if (other_flag) continue;
                new_det_res(sUAV_id, i);
                if ((map_res >> i) & 1){
                    printf("Already Mapped Vessel %c!\n", 'A' + i);
                    continue;
                }
                vsl_id = i;
                // search_tra_finish = 0;
                task_state = MAP;
                StepMapInit();
            }
        }
    }
	
    void StepMapInit(){
        map_init_theta = MyDataFun::angle_2d(vsl_pos[vsl_id], UAV_pos);
        map_init_time = get_time_now();
        if (!((map_res >> vsl_id) & 1)) {
            map_res += (1 << vsl_id);
        }
        half_loop_flag = false;
    }

    void StepMap(){
     	printf("MAP around Vessel %c!!!\n", 'A' + vsl_id);

        double now_theta = MyDataFun::angle_2d(vsl_pos[vsl_id], UAV_pos);
        printf("Initial theta: %.2lf\n", map_init_theta * RAD2DEG);
        printf("Now theta: %.2lf\n", now_theta * RAD2DEG);
        printf("Half loop: %c\n", half_loop_flag?'Y':'N');

        if (abs(now_theta - map_init_theta + PI) <= 5 * DEG2RAD
        || abs(now_theta - map_init_theta + 3 * PI) <= 5 * DEG2RAD
        || abs(now_theta - map_init_theta - PI) <= 5 * DEG2RAD)
            half_loop_flag = true;
        double map_theta = now_theta + 20 / MAP_TRA_RADIUS;

        Point map_point;

        double dis2vsl = sqrt(pow(vsl_pos[vsl_id].y - UAV_pos.y, 2) + pow(vsl_pos[vsl_id].x - UAV_pos.x, 2));

        map_point.x = vsl_pos[vsl_id].x + MAP_TRA_RADIUS * cos(map_theta);
        map_point.y = vsl_pos[vsl_id].y + MAP_TRA_RADIUS * sin(map_theta);
        map_point.z = std::min(UAV_pos.z, dis2vsl * tan(CAMERA_ANGLE * DEG2RAD));
        printf("Next Point: (%.2lf, %.2lf, %.2lf)\n", map_point.x, map_point.y, map_point.z);
        UAV_Control_to_Point_with_facing(map_point, vsl_pos[vsl_id]);
        // if (abs(now_theta - map_init_theta) <= 5 * DEG2RAD && half_loop_flag){
        //     task_state = SEARCH;
        //     half_loop_flag = false;
        // }

        if (cmd == 0){
            task_state = SEARCH;
        }
        if (0){
            task_state = HOLD;
            hold_time = get_time_now();
        }
    }

    void StepBridge(){
        UAV_Control_to_Point_earth(bridge_point);
        printf("Target Vessel @ %s\n", MyDataFun::output_str(vsl_det_pos[TARGET_VESSEL - 'a']).c_str());
        printf("Bridge @ %s (%d of %d)\n", MyDataFun::output_str(bridge_point).c_str(), which_bridge(sUAV_id), bridge_num(vsl_det_pos[TARGET_VESSEL - 'a']));
        if (cmd == 0){
            task_state = SEARCH;
        }
    }
    
    void StepHold(){
        printf("Holding: %.2lf!!!\n", task_time - hold_time);
        UAV_Control_earth(0, 0, 0, 0);
        if (task_time - hold_time >= hold_duration){
            task_state = BACK;
        }
    }

    void StepBack(){
        UAV_Control_to_Point_earth(takeoff_point);
     	printf("Back!!!\n");
        if (is_near(takeoff_point, 1)){
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
                 << UAV_Euler[2] << "\t";
        for (int i = 0; i < VESSEL_NUM; i++){
            log_file << MyDataFun::dis(real_vsl_pos[i], vsl_pos[i]) << "\t";
        }
        log_file << std::endl;
    }

    void timer_callback() {
        update_time();
        std::cout << "\033c" << std::flush;
        printf("Time: %.2lf(%.2lf)\n", task_time, run_clock);
        printf("Phase: %s\nStatus: %s\nScore: %.2lf\n", phase.c_str(), status.c_str(), score);
        printf("sUAV #%d @ %s\n", sUAV_id, MyDataFun::output_str(UAV_pos).c_str());
        printf("Commander cmd: %d\n", cmd);
        // printf("Quaternion by imu: (%.2lf, %.2lf, %.2lf, %.2lf)\n", UAV_att_imu.w, UAV_att_imu.x, UAV_att_imu.y, UAV_att_imu.z);
        // printf("Quaternion by pos: (%.2lf, %.2lf, %.2lf, %.2lf)\n", UAV_att_pos.w, UAV_att_pos.x, UAV_att_pos.y, UAV_att_pos.z);
        printf("Phi %.2lf, Theta %.2lf, Psi %.2lf\n", UAV_Euler[0] * RAD2DEG, UAV_Euler[1] * RAD2DEG, UAV_Euler[2] * RAD2DEG);
        // printf("Transform Matrix: ------\n");
        // for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) printf("%.2lf%c", R_e2b[i][j], (j==2)?'\n':'\t');
        // for (int i = 0; i < VESSEL_NUM; i++){
        //     printf("Vessel %c: %s by vision, %s by cheat, mean=%.4lf, std=%.4lf, rms=%.4lf cnt=%d\n", 'A' + i, MyDataFun::output_str(vsl_pos[i]).c_str(), MyDataFun::output_str(real_vsl_pos[i]).c_str(),
        //      vsl_pos_stat[i].mean, vsl_pos_stat[i].std, vsl_pos_stat[i].rms, vsl_pos_stat[i].cnt);
        // }
        printf("Detection Counter: ");
        for (int i = 0; i < VESSEL_NUM; i++){
            printf("%c:(%d) ", 'A' + i, det_cnt[i]);
        }
        printf("\nDetection Status: ");
        for (int i = 0; i < VESSEL_NUM; i++){
            if (has_det(sUAV_id, i)) printf("%c", 'A' + i);
        }
        printf("\nMap Status: ");
        for (int i = 0; i < VESSEL_NUM; i++){
            if (map_res >> i & 1) printf("%c", 'A' + i);
        }
        printf("\n");
        for (int i = 0; i < VESSEL_NUM; i++){
            printf("%s/", MyDataFun::output_str(vsl_det_pos[i]).c_str());
        }printf("\n");
        printf("det_res:");
        for (int i = 1; i <= UAV_NUM; i++){
            for (int j = 0; j < VESSEL_NUM; j++){
                if (has_det(i, j)) printf("%c", 'A' + j);
            }
            printf("/");
        }printf("\n");

        ros_ign_interfaces::msg::Dataframe com_pub_data;
        com_pub_data.data.resize(VESSEL_NUM * VSL_DET_ENCODE_SIZE);
        for (int i = 0; i < VESSEL_NUM; i++){
            for (int j = 1; j <= UAV_NUM; j++){
                if (has_det(j, i)){
                    if (j == sUAV_id) {
                        MyDataFun::set_value(vsl_det_pos[i], vsl_pos[i]);
                    }
                    com_pub_data.data[i * VSL_DET_ENCODE_SIZE] = j;
                    double tmp[3] = {vsl_det_pos[i].x * 100, vsl_det_pos[i].y * 100, vsl_det_pos[i].z * 100};
                    for (int k = 0; k < 3; k++){
                        if (tmp[k] < 0) {
                            com_pub_data.data[i * VSL_DET_ENCODE_SIZE + k * DOUBLE_ENCODE_SIZE + 1] = 1;
                            tmp[k] = -tmp[k];
                        }
                        else com_pub_data.data[i * VSL_DET_ENCODE_SIZE + k * DOUBLE_ENCODE_SIZE + 1] = 0;
                        com_pub_data.data[i * VSL_DET_ENCODE_SIZE + k * DOUBLE_ENCODE_SIZE + 2] = int(tmp[k]) >> 16;
                        com_pub_data.data[i * VSL_DET_ENCODE_SIZE + k * DOUBLE_ENCODE_SIZE + 3] = (int(tmp[k]) >> 8) & ((1 << 8) - 1);
                        com_pub_data.data[i * VSL_DET_ENCODE_SIZE + k * DOUBLE_ENCODE_SIZE + 4] = int(tmp[k]) & ((1 << 8) - 1); 
                    }
                }
            }
        }
        com_pub_data.src_address = "suav_" + std::to_string(sUAV_id);
        for (int i = 1; i <= UAV_NUM; i++){
            if (i == sUAV_id) continue;
            com_pub_data.dst_address = "suav_" + std::to_string(i);
            com_pub->publish(com_pub_data);
        }
        com_pub_data.dst_address = "usv";
        com_pub->publish(com_pub_data);

        std_msgs::msg::Int16 tmp;
        tmp.data = det_res[sUAV_id];
        // det_pub->publish(tmp); 

        if (if_i_am_bridge(sUAV_id) && cmd != 0){
            bridge_point = bridge_pos<Point>(sUAV_id);
            task_state = BRIDGE;
        }

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
            case BRIDGE:{
                StepBridge();
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
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
    rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr alt_sub;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr nav_sub, vsl_sub[VESSEL_NUM];
    rclcpp::Subscription<target_bbox_msgs::msg::BoundingBoxes>::SharedPtr det_box_sub;
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_cmd_pub;
    rclcpp::Subscription<ros_ign_interfaces::msg::Dataframe>::SharedPtr com_sub;
    rclcpp::Publisher<ros_ign_interfaces::msg::Dataframe>::SharedPtr com_pub;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr cmd_sub;

    rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_sub;
    rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr run_clock_sub;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr score_sub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr phase_sub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr report_stream_pub;
    rclcpp::Publisher<ros_ign_interfaces::msg::StringVec>::SharedPtr report_target_pub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr report_status_sub;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr vsl_det_cmd_pub;
};


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<sUAV>(argv[1]));
    rclcpp::shutdown();
    return 0;
}
