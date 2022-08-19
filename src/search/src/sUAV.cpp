#include "Utils.h"

#include "MyMathFun.h"
#include "MyDataFun.h"

#define sUAV_NUM 14
#define VESSEL_NUM 7
#define KP 0.2
#define X_KP KP
#define Y_KP KP
#define Z_KP KP
#define YAW_KP 1

#define TARGET_VESSEL 'e'
#define COMM_RANGE 800

#define DOUBLE_ENCODE_SIZE 4
#define VSL_DET_ENCODE_SIZE (5 * DOUBLE_ENCODE_SIZE + 1)

#define USE_GROUNDTRUTH

using namespace geometry_msgs::msg;
using namespace std::chrono_literals;
using std::placeholders::_1;

int sUAV_id;

typedef enum TASK_CODE{
    INIT,
    TAKEOFF,
    PREPARE,
    STRETCH,
    SEARCH,
    PURSUE,
    BRIDGE,
    HOLD,
    BACK,
    LAND
} STATE_CODE;


struct REL_LOC{
    double time, delta_x, delta_y, vel_x, vel_y, pos_x, pos_y;
    std::string output_str(){
        char s[200];
        sprintf(s, "{%.2lf: %.2lf,%.2lf %.2lf,%.2lf, %.2lf, %.2lf}", time, delta_x, delta_y, vel_x, vel_y, pos_x, pos_y);
        std::string str(s);
        return str;
    }
};

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

    // Detection status feedback
    std::string status;

    // [Invalid] Groundtruth position of sUAV itself
    Point UAV_pos;

    // Attitude of sUAV itself by imu
    Quaternion UAV_att_imu;
    
    // [Invalid] Groundtruth attitude of sUAV itself
    Quaternion UAV_att_pos;

    // Air pressure height of sUAV itself
    double air_pressure;

    // Velocity command for sUAV itself
    Point UAV_vel;

    // Euler angles of sUAV itself
    double UAV_Euler[3];

    // Transform matrix of sUAV itself
    double R_e2b[3][3];

    // Position of target vessel A-G by vision
    Point vsl_pos[VESSEL_NUM];

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

    // Velocity norm saturation of UAV control
    double sat_norm;

    // Velocity saturation of UAV control 
    Point sat_vel;

    // Yaw rate saturation of UAV control
    double sat_yaw_rate;

    // Some task points in world frame
    Point birth_point, takeoff_point, prepare_point;

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

    // [StepSearch] Search progress of other UAVs
    int search_progress[sUAV_NUM + 1];

    // [StepSearch] Search wait flag
    bool search_wait_flag[1010];

    // [StepSearch] Detecing Results of Other UAVs
    int det_res[sUAV_NUM + 1];

    // [StepSearch] Detection success counter
    int det_cnt[VESSEL_NUM];

    // [StepSearch] Detection started time
    double det_start_time[VESSEL_NUM];

    // [StepSearch] Detection result time stamp
    double det_res_time[VESSEL_NUM];

    // [StepSearch] Vessel position result altogether
    Point vsl_det_pos[VESSEL_NUM];

    // Deque for relative localization
    std::deque<REL_LOC> rel_loc[VESSEL_NUM];
    size_t rel_loc_buffer_size = 200;

    // [StepSearch] Vessel yaw angle
    double vsl_det_yaw[VESSEL_NUM];

    // [StepSearch] Vessel det time
    double vsl_det_time[VESSEL_NUM];
    
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

    // Last communication time in simulation
    double last_comm_time_det;

    // Last comm time
    double last_comm_time_search;

    // Last comm time for target vessel position & yaw
    double last_comm_time_tgt;


    sUAV(char *name) : Node("suav_" + std::string(name)) {
        sUAV_id = std::atoi(name);

        time_t tt = time(NULL);
        tm* t = localtime(&tt);
        char iden_path[256];
        sprintf(iden_path, "%s/coor_ws/src/search/data/%02d-%02d_%02d-%02d_%02d.txt",
        std::getenv("HOME"), t->tm_mon + 1, t->tm_mday, t->tm_hour, t->tm_min, sUAV_id);
        log_file.open(iden_path, std::ios::out);
        while(!log_file) std::cout << "Error: Could not write data!" << std::endl;
        
        name_vec = {"time", "day", "hour", "min", "sec", "control_state", 
                    "uav_pos_x", "uav_pos_y", "uav_pos_z",
                    "uav_roll", "uav_pitch", "uav_yaw"};

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
#ifdef USE_GROUNDTRUTH
        nav_sub = this->create_subscription<geometry_msgs::msg::Pose>(
            "/model/suav_" + std::to_string(sUAV_id) + "/world_pose", 10,
            [this](const geometry_msgs::msg::Pose & msg){
                MyDataFun::set_value(this->UAV_pos, msg.position);
                MyDataFun::set_value_quaternion(this->UAV_att_pos, msg.orientation);
            }
        );
#else
        nav_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            "/suav_" + std::to_string(sUAV_id) + "/odom", 10,
            [this](const nav_msgs::msg::Odometry & msg){
                MyDataFun::set_value(this->UAV_pos, msg.pose.pose.position);
                MyDataFun::set_value_quaternion(this->UAV_att_pos, msg.pose.pose.orientation);
            }
        );
#endif

        // [Valid] Publish Quadrotor Velocity Command
        vel_cmd_pub = this->create_publisher<geometry_msgs::msg::Twist>(
                "/suav_" + std::to_string(sUAV_id) + "/cmd_vel", 10);

        // [Valid] Received data
        com_sub = this->create_subscription<ros_ign_interfaces::msg::Dataframe>(
            "/suav_" + std::to_string(sUAV_id) + "/rx", 10, 
            [this](const ros_ign_interfaces::msg::Dataframe & msg){
                // assert(msg.data.size() == VSL_DET_ENCODE_SIZE * VESSEL_NUM);
                if (msg.data.size() == VSL_DET_ENCODE_SIZE * VESSEL_NUM){
                    for (int i = 0; i < VESSEL_NUM; i++){
                        // if (msg.data[i * VSL_DET_ENCODE_SIZE] != 0 && msg.data[i * VSL_DET_ENCODE_SIZE] != sUAV_id){
                            int other_id = msg.data[i * VSL_DET_ENCODE_SIZE];
                            double tmp[5];
                            for (int j = 0; j < 5; j++){
                                // tmp[j] = 1.0 * MyDataFun::decode_uint8(msg.data, i * VSL_DET_ENCODE_SIZE + j * DOUBLE_ENCODE_SIZE + 2);
                                tmp[j] = 1.0 * double(msg.data[i * VSL_DET_ENCODE_SIZE + j * DOUBLE_ENCODE_SIZE + 2] << 16) 
                                        + double(msg.data[i * VSL_DET_ENCODE_SIZE + j * DOUBLE_ENCODE_SIZE + 3] << 8) 
                                        + double(msg.data[i * VSL_DET_ENCODE_SIZE + j * DOUBLE_ENCODE_SIZE + 4]);
                                tmp[j] /= 100.0;
                                if (msg.data[i * VSL_DET_ENCODE_SIZE + j * DOUBLE_ENCODE_SIZE + 1] == 1) tmp[j] = -tmp[j]; 
                            }
                            if (tmp[4] > det_res_time[i] && other_id != 0){
                                printf("Has get %d's detection result for %d\n", other_id, i);
                                    // del_det_res(det_res[i], i);
                                if (!has_someone_det(i)){
                                    new_det_res(other_id, i);
                                }
                                MyDataFun::set_value(vsl_det_pos[i], tmp);
                                vsl_det_yaw[i] = tmp[3];
                                det_res_time[i] = tmp[4];
                            }
                    }
                }
                else if (msg.data.size() == sUAV_NUM + 1){
                    printf("msg (%ld) time: %.2lf %s\n", msg.data.size(), msg.header.stamp.sec + 1.0 * msg.header.stamp.nanosec / 1e9, msg.src_address.c_str());
                    for (int i = 1; i <= sUAV_NUM; i++){
                        printf("%d", msg.data[i]);
                        if (i == sUAV_id) continue;
                        if (msg.data[i] == 255) continue;
                        if (msg.data[i] > search_progress[i] || search_progress[i] == 255){
                            search_progress[i] = msg.data[i];
                        }
                    }
                    printf("\n");
                }
                else if (msg.data.size() == 1){
                    cmd = msg.data[0];
                }
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
                if (msg.data.find("vessel_id_success") != std::string::npos){
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

        // [Valid] Command for USV
        // tgt_vsl_pub = this->create_publisher<geometry_msgs::msg::Pose2D> (
        //     "/target_vessel_pose", 10
        // );
        tgt_vsl_pub = this->create_publisher<ros_ign_interfaces::msg::Dataframe>(
            "/suav_" + std::to_string(sUAV_id) + "/tx", 10
        );

        timer_ = this->create_wall_timer(50ms, std::bind(&sUAV::timer_callback, this));
        sat_vel.x = 5;
        sat_vel.y = 5;
        sat_vel.z = 5;
        sat_norm = 10;
        sat_yaw_rate = 90 * DEG2RAD;
        loop = 1;
        // double side_length = 3062.28;
        // double search_single_width = side_length / 10, search_signle_depth = side_length;
        // double search_forward_y = (std::abs (sUAV_id - 5.5) - 0.25) * search_single_width  * ((sUAV_id > 5) * 2 - 1);
        // double search_backward_y = (std::abs (sUAV_id - 5.5) + 0.25) * search_single_width * ((sUAV_id > 5) * 2 - 1);
        // double search_backward_x = -side_length / 2.0;
        // double search_forward_x = search_backward_x + search_signle_depth;
        // double search_height = 50;
        // for (int i = 1; i <= loop; i++){
        //     search_tra.push_back(MyDataFun::new_point(search_backward_x, search_forward_y, search_height));
        //     search_tra.push_back(MyDataFun::new_point(search_forward_x, search_forward_y, search_height));
        //     search_tra.push_back(MyDataFun::new_point(search_forward_x, search_backward_y, search_height));
        //     search_tra.push_back(MyDataFun::new_point(search_backward_x, search_backward_y, search_height));
        // }
        // double search_y = search_single_width * (sUAV_id - 5.5);
        // search_tra.push_back(MyDataFun::new_point(search_backward_x, search_y, search_height));
        // for (int i = 1; i <= loop; i++){
            // MyDataFun::put_discrete_points(search_tra, MyDataFun::new_point(search_forward_x, search_y, search_height), 10);
            // search_tra.push_back(MyDataFun::new_point(search_forward_x, search_y, search_height));
            // MyDataFun::put_discrete_points(search_tra, MyDataFun::new_point(search_backward_x, search_y, search_height), 10);
            // search_tra.push_back(MyDataFun::new_point(search_backward_x, search_y, search_height));
        // }
        // MyDataFun::output_vector(search_tra);

        prepare_point = scissor_point(85 * DEG2RAD, 50, 30 + 2 * scissor_part_id(sUAV_id));
        for (int i = 85; i >= 10; i -= 1){
            search_tra.push_back(scissor_point(1.0 * i * DEG2RAD, scissor_length(1.0 * i * DEG2RAD), 100, 50 + 2 * scissor_part_id(sUAV_id)));
        }

        for (size_t i = 0; i < search_tra.size(); i++){
            search_wait_flag[i] = false;
        }


        map_res = 0;

        for (int i = 0; i < VESSEL_NUM; i++){
            double far_away[3] = {5000.0, 5000.0, 5000.0};
            MyDataFun::set_value(vsl_pos[i], far_away);
            vsl_det_time[i] = 0.0;
            det_res_time[i] = 0.0;
        }
        
        for (int i = 1; i <= sUAV_NUM; i++){
            det_res[i] = 0;
            search_progress[i] = 255;
        }

        last_comm_time_det = 0.0;
        last_comm_time_search = 0.0;
        last_comm_time_tgt = 0.0;

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
            vsl_det_time[vis_vsl_num] = get_time_now();
            det_res_time[vis_vsl_num] = get_time_now();
            det_cnt[vis_vsl_num]++;
            if (det_cnt[vis_vsl_num] >= 40 && det_cnt[vis_vsl_num] <= 50){
                det_start_time[vis_vsl_num] = get_time_now();
            }
            rel_loc[vis_vsl_num].push_back({clock, pos_err_v[0], pos_err_v[1], UAV_vel.x, UAV_vel.y, UAV_pos.x, UAV_pos.y});
            while (rel_loc[vis_vsl_num].size() > rel_loc_buffer_size){
                rel_loc[vis_vsl_num].pop_front();
            }
            auto first = rel_loc[vis_vsl_num][0], last = rel_loc[vis_vsl_num][rel_loc[vis_vsl_num].size() - 1];
            printf("Size: %ld\nFirst: %s\nLast: %s\n", rel_loc[vis_vsl_num].size(), first.output_str().c_str(), last.output_str().c_str());
            // double delta_x_vsl = last.delta_x - first.delta_x + (last.vel_x + first.vel_x) / 2.0 * (last.time - first.time);
            // double delta_y_vsl = last.delta_y - first.delta_y + (last.vel_y + first.vel_y) / 2.0 * (last.time - first.time);
            // vsl_det_yaw[vis_vsl_num] = std::atan2(delta_y_vsl, delta_x_vsl);
            double delta_x_vsl = last.delta_x - first.delta_x + (last.pos_x - first.pos_x);
            double delta_y_vsl = last.delta_y - first.delta_y + (last.pos_y - first.pos_y);
            if (status != "vessel_id_success"){
                vsl_det_yaw[vis_vsl_num] = std::atan2(delta_y_vsl, delta_x_vsl);
            }
            printf("dx: %.2lf dy: %.2lf yaw: %.2lf\n", delta_x_vsl, delta_y_vsl, vsl_det_yaw[vis_vsl_num]);
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
    void saturate_length(T &a){
        double norm = MyDataFun::norm(a);
        if (norm < sat_norm) return;
        a = MyDataFun::scale(a, sat_norm / norm);
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
    void b2e(T &a){
        double xx = a.x, yy = a.y, zz = a.z;
        a.x = R_e2b[0][0] * xx + R_e2b[1][0] * yy + R_e2b[2][0] * zz;
        a.y = R_e2b[0][1] * xx + R_e2b[1][1] * yy + R_e2b[2][1] * zz;
        a.z = R_e2b[0][2] * xx + R_e2b[1][2] * yy + R_e2b[2][2] * zz;
    }

    bool has_det(int id, int vsl_id){
        return (det_res[id] >> vsl_id) & 1;
    }

    bool has_someone_det(int vsl_id){
        for (int i = 1; i <= sUAV_NUM; i++){
            if (has_det(i, vsl_id)) return true;
        }
        return false;
    }

    void del_det_res(int id, int vsl_id){
        if (has_det(id, vsl_id)){
            det_res[id] -= (1 << vsl_id);
        }
    }

    void new_det_res(int id, int vsl_id){
        if (!has_det(id, vsl_id)){
            det_res[id] += (1 << vsl_id);
        }
    }

    int tgt_vsl_det_id(){
        int vsl_id = TARGET_VESSEL - 'a';
        for (int i = 1; i <= sUAV_NUM; i++){
            if (has_det(i, vsl_id)) return i;
        }
        return 0;
    }

    int nxt_pursue_id(int x){
        if (x % sUAV_NUM == 0) return x;
        else return x + 1;
    }

    template<typename T>
    double target_dis_from_start(T target_pos){
        return MyDataFun::dis(target_pos, MyDataFun::new_point(-1500, 0, 0));
    }

    template<typename T>
    int bridge_num(T target_pos){
        return std::ceil(target_dis_from_start(target_pos) / COMM_RANGE) - 1;
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
        for (int i = 1; i <= sUAV_NUM; i++){
            if (if_i_am_bridge(i)){
                bdg.push_back(i);
            }
        }
        std::sort(bdg.begin(), bdg.end(), [](int a, int b){return abs(1.0 * a - 5.5) > abs(1.0 * b - 5.5);});
        auto it = std::find(bdg.begin(), bdg.end(), id);
        return std::distance(bdg.begin(), it) + 1;
    }

    int bridge_at(int pos){
        std::vector<int> bdg;
        for (int i = 1; i <= sUAV_NUM; i++){
            if (if_i_am_bridge(i)){
                bdg.push_back(i);
            }
        }
        std::sort(bdg.begin(), bdg.end(), [](int a, int b){return abs(1.0 * a - 5.5) > abs(1.0 * b - 5.5);});
        if (pos < 0) pos = bdg.size() + pos;
        else pos--;
        if (pos < 0 || size_t(pos) >= bdg.size()) return -1;
        return bdg[pos];
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

    bool same_side(int id){
        return (id > (sUAV_NUM / 2)) == (sUAV_id > (sUAV_NUM / 2));
    }

    int scissor_part_id(int id){
        if (id > sUAV_NUM / 2) id -= sUAV_NUM / 2;
        return id;
    }

    template<typename T>
    double scissor_theta(T target){
        return MyDataFun::angle_2d(MyDataFun::new_point(-1450, 0, 0), target);
    }

    double scissor_length(double theta){
        theta = std::abs(theta);
        double width = 3000.0, length = 3000.0;
        double gamma = std::atan(width / length / 2.0);
        if (theta > gamma){
            return (width / 2.0 / std::sin(theta)) / (sUAV_NUM / 2.0 + 1);
        }
        else {
            return (length / std::cos(theta)) / (sUAV_NUM / 2.0 + 1);
        }
    }

    Point scissor_point(double theta, double unit_l, double unit_h, double scissor_height = 50.0){
        Point res;
        int part_id = (sUAV_id > sUAV_NUM / 2) ? (sUAV_id - sUAV_NUM / 2) : sUAV_id;
        res.x = part_id * unit_l;
        res.y = (sUAV_id % 2) ? -unit_h : unit_h;
        res.z = scissor_height;

        theta = (sUAV_id > sUAV_NUM / 2) ? (-abs(theta)) : (abs(theta));

        Point sci;
        sci.x = res.x * std::cos(theta) - res.y * std::sin(theta) - 1450;
        sci.y = res.x * std::sin(theta) + res.y * std::cos(theta);
        sci.z = res.z;

        return sci;
    }

    template<typename T>
    void UAV_Control_earth(T ctrl_cmd, double yaw_rate){
        geometry_msgs::msg::Twist cmd;
        cmd.angular.z = yaw_rate * YAW_KP;
        MyDataFun::set_value(cmd.linear, ctrl_cmd);
        cmd.linear.x *= X_KP;
        cmd.linear.y *= Y_KP;
        cmd.linear.z *= Z_KP;
        // printf("Vel cmd (earth): %.2lf %.2lf %.2lf\n", cmd.linear.x, cmd.linear.y, cmd.linear.z);
        saturate_length(cmd.linear);
        // printf("Sat vel cmd (earth): %.2lf %.2lf %.2lf\n", cmd.linear.x, cmd.linear.y, cmd.linear.z);
        e2b(cmd.linear);
        // printf("Vel cmd (body): %.2lf %.2lf %.2lf\n", cmd.linear.x, cmd.linear.y, cmd.linear.z);
        saturate_vel(cmd.linear);
        // printf("Sat vel cmd (body): %.2lf %.2lf %.2lf\n", cmd.linear.x, cmd.linear.y, cmd.linear.z);
        saturate_yaw_rate(cmd.angular.z);
        // printf("Sat yaw cmd (body): %.2lf\n", cmd.angular.z);
        vel_cmd_pub->publish(cmd);
        b2e(cmd.linear);
        MyDataFun::set_value(UAV_vel, cmd.linear);
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
        MyDataFun::set_value(UAV_vel, cmd.linear);
        vel_cmd_pub->publish(cmd);
        b2e(cmd.linear);
        MyDataFun::set_value(UAV_vel, cmd.linear);
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
        // search_tra[0].z = takeoff_point.z;
        if (cmd != 233){
            return;
        }
        UAV_Control_earth(0, 0, 0, 0);
        task_begin_time = get_time_now();
        search_tra_finish = 255;
        if (UAV_pos.x != 0.0 || UAV_pos.y != 0.0 || UAV_pos.z != 0.0){
            printf("Get Ground Truth Position!!!!!!!!!!!!!!!!!!!!!!\n");
            task_state = TAKEOFF;
        }
    }

    void StepTakeoff(){
     	printf("Takeoff!!!\n");
        UAV_Control_to_Point_earth(takeoff_point);
        if (is_near(takeoff_point, 5)){
            task_state = PREPARE;
            // [Valid] Result of Detecting
            det_box_sub = this->create_subscription<target_bbox_msgs::msg::BoundingBoxes>(
            "/suav_" + std::to_string(sUAV_id) + "/slot0/targets/bboxs", 10, std::bind(&sUAV::det_callback, this, _1));
            memset(det_cnt, 0, sizeof(det_cnt));

            std_msgs::msg::String data;
            data.data = "vessel_det";
            vsl_det_cmd_pub->publish(data);
            for (int i = 1; i <= sUAV_NUM; i++){
                det_res[i] = 0;
                search_progress[i] = 255;
            }
        }
    }

    void StepPrepare(){
     	printf("Prepare!!!\n");
        UAV_Control_to_Point_earth(prepare_point);
        if (is_near(prepare_point, 10)){
            task_state = STRETCH;
            search_time = get_time_now();
        }
    }

    void StepStretch(){
     	printf("Stretch!!!\n");
        UAV_Control_to_Point_earth(search_tra[0]);
        if (has_someone_det(TARGET_VESSEL - 'a') && same_side(tgt_vsl_det_id())){
            task_state = PURSUE;
        }
        if (is_near(search_tra[0], 20)){
            task_state = SEARCH;
            search_tra_finish = 0;
            search_progress[sUAV_id] = search_tra_finish;
            search_time = get_time_now();
        }
    }

    void StepSearch(){
     	printf("Search!!!\n");
        printf("To Point %s(%d/%ld)\n", MyDataFun::output_str(search_tra[search_tra_finish]).c_str(), search_tra_finish, search_tra.size());
        double tra_yaw = MyDataFun::angle_2d(UAV_pos, search_tra[search_tra_finish]);
        if (MyDataFun::dis_2d(search_tra[search_tra_finish], UAV_pos) <= 5 && size_t(search_tra_finish) < search_tra.size() - 1){
            tra_yaw = MyDataFun::angle_2d(UAV_pos, search_tra[search_tra_finish + 1]);
        }
        double shaking_angle = std::sin((get_time_now() - search_time) / 8) * PI / 2;
        double yaw_diff = tra_yaw + shaking_angle - UAV_Euler[2];
        // if (abs(tra_yaw + shaking_angle - UAV_Euler[2])< 5 * DEG2RAD){shaking_label = shaking_label * -1;}
        // shaking_diff = shaking_diff + shaking_label * 0.1;
        // double yaw_diff = tra_yaw + shaking_angle - UAV_Euler[2];
        printf("Desired angle %.2lf while now %.2lf\n", (tra_yaw + shaking_angle) * RAD2DEG, UAV_Euler[2] * RAD2DEG);
        // if (MyDataFun::dis_2d(search_tra[search_tra_finish], UAV_pos) <= 5) yaw_diff = 0;
        UAV_Control_earth(MyDataFun::minus(search_tra[search_tra_finish], UAV_pos), yaw_diff);
        search_progress[sUAV_id] = search_tra_finish;
        if (is_near_2d(search_tra[search_tra_finish], 20)){
            search_wait_flag[search_tra_finish] = true;
        }
        if (search_wait_flag[search_tra_finish]){
            bool wait_flag = false;
            for (int i = 1; i <= sUAV_NUM; i++){
                if (same_side(i)){
                    if (search_progress[i] < search_tra_finish || search_progress[i] == 255){
                        wait_flag = true;
                    }
                }
            }
            if (wait_flag){
                printf("Waiting for others...\n");
            }
            else{
                search_tra_finish++;
                if (search_tra_finish == int(search_tra.size())){
                    search_tra_finish = 0;
                    task_state = BACK;
                    // StepMapInit();
                }
            }
        }    
        // bool finish_flag = true;
        // for (int i = 0; i < VESSEL_NUM; i++){
        //     bool tmp_flag = false;
        //     for (int j = 1; j <= sUAV_NUM; j++){
        //         if (has_det(j, i)){
        //             tmp_flag = true;
        //         }
        //     }
        //     if (!tmp_flag) finish_flag = false;
        // }
        // if (finish_flag){
        //     task_state = BACK;
        // }
        // for (int i = 0; i < VESSEL_NUM; i++){
        //     if (is_near_2d(vsl_pos[i], 3000) && det_cnt[i] >= 50){
        //         printf("Got Vessel %c!\n", 'A' + i);
        //         bool other_flag = false;
        //         for (int j = 1; j <= sUAV_NUM; j++){
        //             if (j == sUAV_id) continue;
        //             if (has_det(j, i)){
        //                 printf("But UAV %d has got it!\n", j);
        //                 other_flag = true;
        //             }
        //         }
        //         if (other_flag) continue;
        //         new_det_res(sUAV_id, i);
        //         if ((map_res >> i) & 1){
        //             printf("Already Mapped Vessel %c!\n", 'A' + i);
        //             continue;
        //         }
        //         vsl_id = i;
        //         // search_tra_finish = 0;
        //         task_state = PURSUE;
        //     }
        // }
        if (is_near_2d(vsl_pos[TARGET_VESSEL - 'a'], 3000) && det_cnt[TARGET_VESSEL - 'a'] >= 50){
            printf("Got Vessel %c!\n", TARGET_VESSEL);
            bool other_flag = false;
            for (int j = 1; j <= sUAV_NUM; j++){
                if (j == sUAV_id) continue;
                if (has_det(j, TARGET_VESSEL - 'a')){
                    printf("But UAV %d has got it!\n", j);
                    other_flag = true;
                }
            }
            det_start_time[TARGET_VESSEL - 'a'] = get_time_now();
            if (!other_flag){
                new_det_res(sUAV_id, TARGET_VESSEL - 'a');
            }
        }
        if (has_someone_det(TARGET_VESSEL - 'a') && same_side(tgt_vsl_det_id())){
            task_state = PURSUE;
        }
    }

    void StepPursue(){
        double theta = scissor_theta(vsl_det_pos[TARGET_VESSEL - 'a']);
        double expect_len = target_dis_from_start(vsl_det_pos[TARGET_VESSEL - 'a']) / scissor_part_id(nxt_pursue_id(tgt_vsl_det_id()));
        printf("%d around target (dis: %.2lf)\n", nxt_pursue_id(tgt_vsl_det_id()), target_dis_from_start(vsl_det_pos[TARGET_VESSEL - 'a']));
        printf("Expect length: %.2lf (instead of %.2lf)\n", expect_len, scissor_length(theta));
        Point pursue_point = scissor_point(theta, expect_len, 100, 50);
        // Point pursue_point = scissor_point(theta, scissor_length(theta), 100, 50);
        printf("Target Vessel @ %s\n", MyDataFun::output_str(vsl_det_pos[TARGET_VESSEL - 'a']).c_str());
        printf("Pursue to %s\n", MyDataFun::output_str(pursue_point).c_str());
        UAV_Control_to_Point_with_facing(pursue_point, vsl_det_pos[TARGET_VESSEL - 'a']);

        int det_id = nxt_pursue_id(tgt_vsl_det_id());
        if (same_side(det_id)){
            std_msgs::msg::String data;
            if (phase == "started"){
                if (det_id == sUAV_id){
                    if (sUAV_id % (sUAV_NUM / 2) == 1){
                        data.data = "vessel_det_one";
                    }
                    // else if (rel_loc[TARGET_VESSEL - 'a'].size() >= rel_loc_buffer_size) {
                    else if (get_time_now() >= det_start_time[TARGET_VESSEL - 'a'] + 20.0 && det_start_time[TARGET_VESSEL - 'a'] >= 10) {
                        data.data = "vessel_det_source_" + std::to_string(sUAV_id - 1);
                    }
                    else {
                        data.data = "vessel_det";
                    }
                }
                else{
                    if (sUAV_id % (sUAV_NUM / 2) == 1){
                        data.data = "vessel_det_upload";
                    }
                    // else if (sUAV_id < det_id){
                    else {
                        data.data = "vessel_det_bridge_" + std::to_string(sUAV_id - 1);
                    }
                }
            }
            else {
                if (sUAV_id % (sUAV_NUM / 2) == 1){
                    data.data = "vessel_det_success_upload";
                }
                // else if (sUAV_id < det_id){
                else {
                    data.data = "vessel_det_success_bridge_" + std::to_string(sUAV_id - 1);
                }
            }
            printf("%s\n", data.data.c_str());
            vsl_det_cmd_pub->publish(data);
        }
        // if (status == "vessel_id_success"){
        //     task_state = BACK;
        // }
    }
	
    void StepBridge(){
        UAV_Control_to_Point_earth(bridge_point);
        printf("Target Vessel @ %s\n", MyDataFun::output_str(vsl_det_pos[TARGET_VESSEL - 'a']).c_str());
        printf("Bridge @ %s (%d of %d)\n", MyDataFun::output_str(bridge_point).c_str(), which_bridge(sUAV_id), bridge_num(vsl_det_pos[TARGET_VESSEL - 'a']));

        std_msgs::msg::String data;
        int nxt = bridge_at(which_bridge(sUAV_id) - 1);
        if (nxt == -1) data.data = "vessel_det_upload";
        else data.data = "vessel_det_bridge_" + std::to_string(nxt);
        vsl_det_cmd_pub->publish(data);

        printf("Comm to %d(%s)\n", nxt, data.data.c_str());

        
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
        log_file << std::endl;
    }

    void timer_callback() {
        update_time();
        std::cout << "\033c" << std::flush;
        printf("Time: %.2lf(%.2lf)\n", task_time, run_clock);
        printf("Phase: %s\nStatus: %s\nScore: %.2lf\n", phase.c_str(), status.c_str(), score);
        printf("sUAV #%d @ %s\n", sUAV_id, MyDataFun::output_str(UAV_pos).c_str());
        printf("Commander cmd: %d\n", cmd);
        
        printf("Phi %.2lf, Theta %.2lf, Psi %.2lf\n", UAV_Euler[0] * RAD2DEG, UAV_Euler[1] * RAD2DEG, UAV_Euler[2] * RAD2DEG);
        
        printf("Prepare @ %s\n", MyDataFun::output_str(prepare_point).c_str());
        printf("Stretch @ %s\n", MyDataFun::output_str(search_tra[0]).c_str());

        printf("Search Progress(%d): ", search_tra_finish);
        for (int i = 1; i <= sUAV_NUM; i++){
            if (same_side(i)){
                printf("%d/", search_progress[i]);
            }
        }
        printf("\n");

        printf("My det: %c(%d) %s %.2lf\n", TARGET_VESSEL, det_cnt[TARGET_VESSEL - 'a'],
                                             MyDataFun::output_str(vsl_pos[TARGET_VESSEL - 'a']).c_str(),
                                             vsl_det_time[TARGET_VESSEL - 'a']);
        
        printf("Everyone det: %s by %d Time: %.2lf Yaw: %.2lf\n", MyDataFun::output_str(vsl_det_pos[TARGET_VESSEL - 'a']).c_str(),
                                           tgt_vsl_det_id(),
                                           det_res_time[TARGET_VESSEL - 'a'], vsl_det_yaw[TARGET_VESSEL - 'a'] * RAD2DEG);

        // printf("Detection Counter: ");
        // for (int i = 0; i < VESSEL_NUM; i++){
        //     printf("%c:(%d) ", 'A' + i, det_cnt[i]);
        // }printf("\n");
        printf("Detection Status: ");
        for (int i = 0; i < VESSEL_NUM; i++){
            if (has_det(sUAV_id, i)) printf("%c", 'A' + i);
        }printf("\n");
        // printf("Map Status: ");
        // for (int i = 0; i < VESSEL_NUM; i++){
        //     if (map_res >> i & 1) printf("%c", 'A' + i);
        // }printf("\n");
        // for (int i = 0; i < VESSEL_NUM; i++){
        //     printf("%s,%.2lf/", MyDataFun::output_str(vsl_det_pos[i]).c_str(), vsl_det_yaw[i] * RAD2DEG);
        // }printf("\n");
        // printf("det_res:");
        // for (int i = 1; i <= sUAV_NUM; i++){
        //     for (int j = 0; j < VESSEL_NUM; j++){
        //         if (has_det(i, j)) printf("%c%.0lf", 'A' + j, det_res_time[j]);
        //     }
        //     printf("/");
        // }printf("\n");
        printf("Last det time @ %.2lf, start @ %.2lf\n", vsl_det_time[TARGET_VESSEL - 'a'], det_start_time[TARGET_VESSEL - 'a']);

        printf("Last comm det @ %.2lf, search @ %.2lf\n", last_comm_time_det, last_comm_time_search);

        if (get_time_now() > last_comm_time_det + 0.5) {
            last_comm_time_det = get_time_now();
            ros_ign_interfaces::msg::Dataframe com_pub_data;
            com_pub_data.data.resize(VESSEL_NUM * VSL_DET_ENCODE_SIZE);
            for (int i = 0; i < VESSEL_NUM; i++){
                for (int j = 1; j <= sUAV_NUM; j++){
                    if (has_det(j, i)){
                        if (j == sUAV_id) {
                            MyDataFun::set_value(vsl_det_pos[i], vsl_pos[i]);
                        }
                        com_pub_data.data[i * VSL_DET_ENCODE_SIZE] = j;
                        double tmp[5] = {vsl_det_pos[i].x * 100, vsl_det_pos[i].y * 100, vsl_det_pos[i].z * 100, vsl_det_yaw[i] * 100, det_res_time[i] * 100};
                        for (int k = 0; k < 5; k++){
                            if (tmp[k] < 0) {
                                com_pub_data.data[i * VSL_DET_ENCODE_SIZE + k * DOUBLE_ENCODE_SIZE + 1] = 1;
                                tmp[k] = -tmp[k];
                            }
                            else com_pub_data.data[i * VSL_DET_ENCODE_SIZE + k * DOUBLE_ENCODE_SIZE + 1] = 0;
                            com_pub_data.data[i * VSL_DET_ENCODE_SIZE + k * DOUBLE_ENCODE_SIZE + 2] = MyDataFun::encode_uint8(uint32_t(tmp[k]), 2);
                            com_pub_data.data[i * VSL_DET_ENCODE_SIZE + k * DOUBLE_ENCODE_SIZE + 3] = MyDataFun::encode_uint8(uint32_t(tmp[k]), 1);
                            com_pub_data.data[i * VSL_DET_ENCODE_SIZE + k * DOUBLE_ENCODE_SIZE + 4] = MyDataFun::encode_uint8(uint32_t(tmp[k]), 0);
                            // com_pub_data.data[i * VSL_DET_ENCODE_SIZE + k * DOUBLE_ENCODE_SIZE + 2] = int(tmp[k]) >> 16;
                            // com_pub_data.data[i * VSL_DET_ENCODE_SIZE + k * DOUBLE_ENCODE_SIZE + 3] = (int(tmp[k]) >> 8) & ((1 << 8) - 1);
                            // com_pub_data.data[i * VSL_DET_ENCODE_SIZE + k * DOUBLE_ENCODE_SIZE + 4] = int(tmp[k]) & ((1 << 8) - 1); 
                        }
                    }
                }
            }

            com_pub_data.src_address = "suav_" + std::to_string(sUAV_id);
            for (int i = 1; i <= sUAV_NUM; i++){
                if (i == sUAV_id) continue;
                if (i <= sUAV_id - 2 || i >= sUAV_id + 2) continue;
                com_pub_data.dst_address = "suav_" + std::to_string(i);
                com_pub->publish(com_pub_data);
            }
            // com_pub_data.dst_address = "usv";
            // com_pub->publish(com_pub_data);
            for (int i = 1; i <= 3; i++){
                com_pub_data.dst_address = "tuav_" + std::to_string(i);
                com_pub->publish(com_pub_data);
            }
        }

        if (get_time_now() > last_comm_time_search + 0.2){
            last_comm_time_search = get_time_now();
            ros_ign_interfaces::msg::Dataframe search_com_data;
            // search_progress[sUAV_id] = search_tra_finish;
            search_com_data.data.resize(sUAV_NUM + 1);
            for (int i = 1; i <= sUAV_NUM; i++){
                search_com_data.data[i] = search_progress[i];
            }

            search_com_data.src_address = "suav_" + std::to_string(sUAV_id);
            for (int i = 1; i <= sUAV_NUM; i++){
                if (i == sUAV_id) continue;
                if (i <= sUAV_id - 2 || i >= sUAV_id + 2) continue;
                search_com_data.dst_address = "suav_" + std::to_string(i);
                com_pub->publish(search_com_data);
            }
        }

        if (get_time_now() > last_comm_time_tgt + 1.0 && vsl_det_time[TARGET_VESSEL - 'a'] <= get_time_now() - 5.0){
            last_comm_time_tgt = get_time_now();
            // if (has_det(sUAV_id, TARGET_VESSEL - 'a')){
                geometry_msgs::msg::Pose2D data;
                data.x = vsl_det_pos[TARGET_VESSEL - 'a'].x;
                data.y = vsl_det_pos[TARGET_VESSEL - 'a'].y;
                data.theta = vsl_det_yaw[TARGET_VESSEL - 'a'];
                ros_ign_interfaces::msg::Dataframe pub_data;
                pub_data.src_address = "suav_" + std::to_string(sUAV_id);
                pub_data.dst_address = "usv";
                pub_data.data.resize(5 * 3);

                double tmp[4] = {data.x * 100, data.y * 100, data.theta * 100};
                for (int k = 0; k < 4; k++){
                    if (tmp[k] < 0) {
                        pub_data.data[k * 5] = 1;
                        tmp[k] = -tmp[k];
                    }
                    else pub_data.data[k * 5] = 0;
                    pub_data.data[k * 5 + 1] = MyDataFun::encode_uint8(uint32_t(tmp[k]), 3);
                    pub_data.data[k * 5 + 2] = MyDataFun::encode_uint8(uint32_t(tmp[k]), 2);
                    pub_data.data[k * 5 + 3] = MyDataFun::encode_uint8(uint32_t(tmp[k]), 1);
                    pub_data.data[k * 5 + 4] = MyDataFun::encode_uint8(uint32_t(tmp[k]), 0);
                }

                
                if (data.x != 0.0 || data.y != 0.0){
                    tgt_vsl_pub->publish(pub_data);
                }
        // }
        }

        std_msgs::msg::Int16 tmp;
        tmp.data = det_res[sUAV_id];
        // det_pub->publish(tmp); 

        if (task_state < BACK){
            // if (if_i_am_bridge(sUAV_id) && cmd != 0){
            //     bridge_point = bridge_pos<Point>(sUAV_id);
            //     task_state = BRIDGE;
            // }
            if (status.find("vessel_id_success") != std::string::npos && task_state != PURSUE){
                task_state = BACK;
            }
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
            case STRETCH:{
                StepStretch();
                break;
            }
            case SEARCH:{
                StepSearch();
                break;
            }
            case PURSUE:{
                StepPursue();
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

#ifdef USE_GROUNDTRUTH
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr nav_sub;
#else
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr nav_sub;
#endif

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

    rclcpp::Publisher<ros_ign_interfaces::msg::Dataframe>::SharedPtr tgt_vsl_pub;
    // rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr tgt_vsl_pub;
};


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<sUAV>(argv[1]));
    rclcpp::shutdown();
    return 0;
}
