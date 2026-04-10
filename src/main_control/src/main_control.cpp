#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <std_srvs/Empty.h>
#include <tf/tf.h>
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <iostream>

// 自定义消息（需根据实际包名调整）
#include <raicom_vision_laser/DetectionInfo.h>

// ============================================================================
// 1. 状态机枚举（已合并冗余状态）
// ============================================================================
enum MissionState {
    INIT_TAKEOFF,               // 起飞
    NAV_TO_RECOG_AREA,          // 导航至目标识别区（原 PASS_OBSTACLES + GO_TO_TARGET_AREA 合并）
    HOVER_RECOG_TARGET,         // 悬停识别目标指示牌（下视YOLO + 像素对准）
    NAV_TO_DROP_AREA,           // 导航至物资投放区（原 GO_TO_ENTRY + AVOID_CYLINDER + GO_TO_DROP_AREA 合并）
    HOVER_RECOG_DROP,           // 悬停识别投放区标识（下视YOLO + 像素对准）
    DROP_SUPPLY,                // 投放物资箱
    MOVE_TO_ATTACK_AREA,        // 移动至攻击目标识别区（导航）
    RECOG_ATTACK_TARGET,        // 识别正确攻击目标（前视YOLO确认字符）
    MOVE_TO_FRONT_OF_TARGET,    // 移动到目标正前方（导航）
    ALIGN_ATTACK_TARGET,        // 前视像素对准目标（准备射击）
    SIMULATE_ATTACK,            // 激光指示攻击
    WAIT_HIT_CONFIRMATION,      // 等待裁判确认
    RETURN_LAND,                // 返回起飞点并降落
    TASK_END
};

// ============================================================================
// 2. 全局管理类
// ============================================================================
class MissionManager {
public:
    MissionManager(ros::NodeHandle &nh) : nh_(nh), current_state_(INIT_TAKEOFF) {
        loadParameters();
        initWaypoints();
    }

    void run();
    void initROSCommunication();
    void waitForConnection();

private:
    // ---------- ROS 通信 ----------
    ros::NodeHandle nh_;
    ros::Publisher  setpoint_pub_;           // FSM直接控制
    ros::Publisher  ego_goal_pub_;           // 导航目标点
    ros::Publisher  drop_trigger_pub_;       // 投放触发
    ros::Publisher  laser_trigger_pub_;      // 激光触发
    ros::Subscriber state_sub_, odom_sub_;
    ros::Subscriber nav_status_sub_;         // 导航状态
    ros::Subscriber detected_target_sub_;    // YOLO确认目标字符
    ros::Subscriber yolo_detect_sub_;        // YOLO检测详情
    ros::Subscriber hit_confirm_sub_;        // 裁判确认

    // 服务客户端
    ros::ServiceClient switch_camera_client_;
    ros::ServiceClient reset_target_client_;
    ros::ServiceClient get_status_client_;
    ros::ServiceClient set_mode_client_;
    ros::ServiceClient arming_client_;

    // ---------- 状态机数据 ----------
    MissionState current_state_;
    ros::Time state_start_time_;
    ros::Time last_request_time_;
    bool init_pos_received_ = false;
    bool mission_finished_ = false;

    // ---------- 无人机状态 ----------
    mavros_msgs::State current_mav_state_;
    nav_msgs::Odometry local_odom_;
    double current_yaw_;
    float  init_pos_x_, init_pos_y_, init_pos_z_;
    double init_yaw_;

    // ---------- 导航状态 ----------
    int8_t nav_status_ = 0;          // 0=IDLE,1=FLYING,2=ARRIVED
    bool nav_goal_sent_ = false;

    // ---------- 视觉识别数据 ----------
    std::string confirmed_target_;   // 确认的字符目标（如"A"）
    bool target_confirmed_ = false;  // 是否已确认
    bool front_camera_active_ = false; // 当前摄像头模式（false=下视，true=前视）

    // YOLO检测详情（用于像素对准）
    struct DetectionData {
        bool detected = false;
        float center_x = 0.0f;
        float center_y = 0.0f;
        float confidence = 0.0f;
        ros::Time last_update;
    };
    DetectionData current_detection_;

    // 攻击目标世界坐标
    Eigen::Vector3f attack_target_world_;
    bool hit_confirmed_ = false;

    // PID控制相关（像素对准）
    ros::Time last_pid_control_time_;
    float pix_integral_x_ = 0.0f;
    float pix_integral_y_ = 0.0f;
    float last_pix_err_x_ = 0.0f;
    float last_pix_err_y_ = 0.0f;

    // ---------- 参数配置 ----------
    struct Config {
        float takeoff_height;
        float max_speed;
        float max_yaw_rate;
        float err_max;
        float hover_vert_tolerance;
        float p_xy, p_z;
        float hover_time_needed;          // 悬停稳定时间
        float target_front_offset;        // 攻击正前方距离
        float nav_goal_timeout;           // 导航到达超时
        float align_pixel_threshold;      // 像素对准阈值
        float shoot_delay;                // 射击后等待时间

        // 像素PID参数
        float PIX_VEL_P, PIX_VEL_I, PIX_VEL_D;
        float PIX_VEL_MAX;
        float PIX_FAR_NORM_DIST;          // 远距离归一化阈值
    } cfg_;

    // 航点（相对于起飞点）—— 仅保留实际使用的航点
    struct Waypoint { float x, y, z; };
    Waypoint wp_target_area_;
    Waypoint wp_drop_area_;
    Waypoint wp_attack_area_;

    // 图像中心（根据分辨率调整）
    const float IMG_CENTER_X = 320.0f;
    const float IMG_CENTER_Y = 240.0f;

    // ---------- 初始化函数 ----------
    void loadParameters();
    void initWaypoints();

    // ---------- 回调函数 ----------
    void stateCallback(const mavros_msgs::State::ConstPtr &msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
    void navStatusCallback(const std_msgs::Int8::ConstPtr &msg);
    void detectedTargetCallback(const std_msgs::String::ConstPtr &msg);
    void yoloDetectCallback(const raicom_vision_laser::DetectionInfo::ConstPtr &msg);
    void hitConfirmCallback(const std_msgs::Bool::ConstPtr &msg);

    // ---------- 控制辅助函数 ----------
    void sendSetpoint(const mavros_msgs::PositionTarget &sp);
    void sendEgoGoal(float x, float y, float z, float yaw = NAN);
    bool waitForNavArrival();
    void positionControl(const Eigen::Vector3f &target_pos, mavros_msgs::PositionTarget &sp);
    void yawControl(double target_yaw, mavros_msgs::PositionTarget &sp, double dt);
    bool reachedTarget(const Eigen::Vector3f &target, float dist_thresh);
    bool isHoveringStable(float vert_tolerance);

    // 像素PID速度计算（复用精准降落逻辑）
    void getPixPidVel(float err_x, float err_y, float dt, float &vel_x, float &vel_y);
    float satfunc(float value, float limit);

    // 服务调用封装
    bool callSwitchCamera();
    bool callResetTarget();

    // ---------- 状态处理函数 ----------
    void handleInitTakeoff();
    void handleNavToRecogArea();          // 合并后的导航至识别区
    void handleHoverRecognizeTarget();
    void handleNavToDropArea();           // 合并后的导航至投放区
    void handleHoverRecognizeDrop();
    void handleDropSupply();
    void handleMoveToAttackArea();
    void handleRecognizeAttackTarget();
    void handleMoveToFrontOfTarget();
    void handleAlignAttackTarget();
    void handleSimulateAttack();
    void handleWaitHitConfirmation();
    void handleReturnLand();
    void handleTaskEnd();
};

// ============================================================================
// 3. 参数加载与航点初始化（移除冗余航点）
// ============================================================================
void MissionManager::loadParameters() {
    // 控制参数
    nh_.param<float>("takeoff_height", cfg_.takeoff_height, 1.2f);
    ROS_INFO("  takeoff_height: %.2f", cfg_.takeoff_height);
    
    nh_.param<float>("max_speed", cfg_.max_speed, 0.8f);
    ROS_INFO("  max_speed: %.2f", cfg_.max_speed);
    
    nh_.param<float>("max_yaw_rate", cfg_.max_yaw_rate, 0.8f);
    ROS_INFO("  max_yaw_rate: %.2f", cfg_.max_yaw_rate);
    
    nh_.param<float>("err_max", cfg_.err_max, 0.25f);
    ROS_INFO("  err_max: %.2f", cfg_.err_max);
    
    nh_.param<float>("hover_vert_tolerance", cfg_.hover_vert_tolerance, 0.03f);
    ROS_INFO("  hover_vert_tolerance: %.3f", cfg_.hover_vert_tolerance);
    
    nh_.param<float>("p_xy", cfg_.p_xy, 0.4f);
    ROS_INFO("  p_xy: %.2f", cfg_.p_xy);
    
    nh_.param<float>("p_z", cfg_.p_z, 0.3f);
    ROS_INFO("  p_z: %.2f", cfg_.p_z);
    
    nh_.param<float>("hover_time_needed", cfg_.hover_time_needed, 3.0f);
    ROS_INFO("  hover_time_needed: %.1f", cfg_.hover_time_needed);
    
    nh_.param<float>("target_front_offset", cfg_.target_front_offset, 1.0f);
    ROS_INFO("  target_front_offset: %.2f", cfg_.target_front_offset);
    
    nh_.param<float>("nav_goal_timeout", cfg_.nav_goal_timeout, 60.0f);
    ROS_INFO("  nav_goal_timeout: %.1f", cfg_.nav_goal_timeout);
    
    nh_.param<float>("align_pixel_threshold", cfg_.align_pixel_threshold, 15.0f);
    ROS_INFO("  align_pixel_threshold: %.1f", cfg_.align_pixel_threshold);
    
    nh_.param<float>("shoot_delay", cfg_.shoot_delay, 2.0f);
    ROS_INFO("  shoot_delay: %.1f", cfg_.shoot_delay);

    // 像素PID参数
    nh_.param<float>("PIX_VEL_P", cfg_.PIX_VEL_P, 0.003f);
    ROS_INFO("  PIX_VEL_P: %.4f", cfg_.PIX_VEL_P);
    
    nh_.param<float>("PIX_VEL_I", cfg_.PIX_VEL_I, 0.0001f);
    ROS_INFO("  PIX_VEL_I: %.4f", cfg_.PIX_VEL_I);
    
    nh_.param<float>("PIX_VEL_D", cfg_.PIX_VEL_D, 0.001f);
    ROS_INFO("  PIX_VEL_D: %.4f", cfg_.PIX_VEL_D);
    
    nh_.param<float>("PIX_VEL_MAX", cfg_.PIX_VEL_MAX, 0.4f);
    ROS_INFO("  PIX_VEL_MAX: %.2f", cfg_.PIX_VEL_MAX);
    
    nh_.param<float>("PIX_FAR_NORM_DIST", cfg_.PIX_FAR_NORM_DIST, 150.0f);
    ROS_INFO("  PIX_FAR_NORM_DIST: %.1f", cfg_.PIX_FAR_NORM_DIST);

    // 航点参数（相对起飞点）
    nh_.param<float>("wp_target_area_x", wp_target_area_.x, 6.0f);
    nh_.param<float>("wp_target_area_y", wp_target_area_.y, 0.0f);
    nh_.param<float>("wp_target_area_z", wp_target_area_.z, cfg_.takeoff_height);
    ROS_INFO("  wp_target_area: (%.2f, %.2f, %.2f)", wp_target_area_.x, wp_target_area_.y, wp_target_area_.z);
    
    nh_.param<float>("wp_drop_area_x", wp_drop_area_.x, 10.0f);
    nh_.param<float>("wp_drop_area_y", wp_drop_area_.y, 0.0f);
    nh_.param<float>("wp_drop_area_z", wp_drop_area_.z, cfg_.takeoff_height);
    ROS_INFO("  wp_drop_area: (%.2f, %.2f, %.2f)", wp_drop_area_.x, wp_drop_area_.y, wp_drop_area_.z);
    
    nh_.param<float>("wp_attack_area_x", wp_attack_area_.x, 12.0f);
    nh_.param<float>("wp_attack_area_y", wp_attack_area_.y, 0.0f);
    nh_.param<float>("wp_attack_area_z", wp_attack_area_.z, cfg_.takeoff_height);
    ROS_INFO("  wp_attack_area: (%.2f, %.2f, %.2f)", wp_attack_area_.x, wp_attack_area_.y, wp_attack_area_.z);

    ROS_INFO("参数加载完成。");
}
void MissionManager::initWaypoints() {
    // 实际坐标在起飞后动态计算
}

// ============================================================================
// 4. ROS通信初始化
// ============================================================================
void MissionManager::initROSCommunication() {
    setpoint_pub_      = nh_.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
    ego_goal_pub_      = nh_.advertise<geometry_msgs::PoseStamped>("/fsm/ego_goal", 1);
    drop_trigger_pub_  = nh_.advertise<std_msgs::Bool>("/uav/drop_trigger", 1);
    laser_trigger_pub_ = nh_.advertise<std_msgs::Bool>("/uav/laser_trigger", 1);

    state_sub_         = nh_.subscribe("/mavros/state", 10, &MissionManager::stateCallback, this);
    odom_sub_          = nh_.subscribe("/mavros/local_position/odom", 10, &MissionManager::odomCallback, this);
    nav_status_sub_    = nh_.subscribe("/ego_controller/status", 10, &MissionManager::navStatusCallback, this);
    detected_target_sub_ = nh_.subscribe("/detected_target", 10, &MissionManager::detectedTargetCallback, this);
    yolo_detect_sub_   = nh_.subscribe("/yolo_detect", 10, &MissionManager::yoloDetectCallback, this);
    hit_confirm_sub_   = nh_.subscribe("/referee/hit_confirmed", 10, &MissionManager::hitConfirmCallback, this);

    // 服务客户端
    switch_camera_client_ = nh_.serviceClient<std_srvs::Empty>("/switch_camera");
    reset_target_client_  = nh_.serviceClient<std_srvs::Empty>("/reset_target");
    get_status_client_    = nh_.serviceClient<std_srvs::Empty>("/get_system_status");
    set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    arming_client_   = nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
}

// ============================================================================
// 5. 回调函数实现
// ============================================================================
void MissionManager::stateCallback(const mavros_msgs::State::ConstPtr &msg) {
    current_mav_state_ = *msg;
}

void MissionManager::odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
    local_odom_ = *msg;
    tf::Quaternion q;
    tf::quaternionMsgToTF(local_odom_.pose.pose.orientation, q);
    double roll, pitch;
    tf::Matrix3x3(q).getRPY(roll, pitch, current_yaw_);

    if (!init_pos_received_ && local_odom_.pose.pose.position.z > -0.5) {
        init_pos_x_ = local_odom_.pose.pose.position.x;
        init_pos_y_ = local_odom_.pose.pose.position.y;
        init_pos_z_ = local_odom_.pose.pose.position.z;
        init_yaw_ = current_yaw_;
        init_pos_received_ = true;
        ROS_INFO("初始位置记录: (%.2f, %.2f, %.2f), 偏航: %.2f°",
                 init_pos_x_, init_pos_y_, init_pos_z_, init_yaw_ * 180 / M_PI);
    }
}

void MissionManager::navStatusCallback(const std_msgs::Int8::ConstPtr &msg) {
    nav_status_ = msg->data;
}

void MissionManager::detectedTargetCallback(const std_msgs::String::ConstPtr &msg) {
    confirmed_target_ = msg->data;
    target_confirmed_ = true;
    ROS_INFO("★★★ 目标确认: %s ★★★", confirmed_target_.c_str());
}

void MissionManager::yoloDetectCallback(const raicom_vision_laser::DetectionInfo::ConstPtr &msg) {
    if (msg->num_detections == 0) {
        current_detection_.detected = false;
        return;
    }

    // 查找与确认目标匹配的检测框（如果已确认目标）
    bool found = false;
    for (int i = 0; i < msg->num_detections; ++i) {
        if (confirmed_target_.empty() || msg->class_names[i] == confirmed_target_) {
            current_detection_.center_x = msg->center_x[i];
            current_detection_.center_y = msg->center_y[i];
            current_detection_.confidence = msg->confidences[i];
            current_detection_.detected = true;
            current_detection_.last_update = ros::Time::now();
            found = true;
            break;
        }
    }
    if (!found) {
        current_detection_.detected = false;
    }
}

void MissionManager::hitConfirmCallback(const std_msgs::Bool::ConstPtr &msg) {
    if (msg->data) {
        hit_confirmed_ = true;
        ROS_INFO("裁判确认击中目标！");
    }
}

// ============================================================================
// 6. 控制辅助函数
// ============================================================================
void MissionManager::sendSetpoint(const mavros_msgs::PositionTarget &sp) {
    setpoint_pub_.publish(sp);
}

void MissionManager::sendEgoGoal(float x, float y, float z, float yaw) {
    geometry_msgs::PoseStamped goal;
    goal.header.stamp = ros::Time::now();
    goal.header.frame_id = "world";
    goal.pose.position.x = x;
    goal.pose.position.y = y;
    goal.pose.position.z = z;
    if (!std::isnan(yaw)) {
        tf::Quaternion q = tf::createQuaternionFromYaw(yaw);
        goal.pose.orientation.x = q.x();
        goal.pose.orientation.y = q.y();
        goal.pose.orientation.z = q.z();
        goal.pose.orientation.w = q.w();
    } else {
        goal.pose.orientation.w = 1.0;
    }
    ego_goal_pub_.publish(goal);
    nav_goal_sent_ = true;
    ROS_INFO("导航目标点: (%.2f, %.2f, %.2f)", x, y, z);
}

bool MissionManager::waitForNavArrival() {
    if ((ros::Time::now() - state_start_time_).toSec() > cfg_.nav_goal_timeout) {
        ROS_WARN("等待导航到达超时！");
        return true;
    }
    return (nav_status_ == 2);
}

void MissionManager::positionControl(const Eigen::Vector3f &target_pos, mavros_msgs::PositionTarget &sp) {
    Eigen::Vector3f err = target_pos - Eigen::Vector3f(local_odom_.pose.pose.position.x,
                                                        local_odom_.pose.pose.position.y,
                                                        local_odom_.pose.pose.position.z);
    float vx = err.x() * cfg_.p_xy;
    float vy = err.y() * cfg_.p_xy;
    float vz = err.z() * cfg_.p_z;
    vx = std::clamp(vx, -cfg_.max_speed, cfg_.max_speed);
    vy = std::clamp(vy, -cfg_.max_speed, cfg_.max_speed);
    vz = std::clamp(vz, -cfg_.max_speed, cfg_.max_speed);

    sp.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    sp.type_mask = 0b100111000011;
    sp.velocity.x = vx;
    sp.velocity.y = vy;
    sp.velocity.z = vz;
    sp.yaw = current_yaw_;
}

bool MissionManager::reachedTarget(const Eigen::Vector3f &target, float dist_thresh) {
    float dx = target.x() - local_odom_.pose.pose.position.x;
    float dy = target.y() - local_odom_.pose.pose.position.y;
    float dz = target.z() - local_odom_.pose.pose.position.z;
    return (dx*dx + dy*dy + dz*dz) < (dist_thresh * dist_thresh);
}

bool MissionManager::isHoveringStable(float vert_tolerance) {
    if (std::abs(local_odom_.twist.twist.linear.z) > 0.05) return false;
    float target_z = init_pos_z_ + cfg_.takeoff_height;
    if (std::abs(local_odom_.pose.pose.position.z - target_z) > vert_tolerance) return false;
    tf::Quaternion q;
    tf::quaternionMsgToTF(local_odom_.pose.pose.orientation, q);
    double roll, pitch;
    tf::Matrix3x3(q).getRPY(roll, pitch, current_yaw_);
    if (std::abs(roll) > 0.05 || std::abs(pitch) > 0.05) return false;
    return true;
}

float MissionManager::satfunc(float value, float limit) {
    return std::clamp(value, -limit, limit);
}

void MissionManager::getPixPidVel(float err_x, float err_y, float dt, float &vel_x, float &vel_y) {
    // 远距离归一化
    float norm_factor = 1.0f;
    float pixel_err = sqrt(err_x*err_x + err_y*err_y);
    if (pixel_err > cfg_.PIX_FAR_NORM_DIST) {
        norm_factor = cfg_.PIX_FAR_NORM_DIST / pixel_err;
    }
    float norm_err_x = err_x * norm_factor;
    float norm_err_y = err_y * norm_factor;

    // 积分
    pix_integral_x_ += norm_err_x * dt;
    pix_integral_y_ += norm_err_y * dt;
    // 微分
    float deriv_x = (norm_err_x - last_pix_err_x_) / dt;
    float deriv_y = (norm_err_y - last_pix_err_y_) / dt;

    float pid_x = cfg_.PIX_VEL_P * norm_err_x + cfg_.PIX_VEL_I * pix_integral_x_ + cfg_.PIX_VEL_D * deriv_x;
    float pid_y = cfg_.PIX_VEL_P * norm_err_y + cfg_.PIX_VEL_I * pix_integral_y_ + cfg_.PIX_VEL_D * deriv_y;

    vel_x = satfunc(pid_x, cfg_.PIX_VEL_MAX);
    vel_y = satfunc(pid_y, cfg_.PIX_VEL_MAX);

    last_pix_err_x_ = norm_err_x;
    last_pix_err_y_ = norm_err_y;
}

bool MissionManager::callSwitchCamera() {
    std_srvs::Empty srv;
    if (switch_camera_client_.call(srv)) {
        front_camera_active_ = !front_camera_active_;
        ROS_INFO("摄像头切换成功，当前模式: %s", front_camera_active_ ? "前视" : "下视");
        return true;
    }
    ROS_WARN("切换摄像头服务调用失败");
    return false;
}

bool MissionManager::callResetTarget() {
    std_srvs::Empty srv;
    if (reset_target_client_.call(srv)) {
        target_confirmed_ = false;
        confirmed_target_.clear();
        current_detection_.detected = false;
        ROS_INFO("目标记忆已重置");
        return true;
    }
    ROS_WARN("重置目标服务调用失败");
    return false;
}

// ============================================================================
// 7. 等待连接
// ============================================================================
void MissionManager::waitForConnection() {
    ros::Rate rate(20);
    while (ros::ok() && !current_mav_state_.connected) {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("MAVROS已连接");

    while (ros::ok() && !init_pos_received_) {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("初始位姿已接收");

    // 等待YOLO服务可用
    ROS_INFO("等待YOLO服务...");
    switch_camera_client_.waitForExistence();
    reset_target_client_.waitForExistence();
    ROS_INFO("YOLO服务已就绪");
}

// ============================================================================
// 8. 各状态处理函数
// ============================================================================

// 8.1 起飞
void MissionManager::handleInitTakeoff() {
    static int sub_state = 0;               // 0: 准备设定点, 1: 切OFFBOARD, 2: 解锁, 3: 爬升稳定
    static ros::Time last_request;
    static int setpoint_count = 0;

    ros::Time now = ros::Time::now();
    float target_z = init_pos_z_ + cfg_.takeoff_height;
    float current_z = local_odom_.pose.pose.position.z;

    // 构造设定点消息（位置控制）
    mavros_msgs::PositionTarget sp;
    sp.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    sp.type_mask = 0b101111111000;          // 仅使用位置+偏航
    sp.position.x = init_pos_x_;
    sp.position.y = init_pos_y_;
    sp.position.z = target_z;
    sp.yaw = init_yaw_;

    // 持续发布设定点（必须高频发布以维持 OFFBOARD）
    sendSetpoint(sp);

    switch (sub_state) {
        case 0: // 预发送设定点
            if (setpoint_count < 100) {
                setpoint_count++;
                if (setpoint_count == 100) {
                    ROS_INFO("已发送100个设定点，准备切换OFFBOARD模式");
                    sub_state = 1;
                    last_request = now;
                }
            }
            break;

        case 1: // 请求切换到 OFFBOARD 模式
            if (current_mav_state_.mode != "OFFBOARD" &&
                (now - last_request) > ros::Duration(3.0)) {
                mavros_msgs::SetMode srv;
                srv.request.custom_mode = "OFFBOARD";
                if (set_mode_client_.call(srv) && srv.response.mode_sent) {
                    ROS_INFO("OFFBOARD模式请求成功");
                    sub_state = 2;
                    last_request = now;
                } else {
                    ROS_WARN("切换OFFBOARD失败，重试中...");
                    last_request = now;
                }
            } else if (current_mav_state_.mode == "OFFBOARD") {
                // 已经处于 OFFBOARD，直接进入解锁阶段
                sub_state = 2;
                last_request = now;
            }
            break;

        case 2: // 请求解锁
            if (!current_mav_state_.armed &&
                (now - last_request) > ros::Duration(3.0)) {
                mavros_msgs::CommandBool srv;
                srv.request.value = true;
                if (arming_client_.call(srv) && srv.response.success) {
                    ROS_INFO("无人机解锁成功");
                    sub_state = 3;
                    last_request = now;
                } else {
                    ROS_WARN("解锁失败，重试中...");
                    last_request = now;
                }
            } else if (current_mav_state_.armed) {
                // 已经解锁，直接进入爬升
                sub_state = 3;
                last_request = now;
            }
            break;

        case 3: // 爬升至目标高度并稳定
            if (fabs(current_z - target_z) < 0.2) {
                // 到达高度后悬停稳定3秒
                if ((now - last_request).toSec() > 3.0) {
                    ROS_INFO("起飞完成，稳定悬停，进入导航至目标识别区阶段");
                    current_state_ = NAV_TO_RECOG_AREA;   // 修改为合并后的状态
                    nav_goal_sent_ = false;
                    state_start_time_ = now;
                    sub_state = 0;
                    setpoint_count = 0;
                }
            } else {
                last_request = now;  // 未到高度则重置计时
            }
            break;
    }

    if (sub_state == 3) {
        ROS_INFO_THROTTLE(1.0, "爬升中... 当前高度: %.2f / %.2f", current_z, target_z);
    }
}

// 8.2 导航至目标识别区（合并原 PASS_OBSTACLES + GO_TO_TARGET_AREA）
void MissionManager::handleNavToRecogArea() {
    if (!nav_goal_sent_) {
        // 直接发送目标识别区终点，ego_planner 自行规划路径（包括穿越障碍）
        float target_x = init_pos_x_ + wp_target_area_.x;
        float target_y = init_pos_y_ + wp_target_area_.y;
        float target_z = init_pos_z_ + wp_target_area_.z;
        sendEgoGoal(target_x, target_y, target_z);
    }

    if (waitForNavArrival()) {
        current_state_ = HOVER_RECOG_TARGET;
        nav_goal_sent_ = false;
        state_start_time_ = ros::Time::now();
        last_request_time_ = ros::Time::now();
        // 确保下视模式，重置目标记忆
        if (front_camera_active_) callSwitchCamera();
        callResetTarget();
        ROS_INFO("到达目标识别区，开始下视识别");
    }
}

// 8.3 悬停识别目标指示牌（下视YOLO + 像素对准）
void MissionManager::handleHoverRecognizeTarget() {
    // 等待目标确认（YOLO节点连续帧确认后发布/detected_target）
    if (!target_confirmed_) {
        // 保持悬停
        mavros_msgs::PositionTarget sp;
        sp.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        sp.type_mask = 0b100111100011;
        sp.velocity.x = sp.velocity.y = 0;
        sp.position.z = init_pos_z_ + cfg_.takeoff_height;
        sp.yaw = current_yaw_;
        sendSetpoint(sp);
        ROS_INFO_THROTTLE(1.0, "等待下视目标确认...");
        return;
    }

    // // 目标已确认，进入像素对准阶段
    // if (!current_detection_.detected) {
    //     ROS_WARN_THROTTLE(1.0, "目标已确认但当前帧未检测到，保持悬停");
    //     mavros_msgs::PositionTarget sp;
    //     sp.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    //     sp.type_mask = 0b100111000011;
    //     sp.velocity.x = sp.velocity.y = sp.velocity.z = 0;
    //     sp.yaw = current_yaw_;
    //     sendSetpoint(sp);
    //     return;
    // }

    // 计算像素误差并生成速度指令
    float err_x = IMG_CENTER_X - current_detection_.center_x;
    float err_y = IMG_CENTER_Y - current_detection_.center_y;
    float pixel_dist = sqrt(err_x*err_x + err_y*err_y);

    ros::Time now = ros::Time::now();
    float dt = (now - last_pid_control_time_).toSec();
    if (last_pid_control_time_.isZero()) dt = 0.05;
    last_pid_control_time_ = now;

    float vel_x, vel_y;
    getPixPidVel(err_x, err_y, dt, vel_x, vel_y);

    // 构造控制消息：X/Y 使用机体速度，Z 使用绝对位置（保持高度）
    mavros_msgs::PositionTarget sp;
    sp.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
    // 掩码设置：忽略 PX, PY, VZ, AFX, AFY, AFZ, YAW_RATE；使用 VX, VY, PZ, YAW
    sp.type_mask = 0b100111100011;  // 等价于 IGNORE_PX | IGNORE_PY | IGNORE_VZ | IGNORE_AFX | IGNORE_AFY | IGNORE_AFZ | IGNORE_YAW_RATE
    sp.velocity.x = vel_y;          // 像素y误差 → 左右速度
    sp.velocity.y = -vel_x;         // 像素x误差 → 前后速度
    sp.position.z = init_pos_z_ + cfg_.takeoff_height; // 锁定起飞高度
    sp.yaw = current_yaw_;
    sendSetpoint(sp);

    ROS_INFO_THROTTLE(0.5, "[下视对准] 像素误差: %.1f px, 速度: (%.2f, %.2f), 锁定高度: %.2f",
                     pixel_dist, sp.velocity.x, sp.velocity.y, sp.position.z);

    // 对准完成条件：误差小于阈值且悬停稳定
    if (pixel_dist < cfg_.align_pixel_threshold && isHoveringStable(cfg_.hover_vert_tolerance)) {
        ROS_INFO("下视对准完成，进入导航至物资投放区");
        current_state_ = NAV_TO_DROP_AREA;
        nav_goal_sent_ = false;
        state_start_time_ = ros::Time::now();
        pix_integral_x_ = pix_integral_y_ = 0;
    }
}

// 8.4 导航至物资投放区（合并原 GO_TO_ENTRY + AVOID_CYLINDER + GO_TO_DROP_AREA）
void MissionManager::handleNavToDropArea() {
    if (!nav_goal_sent_) {
        // 直接发送投放区终点，ego_planner 自行绕过圆柱等障碍
        float target_x = init_pos_x_ + wp_drop_area_.x;
        float target_y = init_pos_y_ + wp_drop_area_.y;
        float target_z = init_pos_z_ + wp_drop_area_.z;
        sendEgoGoal(target_x, target_y, target_z);
    }

    if (waitForNavArrival()) {
        current_state_ = HOVER_RECOG_DROP;
        nav_goal_sent_ = false;
        state_start_time_ = ros::Time::now();
        // 确保下视，重置目标记忆
        if (front_camera_active_) callSwitchCamera();
        callResetTarget();
        ROS_INFO("到达投放区，开始下视识别投放标识");
    }
}

// 8.5 悬停识别投放区标识（复用下视YOLO对准逻辑）
void MissionManager::handleHoverRecognizeDrop() {
    if (!target_confirmed_) {
        mavros_msgs::PositionTarget sp;
        sp.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        sp.type_mask = 0b100111000011;
        sp.velocity.x = sp.velocity.y = sp.velocity.z = 0;
        sp.yaw = current_yaw_;
        sendSetpoint(sp);
        ROS_INFO_THROTTLE(1.0, "等待投放标识确认...");
        return;
    }

    if (!current_detection_.detected) {
        mavros_msgs::PositionTarget sp;
        sp.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        sp.type_mask = 0b100111000011;
        sp.velocity.x = sp.velocity.y = sp.velocity.z = 0;
        sp.yaw = current_yaw_;
        sendSetpoint(sp);
        return;
    }

    float err_x = IMG_CENTER_X - current_detection_.center_x;
    float err_y = IMG_CENTER_Y - current_detection_.center_y;
    float pixel_dist = sqrt(err_x*err_x + err_y*err_y);

    ros::Time now = ros::Time::now();
    float dt = (now - last_pid_control_time_).toSec();
    if (last_pid_control_time_.isZero()) dt = 0.05;
    last_pid_control_time_ = now;

    float vel_x, vel_y;
    getPixPidVel(err_x, err_y, dt, vel_x, vel_y);

    mavros_msgs::PositionTarget sp;
    sp.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
    sp.type_mask = 0b100111000011;
    sp.velocity.x = vel_y;
    sp.velocity.y = vel_x;
    sp.velocity.z = 0;
    sp.yaw = current_yaw_;
    sendSetpoint(sp);

    ROS_INFO_THROTTLE(0.5, "[投放区对准] 像素误差: %.1f px", pixel_dist);

    if (pixel_dist < cfg_.align_pixel_threshold) {
        ROS_INFO("投放区对准完成，开始投放");
        current_state_ = DROP_SUPPLY;
        state_start_time_ = ros::Time::now();
        pix_integral_x_ = pix_integral_y_ = 0;
    }
}

// 8.6 投放物资
void MissionManager::handleDropSupply() {
    static bool dropped = false;
    if (!dropped) {
        std_msgs::Bool trigger;
        trigger.data = true;
        drop_trigger_pub_.publish(trigger);
        ROS_INFO("物资投放指令已发送");
        dropped = true;
        state_start_time_ = ros::Time::now();
    }
    if ((ros::Time::now() - state_start_time_).toSec() > 1.0) {
        current_state_ = MOVE_TO_ATTACK_AREA;
        nav_goal_sent_ = false;
        state_start_time_ = ros::Time::now();
        ROS_INFO("投放完成，前往攻击目标识别区");
    }
}

// 8.7 移动至攻击目标识别区（导航）
void MissionManager::handleMoveToAttackArea() {
    if (!nav_goal_sent_) {
        sendEgoGoal(init_pos_x_ + wp_attack_area_.x, init_pos_y_ + wp_attack_area_.y, init_pos_z_ + wp_attack_area_.z);
    }
    if (waitForNavArrival()) {
        current_state_ = RECOG_ATTACK_TARGET;
        nav_goal_sent_ = false;
        state_start_time_ = ros::Time::now();
        last_request_time_ = ros::Time::now();
        // 切换到前视摄像头，重置目标记忆
        if (!front_camera_active_) callSwitchCamera();
        callResetTarget();
        ROS_INFO("到达攻击目标识别区，开始前视识别正确目标");
    }
}

// 8.8 识别攻击目标（前视YOLO确认字符）
void MissionManager::handleRecognizeAttackTarget() {
    // 保持悬停
    mavros_msgs::PositionTarget sp;
    sp.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    sp.type_mask = 0b100111000011;
    sp.velocity.x = sp.velocity.y = sp.velocity.z = 0;
    sp.yaw = current_yaw_;
    sendSetpoint(sp);

    if (!target_confirmed_) {
        ROS_INFO_THROTTLE(1.0, "等待前视目标确认...");
        return;
    }

    // 目标已确认，记录世界坐标（用于移动到正前方）
    if (current_detection_.detected) {
        // 根据像素坐标和已知高度估算目标世界坐标（简化：假设目标在地面）
        float fx = 500.0f; // 焦距（需标定）
        float fy = 500.0f;
        float z = cfg_.takeoff_height; // 飞行高度
        float world_x = local_odom_.pose.pose.position.x + (current_detection_.center_x - IMG_CENTER_X) * z / fx;
        float world_y = local_odom_.pose.pose.position.y + (current_detection_.center_y - IMG_CENTER_Y) * z / fy;
        attack_target_world_ = Eigen::Vector3f(world_x, world_y, init_pos_z_);
        ROS_INFO("攻击目标世界坐标估算: (%.2f, %.2f)", world_x, world_y);
    }

    // 确认后等待1秒稳定
    if ((ros::Time::now() - state_start_time_).toSec() > 1.0) {
        ROS_INFO("正确攻击目标已确认，移动到目标正前方");
        current_state_ = MOVE_TO_FRONT_OF_TARGET;
        nav_goal_sent_ = false;
        state_start_time_ = ros::Time::now();
    }
}

// 8.9 移动到目标正前方（导航）
void MissionManager::handleMoveToFrontOfTarget() {
    if (!nav_goal_sent_) {
        Eigen::Vector3f front_pos = attack_target_world_ + Eigen::Vector3f(cfg_.target_front_offset * cos(init_yaw_),
                                                                            cfg_.target_front_offset * sin(init_yaw_),
                                                                            0.0f);
        front_pos.z() = init_pos_z_ + cfg_.takeoff_height;
        sendEgoGoal(front_pos.x(), front_pos.y(), front_pos.z());
    }

    if (waitForNavArrival()) {
        current_state_ = ALIGN_ATTACK_TARGET;
        nav_goal_sent_ = false;
        state_start_time_ = ros::Time::now();
        last_pid_control_time_ = ros::Time(0);
        ROS_INFO("已到达攻击位置，开始前视像素对准");
    }
}

// 8.10 前视像素对准目标
void MissionManager::handleAlignAttackTarget() {
    if (!current_detection_.detected) {
        ROS_WARN_THROTTLE(1.0, "前视未检测到目标，保持悬停");
        mavros_msgs::PositionTarget sp;
        sp.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        sp.type_mask = 0b100111000011;
        sp.velocity.x = sp.velocity.y = sp.velocity.z = 0;
        sp.yaw = current_yaw_;
        sendSetpoint(sp);
        return;
    }

    float err_x = IMG_CENTER_X - current_detection_.center_x;
    float err_y = IMG_CENTER_Y - current_detection_.center_y;
    float pixel_dist = sqrt(err_x*err_x + err_y*err_y);

    ros::Time now = ros::Time::now();
    float dt = (now - last_pid_control_time_).toSec();
    if (last_pid_control_time_.isZero()) dt = 0.05;
    last_pid_control_time_ = now;

    float vel_x, vel_y;
    getPixPidVel(err_x, err_y, dt, vel_x, vel_y);

    mavros_msgs::PositionTarget sp;
    sp.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
    sp.type_mask = 0b100111000011;
    sp.velocity.x = vel_y;
    sp.velocity.y = -vel_x;
    sp.velocity.z = 0;
    sp.yaw = current_yaw_;
    sendSetpoint(sp);

    ROS_INFO_THROTTLE(0.5, "[前视攻击对准] 像素误差: %.1f px", pixel_dist);

    if (pixel_dist < cfg_.align_pixel_threshold) {
        ROS_INFO("前视对准完成，准备攻击");
        current_state_ = SIMULATE_ATTACK;
        state_start_time_ = ros::Time::now();
        pix_integral_x_ = pix_integral_y_ = 0;
    }
}

// 8.11 模拟攻击（激光指示）
void MissionManager::handleSimulateAttack() {
    static bool laser_fired = false;
    if (!laser_fired) {
        std_msgs::Bool trigger;
        trigger.data = true;
        laser_trigger_pub_.publish(trigger);
        ROS_INFO("激光指示装置已触发，等待裁判确认...");
        laser_fired = true;
        state_start_time_ = ros::Time::now();
    }

    if (hit_confirmed_) {
        current_state_ = RETURN_LAND;
        state_start_time_ = ros::Time::now();
        ROS_INFO("击中确认，返回起飞点");
    } else if ((ros::Time::now() - state_start_time_).toSec() > 5.0) {
        current_state_ = WAIT_HIT_CONFIRMATION;
        state_start_time_ = ros::Time::now();
        ROS_WARN("未收到确认，进入等待状态");
    }
}

// 8.12 等待裁判确认
void MissionManager::handleWaitHitConfirmation() {
    mavros_msgs::PositionTarget sp;
    sp.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    sp.type_mask = 0b100111000011;
    sp.velocity.x = sp.velocity.y = sp.velocity.z = 0;
    sp.yaw = current_yaw_;
    sendSetpoint(sp);

    if (hit_confirmed_) {
        current_state_ = RETURN_LAND;
        state_start_time_ = ros::Time::now();
        ROS_INFO("裁判确认击中，返回");
    }
}

// 8.13 返回起飞点并降落
void MissionManager::handleReturnLand() {
    static bool returning = false;
    static float landing_target_z = init_pos_z_ + cfg_.takeoff_height;

    if (!returning) {
        Eigen::Vector3f home(init_pos_x_, init_pos_y_, init_pos_z_ + cfg_.takeoff_height);
        mavros_msgs::PositionTarget sp;
        positionControl(home, sp);
        sendSetpoint(sp);
        if (reachedTarget(home, 0.3)) {
            returning = true;
            state_start_time_ = ros::Time::now();
            ROS_INFO("已返回起飞点上方，开始降落");
        }
    } else {
        landing_target_z -= 0.05;
        if (landing_target_z < init_pos_z_ + 0.1) landing_target_z = init_pos_z_ + 0.1;

        mavros_msgs::PositionTarget sp;
        sp.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        sp.type_mask = 0b101111111000;
        sp.position.x = init_pos_x_;
        sp.position.y = init_pos_y_;
        sp.position.z = landing_target_z;
        sp.yaw = init_yaw_;
        sendSetpoint(sp);

        if (landing_target_z <= init_pos_z_ + 0.15 &&
            std::abs(local_odom_.pose.pose.position.z - (init_pos_z_ + 0.15)) < 0.1) {
            current_state_ = TASK_END;
            ROS_INFO("降落完成，任务结束");
        }
    }
}

// 8.14 任务结束
void MissionManager::handleTaskEnd() {
    mavros_msgs::PositionTarget sp;
    sp.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    sp.type_mask = 0b101111111000;
    sp.position.x = init_pos_x_;
    sp.position.y = init_pos_y_;
    sp.position.z = init_pos_z_;
    sp.yaw = init_yaw_;
    sendSetpoint(sp);
    mission_finished_ = true;
    ROS_INFO("任务完成，节点退出");
}

// ============================================================================
// 9. 主循环
// ============================================================================
void MissionManager::run() {
    ros::Rate rate(20);
    while (ros::ok() && !mission_finished_) {
        switch (current_state_) {
            case INIT_TAKEOFF:            handleInitTakeoff();            break;
            case NAV_TO_RECOG_AREA:       handleNavToRecogArea();         break;
            case HOVER_RECOG_TARGET:      handleHoverRecognizeTarget();   break;
            case NAV_TO_DROP_AREA:        handleNavToDropArea();          break;
            case HOVER_RECOG_DROP:        handleHoverRecognizeDrop();     break;
            case DROP_SUPPLY:             handleDropSupply();             break;
            case MOVE_TO_ATTACK_AREA:     handleMoveToAttackArea();       break;
            case RECOG_ATTACK_TARGET:     handleRecognizeAttackTarget();  break;
            case MOVE_TO_FRONT_OF_TARGET: handleMoveToFrontOfTarget();    break;
            case ALIGN_ATTACK_TARGET:     handleAlignAttackTarget();      break;
            case SIMULATE_ATTACK:         handleSimulateAttack();         break;
            case WAIT_HIT_CONFIRMATION:   handleWaitHitConfirmation();    break;
            case RETURN_LAND:             handleReturnLand();             break;
            case TASK_END:                handleTaskEnd();                break;
            default: break;
        }
        ros::spinOnce();
        rate.sleep();
    }
}

// ============================================================================
// 10. 主函数
// ============================================================================
int main(int argc, char **argv) {
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "mission_state_machine");
    ros::NodeHandle nh("~");

    MissionManager manager(nh);
    manager.initROSCommunication();
    manager.waitForConnection();

    std::cout << "按回车键开始任务..." << std::endl;
    std::cin.get();

    manager.run();
    return 0;
}