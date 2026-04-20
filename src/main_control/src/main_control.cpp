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
#include <std_msgs/UInt16.h>
#include <std_srvs/Empty.h>
#include <tf/tf.h>
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <iostream>
#include <algorithm>   // std::clamp, std::max, std::min
#include <limits>      // std::numeric_limits

// 自定义消息（需根据实际包名调整）
#include <raicom_vision_laser/DetectionInfo.h>

// ============================================================================
// 1. 状态机枚举（已合并冗余状态）
// ============================================================================
enum MissionState {
    INIT_TAKEOFF,               // 起飞
    NAV_TO_RECOG_AREA,          // 导航至目标识别区
    HOVER_RECOG_TARGET,         // 悬停识别目标指示牌（下视YOLO + 像素对准）
    NAV_TO_DROP_AREA,           // 导航至物资投放区
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
        // 初始化默认设定点（安全悬停）
        current_setpoint_.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        current_setpoint_.type_mask = 0b100111000111;   // 使用 VX,VY,VZ,YAW
        current_setpoint_.velocity.x = 0.0f;
        current_setpoint_.velocity.y = 0.0f;
        current_setpoint_.velocity.z = 0.0f;
        current_setpoint_.yaw = 0.0f;
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
    double current_roll_ = 0.0;      // 缓存横滚角
    double current_pitch_ = 0.0;     // 缓存俯仰角
    float  init_pos_x_, init_pos_y_, init_pos_z_;
    double init_yaw_;

    // ---------- 导航状态 ----------
    int8_t nav_status_ = 0;          // 0=IDLE,1=FLYING,2=ARRIVED
    bool nav_goal_sent_ = false;
    // ros::Time brake_start_time_;
    // bool is_braking_ = false;

    // ---------- 视觉识别数据 ----------
    std::string confirmed_target_;   // 确认的字符目标（如"A"）
    bool target_confirmed_ = false;  // 是否已确认
    bool front_camera_active_ = false; // 当前摄像头模式（false=下视，true=前视）

    // YOLO检测详情（用于像素对准）
    struct DetectionData {
        bool detected = false;
        float center_x = 0.0f;
        float center_y = 0.0f;
        float width = 0.0f;    // 新增：识别框宽度
        float height = 0.0f;   // 新增：识别框高度
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

    // 投放对准保持计时
    ros::Time drop_alignment_hold_start_;

    // ---------- 新增：当前周期设定点（主循环统一发送）----------
    mavros_msgs::PositionTarget current_setpoint_;

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
        float PIX_INTEGRAL_MAX;
        float PIX_FAR_NORM_DIST;          // 远距离归一化阈值

        // 投放阶段新增参数
        float drop_arrive_threshold;          // 到达投放区的距离阈值
        float drop_detect_timeout;            // 检测丢失超时
        float drop_align_hold_time;           // 对准保持时间
        float drop_release_max_horiz_speed;   // 释放时最大水平速度
        float drop_release_max_vert_speed;    // 释放时最大垂直速度
        float drop_max_tilt;                  // 释放时最大倾角（rad）
        float drop_camera_bias_x_px;          // 相机像素偏置X
        float drop_camera_bias_y_px;          // 相机像素偏置Y
        float drop_release_bias_x_px;         // 释放点像素偏置X
        float drop_release_bias_y_px;         // 释放点像素偏置Y
        float drop_fine_pixel_radius;         // 精细速度缩放半径
        float drop_fine_vel_scale;            // 精细速度缩放系数
        float drop_descend_distance;          // 投放前下降距离（m）
        int drop_on;
        int drop_off;
    } cfg_;

    // 航点（相对于起飞点）
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

    // 新增辅助函数
    float getHorizontalSpeed() const;
    bool isDropWindowStable(float target_z) const;

    // 像素PID速度计算
    void getPixPidVel(float err_x, float err_y, float dt, float &vel_x, float &vel_y);
    float satfunc(float value, float limit);

    // 服务调用封装
    bool callSwitchCamera();
    bool callResetTarget();

    // ---------- 状态处理函数 ----------
    void handleInitTakeoff();
    void handleNavToRecogArea();
    void handleHoverRecognizeTarget();
    void handleNavToDropArea();
    void handleHoverRecognizeDrop();
    void handleDropSupply();
    void handleMoveToAttackArea();
    // void handleRecognizeAttackTarget();
    // void handleMoveToFrontOfTarget();
    void handleAlignAttackTarget();
    void handleSimulateAttack();
    void handleWaitHitConfirmation();
    void handleReturnLand();
    void handleTaskEnd();
};

// ============================================================================
// 3. 参数加载与航点初始化
// ============================================================================
void MissionManager::loadParameters() {
    // 控制参数
    nh_.param<float>("takeoff_height", cfg_.takeoff_height, 1.2f);
    nh_.param<float>("max_speed", cfg_.max_speed, 0.8f);
    nh_.param<float>("max_yaw_rate", cfg_.max_yaw_rate, 0.8f);
    nh_.param<float>("err_max", cfg_.err_max, 0.25f);
    nh_.param<float>("hover_vert_tolerance", cfg_.hover_vert_tolerance, 0.03f);
    nh_.param<float>("p_xy", cfg_.p_xy, 0.4f);
    nh_.param<float>("p_z", cfg_.p_z, 0.3f);
    nh_.param<float>("hover_time_needed", cfg_.hover_time_needed, 3.0f);
    nh_.param<float>("target_front_offset", cfg_.target_front_offset, 1.0f);
    nh_.param<float>("nav_goal_timeout", cfg_.nav_goal_timeout, 60.0f);
    nh_.param<float>("align_pixel_threshold", cfg_.align_pixel_threshold, 15.0f);
    nh_.param<float>("shoot_delay", cfg_.shoot_delay, 2.0f);

    // 像素PID参数
    nh_.param<float>("PIX_VEL_P", cfg_.PIX_VEL_P, 0.003f);
    nh_.param<float>("PIX_VEL_I", cfg_.PIX_VEL_I, 0.0001f);
    nh_.param<float>("PIX_VEL_D", cfg_.PIX_VEL_D, 0.001f);
    nh_.param<float>("PIX_VEL_MAX", cfg_.PIX_VEL_MAX, 0.4f);
    nh_.param<float>("PIX_FAR_NORM_DIST", cfg_.PIX_FAR_NORM_DIST, 150.0f);

    // 航点参数
    nh_.param<float>("wp_target_area_x", wp_target_area_.x, 6.0f);
    nh_.param<float>("wp_target_area_y", wp_target_area_.y, 0.0f);
    nh_.param<float>("wp_target_area_z", wp_target_area_.z, cfg_.takeoff_height);
    nh_.param<float>("wp_drop_area_x", wp_drop_area_.x, 10.0f);
    nh_.param<float>("wp_drop_area_y", wp_drop_area_.y, 0.0f);
    nh_.param<float>("wp_drop_area_z", wp_drop_area_.z, cfg_.takeoff_height);
    nh_.param<float>("wp_attack_area_x", wp_attack_area_.x, 12.0f);
    nh_.param<float>("wp_attack_area_y", wp_attack_area_.y, 0.0f);
    nh_.param<float>("wp_attack_area_z", wp_attack_area_.z, cfg_.takeoff_height);

    // 投放阶段参数
    nh_.param<float>("drop_arrive_threshold", cfg_.drop_arrive_threshold, 0.35f);
    nh_.param<float>("drop_detect_timeout", cfg_.drop_detect_timeout, 0.30f);
    nh_.param<float>("drop_align_hold_time", cfg_.drop_align_hold_time, 0.35f);
    nh_.param<float>("drop_release_max_horiz_speed", cfg_.drop_release_max_horiz_speed, 0.12f);
    nh_.param<float>("drop_release_max_vert_speed", cfg_.drop_release_max_vert_speed, 0.06f);
    nh_.param<float>("drop_max_tilt", cfg_.drop_max_tilt, 0.08f);
    nh_.param<float>("drop_camera_bias_x_px", cfg_.drop_camera_bias_x_px, 0.0f);
    nh_.param<float>("drop_camera_bias_y_px", cfg_.drop_camera_bias_y_px, 0.0f);
    nh_.param<float>("drop_release_bias_x_px", cfg_.drop_release_bias_x_px, 0.0f);
    nh_.param<float>("drop_release_bias_y_px", cfg_.drop_release_bias_y_px, 0.0f);
    nh_.param<float>("drop_fine_pixel_radius", cfg_.drop_fine_pixel_radius, 35.0f);
    nh_.param<float>("drop_fine_vel_scale", cfg_.drop_fine_vel_scale, 0.45f);
    nh_.param<float>("drop_descend_distance", cfg_.drop_descend_distance, 0.0f);
    nh_.param<int>("drop_on", cfg_.drop_on, 160);
    nh_.param<int>("drop_off", cfg_.drop_off, 160);

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
    drop_trigger_pub_  = nh_.advertise<std_msgs::UInt16>("/servo_control", 1);
    laser_trigger_pub_ = nh_.advertise<std_msgs::Bool>("/uav/laser_trigger", 1);

    state_sub_         = nh_.subscribe("/mavros/state", 10, &MissionManager::stateCallback, this);
    odom_sub_          = nh_.subscribe("/mavros/local_position/odom", 10, &MissionManager::odomCallback, this);
    nav_status_sub_    = nh_.subscribe("/ego_controller/status", 10, &MissionManager::navStatusCallback, this);
    detected_target_sub_ = nh_.subscribe("/detected_target", 10, &MissionManager::detectedTargetCallback, this);
    yolo_detect_sub_   = nh_.subscribe("/ocr_detect", 10, &MissionManager::yoloDetectCallback, this);
    hit_confirm_sub_   = nh_.subscribe("/referee/hit_confirmed", 10, &MissionManager::hitConfirmCallback, this);

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
    tf::Matrix3x3(q).getRPY(current_roll_, current_pitch_, current_yaw_);

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
    // ROS_INFO_THROTTLE(1.0, "YOLO callback: %d detections", msg->num_detections);
    if (msg->num_detections == 0) {
        current_detection_.detected = false;
        return;
    }

    bool found = false;
    for (int i = 0; i < msg->num_detections; ++i) {
        // ROS_INFO_THROTTLE(1.0, "Detected class: [%s]", msg->class_names[i].c_str());
        if (confirmed_target_.empty() || msg->class_names[i] == confirmed_target_) {
            current_detection_.center_x = msg->center_x[i];
            current_detection_.center_y = msg->center_y[i];
            
            // 【新增】：计算边界框的宽度和高度
            current_detection_.width = std::abs(msg->x2[i] - msg->x1[i]);
            current_detection_.height = std::abs(msg->y2[i] - msg->y1[i]);

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
        current_state_ = TASK_END;
        return false;
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
    sp.type_mask = 0b100111000011;   // 注意：这里原为速度控制，我们保留原逻辑，但后续会在状态函数中覆盖
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
    if (std::abs(local_odom_.twist.twist.linear.z) > 0.15) return false;
    // float target_z = init_pos_z_ + cfg_.takeoff_height;
    // if (std::abs(local_odom_.pose.pose.position.z - target_z) > vert_tolerance) return false;
    if (std::abs(current_roll_) > 0.15 || std::abs(current_pitch_) > 0.15) return false;
    return true;
}

float MissionManager::getHorizontalSpeed() const {
    return std::hypot(local_odom_.twist.twist.linear.x, local_odom_.twist.twist.linear.y);
}

bool MissionManager::isDropWindowStable(float target_z) const {
    return getHorizontalSpeed() < cfg_.drop_release_max_horiz_speed &&
           std::abs(local_odom_.twist.twist.linear.z) < cfg_.drop_release_max_vert_speed &&
           std::abs(local_odom_.pose.pose.position.z - target_z) < cfg_.hover_vert_tolerance &&
           std::abs(current_roll_) < cfg_.drop_max_tilt &&
           std::abs(current_pitch_) < cfg_.drop_max_tilt;
}

float MissionManager::satfunc(float value, float limit) {
    return std::clamp(value, -limit, limit);
}

void MissionManager::getPixPidVel(float err_x, float err_y, float dt, float &vel_x, float &vel_y) {
    // 限制 dt，防止由于长时间未识别导致的单次计算爆炸
    dt = std::clamp(dt, 0.02f, 1.0f);

    float norm_factor = 1.0f;
    float pixel_err = std::sqrt(err_x * err_x + err_y * err_y);
    if (pixel_err > cfg_.PIX_FAR_NORM_DIST) {
        norm_factor = cfg_.PIX_FAR_NORM_DIST / pixel_err;
    }
    float norm_err_x = err_x * norm_factor;
    float norm_err_y = err_y * norm_factor;

    //消除 D 项“初见杀” 
    // 如果是刚刚进入对准状态，把 last_pix_err 强行对齐当前误差，让第一帧的 D 项为 0
    if (last_pix_err_x_ == 0.0f && last_pix_err_y_ == 0.0f) {
        last_pix_err_x_ = err_x;
        last_pix_err_y_ = err_y;
    }

    // 只有当目标进入画面中心区域 (比如 100 像素以内)，才开启积分
    if (pixel_err < 80.0f) {
        pix_integral_x_ += norm_err_x * dt;
        pix_integral_y_ += norm_err_y * dt;

    } else {
        pix_integral_x_ = 0.0f; // 误差过大时清空积分，防止炸弹累加
        pix_integral_y_ = 0.0f;
    }

    pix_integral_x_ = std::clamp(pix_integral_x_, -cfg_.PIX_INTEGRAL_MAX, cfg_.PIX_INTEGRAL_MAX);
    pix_integral_y_ = std::clamp(pix_integral_y_, -cfg_.PIX_INTEGRAL_MAX, cfg_.PIX_INTEGRAL_MAX);

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

    ROS_INFO("等待YOLO服务...");
    switch_camera_client_.waitForExistence();
    reset_target_client_.waitForExistence();
    ROS_INFO("YOLO服务已就绪");
}

// ============================================================================
// 8. 各状态处理函数（修改为填充 current_setpoint_）
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
    current_setpoint_.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    current_setpoint_.type_mask = 0b101111111000;          // 仅使用位置+偏航
    current_setpoint_.position.x = init_pos_x_;
    current_setpoint_.position.y = init_pos_y_;
    current_setpoint_.position.z = target_z;
    current_setpoint_.yaw = init_yaw_;

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
                sub_state = 3;
                last_request = now;
            }
            break;

        case 3: // 爬升至目标高度并稳定
            if (fabs(current_z - target_z) < 0.2) {
                if ((now - last_request).toSec() > 3.0) {
                    ROS_INFO("起飞完成，稳定悬停，进入导航至目标识别区阶段");
                    current_state_ = NAV_TO_RECOG_AREA;
                    nav_goal_sent_ = false;
                    state_start_time_ = now;
                    sub_state = 0;
                    setpoint_count = 0;
                }
            } else {
                last_request = now;
            }
            break;
    }

    if (sub_state == 3) {
        ROS_INFO_THROTTLE(1.0, "爬升中... 当前高度: %.2f / %.2f", current_z, target_z);
    }
}

// 8.2 导航至目标识别区
void MissionManager::handleNavToRecogArea() {
    if (!nav_goal_sent_) {
        float target_x = init_pos_x_ + wp_target_area_.x;
        float target_y = init_pos_y_ + wp_target_area_.y;
        float target_z = init_pos_z_ + wp_target_area_.z;
        sendEgoGoal(target_x, target_y, target_z);
    }

    // 导航期间发送安全悬停设定点（ego_planner会覆盖）
    current_setpoint_.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    current_setpoint_.type_mask = 0b100111000111;   // 使用速度控制
    current_setpoint_.velocity.x = current_setpoint_.velocity.y = current_setpoint_.velocity.z = 0.0f;
    current_setpoint_.yaw = current_yaw_;

    if (waitForNavArrival()) {
        current_state_ = HOVER_RECOG_TARGET;
        nav_goal_sent_ = false;
        state_start_time_ = ros::Time::now();
        last_request_time_ = ros::Time::now();
        if (front_camera_active_) callSwitchCamera();
        callResetTarget();
        ROS_INFO("到达目标识别区，开始下视识别");
        current_state_ = HOVER_RECOG_TARGET;
        // is_braking_ = true;                     // 开启刹车模式
        // brake_start_time_ = ros::Time::now();   // 记录刹车开始时间
    }
}

// 8.3 悬停识别目标指示牌
void MissionManager::handleHoverRecognizeTarget() {
    // if (is_braking_) {
    //     // 发送纯悬停指令 (速度全0)
    //     current_setpoint_.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    //     current_setpoint_.type_mask = 0b100111000111;
    //     current_setpoint_.velocity.x = current_setpoint_.velocity.y = current_setpoint_.velocity.z = 0.0f;
    //     current_setpoint_.yaw = current_yaw_;
        
    //     if ((ros::Time::now() - brake_start_time_).toSec() > 1.5) { // 强行镇定 1.5 秒
    //         is_braking_ = false;
    //         last_pid_control_time_ = ros::Time::now(); // 重置 PID 时间，防止 dt 过大
    //         pix_integral_x_ = pix_integral_y_ = 0.0f;  // 清空脏积分
    //         ROS_INFO("刹车镇定完成，开始视觉对准");
    //     } else {
    //         ROS_INFO_THROTTLE(0.5, "导航到达，刹车镇定中...");
    //         return; // 拦截，不执行后续的视觉 PID
    //     }
    // }

    if (!target_confirmed_) {
        // 保持悬停
        current_setpoint_.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        current_setpoint_.type_mask = 0b100111000111;
        current_setpoint_.velocity.x = current_setpoint_.velocity.y = current_setpoint_.velocity.z = 0.0f;
        current_setpoint_.yaw = current_yaw_;
        ROS_INFO_THROTTLE(1.0, "等待下视目标确认...");
        return;
    }

    // 检查检测有效性（包括超时）
    const double detect_age = current_detection_.last_update.isZero()
                                  ? std::numeric_limits<double>::infinity()
                                  : (ros::Time::now() - current_detection_.last_update).toSec();
    if (!current_detection_.detected || detect_age > 0.5) {   // 0.5秒超时
        // 悬停保护
        current_setpoint_.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        current_setpoint_.type_mask = 0b100111000111;
        current_setpoint_.velocity.x = current_setpoint_.velocity.y = current_setpoint_.velocity.z = 0.0f;
        current_setpoint_.yaw = current_yaw_;
        ROS_WARN_THROTTLE(1.0, "目标检测丢失或超时，保持悬停");
        // 重置 PID 积分
        pix_integral_x_ = pix_integral_y_ = 0.0f;
        last_pid_control_time_ = ros::Time(0);
        return;
    }
    

    float err_x = IMG_CENTER_X - current_detection_.center_x;
    float err_y = IMG_CENTER_Y - current_detection_.center_y;
    float pixel_dist = sqrt(err_x*err_x + err_y*err_y);

    ros::Time now = ros::Time::now();
    float dt = (now - last_pid_control_time_).toSec();
    if (last_pid_control_time_.isZero()) dt = 0.05f;
    last_pid_control_time_ = now;

    float vel_x, vel_y;
    getPixPidVel(err_x, err_y, dt, vel_x, vel_y);

    current_setpoint_.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
    current_setpoint_.type_mask = 0b100111000111;   // 与你的相同
    current_setpoint_.velocity.x =  vel_y;
    current_setpoint_.velocity.y =  vel_x;
    current_setpoint_.velocity.z =  0.0f;
    current_setpoint_.yaw = init_yaw_-1.57;              // 固定起飞偏航

    ROS_INFO_THROTTLE(0.5, "[下视对准] err_x=%.1f, err_y=%.1f | vel_x_cmd=%.2f, vel_y_cmd=%.2f | yaw=%.2f",
                 err_x, err_y, current_setpoint_.velocity.x, current_setpoint_.velocity.y, current_yaw_ * 180.0 / M_PI);

    ROS_INFO_THROTTLE(0.5, "[下视对准] 像素误差: %.1f px, 速度: (%.2f, %.2f)",
                     pixel_dist, current_setpoint_.velocity.x, current_setpoint_.velocity.y);

    if (pixel_dist < cfg_.align_pixel_threshold && isHoveringStable(cfg_.hover_vert_tolerance)) {
        ROS_INFO("下视对准完成，进入导航至物资投放区");
        current_state_ = NAV_TO_DROP_AREA;
        nav_goal_sent_ = false;
        state_start_time_ = ros::Time::now();
        pix_integral_x_ = pix_integral_y_ = 0.0f;
    }
}

// 8.4 导航至物资投放区
void MissionManager::handleNavToDropArea() {
    Eigen::Vector3f drop_target(init_pos_x_ + wp_drop_area_.x,
                                init_pos_y_ + wp_drop_area_.y,
                                init_pos_z_ + wp_drop_area_.z);

    if (!nav_goal_sent_) {
        sendEgoGoal(drop_target.x(), drop_target.y(), drop_target.z());
    }
    
    // 安全悬停设定点（等待 ego_planner 接管或到达终点时保持稳定）
    current_setpoint_.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    current_setpoint_.type_mask = 0b100111000111;
    current_setpoint_.velocity.x = current_setpoint_.velocity.y = current_setpoint_.velocity.z = 0.0f;
    current_setpoint_.yaw = current_yaw_;

    if (waitForNavArrival() && isDropWindowStable(drop_target.z())) {
        current_state_ = HOVER_RECOG_DROP;
        nav_goal_sent_ = false;
        state_start_time_ = ros::Time::now();
        if (front_camera_active_) callSwitchCamera();
        callResetTarget();
        last_pid_control_time_ = ros::Time(0);
        drop_alignment_hold_start_ = ros::Time(0);
        pix_integral_x_ = pix_integral_y_ = 0.0f;
        last_pix_err_x_ = last_pix_err_y_ = 0.0f;
        ROS_INFO("到达投放区 (ego_planner)，开始下视识别投放标识");
    }
}

// 8.5 悬停识别投放区标识
void MissionManager::handleHoverRecognizeDrop() {
    auto holdDropHover = [this]() {
        current_setpoint_.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        current_setpoint_.type_mask = 0b100111000111;
        current_setpoint_.velocity.x = current_setpoint_.velocity.y = 0.0f;
        current_setpoint_.velocity.z = (init_pos_z_ + cfg_.takeoff_height - local_odom_.pose.pose.position.z) * cfg_.p_z;
        current_setpoint_.yaw = init_yaw_;
    };

    if (!target_confirmed_) {
        holdDropHover();
        drop_alignment_hold_start_ = ros::Time(0);
        last_pid_control_time_ = ros::Time(0);
        pix_integral_x_ = pix_integral_y_ = 0.0f;
        last_pix_err_x_ = last_pix_err_y_ = 0.0f;
        ROS_INFO_THROTTLE(1.0, "等待投放标识确认...");
        return;
    }

    const double detect_age = current_detection_.last_update.isZero()
                                  ? std::numeric_limits<double>::infinity()
                                  : (ros::Time::now() - current_detection_.last_update).toSec();
    if (!current_detection_.detected || detect_age > cfg_.drop_detect_timeout) {
        holdDropHover();
        drop_alignment_hold_start_ = ros::Time(0);
        last_pid_control_time_ = ros::Time(0);
        pix_integral_x_ = pix_integral_y_ = 0.0f;
        last_pix_err_x_ = last_pix_err_y_ = 0.0f;
        ROS_WARN_THROTTLE(1.0, "投放标识检测丢失或超时，保持悬停");
        return;
    }

    const float aim_center_x = IMG_CENTER_X + cfg_.drop_camera_bias_x_px + cfg_.drop_release_bias_x_px;
    const float aim_center_y = IMG_CENTER_Y + cfg_.drop_camera_bias_y_px + cfg_.drop_release_bias_y_px;

    float err_x = aim_center_x - current_detection_.center_x;
    float err_y = aim_center_y - current_detection_.center_y;
    float pixel_dist = std::sqrt(err_x * err_x + err_y * err_y);

    ros::Time now = ros::Time::now();
    float dt = (now - last_pid_control_time_).toSec();
    if (last_pid_control_time_.isZero()) dt = 0.05f;
    last_pid_control_time_ = now;

    float vel_x, vel_y;
    getPixPidVel(err_x, err_y, dt, vel_x, vel_y);

    if (pixel_dist < cfg_.drop_fine_pixel_radius) {
        vel_x *= cfg_.drop_fine_vel_scale;
        vel_y *= cfg_.drop_fine_vel_scale;
    }

    current_setpoint_.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
    current_setpoint_.type_mask = 0b100111000111;
    current_setpoint_.velocity.x = vel_y;       // 左右
    current_setpoint_.velocity.y = vel_x;      // 前后
    current_setpoint_.velocity.z = (init_pos_z_ + cfg_.takeoff_height - local_odom_.pose.pose.position.z) * cfg_.p_z;
    current_setpoint_.yaw = init_yaw_-1.57;

    ROS_INFO_THROTTLE(0.5,
                      "[投放区对准] 像素误差: %.1f px, 有效中心:(%.1f, %.1f), 机体速度: %.2f m/s",
                      pixel_dist, aim_center_x, aim_center_y, getHorizontalSpeed());

    const bool ready_to_drop = pixel_dist < cfg_.align_pixel_threshold &&
                               isDropWindowStable(init_pos_z_ + wp_drop_area_.z);
    if (!ready_to_drop) {
        drop_alignment_hold_start_ = ros::Time(0);
        return;
    }

    if (drop_alignment_hold_start_.isZero()) {
        drop_alignment_hold_start_ = now;
        return;
    }

    if ((now - drop_alignment_hold_start_).toSec() >= cfg_.drop_align_hold_time) {
        ROS_INFO("投放区对准完成，进入投放");
        current_state_ = DROP_SUPPLY;
        state_start_time_ = ros::Time::now();
        drop_alignment_hold_start_ = ros::Time(0);
        pix_integral_x_ = pix_integral_y_ = 0.0f;
        last_pix_err_x_ = last_pix_err_y_ = 0.0f;
    }
}

// 8.6 投放物资
void MissionManager::handleDropSupply() {
    static bool dropped = false;
    static bool drop_profile_initialized = false;
    static int drop_phase = 0;          // 0=下降, 1=释放, 2=回升
    static float hold_x = 0.0f;
    static float hold_y = 0.0f;
    static float cruise_z = 0.0f;
    static float release_z = 0.0f;

    const ros::Time now = ros::Time::now();

    if (!drop_profile_initialized) {
        hold_x = local_odom_.pose.pose.position.x;
        hold_y = local_odom_.pose.pose.position.y;
        cruise_z = local_odom_.pose.pose.position.z;
        release_z = cruise_z;

        if (cfg_.drop_descend_distance > 0.0f) {
            release_z = std::max(init_pos_z_ + 0.20f, cruise_z - cfg_.drop_descend_distance);
        }

        drop_phase = (cfg_.drop_descend_distance > 0.0f && (cruise_z - release_z) > 1e-3f) ? 0 : 1;
        dropped = false;
        drop_profile_initialized = true;
        state_start_time_ = now;
        ROS_INFO("投放剖面初始化: 巡航高度 %.2f m, 释放高度 %.2f m, 起始阶段 %d",
                 cruise_z, release_z, drop_phase);
    }

    // 基础设定点（位置控制）
    current_setpoint_.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    current_setpoint_.type_mask = 0b101111111000;   // 使用位置控制
    current_setpoint_.position.x = hold_x;
    current_setpoint_.position.y = hold_y;
    current_setpoint_.yaw = init_yaw_;

    if (drop_phase == 0) {
        current_setpoint_.position.z = release_z;

        const bool reached_descend_height =
            std::abs(local_odom_.pose.pose.position.z - release_z) < cfg_.hover_vert_tolerance;
        const bool stable_at_release_height =
            getHorizontalSpeed() < cfg_.drop_release_max_horiz_speed &&
            std::abs(local_odom_.twist.twist.linear.z) < cfg_.drop_release_max_vert_speed &&
            std::abs(current_roll_) < cfg_.drop_max_tilt &&
            std::abs(current_pitch_) < cfg_.drop_max_tilt;

        if (reached_descend_height && stable_at_release_height) {
            ROS_INFO("已下降至释放高度并稳定，进入释放阶段");
            drop_phase = 1;
            state_start_time_ = now;
        }
        return;
    }

    if (drop_phase == 1) {
        current_setpoint_.position.z = release_z;

        if (!isDropWindowStable(release_z)) {
            ROS_WARN_THROTTLE(1.0, "投放窗口不稳定，继续等待速度和姿态收敛");
            return;
        }

        if (!dropped) {
            std_msgs::Bool trigger;
            trigger.data = cfg_.drop_on;
            drop_trigger_pub_.publish(trigger);
            dropped = true;
            state_start_time_ = now;
            ROS_INFO("物资投放指令已发送");
            return;
        }

        if ((now - state_start_time_).toSec() > 1.0) {
            ROS_INFO("释放完成，开始回升");
            drop_phase = 2;
            state_start_time_ = now;
        }
        return;
    }

    // 阶段2：回升
    current_setpoint_.position.z = cruise_z;

    const bool reached_cruise_height =
        std::abs(local_odom_.pose.pose.position.z - cruise_z) < cfg_.hover_vert_tolerance;
    const bool stable_after_ascend =
        getHorizontalSpeed() < cfg_.drop_release_max_horiz_speed &&
        std::abs(local_odom_.twist.twist.linear.z) < cfg_.drop_release_max_vert_speed &&
        std::abs(current_roll_) < cfg_.drop_max_tilt &&
        std::abs(current_pitch_) < cfg_.drop_max_tilt;

    if (reached_cruise_height && stable_after_ascend) {
        ROS_INFO("已回升至巡航高度并稳定，投放任务完成，前往攻击目标识别区");
        dropped = false;
        drop_profile_initialized = false;
        drop_phase = 0;
        // current_state_ = MOVE_TO_ATTACK_AREA;
        current_state_ = MOVE_TO_ATTACK_AREA;
        nav_goal_sent_ = false;
        state_start_time_ = now;
    }
}

// 8.7 移动至攻击目标识别区 (定点直飞模式)
void MissionManager::handleMoveToAttackArea() {
    // 1. 计算攻击目标识别区的绝对世界坐标
    Eigen::Vector3f attack_area_target(init_pos_x_ + wp_attack_area_.x,
                                       init_pos_y_ + wp_attack_area_.y,
                                       init_pos_z_ + wp_attack_area_.z);

    // 2. 使用自带的位置比例控制器生成设定点 (平滑的速度控制)
    mavros_msgs::PositionTarget sp;
    positionControl(attack_area_target, sp);
    sp.yaw = init_yaw_; // 飞行过程中保持起飞偏航角，避免乱转
    current_setpoint_ = sp;

    // 3. 计算误差用于日志打印和到达判定
    const float dx = attack_area_target.x() - local_odom_.pose.pose.position.x;
    const float dy = attack_area_target.y() - local_odom_.pose.pose.position.y;
    const float dz = attack_area_target.z() - local_odom_.pose.pose.position.z;
    
    ROS_INFO_THROTTLE(0.5, "[前往攻击区] 直飞位置误差: xy=%.2f m, z=%.2f m", 
                      std::hypot(dx, dy), dz);

    // 4. 到达判定：距离小于阈值，且飞行器状态(速度、姿态)稳定
    // 此处复用了投放区的到达阈值 cfg_.drop_arrive_threshold，你也可以在 yaml 中单独建一个参数
    if (reachedTarget(attack_area_target, cfg_.drop_arrive_threshold) && 
        isDropWindowStable(attack_area_target.z())) {
        
        current_state_ = RECOG_ATTACK_TARGET; 
        
        nav_goal_sent_ = false; // 虽然没用 ego_planner，但也归零复位状态
        state_start_time_ = ros::Time::now();
        last_request_time_ = ros::Time::now();
        
        // 切换为前视摄像头，并清空上一次的目标记忆
        if (!front_camera_active_) callSwitchCamera();
        callResetTarget();
        
        ROS_INFO("已到达攻击目标识别区，开始前视识别正确目标");
    }
}

/*

// 8.8 识别攻击目标
void MissionManager::handleRecognizeAttackTarget() {
    current_setpoint_.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    current_setpoint_.type_mask = 0b100111000111;
    current_setpoint_.velocity.x = current_setpoint_.velocity.y = current_setpoint_.velocity.z = 0.0f;
    current_setpoint_.yaw = current_yaw_;

    if (!target_confirmed_) {
        ROS_INFO_THROTTLE(1.0, "等待前视目标确认...");
        return;
    }

    if (current_detection_.detected) {
        float fx = 500.0f;
        float fy = 500.0f;
        float z = cfg_.takeoff_height;
        float world_x = local_odom_.pose.pose.position.x + (current_detection_.center_x - IMG_CENTER_X) * z / fx;
        float world_y = local_odom_.pose.pose.position.y + (current_detection_.center_y - IMG_CENTER_Y) * z / fy;
        attack_target_world_ = Eigen::Vector3f(world_x, world_y, init_pos_z_);
        ROS_INFO("攻击目标世界坐标估算: (%.2f, %.2f)", world_x, world_y);
    }

    if ((ros::Time::now() - state_start_time_).toSec() > 1.0) {
        ROS_INFO("正确攻击目标已确认，移动到目标正前方");
        current_state_ = MOVE_TO_FRONT_OF_TARGET;
        nav_goal_sent_ = false;
        state_start_time_ = ros::Time::now();
    }
}

// 8.9 移动到目标正前方
void MissionManager::handleMoveToFrontOfTarget() {
    if (!nav_goal_sent_) {
        Eigen::Vector3f front_pos = attack_target_world_ + Eigen::Vector3f(cfg_.target_front_offset * cos(init_yaw_),
                                                                            cfg_.target_front_offset * sin(init_yaw_),
                                                                            0.0f);
        front_pos.z() = init_pos_z_ + cfg_.takeoff_height;
        sendEgoGoal(front_pos.x(), front_pos.y(), front_pos.z());
    }

    current_setpoint_.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    current_setpoint_.type_mask = 0b100111000111;
    current_setpoint_.velocity.x = current_setpoint_.velocity.y = current_setpoint_.velocity.z = 0.0f;
    current_setpoint_.yaw = current_yaw_;

    if (waitForNavArrival()) {
        current_state_ = ALIGN_ATTACK_TARGET;
        nav_goal_sent_ = false;
        state_start_time_ = ros::Time::now();
        last_pid_control_time_ = ros::Time(0);
        ROS_INFO("已到达攻击位置，开始前视像素对准");
    }
}

*/

// 8.10 前视像素对准目标与距离保持
void MissionManager::handleAlignAttackTarget() {
    // 静态变量：用于确认对准稳定时间
    static ros::Time align_start = ros::Time(0);

    if (!current_detection_.detected) {
        // 目标丢失，紧急刹车并在原高度悬停（利用P_z维持高度）
        current_setpoint_.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        current_setpoint_.type_mask = 0b100111000111;
        current_setpoint_.velocity.x = 0.0f;
        current_setpoint_.velocity.y = 0.0f;
        current_setpoint_.velocity.z = (init_pos_z_ + cfg_.takeoff_height - local_odom_.pose.pose.position.z) * cfg_.p_z;
        current_setpoint_.yaw = -init_yaw_;
        ROS_WARN_THROTTLE(1.0, "前视未检测到目标，保持悬停");
        last_pid_control_time_ = ros::Time(0);
        align_start = ros::Time(0); // 丢失目标，重置对准计时
        return;
    }

    // --- 1. 计算十字准星的平面误差 (控制左右和上下) ---
    float err_x = IMG_CENTER_X - current_detection_.center_x;
    float err_y = IMG_CENTER_Y - current_detection_.center_y;
    float pixel_dist = std::hypot(err_x, err_y);

    ros::Time now = ros::Time::now();
    float dt = (now - last_pid_control_time_).toSec();
    if (last_pid_control_time_.isZero()) dt = 0.05f;
    last_pid_control_time_ = now;

    // 使用你写好的 PID 计算平面修正速度
    float vis_vel_x, vis_vel_y;
    getPixPidVel(err_x, err_y, dt, vis_vel_x, vis_vel_y);

    // --- 2. 计算距离误差 (控制前后) ---
    // 假设在完美射击距离下，靶子在画面中的宽度应该是 150 像素（该值需要你实测后写入 yaml 参数中）
    float target_box_width_px = 150.0f; 
    float dist_err = target_box_width_px - current_detection_.width;
    
    // 简单的 P 控制器计算前后速度 (P_dist 建议写入 yaml)
    float P_dist = 0.005f; 
    float forward_vel = dist_err * P_dist; 
    // 如果框比目标小 (离得远), dist_err > 0, 向前飞
    // 如果框比目标大 (离得近), dist_err < 0, 向后退
    forward_vel = std::clamp(forward_vel, -0.3f, 0.3f); // 前后速度限幅，防止冲过头

    // --- 3. 速度下发 (切换为机体坐标系 BODY_NED) ---
    current_setpoint_.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
    current_setpoint_.type_mask = 0b100111000111;
    
    // 运动学映射 (极其关键)
    current_setpoint_.velocity.x = forward_vel;  // X轴：控制前后距离
    current_setpoint_.velocity.y = vis_vel_x;    // Y轴：控制左右对准 (如运动反向，改为 -vis_vel_x)
    current_setpoint_.velocity.z = -vis_vel_y;   // Z轴：控制上下对准 (NED坐标系Z向下，所以上升给负值。如运动反向，去掉负号)

    current_setpoint_.yaw = -init_yaw_; // 锁死偏航角，防疯转

    ROS_INFO_THROTTLE(0.5, "[前视对准] 准星误差:%.1fpx, 框宽:%.1fpx(目标%.0f) | 速度(前/左/上): %.2f, %.2f, %.2f", 
                      pixel_dist, current_detection_.width, target_box_width_px,
                      current_setpoint_.velocity.x, current_setpoint_.velocity.y, current_setpoint_.velocity.z);

    // --- 4. 射击确认判定 ---
    // 条件：十字准星对准误差极小 且 前后距离也在合理区间内
    bool is_aimed = (pixel_dist < cfg_.align_pixel_threshold);
    bool is_dist_ok = (std::abs(dist_err) < 25.0f); // 距离误差容忍度：±25像素

    if (is_aimed && is_dist_ok) {
        if (align_start.isZero()) align_start = now;

        // 持续稳定瞄准 1 秒后触发射击
        if ((now - align_start).toSec() > 1.0) { 
            ROS_INFO("前视对准与距离保持稳定，准备攻击");
            current_state_ = SIMULATE_ATTACK;
            state_start_time_ = ros::Time::now();
            pix_integral_x_ = pix_integral_y_ = 0.0f;
            align_start = ros::Time(0);
        }
    } else {
        // 如果中间有任何一帧误差变大，重置倒计时，必须连续稳定才开枪
        align_start = ros::Time(0); 
    }
}

// 8.11 模拟攻击
void MissionManager::handleSimulateAttack() {
    static bool laser_fired = false;

    current_setpoint_.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    current_setpoint_.type_mask = 0b100111000111;
    current_setpoint_.velocity.x = current_setpoint_.velocity.y = current_setpoint_.velocity.z = 0.0f;
    current_setpoint_.yaw = current_yaw_;

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
    current_setpoint_.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    current_setpoint_.type_mask = 0b100111000111;
    current_setpoint_.velocity.x = current_setpoint_.velocity.y = current_setpoint_.velocity.z = 0.0f;
    current_setpoint_.yaw = current_yaw_;

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
        // --- 1. 使用 ego_planner 导航返回起飞点 ---
        if (!nav_goal_sent_) {
            float target_x = init_pos_x_;
            float target_y = init_pos_y_;
            float target_z = init_pos_z_ + cfg_.takeoff_height;
            sendEgoGoal(target_x, target_y, target_z);
        }

        // 导航期间发送安全悬停设定点（ego_planner会覆盖速度指令）
        current_setpoint_.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        current_setpoint_.type_mask = 0b100111000111;   // 使用速度控制
        current_setpoint_.velocity.x = current_setpoint_.velocity.y = current_setpoint_.velocity.z = 0.0f;
        current_setpoint_.yaw = current_yaw_;

        if (waitForNavArrival()) {
            returning = true;
            nav_goal_sent_ = false;
            state_start_time_ = ros::Time::now();
            ROS_INFO("已通过ego_planner返回起飞点上方，开始降落");
        }
    } else {
        // --- 2. 原有的缓慢降落逻辑 ---
        landing_target_z -= 0.05f;
        if (landing_target_z < init_pos_z_ + 0.1f) landing_target_z = init_pos_z_ + 0.1f;

        current_setpoint_.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        current_setpoint_.type_mask = 0b101111111000;   // 切换为位置控制
        current_setpoint_.position.x = init_pos_x_;
        current_setpoint_.position.y = init_pos_y_;
        current_setpoint_.position.z = landing_target_z;
        current_setpoint_.yaw = init_yaw_; // 降落时保持起飞朝向更安全

        if (landing_target_z <= init_pos_z_ + 0.15f &&
            std::abs(local_odom_.pose.pose.position.z - (init_pos_z_ + 0.15f)) < 0.1f) {
            current_state_ = TASK_END;
            ROS_INFO("降落完成，任务结束");
        }
    }
}

// 8.14 任务结束
void MissionManager::handleTaskEnd() {
    current_setpoint_.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    current_setpoint_.type_mask = 0b101111111000;
    current_setpoint_.position.x = init_pos_x_;
    current_setpoint_.position.y = init_pos_y_;
    current_setpoint_.position.z = init_pos_z_;
    current_setpoint_.yaw = init_yaw_;
    mission_finished_ = true;
    ROS_INFO("任务完成，节点退出");
}

// ============================================================================
// 9. 主循环（修改为统一发布设定点）
// ============================================================================
void MissionManager::run() {
    ros::Rate rate(20);
    while (ros::ok() && !mission_finished_) {
        // 1. 默认安全设定点（悬停，零速度）
        current_setpoint_.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        current_setpoint_.type_mask = 0b100111000111;
        current_setpoint_.velocity.x = current_setpoint_.velocity.y = current_setpoint_.velocity.z = 0.0f;
        current_setpoint_.yaw = current_yaw_;

        // 2. 执行状态逻辑（各状态函数会覆盖 current_setpoint_）
        switch (current_state_) {
            case INIT_TAKEOFF:            handleInitTakeoff();            break;
            case NAV_TO_RECOG_AREA:       handleNavToRecogArea();         break;
            case HOVER_RECOG_TARGET:      handleHoverRecognizeTarget();   break;
            case NAV_TO_DROP_AREA:        handleNavToDropArea();          break;
            case HOVER_RECOG_DROP:        handleHoverRecognizeDrop();     break;
            case DROP_SUPPLY:             handleDropSupply();             break;
            case MOVE_TO_ATTACK_AREA:     handleMoveToAttackArea();       break;
            // case RECOG_ATTACK_TARGET:     handleRecognizeAttackTarget();  break;
            // case MOVE_TO_FRONT_OF_TARGET: handleMoveToFrontOfTarget();    break;
            case ALIGN_ATTACK_TARGET:     handleAlignAttackTarget();      break;
            case SIMULATE_ATTACK:         handleSimulateAttack();         break;
            case WAIT_HIT_CONFIRMATION:   handleWaitHitConfirmation();    break;
            case RETURN_LAND:             handleReturnLand();             break;
            case TASK_END:                handleTaskEnd();                break;
            default: break;
        }

        // 3. 无条件发布设定点（心跳保证）
        sendSetpoint(current_setpoint_);

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