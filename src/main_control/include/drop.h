/*
 * 这个文件不是拿来直接编译的，而是给“最终合进单个 cpp”时使用的改动清单。
 * 你可以把下面各段按标题，逐段贴回 main_control.cpp。
 *
 * 本版方案是“临时可用版”：
 * 1. 去投放区时不走 ego，直接发 setpoint 定点飞；
 * 2. 下视微调依赖固定高度 + 固定朝向 + 速度/姿态门限；
 * 3. 用经验像素偏置补偿相机安装误差；
 * 4. 用经验像素偏置补偿投物点相对相机的固定偏移；
 * 5. 投放前支持先下落，投完后自动回升；
 * 6. 舵机释放接口先留 TODO，占位说明已经写在代码里。
 */

// ============================================================================
// 0. 需要补充到 main_control.cpp 顶部的头文件
// 作用：
// - std::max 需要 <algorithm>
// - std::numeric_limits 需要 <limits>
// ============================================================================
#include <algorithm>
#include <limits>

// ============================================================================
// 1. 需要补充到 MissionManager 类内的成员变量/参数/函数声明
// 放置位置：
// - current_roll_ / current_pitch_ 放在“无人机状态”里
// - drop_alignment_hold_start_ 放在“PID控制相关”里
// - Config 新参数放在 Config 结构体里
// - 两个 helper 声明放在“控制辅助函数”声明区
// ============================================================================

// ---------- 无人机状态：缓存当前横滚/俯仰，避免每次都重复解四元数 ----------
double current_roll_ = 0.0;
double current_pitch_ = 0.0;

// ---------- PID控制相关：满足投放条件后，保持一小段时间再释放 ----------
ros::Time drop_alignment_hold_start_;

// ---------- Config：投放阶段新增参数 ----------
float drop_arrive_threshold;         // 直飞到投放区时的位置阈值
float drop_detect_timeout;           // 检测结果允许的最大滞后时间
float drop_align_hold_time;          // 满足投放条件后还要持续保持多久
float drop_release_max_horiz_speed;  // 投放时允许的最大水平速度
float drop_release_max_vert_speed;   // 投放时允许的最大垂向速度
float drop_max_tilt;                 // 投放时允许的最大 roll/pitch
float drop_camera_bias_x_px;         // 相机安装误差的像素补偿
float drop_camera_bias_y_px;
float drop_release_bias_x_px;        // 投物点相对相机中心的像素补偿
float drop_release_bias_y_px;
float drop_fine_pixel_radius;        // 进入近距离细调的像素半径
float drop_fine_vel_scale;           // 细调阶段速度缩放比例
float drop_descend_distance;         // 投放前下落距离，设为0则跳过下落

// ---------- 控制辅助函数声明 ----------
float getHorizontalSpeed() const;
bool isDropWindowStable(float target_z) const;

// ============================================================================
// 2. loadParameters() 里需要新增的参数读取
// 作用：
// - 让现场调试时直接在 launch 里改参数，不用再反复改 cpp
// ============================================================================
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

// ============================================================================
// 3. odomCallback() 里的姿态缓存替换片段
// 作用：
// - 后面投放稳定性判断会直接复用 current_roll_ / current_pitch_
// - 用这一句替换原来只拿 yaw 的那一行
// ============================================================================
tf::Matrix3x3(q).getRPY(current_roll_, current_pitch_, current_yaw_);

// ============================================================================
// 4. 替换 getPixPidVel()
// 作用：
// - 首帧或掉帧后 dt 可能过小，先做一个下限保护，避免微分项炸掉
// ============================================================================
void MissionManager::getPixPidVel(float err_x, float err_y, float dt, float &vel_x, float &vel_y)
{
    dt = std::max(dt, 0.02f);

    // 远距离时先做归一化，避免刚看见目标时速度给得过猛
    float norm_factor = 1.0f;
    float pixel_err = std::sqrt(err_x * err_x + err_y * err_y);
    if (pixel_err > cfg_.PIX_FAR_NORM_DIST)
    {
        norm_factor = cfg_.PIX_FAR_NORM_DIST / pixel_err;
    }
    float norm_err_x = err_x * norm_factor;
    float norm_err_y = err_y * norm_factor;

    pix_integral_x_ += norm_err_x * dt;
    pix_integral_y_ += norm_err_y * dt;

    float deriv_x = (norm_err_x - last_pix_err_x_) / dt;
    float deriv_y = (norm_err_y - last_pix_err_y_) / dt;

    float pid_x = cfg_.PIX_VEL_P * norm_err_x +
                  cfg_.PIX_VEL_I * pix_integral_x_ +
                  cfg_.PIX_VEL_D * deriv_x;
    float pid_y = cfg_.PIX_VEL_P * norm_err_y +
                  cfg_.PIX_VEL_I * pix_integral_y_ +
                  cfg_.PIX_VEL_D * deriv_y;

    vel_x = satfunc(pid_x, cfg_.PIX_VEL_MAX);
    vel_y = satfunc(pid_y, cfg_.PIX_VEL_MAX);

    last_pix_err_x_ = norm_err_x;
    last_pix_err_y_ = norm_err_y;
}

// ============================================================================
// 5. 新增 helper：投放稳定窗口判断
// 作用：
// - 只有“速度小 + 高度稳 + 姿态平”时才允许进入最终投放
// - 这就是临时方案里替代正式几何补偿的一层安全兜底
// ============================================================================
float MissionManager::getHorizontalSpeed() const
{
    return std::hypot(local_odom_.twist.twist.linear.x, local_odom_.twist.twist.linear.y);
}

bool MissionManager::isDropWindowStable(float target_z) const
{
    return getHorizontalSpeed() < cfg_.drop_release_max_horiz_speed &&
           std::abs(local_odom_.twist.twist.linear.z) < cfg_.drop_release_max_vert_speed &&
           std::abs(local_odom_.pose.pose.position.z - target_z) < cfg_.hover_vert_tolerance &&
           std::abs(current_roll_) < cfg_.drop_max_tilt &&
           std::abs(current_pitch_) < cfg_.drop_max_tilt;
}

// ============================================================================
// 6. 替换 handleGoToDropArea()
// 作用：
// - 去投放区这段不再走 ego，直接用本地位置控制去定点
// - 到达后会先重置检测状态，再进入下视微调
// ============================================================================
void MissionManager::handleGoToDropArea()
{
    Eigen::Vector3f drop_target(init_pos_x_ + wp_drop_area_.x,
                                init_pos_y_ + wp_drop_area_.y,
                                init_pos_z_ + wp_drop_area_.z);

    mavros_msgs::PositionTarget sp;
    positionControl(drop_target, sp);
    sp.yaw = init_yaw_;
    sendSetpoint(sp);

    const float dx = drop_target.x() - local_odom_.pose.pose.position.x;
    const float dy = drop_target.y() - local_odom_.pose.pose.position.y;
    const float dz = drop_target.z() - local_odom_.pose.pose.position.z;
    ROS_INFO_THROTTLE(0.5, "[投放区直飞] 位置误差: xy=%.2f m, z=%.2f m",
                      std::hypot(dx, dy), dz);

    if (reachedTarget(drop_target, cfg_.drop_arrive_threshold) && isDropWindowStable(drop_target.z()))
    {
        current_state_ = HOVER_RECOG_DROP;
        nav_goal_sent_ = false;
        state_start_time_ = ros::Time::now();

        // 投放识别阶段必须确保是下视相机
        if (front_camera_active_)
            callSwitchCamera();
        callResetTarget();

        // 清掉上一阶段残留的 PID 状态，避免刚进投放时就带着旧积分冲出去
        last_pid_control_time_ = ros::Time(0);
        drop_alignment_hold_start_ = ros::Time(0);
        pix_integral_x_ = pix_integral_y_ = 0.0f;
        last_pix_err_x_ = last_pix_err_y_ = 0.0f;
        ROS_INFO("到达投放区，开始下视识别投放标识");
    }
}

// ============================================================================
// 7. 替换 handleHoverRecognizeDrop()
// 作用：
// - 先悬停等识别
// - 识别到目标后，用“图像中心 + 相机偏置 + 释放点偏置”作为真正的目标中心
// - 微调完成后不立刻投，而是还要保持一小段时间
// ============================================================================
void MissionManager::handleHoverRecognizeDrop()
{
    auto holdDropHover = [this]()
    {
        mavros_msgs::PositionTarget sp;
        sp.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        sp.type_mask = 0b100111000011;
        sp.velocity.x = sp.velocity.y = sp.velocity.z = 0;
        sp.yaw = init_yaw_;
        sendSetpoint(sp);
    };

    if (!target_confirmed_)
    {
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
    if (!current_detection_.detected || detect_age > cfg_.drop_detect_timeout)
    {
        holdDropHover();
        drop_alignment_hold_start_ = ros::Time(0);
        last_pid_control_time_ = ros::Time(0);
        pix_integral_x_ = pix_integral_y_ = 0.0f;
        last_pix_err_x_ = last_pix_err_y_ = 0.0f;
        ROS_WARN_THROTTLE(1.0, "投放标识检测丢失或超时，保持悬停");
        return;
    }

    // 临时方案的关键：
    // 真正对准的不是原始图像中心，而是“图像中心 + 安装偏差 + 投物点偏差”
    const float aim_center_x = IMG_CENTER_X + cfg_.drop_camera_bias_x_px + cfg_.drop_release_bias_x_px;
    const float aim_center_y = IMG_CENTER_Y + cfg_.drop_camera_bias_y_px + cfg_.drop_release_bias_y_px;

    float err_x = aim_center_x - current_detection_.center_x;
    float err_y = aim_center_y - current_detection_.center_y;
    float pixel_dist = std::sqrt(err_x * err_x + err_y * err_y);

    ros::Time now = ros::Time::now();
    float dt = (now - last_pid_control_time_).toSec();
    if (last_pid_control_time_.isZero())
        dt = 0.05f;
    last_pid_control_time_ = now;

    float vel_x, vel_y;
    getPixPidVel(err_x, err_y, dt, vel_x, vel_y);

    // 越接近靶心，速度越小，避免最后几像素来回抖
    if (pixel_dist < cfg_.drop_fine_pixel_radius)
    {
        vel_x *= cfg_.drop_fine_vel_scale;
        vel_y *= cfg_.drop_fine_vel_scale;
    }

    mavros_msgs::PositionTarget sp;
    sp.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
    sp.type_mask = 0b100111000011;
    sp.velocity.x = vel_y;
    sp.velocity.y = -vel_x;
    sp.velocity.z = 0;
    sp.yaw = init_yaw_;
    sendSetpoint(sp);

    ROS_INFO_THROTTLE(0.5,
                      "[投放区对准] 像素误差: %.1f px, 有效中心:(%.1f, %.1f), 机体速度: %.2f m/s",
                      pixel_dist, aim_center_x, aim_center_y, getHorizontalSpeed());

    const bool ready_to_drop = pixel_dist < cfg_.align_pixel_threshold &&
                               isDropWindowStable(init_pos_z_ + wp_drop_area_.z);
    if (!ready_to_drop)
    {
        drop_alignment_hold_start_ = ros::Time(0);
        return;
    }

    // 第一次满足条件时只记时间，不立刻切状态
    if (drop_alignment_hold_start_.isZero())
    {
        drop_alignment_hold_start_ = now;
        return;
    }

    if ((now - drop_alignment_hold_start_).toSec() >= cfg_.drop_align_hold_time)
    {
        ROS_INFO("投放区对准完成，进入投放");
        current_state_ = DROP_SUPPLY;
        state_start_time_ = ros::Time::now();
        drop_alignment_hold_start_ = ros::Time(0);
        pix_integral_x_ = pix_integral_y_ = 0.0f;
        last_pix_err_x_ = last_pix_err_y_ = 0.0f;
    }
}

// ============================================================================
// 8. 替换 handleDropSupply()
// 作用：
// - 进入投放状态后，根据参数决定是否先下落到更低高度
// - 下落距离为0时，直接在当前高度执行投放
// - 投完后会自动回升到进入投放状态时的高度
// - 舵机接口先保留 TODO 说明，后面直接替换这一段即可
// ============================================================================
void MissionManager::handleDropSupply()
{
    static bool dropped = false;
    static bool drop_profile_initialized = false;
    static int drop_phase = 0; // 0=下降, 1=释放, 2=回升
    static float hold_x = 0.0f;
    static float hold_y = 0.0f;
    static float cruise_z = 0.0f;
    static float release_z = 0.0f;

    const ros::Time now = ros::Time::now();

    if (!drop_profile_initialized)
    {
        hold_x = local_odom_.pose.pose.position.x;
        hold_y = local_odom_.pose.pose.position.y;
        cruise_z = local_odom_.pose.pose.position.z;
        release_z = cruise_z;

        if (cfg_.drop_descend_distance > 0.0f)
        {
            // 给一个最低高度保护，避免参数填太大导致离地过低。
            release_z = std::max(init_pos_z_ + 0.20f, cruise_z - cfg_.drop_descend_distance);
        }

        drop_phase = (cfg_.drop_descend_distance > 0.0f && (cruise_z - release_z) > 1e-3f) ? 0 : 1;
        dropped = false;
        drop_profile_initialized = true;
        state_start_time_ = now;

        if (drop_phase == 0)
        {
            ROS_INFO("投放阶段初始化: 先下降 %.2f m 后投放", cruise_z - release_z);
        }
        else
        {
            ROS_INFO("投放阶段初始化: 下落距离为0，直接执行投放");
        }
    }

    mavros_msgs::PositionTarget sp;
    sp.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    sp.type_mask = 0b101111111000;
    sp.position.x = hold_x;
    sp.position.y = hold_y;
    sp.yaw = init_yaw_;

    if (drop_phase == 0)
    {
        sp.position.z = release_z;
        sendSetpoint(sp);

        ROS_INFO_THROTTLE(0.5, "[投放下降] 当前高度: %.2f m, 目标高度: %.2f m",
                          local_odom_.pose.pose.position.z, release_z);

        const bool reached_descend_height =
            std::abs(local_odom_.pose.pose.position.z - release_z) < cfg_.hover_vert_tolerance;
        const bool stable_at_release_height =
            getHorizontalSpeed() < cfg_.drop_release_max_horiz_speed &&
            std::abs(local_odom_.twist.twist.linear.z) < cfg_.drop_release_max_vert_speed &&
            std::abs(current_roll_) < cfg_.drop_max_tilt &&
            std::abs(current_pitch_) < cfg_.drop_max_tilt;

        if (reached_descend_height && stable_at_release_height)
        {
            drop_phase = 1;
            state_start_time_ = now;
            ROS_INFO("已下降到投放高度，准备释放");
        }
        return;
    }

    if (drop_phase == 1)
    {
        sp.position.z = release_z;
        sendSetpoint(sp);

        if (!isDropWindowStable(release_z))
        {
            ROS_WARN_THROTTLE(1.0, "投放窗口不稳定，继续等待速度和姿态收敛");
            return;
        }

        if (!dropped)
        {
            // TODO: 这里替换成已封装好的舵机释放接口，例如 drop_controller_.releaseOnce();
            // 当前先沿用触发话题作为占位，保证整体流程先能跑通。
            std_msgs::Bool trigger;
            trigger.data = true;
            drop_trigger_pub_.publish(trigger);
            ROS_INFO("物资投放指令已发送");
            dropped = true;
            state_start_time_ = now;
            return;
        }

        if ((now - state_start_time_).toSec() > 1.0)
        {
            drop_phase = 2;
            state_start_time_ = now;
            ROS_INFO("投放完成，开始回升");
        }
        return;
    }

    sp.position.z = cruise_z;
    sendSetpoint(sp);

    ROS_INFO_THROTTLE(0.5, "[投放回升] 当前高度: %.2f m, 目标高度: %.2f m",
                      local_odom_.pose.pose.position.z, cruise_z);

    const bool reached_cruise_height =
        std::abs(local_odom_.pose.pose.position.z - cruise_z) < cfg_.hover_vert_tolerance;
    const bool stable_after_ascend =
        getHorizontalSpeed() < cfg_.drop_release_max_horiz_speed &&
        std::abs(local_odom_.twist.twist.linear.z) < cfg_.drop_release_max_vert_speed &&
        std::abs(current_roll_) < cfg_.drop_max_tilt &&
        std::abs(current_pitch_) < cfg_.drop_max_tilt;

    if (reached_cruise_height && stable_after_ascend)
    {
        dropped = false;
        drop_profile_initialized = false;
        drop_phase = 0;
        current_state_ = MOVE_TO_ATTACK_AREA;
        nav_goal_sent_ = false;
        state_start_time_ = now;
        ROS_INFO("投放完成并已回升，前往攻击目标识别区");
    }
}
