# 投放逻辑改动汇总

这份文档把本次改动集中整理出来，便于直接发给需要合并代码的人。

- 非 `launch` 的代码改动，已经全部整理进 [drop.h](/home/jetson/raicom/src/main_control/include/drop.h)。
- `launch` 改动保留在 [main_control.launch](/home/jetson/raicom/src/main_control/launch/main_control.launch)。
- 本文同时把两部分都列出来，方便统一查看。

## 1. 非 Launch 改动

### 1.1 需要补充的头文件

```cpp
#include <algorithm>
#include <limits>
```

### 1.2 需要补充到 MissionManager 类内的成员变量/参数/函数声明

```cpp
// ---------- 无人机状态：缓存当前横滚/俯仰，避免每次都重复解四元数 ----------
double current_roll_ = 0.0;
double current_pitch_ = 0.0;

// ---------- PID控制相关：满足投放条件后，保持一小段时间再释放 ----------
ros::Time drop_alignment_hold_start_;

// ---------- Config：投放阶段新增参数 ----------
float drop_arrive_threshold;
float drop_detect_timeout;
float drop_align_hold_time;
float drop_release_max_horiz_speed;
float drop_release_max_vert_speed;
float drop_max_tilt;
float drop_camera_bias_x_px;
float drop_camera_bias_y_px;
float drop_release_bias_x_px;
float drop_release_bias_y_px;
float drop_fine_pixel_radius;
float drop_fine_vel_scale;
float drop_descend_distance;

// ---------- 控制辅助函数声明 ----------
float getHorizontalSpeed() const;
bool isDropWindowStable(float target_z) const;
```

### 1.3 loadParameters() 新增参数

```cpp
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
```

### 1.4 odomCallback() 姿态缓存替换片段

```cpp
tf::Matrix3x3(q).getRPY(current_roll_, current_pitch_, current_yaw_);
```

### 1.5 替换 getPixPidVel()

```cpp
void MissionManager::getPixPidVel(float err_x, float err_y, float dt, float &vel_x, float &vel_y)
{
    dt = std::max(dt, 0.02f);

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
```

### 1.6 新增 helper

```cpp
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
```

### 1.7 替换 handleGoToDropArea()

```cpp
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

        if (front_camera_active_)
            callSwitchCamera();
        callResetTarget();

        last_pid_control_time_ = ros::Time(0);
        drop_alignment_hold_start_ = ros::Time(0);
        pix_integral_x_ = pix_integral_y_ = 0.0f;
        last_pix_err_x_ = last_pix_err_y_ = 0.0f;
        ROS_INFO("到达投放区，开始下视识别投放标识");
    }
}
```

### 1.8 替换 handleHoverRecognizeDrop()

```cpp
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
```

### 1.9 替换 handleDropSupply()

```cpp
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
            release_z = std::max(init_pos_z_ + 0.20f, cruise_z - cfg_.drop_descend_distance);
        }

        drop_phase = (cfg_.drop_descend_distance > 0.0f && (cruise_z - release_z) > 1e-3f) ? 0 : 1;
        dropped = false;
        drop_profile_initialized = true;
        state_start_time_ = now;
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
            dropped = true;
            state_start_time_ = now;
            return;
        }

        if ((now - state_start_time_).toSec() > 1.0)
        {
            drop_phase = 2;
            state_start_time_ = now;
        }
        return;
    }

    sp.position.z = cruise_z;
    sendSetpoint(sp);

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
```

## 2. Launch 改动

下面这些参数已经加到 [main_control.launch](/home/jetson/raicom/src/main_control/launch/main_control.launch) 里了：

```xml
<!-- 投放阶段调参 -->
<param name="drop_arrive_threshold" value="0.35" />
<param name="drop_detect_timeout" value="0.30" />
<param name="drop_align_hold_time" value="0.35" />
<param name="drop_release_max_horiz_speed" value="0.12" />
<param name="drop_release_max_vert_speed" value="0.06" />
<param name="drop_max_tilt" value="0.08" />

<!-- 经验像素偏置：先从 0 开始，实飞后慢慢调 -->
<param name="drop_camera_bias_x_px" value="0.0" />
<param name="drop_camera_bias_y_px" value="0.0" />
<param name="drop_release_bias_x_px" value="0.0" />
<param name="drop_release_bias_y_px" value="0.0" />

<param name="drop_fine_pixel_radius" value="35.0" />
<param name="drop_fine_vel_scale" value="0.45" />
<param name="drop_descend_distance" value="0.0" />
```

## 3. 调参建议

- 第一次试飞时，把四个偏置参数都先设成 `0`。
- 先调 `drop_camera_bias_x_px / drop_camera_bias_y_px`，让“视觉中心”接近真实正下方。
- 再调 `drop_release_bias_x_px / drop_release_bias_y_px`，补偿投物机构相对相机的固定偏差。
- 如果最后几像素来回抖，就把 `drop_fine_vel_scale` 再调小一点。
- 如果已经对准了但总进不了投放，先放宽 `drop_release_max_horiz_speed` 和 `drop_align_hold_time`。
- 如果想加大命中把握，可以逐步增大 `drop_descend_distance`；如果设成 `0`，就会跳过下落，直接投放。
