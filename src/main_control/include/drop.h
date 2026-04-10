bool target_confirmed_ = false; // 是否已确认

// 8.8 悬停识别投放区标识（复用下视YOLO对准逻辑）
void MissionManager::handleHoverRecognizeDrop()
{
    if (!target_confirmed_)
    {
        mavros_msgs::PositionTarget sp;
        sp.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        sp.type_mask = 0b100111000011;
        sp.velocity.x = sp.velocity.y = sp.velocity.z = 0;
        sp.yaw = current_yaw_;
        sendSetpoint(sp);
        ROS_INFO_THROTTLE(1.0, "等待投放标识确认...");
        return;
    }

    if (!current_detection_.detected)
    {
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
    float pixel_dist = sqrt(err_x * err_x + err_y * err_y);

    ros::Time now = ros::Time::now();
    float dt = (now - last_pid_control_time_).toSec();
    if (last_pid_control_time_.isZero())
        dt = 0.05;
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

    ROS_INFO_THROTTLE(0.5, "[投放区对准] 像素误差: %.1f px", pixel_dist);

    if (pixel_dist < cfg_.align_pixel_threshold)
    {
        ROS_INFO("投放区对准完成，开始投放");
        current_state_ = DROP_SUPPLY;
        state_start_time_ = ros::Time::now();
        pix_integral_x_ = pix_integral_y_ = 0;
    }
}

void MissionManager::handleDropSupply()
{
    static bool dropped = false;
    if (!dropped)
    {
        std_msgs::Bool trigger;
        trigger.data = true;
        drop_trigger_pub_.publish(trigger);
        ROS_INFO("物资投放指令已发送");
        dropped = true;
        state_start_time_ = ros::Time::now();
    }
    if ((ros::Time::now() - state_start_time_).toSec() > 1.0)
    {
        current_state_ = MOVE_TO_ATTACK_AREA;
        nav_goal_sent_ = false;
        state_start_time_ = ros::Time::now();
        ROS_INFO("投放完成，前往攻击目标识别区");
    }
}