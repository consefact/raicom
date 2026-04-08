#!/bin/zsh

# ============================================
# 简单 tmux 任务启动脚本
# ============================================
export LANG=zh_CN.UTF-8
export LC_ALL=zh_CN.UTF-8

SESSION="mission"
WS=~/raicom_ws
SIM=~/catkin_ws
PX4_PATH=~/Libraries/PX4-Autopilot

# 清理旧会话
tmux kill-session -t $SESSION 2>/dev/null
sleep 1

echo "======================================"
echo "  无人机竞赛任务启动"
echo "======================================"

# 创建新会话，第一个窗口用于 roscore
tmux new-session -d -s $SESSION -n "roscore"
tmux send-keys -t $SESSION:0 "source $WS/devel/setup.zsh; \
export LANG=zh_CN.UTF-8; export LC_ALL=zh_CN.UTF-8; \
roscore" C-m

# 窗口1：仿真环境 sim.launch（需等待 roscore 启动）
tmux new-window -t $SESSION -n "sim"
tmux send-keys -t $SESSION:1 "sleep 3; \
source $SIM/devel/setup.zsh; \
source $PX4_PATH/Tools/setup_gazebo.zsh $PX4_PATH $PX4_PATH/build/px4_sitl_default; \
export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:$PX4_PATH:$PX4_PATH/Tools/sitl_gazebo; \
roslaunch tutorial_gazebo sim.launch" C-m

# 窗口2：主控状态机
tmux new-window -t $SESSION -n "main"
tmux send-keys -t $SESSION:2 "sleep 10; source $WS/devel/setup.zsh; \
export LANG=zh_CN.UTF-8; export LC_ALL=zh_CN.UTF-8; \
roslaunch main_control main_control.launch" C-m

# 窗口3：PCL 点云检测
tmux new-window -t $SESSION -n "pcl"
tmux send-keys -t $SESSION:3 "sleep 12; source $WS/devel/setup.zsh; \
export LANG=zh_CN.UTF-8; export LC_ALL=zh_CN.UTF-8; \
roslaunch pcl_detection2 pcl_detection2.launch" C-m

# 窗口4：Ego-Planner 导航
tmux new-window -t $SESSION -n "nav"
tmux send-keys -t $SESSION:4 "sleep 14; \
source $WS/devel/setup.zsh; \
source ~/ego_ws/devel/setup.zsh --extend; \
export LANG=zh_CN.UTF-8; export LC_ALL=zh_CN.UTF-8; \
roslaunch uav_navigation ego_nav.launch" C-m

# 窗口5：YOLO 视觉
tmux new-window -t $SESSION -n "yolo"
tmux send-keys -t $SESSION:5 "sleep 16; \
source $WS/devel/setup.zsh; \
roslaunch raicom_vision_laser raicom_vision_laser.launch" C-m

# 窗口6：状态监控
tmux new-window -t $SESSION -n "monitor"
tmux split-window -v -t $SESSION:6
tmux send-keys -t $SESSION:6.0 "sleep 18; rostopic echo /mavros/local_position/pose" C-m
tmux send-keys -t $SESSION:6.1 "sleep 18; rostopic echo /ego_controller/status" C-m
tmux select-layout -t $SESSION:6 tiled

# 回到监控窗口，附加会话
tmux select-window -t $SESSION:6
tmux attach-session -t $SESSION