#!/bin/zsh

# ============================================
# 无人机竞赛 tmux 任务启动脚本 (均分四宫格版)
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
echo "  无人机竞赛任务启动中..."
echo "======================================"

# ---------------------------------------------------------
# 窗口 0：核心与仿真 (左右分屏)
# 左: roscore | 右: sim.launch
# ---------------------------------------------------------
tmux new-session -d -s $SESSION -n "Core_Sim"

# 左侧: roscore
tmux send-keys -t $SESSION:0 "source $WS/devel/setup.zsh; roscore" C-m

# 左右分屏 (右侧: sim.launch)
tmux split-window -h -t $SESSION:0
tmux send-keys -t $SESSION:0 "sleep 3; \
source $SIM/devel/setup.zsh; \
source $PX4_PATH/Tools/setup_gazebo.zsh $PX4_PATH $PX4_PATH/build/px4_sitl_default; \
export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:$PX4_PATH:$PX4_PATH/Tools/sitl_gazebo; \
roslaunch tutorial_gazebo sim.launch" C-m


# ---------------------------------------------------------
# 窗口 1：主控、监控与视觉 (四等分 2x2)
# 布局:
# [ 左上: 状态监控 1 ] | [ 右上: 主控 ]
# -------------------------------------
# [ 左下: 状态监控 2 ] | [ 右下: YOLO ]
# ---------------------------------------------------------
tmux new-window -t $SESSION -n "Control_Vision"

# 步骤 1 (左上角)：状态监控 1
tmux send-keys -t $SESSION:1 "sleep 18; rostopic echo /mavros/local_position/pose" C-m

# 步骤 2：左右对半切分，焦点转到右半边 (右上角)，启动主控
tmux split-window -h -t $SESSION:1
tmux send-keys -t $SESSION:1 "sleep 10; source $WS/devel/setup.zsh; roslaunch main_control main_control.launch" C-m

# 步骤 3：将右半边上下切分 (右下角)，启动 YOLO
tmux split-window -v -t $SESSION:1
# tmux send-keys -t $SESSION:1 "sleep 16; conda run -n yolov11n --no-capture-output zsh -c 'export PYTHONPATH=~/cv_bridge_ws/devel/lib/python3/dist-packages:$PYTHONPATH && source $WS/devel/setup.zsh && roslaunch raicom_vision_laser raicom_vision_laser.launch'" C-m
tmux send-keys -t $SESSION:1 "sleep 16; source $WS/devel/setup.zsh; roslaunch raicom_vision_laser raicom_ocr_laser.launch" C-m

# 步骤 4：通过相对方向焦点回到最左侧，将左半边上下切分 (左下角)，启动状态监控 2
tmux select-pane -L -t $SESSION:1
tmux split-window -v -t $SESSION:1
tmux send-keys -t $SESSION:1 "sleep 18; rostopic echo /ego_controller/status" C-m

# 步骤 5：强制应用平铺布局，确保四个方块绝对均等
tmux select-layout -t $SESSION:1 tiled


# ---------------------------------------------------------
# 窗口 2：感知与导航 (左右分屏)
# 左: PCL 点云检测 | 右: Ego-Planner 导航
# ---------------------------------------------------------
tmux new-window -t $SESSION -n "Perception_Nav"

# 左侧: PCL
tmux send-keys -t $SESSION:2 "sleep 12; source $WS/devel/setup.zsh; roslaunch pcl_detection2 pcl_detection2.launch" C-m

# 左右分屏 (右侧: Nav)
tmux split-window -h -t $SESSION:2
tmux send-keys -t $SESSION:2 "sleep 14; \
source $WS/devel/setup.zsh; \
source ~/ego_ws/devel/setup.zsh --extend; \
roslaunch uav_navigation ego_nav.launch" C-m


# ============================================
# 完成配置并附加会话
# ============================================
# 回到四等分的窗口，方便直接观察整体状态
tmux select-window -t $SESSION:1
tmux attach-session -t $SESSION