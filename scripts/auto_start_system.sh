#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "${SCRIPT_DIR}")"  
SESSION="sim_auto"

# 检查旧的 tmux 会话是否存在，如果有就清理
if tmux has-session -t $SESSION 2>/dev/null; then
  echo "⚠️  tmux session '$SESSION' already exists. Killing it..."
  tmux kill-session -t $SESSION
fi

# 创建新会话，同时初始化第一个窗口为 odom_pub
tmux new-session -d -s $SESSION -n odom_pub

# 窗口 0：补发 odom
tmux send-keys -t $SESSION:0 "
source ${WORKSPACE_DIR}/devel/setup.bash
python3 ${SCRIPT_DIR}/tf_to_odom_publisher.py
" C-m

# 窗口 1：启动 move_base_benchmark
tmux new-window -t $SESSION -n benchmark
tmux send-keys -t $SESSION:1 "source ${WORKSPACE_DIR}/devel/setup.bash && roslaunch move_base_benchmark aa.launch scene:=Test" C-m
sleep 4

# 窗口 2：启动 mapf_base
tmux new-window -t $SESSION -n mapf
tmux send-keys -t $SESSION:2 "source ${WORKSPACE_DIR}/devel/setup.bash && roslaunch mapf_base mapf_example.launch" C-m
sleep 2

# 窗口 3：发布目标点 & 启动规划
tmux new-window -t $SESSION -n goals
tmux send-keys -t $SESSION:3 "
source ${WORKSPACE_DIR}/devel/setup.bash
rostopic pub --once /mapf_base/p1/goal geometry_msgs/PoseStamped \
\"header:
  frame_id: 'map'
pose:
  position:
    x: -8.0
    y: 8.0
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.707
    w: 0.707\"

rostopic pub --once /mapf_base/p2/goal geometry_msgs/PoseStamped \
\"header:
  frame_id: 'map'
pose:
  position:
    x: -10.0
    y: -1.0
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.707
    w: -0.707\"

rostopic pub --once /mapf_base/p3/goal geometry_msgs/PoseStamped \
\"header:
  frame_id: 'map'
pose:
  position:
    x: -3.0
    y: 0.0
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.707
    w: -0.707\"

rostopic pub --once /mapf_base/p4/goal geometry_msgs/PoseStamped \
\"header:
  frame_id: 'map'
pose:
  position:
    x: -3.5
    y: -14.0
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.707
    w: 0.707\"
    
rostopic pub --once /mapf_base/p5/goal geometry_msgs/PoseStamped \
\"header:
  frame_id: 'map'
pose:
  position:
    x: 5.0
    y: -13.0
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.707
    w: 0.707\"
    
rostopic pub --once /mapf_base/p6/goal geometry_msgs/PoseStamped \
\"header:
  frame_id: 'map'
pose:
  position:
    x: 4.0
    y: 2.0
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0\"

sleep 1
rostopic pub --once /mapf_base/goal_init_flag std_msgs/Bool \"data: true\"
" C-m


# 窗口 4：rosbag record，bag
tmux new-window -t $SESSION -n rosbag
tmux send-keys -t $SESSION:4 "
cd ${WORKSPACE_DIR}
bash ${SCRIPT_DIR}/record_multi_robot_metrics.sh
" C-m

