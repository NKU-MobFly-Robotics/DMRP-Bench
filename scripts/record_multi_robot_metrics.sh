#!/bin/bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"   
BAG_DIR="${SCRIPT_DIR}/../bags"                             
mkdir -p "${BAG_DIR}"                                         

DATE=$(date +%Y%m%d_%H%M%S)
BAG_NAME="multi_robot_metrics_${DATE}.bag"
BAG_PATH="${BAG_DIR}/${BAG_NAME}"

echo "📦 Recording rosbag to: ${BAG_PATH}"

rosbag record -O "${BAG_PATH}" \
/clock \
/tf \
/mapf_base/planning_time \
/mapf_base/mission_time \
/p1/odom /p2/odom /p3/odom /p4/odom \
/p1/RosAria/cmd_vel /p2/RosAria/cmd_vel /p3/RosAria/cmd_vel /p4/RosAria/cmd_vel \
/p1/move_base/result /p2/move_base/result /p3/move_base/result /p4/move_base/result \
/p1/move_base/status /p2/move_base/status /p3/move_base/status /p4/move_base/status \
/p1/move_base_benchmark/TebLocalPlannerROS/local_plan \
/p2/move_base_benchmark/TebLocalPlannerROS/local_plan \
/p3/move_base_benchmark/TebLocalPlannerROS/local_plan \
/p4/move_base_benchmark/TebLocalPlannerROS/local_plan \
/mapf_base/p1/plan \
/mapf_base/p2/plan \
/mapf_base/p3/plan \
/mapf_base/p4/plan \
/target_pose/F_Business_02 \
/target_pose/F_Medical_01 \
/target_pose/M_Medical_01 \
/target_pose/Police_Female_01 \
/target_pose/Police_Male_04 \
/target_pose/Police_Female_02 \
/target_pose/Police_Female_03 \
> /dev/null 2>&1 &
ROS_PID=$!
echo $ROS_PID > /tmp/rosbag_record.pid
echo "📋 rosbag PID saved: $ROS_PID"
# 等待 rosbag 保持运行
wait $ROS_PID
