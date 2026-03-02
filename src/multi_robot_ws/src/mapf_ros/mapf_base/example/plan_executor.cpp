#include <cmath>
#include <fstream>
#include <string>
#include <vector>
#include <mutex>
#include <atomic>
#include "ros/package.h"
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Bool.h"
#include "tf/tf.h"
#include "actionlib/client/simple_action_client.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "mapf_msgs/GlobalPlan.h"
#include "mapf_msgs/SinglePlan.h"
#include "mapf_ros/utils/utility.hpp"
#include "mapf_ros/utils/spline.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

class ParamServer {
public:
    ros::NodeHandle ps_nh_;
    int agent_num_;
    std::vector<std::string> agent_name_;
    std::string global_frame_id_;
    std::vector<std::string> base_frame_id_;
    std::vector<std::string> plan_topic_;

    ParamServer() {
        ps_nh_.param<int>("agent_num", agent_num_, 1);
        ps_nh_.param<std::string>("global_frame_id", global_frame_id_, "map");
        agent_name_.resize(agent_num_);
        base_frame_id_.resize(agent_num_);
        plan_topic_.resize(agent_num_);

        for (int i = 0; i < agent_num_; ++i) {
            ps_nh_.param<std::string>("base_frame_id/agent_" + std::to_string(i),
                base_frame_id_[i], "base_link");
            ps_nh_.param<std::string>("plan_topic/agent_" + std::to_string(i),
                plan_topic_[i], "plan");
            ps_nh_.param<std::string>("agent_name/agent_" + std::to_string(i),
                agent_name_[i], "agent_name_0");
        }
    }
};

class PlanExecutor : public ParamServer {
private:
    mutable std::mutex plan_mtx_;
    mutable std::mutex goals_mtx_;
    ros::NodeHandle nh_;
    ros::Subscriber sub_mapf_plan_;
    ros::Publisher smoothed_global_plan_pub_;
    std::vector<ros::Publisher> viz_path_publishers_;  // 用于rviz可视化的发布器
    std::vector<ros::Publisher> path_publishers_; // 添加路径发布器
    bool waiting_for_plans_;  // 添加此变量
    std::vector<bool> goals_waiting_;  // 添加此变量，用于跟踪等待规划的目标点
    int make_span_;
    std::vector<mapf_msgs::SinglePlan> plan_arr_;
    std::atomic<bool> get_plan_;
    
    typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseActionClient;
    typedef std::shared_ptr<MoveBaseActionClient> MoveBaseActionClientPtr;
    std::vector<MoveBaseActionClientPtr> ac_ptr_arr_;
    
    std::vector<ros::Subscriber> goal_subs_;
    std::vector<geometry_msgs::PoseStamped> original_goals_;
    std::vector<bool> received_goals_;
    
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    
    static constexpr double POSITION_TOLERANCE = 0.1;
    static constexpr double ANGLE_TOLERANCE = 0.1;

public:
    PlanExecutor() : ParamServer(), 
        make_span_(0),
        get_plan_(false),
        ac_ptr_arr_(agent_num_, nullptr),
        tf_listener_(tf_buffer_)
    {
        // 初始化向量
        plan_arr_.resize(agent_num_);
        ac_ptr_arr_.resize(agent_num_);
        goal_subs_.resize(agent_num_);
        original_goals_.resize(agent_num_);
        received_goals_.resize(agent_num_, false);
        viz_path_publishers_.resize(agent_num_);  // 初始化可视化发布器数组
        path_publishers_.resize(agent_num_); // 初始化发布器数组
        goals_waiting_.resize(agent_num_, false);

        // 初始化发布器和订阅器
        smoothed_global_plan_pub_ = nh_.advertise<mapf_msgs::GlobalPlan>(
            "/mapf_base/smoothed_global_plan", 1, true);

        // 初始化rviz可视化发布器
        for (int i = 0; i < agent_num_; ++i) {
            // 使用与原mapf_base相同的话题名
            std::string viz_topic = "/mapf_base/" + agent_name_[i] + "/plan";
            viz_path_publishers_[i] = nh_.advertise<nav_msgs::Path>(
                viz_topic, 1, false);
            ROS_ERROR("Created visualization publisher for agent %d on topic: %s", 
                     i, viz_topic.c_str());
        }

        // 初始化action clients和订阅器
        for (int i = 0; i < agent_num_; ++i) {
            std::string move_base_name = "/" + agent_name_[i] + "/move_base";
            ac_ptr_arr_[i] = std::make_shared<MoveBaseActionClient>(move_base_name, true);

            // 等待action server
            ROS_INFO("Waiting for move_base action server %s...", move_base_name.c_str());
            ac_ptr_arr_[i]->waitForServer(ros::Duration(5.0));

            // 订阅目标点话题
            std::string goal_topic = "/mapf_base/" + agent_name_[i] + "/goal";
            int idx = i;
            goal_subs_[i] = nh_.subscribe<geometry_msgs::PoseStamped>(
                goal_topic, 1,
                [this, idx](const geometry_msgs::PoseStamped::ConstPtr& msg) {
                    std::lock_guard<std::mutex> lock(goals_mtx_);
                    original_goals_[idx] = *msg;
                    received_goals_[idx] = true;
                    goals_waiting_[idx] = true;  // 标记为等待规划
                    ROS_ERROR("Received goal for agent %d, waiting for MAPF planning", idx);
                    //sendFinalGoal(idx);
                });

        }

        // 订阅全局规划
        sub_mapf_plan_ = nh_.subscribe<mapf_msgs::GlobalPlan>(
            "global_plan", 1, &PlanExecutor::planCallback, this);
    }
    
    ~PlanExecutor() {
    ROS_INFO("PlanExecutor destructor called");
    
    try {
        // 安全地清理订阅器
        for (auto& sub : goal_subs_) {
            sub.shutdown();
        }
        sub_mapf_plan_.shutdown();

        // 安全地清理发布器
        for (auto& pub : viz_path_publishers_) {
            pub.shutdown();
        }
        smoothed_global_plan_pub_.shutdown();

        // 清理其他资源
        plan_arr_.clear();
        original_goals_.clear();
        received_goals_.clear();
        goals_waiting_.clear();

        ROS_INFO("PlanExecutor cleanup completed");
    }
    catch (const std::exception& e) {
        ROS_ERROR("Exception during cleanup: %s", e.what());
    }
  }
  
mapf_msgs::SinglePlan smoothPathWithSpline(const mapf_msgs::SinglePlan& input_plan, double sampling_step = 0.1) {
    mapf_msgs::SinglePlan smoothed_plan;
    const nav_msgs::Path& input_path = input_plan.plan;

    if (input_path.poses.size() < 4) {
        ROS_WARN("Path has too few points to smooth, returning original plan.");
        return input_plan;
    }
    smoothed_plan.plan.header = input_path.header;

    // --- 步骤 1: 提取x, y, 和 time_step 数据 ---
    std::vector<double> x_points, y_points, t_params, time_step_values;
    t_params.push_back(0.0);
    x_points.push_back(input_path.poses[0].pose.position.x);
    y_points.push_back(input_path.poses[0].pose.position.y);
    time_step_values.push_back(input_plan.time_step[0]);
    double cumulative_distance = 0.0;

    for (size_t i = 1; i < input_path.poses.size(); ++i) {
        double dx = input_path.poses[i].pose.position.x - input_path.poses[i-1].pose.position.x;
        double dy = input_path.poses[i].pose.position.y - input_path.poses[i-1].pose.position.y;
        cumulative_distance += std::sqrt(dx*dx + dy*dy);
        
        t_params.push_back(cumulative_distance);
        x_points.push_back(input_path.poses[i].pose.position.x);
        y_points.push_back(input_path.poses[i].pose.position.y);
        time_step_values.push_back(input_plan.time_step[i]);
    }

    // --- 步骤 2: 为 x, y, 和 time_step 创建样条曲线 ---
    tk::spline sx, sy, st;
    sx.set_points(t_params, x_points);
    sy.set_points(t_params, y_points);
    st.set_points(t_params, time_step_values); // 为时间步创建样条

    // --- 步骤 3: 重新离散化，并插值所有值 ---
    for (double t = 0.0; t < cumulative_distance; t += sampling_step) {
        geometry_msgs::PoseStamped pose;
        pose.header = input_path.header;
        pose.pose.position.x = sx(t);
        pose.pose.position.y = sy(t);
        pose.pose.position.z = 0;
        
        smoothed_plan.plan.poses.push_back(pose);
        
        // 插值时间步，并四舍五入到最近的整数
        smoothed_plan.time_step.push_back(std::round(st(t)));
    }
    
    // 添加最后一个点，确保路径和时间步完整
    smoothed_plan.plan.poses.push_back(input_path.poses.back());
    smoothed_plan.time_step.push_back(input_plan.time_step.back());

    // --- 步骤 4: 计算新路径点的方向 ---
    if (smoothed_plan.plan.poses.size() > 1) {
        for (size_t i = 0; i < smoothed_plan.plan.poses.size() - 1; ++i) {
            double dx = smoothed_plan.plan.poses[i+1].pose.position.x - smoothed_plan.plan.poses[i].pose.position.x;
            double dy = smoothed_plan.plan.poses[i+1].pose.position.y - smoothed_plan.plan.poses[i].pose.position.y;
            double yaw = std::atan2(dy, dx);
            tf2::Quaternion q;
            q.setRPY(0, 0, yaw);
            smoothed_plan.plan.poses[i].pose.orientation = tf2::toMsg(q);
        }
        smoothed_plan.plan.poses.back().pose.orientation = smoothed_plan.plan.poses[smoothed_plan.plan.poses.size()-2].pose.orientation;
    }

    return smoothed_plan;
}

void processPath(mapf_msgs::SinglePlan& single_plan) {
    if (single_plan.plan.poses.size() < 2) return;

    // 保存原始终点姿态
    auto final_pose = single_plan.plan.poses.back();
    
    tf2::Quaternion q0;
    q0.setRPY(0, 0, 1.571);
    single_plan.plan.poses[0].pose.orientation = tf2::toMsg(q0);
    
    const double EPSILON = 0.001; //添加一个阈值
    
    // 每隔一个点取一个，确保包含第一个点
    for (size_t i = 1; i < single_plan.plan.poses.size() - 1; i++) {
        // 计算到下一个点的方向
        double dx = single_plan.plan.poses[i].pose.position.x - single_plan.plan.poses[i-1].pose.position.x;
        double dy = single_plan.plan.poses[i].pose.position.y - single_plan.plan.poses[i-1].pose.position.y;
        double yaw ;  //= std::atan2(dy, dx);
        if (std::abs(dx) > std::abs(dy)) {
            // 主要是水平移动
            yaw = (dx > 0.05) ? 0.0 : M_PI;
        } else {
            // 主要是垂直移动
            yaw = (dy > 0.05) ? M_PI/2 : -M_PI/2;
        }
        
        // 设置方向四元数
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);
        single_plan.plan.poses[i].pose.orientation = tf2::toMsg(q);
    }
    
    // 最后一个点使用原始目标姿态
    single_plan.plan.poses.back() = final_pose;
}


    void planCallback(const mapf_msgs::GlobalPlan::ConstPtr& mapf_global_plan) {
        ROS_ERROR("Entering planCallback");  // 调试信息
        std::lock_guard<std::mutex> lock(plan_mtx_);
        make_span_ = mapf_global_plan->makespan;

      if (!mapf_global_plan) {
          ROS_WARN("Received null global plan");
          return;
      }

      if (mapf_global_plan->global_plan.empty()) {
          ROS_WARN("Received empty global plan");
          return;
      }

      if (mapf_global_plan->global_plan.size() != agent_num_) {
          ROS_ERROR("Received plan size (%zu) does not match agent_num_ (%d)", 
                    mapf_global_plan->global_plan.size(), agent_num_);
          return;
      }

        // 创建平滑后的全局规划消息
        mapf_msgs::GlobalPlan smoothed_global_plan;
        smoothed_global_plan.makespan = mapf_global_plan->makespan;
        smoothed_global_plan.global_plan.resize(agent_num_);

        for (int i = 0; i < agent_num_; ++i) {
            ROS_ERROR("Processing agent %d", i);  // 调试信息
            const auto& original_single_plan = mapf_global_plan->global_plan[i];

            // 调试输出：检查原始路径信息
        ROS_ERROR("Agent %d - Original plan info:", i);
        ROS_ERROR("  Path frame_id: %s", original_single_plan.plan.header.frame_id.c_str());
        ROS_ERROR("  Number of poses: %zu", original_single_plan.plan.poses.size());
        if (!original_single_plan.plan.poses.empty()) {
            ROS_ERROR("  First pose: (%.3f, %.3f)", 
                     original_single_plan.plan.poses[0].pose.position.x,
                     original_single_plan.plan.poses[0].pose.position.y);
            ROS_ERROR("  Last pose: (%.3f, %.3f)", 
                     original_single_plan.plan.poses.back().pose.position.x,
                     original_single_plan.plan.poses.back().pose.position.y);
        }

        // 获取机器人当前位置
        try {
            geometry_msgs::TransformStamped transform = 
                tf_buffer_.lookupTransform("map", base_frame_id_[i], ros::Time(0));
            double current_x = transform.transform.translation.x;
            double current_y = transform.transform.translation.y;
            ROS_ERROR("Agent %d current position: (%.3f, %.3f)", i, current_x, current_y);
            
            // 计算与路径起点的距离
            if (!original_single_plan.plan.poses.empty()) {
                double start_x = original_single_plan.plan.poses[0].pose.position.x;
                double start_y = original_single_plan.plan.poses[0].pose.position.y;
                double dx = start_x - current_x;
                double dy = start_y - current_y;
                double dist = sqrt(dx*dx + dy*dy);
                ROS_ERROR("Agent %d: distance from current to path start: %.3f meters", i, dist);
                
                // 如果距离过大，发出警告
                if (dist > 1.0) {
                    ROS_WARN("Agent %d: Large distance between current position and path start!", i);
                }
            }
        } catch (tf2::TransformException &ex) {
            ROS_WARN("Could not get current position for agent %d: %s", i, ex.what());
        }
            
            //hzj-注释掉
            // mapf_msgs::SinglePlan smoothed_single_plan = smoothPathWithSpline(original_single_plan, 0.8);
            
            // smoothed_global_plan.global_plan[i] = smoothed_single_plan;
            mapf_msgs::SinglePlan smoothed_single_plan = original_single_plan; // 直接使用原始规划

            // 获取机器人当前位置
        try {
            geometry_msgs::TransformStamped transform = 
                tf_buffer_.lookupTransform("map", base_frame_id_[i], ros::Time(0));
            double current_x = transform.transform.translation.x;
            double current_y = transform.transform.translation.y;
            
            // 将当前位置作为路径的第一个点
            if (!smoothed_single_plan.plan.poses.empty()) {
                // 计算当前位置与原始路径起点的距离
                double start_x = smoothed_single_plan.plan.poses[0].pose.position.x;
                double start_y = smoothed_single_plan.plan.poses[0].pose.position.y;
                double dx = start_x - current_x;
                double dy = start_y - current_y;
                double dist = sqrt(dx*dx + dy*dy);
                
                // 如果距离大于阈值，修正路径起点
                if (dist > 0.1) { // 10厘米阈值
                    ROS_WARN("Agent %d: Correcting path start from (%.3f, %.3f) to (%.3f, %.3f), distance: %.3f", 
                            i, start_x, start_y, current_x, current_y, dist);
                    
                    // 将当前位置设置为路径起点
                    smoothed_single_plan.plan.poses[0].pose.position.x = current_x;
                    smoothed_single_plan.plan.poses[0].pose.position.y = current_y;
                    
                    // 如果路径中有多个点，还需要调整第一个点的方向
                    if (smoothed_single_plan.plan.poses.size() > 1) {
                        double next_dx = smoothed_single_plan.plan.poses[1].pose.position.x - current_x;
                        double next_dy = smoothed_single_plan.plan.poses[1].pose.position.y - current_y;
                        double yaw = std::atan2(next_dy, next_dx);
                        
                        tf2::Quaternion q;
                        q.setRPY(0, 0, yaw);
                        smoothed_single_plan.plan.poses[0].pose.orientation = tf2::toMsg(q);
                    }
                }
            }
        } catch (tf2::TransformException &ex) {
            ROS_WARN("Could not get current position for agent %d: %s", i, ex.what());
        }
        
            smoothed_global_plan.global_plan[i] = smoothed_single_plan;
        
                // 发布路径用于可视化
                nav_msgs::Path viz_path = smoothed_single_plan.plan;
                viz_path.header.frame_id = "map";
                viz_path.header.stamp = ros::Time::now();
                viz_path_publishers_[i].publish(viz_path);
                
            ROS_ERROR("Published smoothed visualization path for agent %d with %zu poses",
                     i, viz_path.poses.size());

                // 只有在收到MAPF规划后才发送目标点
                if (goals_waiting_[i]) {
                    sendFinalGoal(i);
                    goals_waiting_[i] = false;
                    ROS_ERROR("1111111111111111111111111111111");
                }
         ROS_ERROR("Finished smoothing for agent %d", i);  // 调试信息
        }
        
        ROS_ERROR("Exiting planCallback");  // 调试信息

        smoothed_global_plan_pub_.publish(smoothed_global_plan);
         waiting_for_plans_ = false;
    }

private:
    void sendFinalGoal(int agent_id) {
        if (!ac_ptr_arr_[agent_id] || !ac_ptr_arr_[agent_id]->isServerConnected()) {
            ROS_WARN("Move base action server for agent %d not connected", agent_id);
            return;
        }

        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose = original_goals_[agent_id];
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.header.frame_id = "map";

        ROS_INFO("Sending final goal to agent %d: position (%.2f, %.2f), orientation (%.2f, %.2f, %.2f, %.2f)",
            agent_id,
            goal.target_pose.pose.position.x,
            goal.target_pose.pose.position.y,
            goal.target_pose.pose.orientation.x,
            goal.target_pose.pose.orientation.y,
            goal.target_pose.pose.orientation.z,
            goal.target_pose.pose.orientation.w);

        ac_ptr_arr_[agent_id]->sendGoal(goal);
    }
};

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "plan_executor_node");
    
    try {
        ROS_INFO("Starting plan_executor_node");
        PlanExecutor plan_executor;
        ros::MultiThreadedSpinner spinner(2);
        ROS_INFO("Starting spinner");
        spinner.spin();
    }
    catch (const std::exception& e) {
        ROS_ERROR("Exception in plan_executor_node: %s", e.what());
        return 1;
    }
    
    catch (...) {
        ROS_ERROR("Unknown exception in plan_executor_node");
        return 1;
    }
    
    return 0;
}
