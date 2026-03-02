#include <pluginlib/class_list_macros.h>
#include "mapf_ros/aaasipp/aaasipp_ros.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

// Register this class as a ROS plugin
PLUGINLIB_EXPORT_CLASS(mapf::AAASIPPROS, mapf::MAPFROS)

namespace mapf {

AAASIPPROS::AAASIPPROS() : costmap_ros_(nullptr) {
}

AAASIPPROS::~AAASIPPROS() {
}

void AAASIPPROS::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
    ROS_INFO("Initializing AAASIPPROS plugin");
    costmap_ros_ = costmap_ros;
    ros::NodeHandle private_nh("~/");

    // Load parameters from ROS parameter server, replacing the functionality of your old config.xml
    private_nh.param<bool>("allowanyangle", config_.allowanyangle, CN_DEFAULT_ALLOWANYANGLE);
    private_nh.param<double>("timelimit", config_.timelimit, CN_DEFAULT_TIMELIMIT);
    private_nh.param<int>("connectedness", config_.connectedness, CN_DEFAULT_CONNECTEDNESS);
    private_nh.param<int>("rescheduling", config_.rescheduling, CN_DEFAULT_RESCHEDULING);
    private_nh.param<int>("initialprioritization", config_.initialprioritization, CN_DEFAULT_INITIALPRIORITIZATION);
    private_nh.param<bool>("planforturns", config_.planforturns, CN_DEFAULT_PLANFORTURNS);
    private_nh.param<double>("inflatecollisionintervals", config_.inflatecollisionintervals, CN_DEFAULT_INFLATEINTERVALS);
    
    // Create instances of your core objects
    map_ = std::make_unique<Map>();
    task_ = std::make_unique<Task>();
    shapes_ = std::make_unique<Shape>();
    dynamic_obstacles_ = std::make_unique<DynamicObstacles>();
    
    // // --- 新增：初始化专用发布器 ---
    // dedicated_plan_pub_ = private_nh.advertise<mapf_msgs::AAASIPPGlobalPlan>("aaasipp_global_plan", 1);

    // For simplicity, we assume one single agent shape.
    // In a more complex scenario, you could load multiple shapes from a parameter list.
    // We'll use a simple circle shape here, which is the default for most ROS planners.
    // If you need to support custom shapes, this is where you'd read a list of vertices from a parameter.
    // std::vector<Location> circle_shape;
    // circle_shape.push_back({0, 0}); // A simple point for a circle-like agent
    // shapes_->addShape(circle_shape);
    XmlRpc::XmlRpcValue shape_list;
    if (private_nh.getParam("agents", shape_list)) {
        ROS_ASSERT(shape_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
        for (int i = 0; i < shape_list.size(); ++i) {
            XmlRpc::XmlRpcValue& agent_entry = shape_list[i];
            if (agent_entry.getType() == XmlRpc::XmlRpcValue::TypeStruct && agent_entry.hasMember("shape")) {
                XmlRpc::XmlRpcValue& shape_vertices_list = agent_entry["shape"];
                ROS_ASSERT(shape_vertices_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
                std::vector<Location> vertices;
                for (int j = 0; j < shape_vertices_list.size(); ++j) {
                    XmlRpc::XmlRpcValue& vertex = shape_vertices_list[j];
                    ROS_ASSERT(vertex.getType() == XmlRpc::XmlRpcValue::TypeArray);
                    ROS_ASSERT(vertex.size() == 2);
                    // vertices.push_back({static_cast<double>(vertex[0]), static_cast<double>(vertex[1])});

                    double x, y;
                    if (vertex[0].getType() == XmlRpc::XmlRpcValue::TypeInt) {
                        x = static_cast<double>(static_cast<int>(vertex[0]));
                    } else if (vertex[0].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
                        x = static_cast<double>(vertex[0]);
                    } else {
                        ROS_ERROR("Invalid type for x coordinate of agent %d's shape vertex.", i);
                        continue; // 跳过此顶点
                    }

                    if (vertex[1].getType() == XmlRpc::XmlRpcValue::TypeInt) {
                        y = static_cast<double>(static_cast<int>(vertex[1]));
                    } else if (vertex[1].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
                        y = static_cast<double>(vertex[1]);
                    } else {
                        ROS_ERROR("Invalid type for y coordinate of agent %d's shape vertex.", i);
                        continue; // 跳过此顶点
                    }

                    vertices.push_back({x, y});
                }
                shapes_->shape_all.push_back(vertices);
            } else {
                ROS_ERROR("Agent entry at index %d in 'aaasipp_shapes_params.yaml' is invalid. Missing 'shape' member.", i);
            }
        }
    } else {
        ROS_WARN("No 'agents' parameter found. Make sure 'aaasipp_shapes_params.yaml' is loaded correctly.");
        // Fallback: add a simple point shape if no shapes are found
        std::vector<Location> point_shape;
        point_shape.push_back({0, 0});
        shapes_->shape_all.push_back(point_shape);
    }
    
    // Instantiate the core AA_SIPP planner
    planner_ = std::make_unique<AA_SIPP>(config_);

    ROS_INFO("AAASIPPROS plugin initialized successfully.");
}

bool AAASIPPROS::makePlan(const nav_msgs::Path &start, const nav_msgs::Path &goal,
                          mapf_msgs::GlobalPlan &plan, double &cost,
                          const double &time_tolerance)  {
    if (start.poses.empty() || goal.poses.empty() || costmap_ros_ == nullptr) {
        ROS_ERROR("Received empty start/goal path or costmap is not initialized.");
        return false;
    }

    // Step 1: Convert ROS map to AA-SIPP-m map
    loadMapFromROS();

    // Step 2: Convert ROS paths to AA-SIPP-m task
    loadTaskFromROS(start, goal);

    // Step 3: Handle dynamic obstacles. 
    // Since your original code reads from XML, we'll leave this empty for now.
    // In a real-world ROS application, you would subscribe to a topic for dynamic obstacles.
    // We'll skip this for the first implementation.
    
    // Step 4: Call the core AAA-SIPP-m planning algorithm
    ROS_INFO("Calling core AAA-SIPP-m planner for %lu agents...", task_->getNumberOfAgents());
    SearchResult result = planner_->startSearch(*map_, *task_, *dynamic_obstacles_, *shapes_);
    ROS_INFO("AAA-SIPP-m search finished in %f seconds.", result.runtime);

    if (!result.pathfound) {
        ROS_ERROR("AAA-SIPP-m failed to find a plan for this mission. Check the start/goal configurations or map validity.");
        // 由于 makePlan 必须返回 GlobalPlan，我们返回一个空的或默认的实例
        plan.makespan = -1;
        plan.global_plan.clear();
        return false;
    }

    ROS_INFO("AAASIPPROS successfully found a plan for %d out of %d agents. Makespan: %f, Flowtime: %f",
             result.agentsSolved, result.agents, result.makespan, result.flowtime);

    // Step 5: Convert AAA-SIPP-m results back to the standard GlobalPlan message
    plan.makespan = static_cast<int>(std::round(result.makespan));
    plan.global_plan.clear();

    // for (size_t i = 0; i < result.pathInfo.size(); ++i) {
    //     if (!result.pathInfo[i].pathfound) {
    //         mapf_msgs::SinglePlan empty_plan;
    //         plan.global_plan.push_back(empty_plan);
    //         continue;
    //     }

    //     mapf_msgs::SinglePlan single_plan;
    //     single_plan.plan.header.frame_id = costmap_ros_->getGlobalFrameID();
    //     single_plan.plan.header.stamp = ros::Time::now();
    //     single_plan.plan.poses.clear();
    //     single_plan.time_step.clear();
        
    //     for (const auto& node : result.pathInfo[i].path) {
    //         geometry_msgs::PoseStamped pose;
    //         pose.header.frame_id = single_plan.plan.header.frame_id;
    //         pose.header.stamp = single_plan.plan.header.stamp;
    //         pose.pose.position.x = costmap_ros_->getCostmap()->getOriginX() + (node.j + 0.5) * costmap_ros_->getCostmap()->getResolution();
    //         pose.pose.position.y = costmap_ros_->getCostmap()->getOriginY() + (node.i + 0.5) * costmap_ros_->getCostmap()->getResolution();

    //         tf2::Quaternion q;
    //         q.setRPY(0, 0, node.heading);
    //         pose.pose.orientation = tf2::toMsg(q);
            
    //         single_plan.plan.poses.push_back(pose);
    //         single_plan.time_step.push_back(static_cast<int>(std::round(node.g)));
    //     }
    //     plan.global_plan.push_back(single_plan);
    // }

    // 在makePlan函数中，替换原来的路径生成部分：

for (size_t i = 0; i < result.pathInfo.size(); ++i) {
    if (!result.pathInfo[i].pathfound) {
        mapf_msgs::SinglePlan empty_plan;
        plan.global_plan.push_back(empty_plan);
        continue;
    }

//     ROS_ERROR("Agent %zu: path found, sections size: %zu", i, result.pathInfo[i].sections.size());
    
//     if (result.pathInfo[i].sections.empty()) {
//         ROS_WARN("Agent %zu: sections is empty, using fallback path generation", i);
//         // 回退到原来的路径生成方式
//         mapf_msgs::SinglePlan single_plan;
//         single_plan.plan.header.frame_id = costmap_ros_->getGlobalFrameID();
//         single_plan.plan.header.stamp = ros::Time::now();
//         single_plan.plan.poses.clear();
//         single_plan.time_step.clear();
        
//         for (const auto& node : result.pathInfo[i].path) {
//             geometry_msgs::PoseStamped pose;
//             pose.header.frame_id = single_plan.plan.header.frame_id;
//             pose.header.stamp = single_plan.plan.header.stamp;
//             pose.pose.position.x = costmap_ros_->getCostmap()->getOriginX() + (node.j + 0.5) * costmap_ros_->getCostmap()->getResolution();
//             pose.pose.position.y = costmap_ros_->getCostmap()->getOriginY() + (node.i + 0.5) * costmap_ros_->getCostmap()->getResolution();

//             tf2::Quaternion q;
//             q.setRPY(0, 0, node.heading);
//             pose.pose.orientation = tf2::toMsg(q);
            
//             single_plan.plan.poses.push_back(pose);
//         }
//         plan.global_plan.push_back(single_plan);
//         continue;
//     }


    mapf_msgs::SinglePlan single_plan;
    single_plan.plan.header.frame_id = costmap_ros_->getGlobalFrameID();
    single_plan.plan.header.stamp = ros::Time::now();
    single_plan.plan.poses.clear();
    single_plan.time_step.clear(); // 清空时间步，不再使用
    
    // 使用新的转换函数，传入hppath（sections）
    auto continuous_path = convertActionSequenceToPath(
        result.pathInfo[i].sections, 
        costmap_ros_->getCostmap(),
        costmap_ros_->getCostmap()->getResolution()
    );
    
    // 设置路径点的时间戳（可选，用于表示顺序）
    ros::Time base_time = ros::Time::now();
    for (size_t j = 0; j < continuous_path.size(); ++j) {
        continuous_path[j].header.frame_id = single_plan.plan.header.frame_id;
        continuous_path[j].header.stamp = base_time + ros::Duration(j * 0.1); // 每点间隔0.1秒
        single_plan.plan.poses.push_back(continuous_path[j]);
    }
    
    plan.global_plan.push_back(single_plan);
}
    
    // // Step 5: Convert AAA-SIPP-m results back to ROS messages
    // // --- 新增：使用新消息类型填充数据并发布 ---
    // mapf_msgs::AAASIPPGlobalPlan aaasipp_plan;
    // aaasipp_plan.header.stamp = ros::Time::now();
    // aaasipp_plan.header.frame_id = costmap_ros_->getGlobalFrameID();
    // aaasipp_plan.makespan = result.makespan;
    // aaasipp_plan.flowtime = result.flowtime;
    // aaasipp_plan.success = result.pathfound;
    // aaasipp_plan.agent_plans.clear();

    // for (size_t i = 0; i < result.pathInfo.size(); ++i) {
    //     mapf_msgs::AAASIPPSinglePlan single_plan;
    //     single_plan.plan_found = result.pathInfo[i].pathfound;

    //     for (const auto& node : result.pathInfo[i].path) {
    //         geometry_msgs::PoseStamped pose;
    //         pose.header.frame_id = aaasipp_plan.header.frame_id;
    //         pose.header.stamp = aaasipp_plan.header.stamp;
    //         pose.pose.position.x = costmap_ros_->getCostmap()->getOriginX() + (node.j + 0.5) * costmap_ros_->getCostmap()->getResolution();
    //         pose.pose.position.y = costmap_ros_->getCostmap()->getOriginY() + (node.i + 0.5) * costmap_ros_->getCostmap()->getResolution();

    //         tf2::Quaternion q;
    //         q.setRPY(0, 0, node.heading);
    //         pose.pose.orientation = tf2::toMsg(q);
            
    //         single_plan.plan.poses.push_back(pose);
    //         single_plan.time_step.push_back(node.g);
    //     }
    //     aaasipp_plan.agent_plans.push_back(single_plan);
    // }
    
    // dedicated_plan_pub_.publish(aaasipp_plan);
    
// --- [调试代码开始]：检查最终的 GlobalPlan 内容 ---
    ROS_ERROR("--- Final Global Plan Debug (ROS) ---");
    ROS_ERROR("Makespan: %d", plan.makespan);
    ROS_ERROR("Total Agents (Plans): %zu", plan.global_plan.size());

    for (size_t i = 0; i < plan.global_plan.size(); ++i) {
        const mapf_msgs::SinglePlan& single_plan = plan.global_plan[i];
        ROS_ERROR("  Agent %zu Plan: %zu total poses (Expected > 4 points now)", i, single_plan.plan.poses.size());

        if (single_plan.plan.poses.empty()) {
            ROS_ERROR("  Agent %zu: No path found (empty plan).", i);
            continue;
        }

        // 打印起点和终点坐标
        const geometry_msgs::PoseStamped& start_pose = single_plan.plan.poses.front();
        const geometry_msgs::PoseStamped& end_pose = single_plan.plan.poses.back();
        
        ROS_ERROR("    Start Pos (World): (%.2f, %.2f)", start_pose.pose.position.x, start_pose.pose.position.y);
        ROS_ERROR("    End Pos (World): (%.2f, %.2f)", end_pose.pose.position.x, end_pose.pose.position.y);

        // 可选：打印路径中段的几个点 (以检查插值是否成功)
        if (single_plan.plan.poses.size() >= 3) {
            const geometry_msgs::PoseStamped& mid_pose = single_plan.plan.poses[single_plan.plan.poses.size() / 2];
            ROS_ERROR("    Mid Pos (World): (%.2f, %.2f) - Used %zu interpolation steps", mid_pose.pose.position.x, mid_pose.pose.position.y, single_plan.plan.poses.size() / 2);
        }
    }
    ROS_ERROR("---------------------------------------------");

    // 填充 cost 参数
    cost = result.flowtime;

    ROS_ERROR("AAASIPPROS: Published global plan with %zu agents", plan.global_plan.size());
    for (size_t i = 0; i < plan.global_plan.size(); ++i) {
        ROS_ERROR("  Agent %zu: %zu path points", i, plan.global_plan[i].plan.poses.size());
    }

    return true;
}

void AAASIPPROS::loadMapFromROS() {
    costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();

    // [新增调试输出]：打印 Costmap 分辨率
    // =================================================================
    double resolution = costmap->getResolution();
    ROS_WARN("--- MAP RESOLUTION INFO ---");
    ROS_WARN("Costmap Resolution (meters/cell): %.4f", resolution);
    ROS_WARN("Thus, 1 unit in aaasipp_shapes_params.yaml equals %.4f meters.", resolution);
    
    if (resolution >= 1.0) {
        ROS_WARN("WARNING: Resolution is 1.0m/cell or larger. 1 unit = 1 meter or more.");
    } else if (resolution <= 0.01) {
        ROS_WARN("WARNING: Resolution is 0.01m/cell (1cm) or smaller. 1 unit = 1 cm or less.");
    }
    ROS_WARN("---------------------------");
    // =================================================================
    
    map_->height = costmap->getSizeInCellsY();
    map_->width = costmap->getSizeInCellsX();
    map_->Grid.resize(map_->height, std::vector<int>(map_->width, 0));

    // Convert ROS costmap to your Map's Grid
    for (unsigned int i = 0; i < map_->height; ++i) {
        for (unsigned int j = 0; j < map_->width; ++j) {
            unsigned char cost = costmap->getCost(j, i);
            if (cost != costmap_2d::NO_INFORMATION && cost != 0) {  // cost >= costmap_2d::LETHAL_OBSTACLE
                map_->Grid[i][j] = CN_OBSTL;
            } else {
                map_->Grid[i][j] = 0;
            }
        }
    }
}

void AAASIPPROS::loadTaskFromROS(const nav_msgs::Path &start,
                                 const nav_msgs::Path &goal) {
    task_->agents.clear();

    costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();

    for (size_t i = 0; i < start.poses.size(); ++i) {
        Agent agent;
        agent.id = std::to_string(i);

        unsigned int start_i, start_j;
        costmap->worldToMap(start.poses[i].pose.position.x, start.poses[i].pose.position.y, start_j, start_i);
        agent.start_i = start_i;
        agent.start_j = start_j;

        unsigned int goal_i, goal_j;
        costmap->worldToMap(goal.poses[i].pose.position.x, goal.poses[i].pose.position.y, goal_j, goal_i);
        agent.goal_i = goal_i;
        agent.goal_j = goal_j;

        if (i < shapes_->shape_all.size()) {
            double max_dist_sq = 0.0;
            for (const auto& vertex : shapes_->shape_all[i]) {
                max_dist_sq = std::max(max_dist_sq, vertex.x*vertex.x + vertex.y*vertex.y);
            }
            agent.size = std::sqrt(max_dist_sq);
        } else {
            ROS_WARN("Agent %zu does not have a defined shape. Using default size 0.5.", i);
            agent.size = 0.5;
        }

        agent.mspeed = 1.0;
        agent.rspeed = 1.0;

        tf2::Quaternion q;
        tf2::fromMsg(start.poses[i].pose.orientation, q);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        agent.start_heading = yaw * 180.0 / CN_PI;

        tf2::fromMsg(goal.poses[i].pose.orientation, q);
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        agent.goal_heading = yaw * 180.0 / CN_PI;

        task_->agents.push_back(agent);
    }
}

std::vector<geometry_msgs::PoseStamped> 
AAASIPPROS::convertActionSequenceToPath(const std::vector<Node>& hppath, 
                           costmap_2d::Costmap2D* costmap,
                           double resolution) {
    std::vector<geometry_msgs::PoseStamped> path;
    
    if (hppath.empty()) {
        ROS_ERROR("convertActionSequenceToPath: hppath is empty");
        return path;
    }
    
    ROS_ERROR("convertActionSequenceToPath: processing %zu nodes", hppath.size());

    // 添加第一个点
    geometry_msgs::PoseStamped first_pose;
    first_pose.pose.position.x = costmap->getOriginX() + (hppath[0].j + 0.5) * resolution;
    first_pose.pose.position.y = costmap->getOriginY() + (hppath[0].i + 0.5) * resolution;
    
    tf2::Quaternion q_first;
    q_first.setRPY(0, 0, hppath[0].heading * M_PI / 180.0);
    first_pose.pose.orientation = tf2::toMsg(q_first);
    
    path.push_back(first_pose);
    
    // 处理后续点
    for (size_t i = 1; i < hppath.size(); ++i) {
        const Node& prev = hppath[i-1];
        const Node& curr = hppath[i];
        
        // 检查是否是原地转向动作
        if (prev.i == curr.i && prev.j == curr.j) {
            // 原地转向 - 插入多个中间朝向点
            double angle_diff = curr.heading - prev.heading;
            // 处理角度环绕
            if (angle_diff > 180) angle_diff -= 360;
            if (angle_diff < -180) angle_diff += 360;
            
            int steps = std::max(1, static_cast<int>(std::abs(angle_diff) / 15)); // 每15度一个中间点
            steps = std::min(steps, 12); // 最多12个中间点
            
            for (int j = 1; j <= steps; ++j) {
                double ratio = static_cast<double>(j) / steps;
                double interp_heading = prev.heading + ratio * angle_diff;
                
                geometry_msgs::PoseStamped pose;
                pose.pose.position.x = costmap->getOriginX() + (curr.j + 0.5) * resolution;
                pose.pose.position.y = costmap->getOriginY() + (curr.i + 0.5) * resolution;
                
                tf2::Quaternion q;
                q.setRPY(0, 0, interp_heading * M_PI / 180.0);
                pose.pose.orientation = tf2::toMsg(q);
                
                path.push_back(pose);
            }
        } else {
            // 移动动作 - 插入多个中间位置点
            double distance = sqrt(pow(curr.i - prev.i, 2) + pow(curr.j - prev.j, 2));
            int steps = std::max(1, static_cast<int>(distance * 2)); // 每0.5格一个中间点

            ROS_ERROR("  Detected movement: distance %.2f, adding %d movement steps", distance, steps);
            
            for (int j = 1; j <= steps; ++j) {
                double ratio = static_cast<double>(j) / steps;
                double interp_i = prev.i + ratio * (curr.i - prev.i);
                double interp_j = prev.j + ratio * (curr.j - prev.j);
                double interp_heading = prev.heading + ratio * (curr.heading - prev.heading);
                
                geometry_msgs::PoseStamped pose;
                pose.pose.position.x = costmap->getOriginX() + (interp_j + 0.5) * resolution;
                pose.pose.position.y = costmap->getOriginY() + (interp_i + 0.5) * resolution;
                
                tf2::Quaternion q;
                q.setRPY(0, 0, interp_heading * M_PI / 180.0);
                pose.pose.orientation = tf2::toMsg(q);
                
                path.push_back(pose);
            }
            
            // 确保最后一个点精确
            geometry_msgs::PoseStamped final_pose;
            final_pose.pose.position.x = costmap->getOriginX() + (curr.j + 0.5) * resolution;
            final_pose.pose.position.y = costmap->getOriginY() + (curr.i + 0.5) * resolution;
            
            tf2::Quaternion q_final;
            q_final.setRPY(0, 0, curr.heading * M_PI / 180.0);
            final_pose.pose.orientation = tf2::toMsg(q_final);

            ROS_ERROR("convertActionSequenceToPath: generated %zu path points", path.size());
            
            path.push_back(final_pose);
        }
    }
    
    return path;
}

} // namespace mapf
