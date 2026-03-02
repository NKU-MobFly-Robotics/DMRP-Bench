#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>
#include "mapf_msgs/GlobalPlan.h"
#include "mapf_msgs/Goal.h"
#include "mapf_msgs/SinglePlan.h"
#include "mapf_ros/eecbs/eecbs_ros.hpp"

PLUGINLIB_EXPORT_CLASS(mapf::EECBSROS, mapf::MAPFROS)

namespace mapf {

EECBSROS::EECBSROS() : costmap_(nullptr), initialized_(false) {}

EECBSROS::EECBSROS(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
    : costmap_(nullptr), initialized_(false) {
    initialize(name, costmap_ros);
}

void EECBSROS::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
    if (!initialized_) {
        ros::NodeHandle nh("~");
        nh.param<float>("eecbs/suboptimality", suboptimality_, 1.1f);
        nh.param<double>("eecbs/k1", k1_, 1.1);
        costmap_ = costmap_ros->getCostmap();
        global_frame_ = costmap_ros->getGlobalFrameID();
        update_obstacle_thread_ = new boost::thread(boost::bind(&EECBSROS::updateObstacleThread, this));
        initialized_ = true;
    }
}

void EECBSROS::updateObstacleThread() {
    ROS_INFO_NAMED("update_obstacle_thread", "Updating obstacle state...");
    ros::NodeHandle thread_nh;
    ros::Rate loop_rate(0.5);  // update obstacle every 2s

    try {
        while (thread_nh.ok()) {
            int dimx = costmap_->getSizeInCellsX(), dimy = costmap_->getSizeInCellsY();
            const unsigned char* costarr = costmap_->getCharMap();
            
            {
                std::unique_lock<std::mutex> ulock(mtx_obs_update_, std::try_to_lock);
                if (ulock.owns_lock()) {
                    obstacles_.clear();
                    int offset = 0, num_obs = 0;
                    for (int i = 0; i < dimy; ++i) {
                        for (int j = 0; j < dimx; ++j) {
                            if (costarr[offset] >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
                                obstacles_.insert(Location(j, i));
                                num_obs++;
                            }
                            offset++;
                        }
                    }
                }
            }
            
            loop_rate.sleep();
            boost::this_thread::interruption_point();
        }
    } catch (boost::thread_interrupted const&) {
        ROS_INFO_NAMED("eecbs_planner", "Boost interrupt Exit Obstacle.");
    }
}

bool EECBSROS::makePlan(const nav_msgs::Path& start, const nav_msgs::Path& goal,
                        mapf_msgs::GlobalPlan& plan, double& cost,
                        const double& time_tolerance) {
    if (!validateInputs(start, goal)) {
        return false;
    }

    std::lock_guard<std::mutex> lock(mtx_obs_update_);
    int agent_num = start.poses.size();

    std::vector<State> startStates;
    std::vector<Location> goals;
    
    if (!prepareStatesAndGoals(start, goal, startStates, goals)) {
        return false;
    }

    int dimx = costmap_->getSizeInCellsX(), dimy = costmap_->getSizeInCellsY();
    std::vector<PlanResult<State, Action, int>> solution;
    Environment mapf(dimx, dimy, obstacles_, goals, false);
    EECBS<State, Action, int, Conflict, Constraints, Environment> eecbs(mapf, suboptimality_, k1_);

    Timer timer;
    bool success = eecbs.search(startStates, solution, time_tolerance);

    timer.stop();
    if (timer.elapsedSeconds() > time_tolerance) {
        ROS_ERROR("Planning time out! Current time tolerance is %lf", time_tolerance);
        return false;
    }

    if (success) {
        cost = 0;
        generatePlan(solution, goal, plan, cost);
        logPlanningResults(timer, cost, plan.makespan, mapf);
    } else {
        ROS_ERROR("Planning NOT successful!");
    }

    return success;
}

bool EECBSROS::validateInputs(const nav_msgs::Path& start, const nav_msgs::Path& goal) {
    if (goal.header.frame_id != global_frame_) {
        ROS_ERROR("The goal pose must be in the %s frame. It is in the %s frame.",
                  global_frame_.c_str(), goal.header.frame_id.c_str());
        return false;
    }
    if (start.header.frame_id != global_frame_) {
        ROS_ERROR("The start pose must be in the %s frame. It is in the %s frame.",
                  global_frame_.c_str(), start.header.frame_id.c_str());
        return false;
    }
    if (start.poses.empty() || goal.poses.empty()) {
        ROS_ERROR("Start and goal vectors are empty!");
        return false;
    }
    if (start.poses.size() != goal.poses.size()) {
        ROS_ERROR("Start and goal vectors are not the same length!");
        return false;
    }
    return true;
}

bool EECBSROS::prepareStatesAndGoals(const nav_msgs::Path& start, const nav_msgs::Path& goal,
                                     std::vector<State>& startStates, std::vector<Location>& goals) {
    for (size_t i = 0; i < start.poses.size(); ++i) {
        unsigned int start_x_i, start_y_i, goal_x_i, goal_y_i;
        worldToMap(start.poses[i].pose.position.x, start.poses[i].pose.position.y, start_x_i, start_y_i);
        worldToMap(goal.poses[i].pose.position.x, goal.poses[i].pose.position.y, goal_x_i, goal_y_i);

        if (checkSurroundObstacle(goal_x_i, goal_y_i)) {
            ROS_ERROR("Goal is surrounded by Obstacles");
            return false;
        }

        clearCell(start_x_i, start_y_i);
        clearCell(goal_x_i, goal_y_i);

        startStates.emplace_back(State(0, start_x_i, start_y_i));
        goals.emplace_back(Location(goal_x_i, goal_y_i));
    }
    return true;
}

void EECBSROS::generatePlan(const std::vector<PlanResult<State, Action, int>>& solution,
                            const nav_msgs::Path& goal, mapf_msgs::GlobalPlan& plan, double& cost) {
    int& makespan = plan.makespan;
    makespan = 0;
    for (const auto& s : solution) {
        cost += s.cost;
        makespan = std::max<int>(makespan, s.cost);
    }
    makespan += 1;

    plan.global_plan.resize(solution.size());
    for (size_t i = 0; i < solution.size(); ++i) {
        mapf_msgs::SinglePlan& single_plan = plan.global_plan[i];
        nav_msgs::Path& single_path = single_plan.plan;
        single_path.header.frame_id = global_frame_;
        single_path.header.stamp = ros::Time::now();

        for (const auto& state : solution[i].states) {
            geometry_msgs::PoseStamped cur_pose;
            cur_pose.header.frame_id = single_path.header.frame_id;
            cur_pose.pose.orientation.w = 1;
            mapToWorld(state.first.x, state.first.y, cur_pose.pose.position.x, cur_pose.pose.position.y);
            single_path.poses.push_back(cur_pose);
            single_plan.time_step.push_back(state.second);
        }
        single_path.poses.back() = goal.poses[i];
    }
}

void EECBSROS::worldToMap(const double& wx, const double& wy, unsigned int& mx, unsigned int& my) {
    if (!costmap_->worldToMap(wx, wy, mx, my)) {
        ROS_WARN("The robot's start position is off the global costmap. Planning will always fail, are you sure the robot has been properly localized?");
    }
}

void EECBSROS::mapToWorld(const unsigned int& mx, const unsigned int& my, double& wx, double& wy) {
    costmap_->mapToWorld(mx, my, wx, wy);
}

void EECBSROS::clearCell(const unsigned int& mx, const unsigned int& my) {
    obstacles_.erase(Location(mx, my));
}

bool EECBSROS::checkIsObstacle(const unsigned int& mx, const unsigned int& my) {
    return (costmap_->getCost(mx, my) >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
}

bool EECBSROS::checkSurroundObstacle(const unsigned int& mx, const unsigned int& my) {
    int dimx = costmap_->getSizeInCellsX(), dimy = costmap_->getSizeInCellsY();
    std::vector<std::pair<int, int>> steps{{0, 1}, {0, -1}, {-1, 0}, {1, 0}};
    
    for (const auto& step : steps) {
        int x = mx + step.first, y = my + step.second;
        if (x >= 0 && x < dimx && y >= 0 && y < dimy && !checkIsObstacle(x, y)) {
            return false;
        }
    }
    return true;
}

void EECBSROS::logPlanningResults(const Timer& timer, double cost, int makespan, const Environment& mapf) {
    ROS_DEBUG_STREAM("Planning successful!");
    ROS_DEBUG_STREAM("runtime: " << timer.elapsedSeconds());
    ROS_DEBUG_STREAM("cost: " << cost);
    ROS_DEBUG_STREAM("makespan(involve start & end): " << makespan);
    ROS_DEBUG_STREAM("highLevelExpanded: " << mapf.highLevelExpanded());
    ROS_DEBUG_STREAM("lowLevelExpanded: " << mapf.lowLevelExpanded());
}

EECBSROS::~EECBSROS() {
    if (update_obstacle_thread_) {
        update_obstacle_thread_->interrupt();
        update_obstacle_thread_->join();
        delete update_obstacle_thread_;
    }
    costmap_ = nullptr;
    ROS_INFO("Exit EECBS planner.");
}

} // namespace mapf