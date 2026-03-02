#pragma once

#ifndef EECBS_ROS_H
#define EECBS_ROS_H

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include "../utils/timer.hpp"
#include "eecbs.hpp"
#include "eecbs_env.hpp"
#include "mapf_ros/mapf_ros.hpp"

namespace mapf {

class EECBSROS : public mapf::MAPFROS {
public:
    EECBSROS();
    EECBSROS(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) override;
bool makePlan(const nav_msgs::Path& start, const nav_msgs::Path& goal,
                  mapf_msgs::GlobalPlan& plan, double& cost,
                  const double& time_tolerance) override;

    ~EECBSROS();

protected:
    void updateObstacleThread();
    void worldToMap(const double& wx, const double& wy, unsigned int& mx, unsigned int& my);
    void mapToWorld(const unsigned int& mx, const unsigned int& my, double& wx, double& wy);
    void generatePlan(const std::vector<PlanResult<State, Action, int>>& solution,
                      const nav_msgs::Path& goal, mapf_msgs::GlobalPlan& plan, double& cost);
    void clearCell(const unsigned int& mx, const unsigned int& my);
    bool checkIsObstacle(const unsigned int& mx, const unsigned int& my);
    bool checkSurroundObstacle(const unsigned int& mx, const unsigned int& my);
    bool validateInputs(const nav_msgs::Path& start, const nav_msgs::Path& goal);
    bool prepareStatesAndGoals(const nav_msgs::Path& start, const nav_msgs::Path& goal,
                               std::vector<State>& startStates, std::vector<Location>& goals);
    void logPlanningResults(const Timer& timer, double cost, int makespan, const Environment& mapf);


private:
    std::mutex mtx_obs_update_;
    costmap_2d::Costmap2D* costmap_;
    std::string global_frame_;
    boost::thread* update_obstacle_thread_;
    std::unordered_set<Location> obstacles_;
    bool initialized_;
    float suboptimality_;
    double k1_; 
};

} // namespace mapf

#endif // EECBS_ROS_H