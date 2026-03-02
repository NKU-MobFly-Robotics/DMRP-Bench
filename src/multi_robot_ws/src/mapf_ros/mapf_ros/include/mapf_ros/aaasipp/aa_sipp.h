#ifndef AA_SIPP_H
#define AA_SIPP_H

#include "constraints.h"
#include "lineofsight.h"
#include "config.h"
#include "searchresult.h"
#include "task.h"
#include "dynamicobstacles.h"
#include "shapes.hpp"
#include <math.h>
#include <memory>
#include <algorithm>
#include <unordered_map>
#include <random>
#ifdef __linux__
    #include <sys/time.h>
#else
    #include <windows.h>
#endif
class AA_SIPP
{

public:

    AA_SIPP(const Config &config);
    ~AA_SIPP();
    SearchResult startSearch(Map &map, Task &task, DynamicObstacles &obstacles, Shape &shapes);
    SearchResult sresult;
private:

    void addOpen(Node &newNode, const Map &map);
    // std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open;
    std::vector<std::list<Node>> open;
    void addOpen(Node &newNode);
    Node findMin(int size);
    bool stopCriterion(const Node &curNode, Node &goalNode);
    bool testGoal(const Node &curNode, Node &goalNode);
    double getCost(int a_i, int a_j, int b_i, int b_j);
    double getRCost(double headingA, double headingB);
    double calcHeading(const Node &node, const Node &son);
    std::list<Node> findSuccessors(const Node curNode, const Node goalNode,const Map &map, const std::vector<Location> &shape);
    // Node touchGoal(const Node curNode, const Node goalNode, const Map &map, const std::vector<Location> &shape);
    void makePrimaryPath(Node curNode);
    void makeSecondaryPath(Node curNode);
    void calculateLineSegment(std::vector<Node> &line, const Node &start, const Node &goal);
    void addConstraints(){}
    Node resetParent(Node current, Node Parent, const Map &map);
    Node resetShapeParent(Node current, Node Parent, std::vector<Location> &shape_ori, const Map &map);
    bool resetShapeParent_bool(Node current, Node Parent, std::vector<Location> &shape_ori, const Map &map);
    bool findPath(unsigned int numOfCurAgent, const Map &map, const std::vector<Location> &shape, const std::vector<Location> &shape_heading);
    std::vector<conflict> CheckConflicts(const Task &task);//bruteforce checker. It splits final(already built) trajectories into sequences of points and checks distances between them
    void setPriorities(const Task &task, const std::vector<std::vector<Location>> &shapes);
    double calculateSH_K(std::vector<Location> shape);
    double calculateSH_P(std::vector<Location> shape);

    double getHValue(int i, int j);
    bool changePriorities(int bad_i);
    unsigned int openSize;
    std::list<Node> lppath;
    std::unordered_multimap<int, Node> close;
    std::vector<Node> hppath;
    std::vector<std::vector<int>> priorities;
    std::vector<int> current_priorities;
    LineOfSight lineofsight;
    Agent curagent;
    Constraints *constraints;
    std::shared_ptr<const Config> config;
};

#endif // AA_SIPP_H
