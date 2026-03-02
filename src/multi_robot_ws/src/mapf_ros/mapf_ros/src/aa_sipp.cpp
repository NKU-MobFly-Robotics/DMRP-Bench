#include "mapf_ros/aaasipp/aa_sipp.h"
#include <chrono>
#include <map>

AA_SIPP::AA_SIPP(const Config &config)
{
    this->config = std::make_shared<const Config>(config);
    openSize = 0;
    constraints = nullptr;
}

AA_SIPP::~AA_SIPP()
{
}

bool AA_SIPP::stopCriterion(const Node &curNode, Node &goalNode)
{

    if (curNode.i == curagent.goal_i && curNode.j == curagent.goal_j && curNode.interval.end == CN_INFINITY)
    {
        if (!config->planforturns || curagent.goal_heading == CN_HEADING_WHATEVER)
            goalNode = curNode;
        else if (goalNode.g > curNode.g + getRCost(curNode.heading, curagent.goal_heading))
        {
            goalNode = curNode;
            goalNode.g = curNode.g + getRCost(curNode.heading, curagent.goal_heading);
            goalNode.F = curNode.F + getRCost(curNode.heading, curagent.goal_heading);
        }
    }
    if (openSize == 0)
    {
        std::cout << "OPEN list is empty! " << std::endl;
        return true;
    }
    if (goalNode.F - CN_EPSILON < curNode.F)
        return true;
    return false;
}

double AA_SIPP::getCost(int a_i, int a_j, int b_i, int b_j)
{
    return sqrt((a_i - b_i) * (a_i - b_i) + (a_j - b_j) * (a_j - b_j));
}

double AA_SIPP::getHValue(int i, int j)
{
    if (config->allowanyangle || config->connectedness > 3) // euclid
        return (sqrt(pow(i - curagent.goal_i, 2) + pow(j - curagent.goal_j, 2))) / curagent.mspeed;
    else if (config->connectedness == 2) // manhattan
        return (abs(i - curagent.goal_i) + abs(j - curagent.goal_j)) / curagent.mspeed;
    else // k=3, use diagonal
        return (abs(abs(i - curagent.goal_i) - abs(j - curagent.goal_j)) + sqrt(2.0) * std::min(abs(i - curagent.goal_i), abs(j - curagent.goal_j))) / curagent.mspeed;
}

double AA_SIPP::getRCost(double headingA, double headingB)
{
    if (config->planforturns)
        return std::min(360 - fabs(headingA - headingB), fabs(headingA - headingB)) / (curagent.rspeed * 180.0);
    else
        return 0;
}

double AA_SIPP::calcHeading(const Node &node, const Node &son)
{
    double heading = atan2(son.i - node.i, son.j - node.j) * 180 / CN_PI;
    if (heading < 0)
        heading += 360;
    return heading;
}

// Node AA_SIPP::touchGoal(const Node curNode, const Node goalNode, const Map &map, const std::vector<Location> &shape)
// {
//     // 这里的shape传入的是shape_ori
//     Node newNode, angleNode, touchNode;
//     std::list<Node> successors;
//     std::vector<double> EAT;             // 每个后继节点的最早到达时间
//     std::vector<SafeInterval> intervals; //
//     double h_value;
//     auto parent = &(close.find(curNode.i * map.width + curNode.j)->second); // 当前节点的父节点
//     // LineOfSight l;

//     // 首先检查curNode可否直接到达goal
//     touchNode.i = -1;
//     int x1 = curNode.i, y1 = curNode.j, x2 = goalNode.i, y2 = goalNode.j;
//     std::vector<Location> shape_ori = shape;
//     if (lineofsight.checkShapeLine(x1, y1, x2, y2, shape_ori, map))
//     {

//         newNode.i = goalNode.i; // 新节点的坐标
//         newNode.j = goalNode.j;
//         newNode.heading = calcHeading(curNode, newNode); // 计算前往新节点的朝向
//         // 计算当前move朝向旋转后的shape
//         Shape_Collide s_temp;
//         Location ori = Location(0, 0);
//         // 未旋转的shape
//         std::vector<Location> shape_ori = shape;
//         // 朝向该move移动的shape
//         std::vector<Location> shape_cur = s_temp.rotate_shape(ori, newNode.heading, shape_ori);
//         constraints->updateShapeCellSafeIntervals({curagent.start_i, curagent.start_j}, shape_cur, map);
//         // constraints->updateCellSafeIntervals_Round({curagent.start_i, curagent.start_j}, shape_cur);
//         angleNode = curNode; // the same state, but with extended g-value
//         // Node tempNode = curNode;
//         // 转向所需要的时间！旋转代价（getRCost）以及额外等待时间
//         angleNode.g += getRCost(angleNode.heading, newNode.heading) + config->additionalwait; // to compensate the amount of time required for rotation

//         // angleNode.heading = newNode.heading;
//         // angleNode.Parent = &tempNode;

//         double dis2goal = sqrt((angleNode.i - goalNode.i) * (angleNode.i - goalNode.i) + (angleNode.j - goalNode.j) * (angleNode.j - goalNode.j));
//         newNode.g = angleNode.g + dis2goal / curagent.mspeed; // 更新前往新节点的成本
//         newNode.Parent = &angleNode;                          // 新节点的父节点
//         h_value = getHValue(newNode.i, newNode.j);            // 计算新节点的启发值

//         if (angleNode.g <= angleNode.interval.end)
//         {
//             // 如果当前节点成本（angleNode.g）小于等于父节点安全区间的结束时间（angleNode.interval.end），
//             // 则调用 constraints->findIntervals 函数寻找newNode的安全区间（intervals）和最早到达时间（EAT）
//             intervals = constraints->findIntervals(newNode, EAT, close, shape_ori, map, false);
//             for (unsigned int k = 0; k < intervals.size(); k++)
//             {
//                 // 遍历找到的安全区间，为每个安全区间创建一个newNode，并计算其F值，然后将其添加到successors列表中。
//                 newNode.interval = intervals[k];
//                 newNode.Parent = parent;
//                 newNode.g = EAT[k];
//                 newNode.F = newNode.g + h_value;
//                 // successors.push_front(newNode);
//                 touchNode = newNode;
//                 // std::cout << "Can Touch Goal at : " << curNode.i << " , " << curNode.j << std::endl;
//                 // std::cout << "touchNode.F : " << touchNode.F << std::endl;
//                 // std::cout << "touchNode.g : " << touchNode.g << std::endl;
//             }
//         }
//     }
//     return touchNode;
// }

std::list<Node> AA_SIPP::findSuccessors(const Node curNode, const Node goalNode, const Map &map, const std::vector<Location> &shape) // shape需要传进来
{
    // 这里的shape传入的是shape_ori
    Node newNode, angleNode;
    std::list<Node> successors;
    std::vector<double> EAT;             // 每个后继节点的最早到达时间
    std::vector<SafeInterval> intervals; //
    double h_value;
    // auto parent = &(close.find(curNode.i * map.width + curNode.j)->second); // 当前节点的父节点

    auto it = close.find(curNode.i * map.width + curNode.j);
    
    // 關鍵檢查：確保找到了節點再進行後續操作
    if (it == close.end()) {
        // 如果在 close 列表中找不到當前節點，這是一個嚴重的邏輯錯誤。
        // 記錄錯誤並返回一個空的後繼列表，以避免崩潰。
        std::cerr << "CRITICAL ERROR: Node (" << curNode.i << ", " << curNode.j << ") not found in CLOSE set!" << std::endl;
        return {}; // 返回一個空的後繼列表
    }
    
    auto parent = &(it->second); // 将“parent”设置为close中的curNode

    // 首先检查curNode可否直接到达goal
    // if(lineofsight.checkShapeLine(curNode.i, curNode.j, goalNode.i, goalNode.j, shape, map))
    // touchNode.i = goalNode.i;
    // std::vector<Node> moves = map.getValidMoves(curNode.i, curNode.j, config->connectedness, curagent.size);
    double round_radius = 0;
    for (const auto &s : shape)
    {
        round_radius = std::max(round_radius, s.x * s.x + s.y * s.y);
    }
    round_radius = std::sqrt(round_radius);

    std::vector<Node> moves = map.getShapeValidMoves(curNode.i, curNode.j, config->connectedness, curagent.size, curNode.heading, shape);

    for (auto m : moves) // 遍历所有可行的移动
    {
        newNode.i = curNode.i + m.i; // 新子节点的坐标
        newNode.j = curNode.j + m.j;
        newNode.heading = calcHeading(curNode, newNode); // 计算前往新节点的朝向
        // 计算当前move朝向旋转后的shape
        Shape_Collide s_temp;
        Location ori = Location(0, 0);
        // 未旋转的shape
        std::vector<Location> shape_ori = shape;
        // 朝向该move移动的shape
        std::vector<Location> shape_cur = s_temp.rotate_shape(ori, newNode.heading, shape_ori);

        // constraints->updateCellSafeIntervals({curagent.start_i, curagent.start_j});
        // constraints->updateShapeCellSafeIntervals({newNode.i, newNode.j}, shape_cur, map);
        constraints->updateCellSafeIntervals_Round({newNode.i, newNode.j}, shape_cur, map);

        angleNode = curNode; // the same state, but with extended g-value
        // 转向所需要的时间！旋转代价（getRCost）以及额外等待时间
        angleNode.g += getRCost(angleNode.heading, newNode.heading) + config->additionalwait; // to compensate the amount of time required for rotation
        newNode.g = angleNode.g + m.g / curagent.mspeed;                                      // 更新前往新节点的成本
        newNode.Parent = &angleNode;                                                          // 新节点的父节点
        h_value = getHValue(newNode.i, newNode.j);                                            // 计算新节点的启发值

        if (angleNode.g <= angleNode.interval.end)
        {
            // auto s4 = std::chrono::high_resolution_clock::now();
            intervals = constraints->findIntervals(newNode, EAT, close, shape_ori, map, false);
            // auto e4 = std::chrono::high_resolution_clock::now();
            // std::chrono::duration<double, std::milli> duration4 = e4 - s4;
            // std::cout << "-- Time for findIntervals using time: " << duration4.count() << " (ms)" << std::endl;
            for (unsigned int k = 0; k < intervals.size(); k++)
            {
                // 遍历找到的安全区间，为每个安全区间创建一个newNode，并计算其F值，然后将其添加到successors列表中。
                newNode.interval = intervals[k];
                newNode.Parent = parent;
                newNode.g = EAT[k];
                newNode.F = newNode.g + h_value;
                successors.push_front(newNode);
            }
        }
        if (config->allowanyangle) // 如果允许任何角度移动，重置新节点的父节点，并重新计算朝向、成本和父节点
        {
            // newNode = resetShapeParent(newNode, curNode, shape_ori, map);
            // if (parent->Parent != nullptr)
            // {
            //     std::cout << "------------------ AllowAnyAngle is true ------------------" << std::endl;
            //     std::cout << "GrandParent heading is : " << parent->Parent->heading << std::endl;
            //     double new_heading = calcHeading(*parent->Parent, newNode);
            //     double heading2 = calcHeading(*parent->Parent, *parent);
            //     std::cout << "GrandPar To new_heading is : " << new_heading << " and Grand to curNode heading is :" << heading2 << std::endl;
            //     std::cout << "GrandPar is at : (" << parent->Parent->i << "," << parent->Parent->j << ") and curNode is at : (" << curNode.i << "," << curNode.j << ") and newNode is at:(" << newNode.i << "," << newNode.j << ")" << std::endl;
            // }

            bool reset_par = resetShapeParent_bool(newNode, curNode, shape_ori, map);
            if (!reset_par)
            {
                continue;
            }
                
            // // 祖父节点A 父节点B 节点C  
            // // 若A迭代为C的父节点，A原地旋转为A'，然后A'->C,需要分别检测 A->A' 和 A'->C 是否与动静态障碍物碰撞
            // // 如果都不碰撞，再进行实际的父节点迭代操作
            // Node A = *curNode.Parent; // 祖父节点A
            // Node C = newNode;         // 节点C
            // double heading_AC = calcHeading(A, C); // A->C,也就是A'和C的朝向
            // double heading_A = A.heading; // A的朝向
            // std::vector<Location> shape_AC = s_temp.rotate_shape(ori, heading_AC, shape_ori);
            
            // bug: 应该计算的是： 1. A->A'是否与动、静态障碍物碰撞, 这里只计算了A'是否与静态障碍物碰撞
            // LineOfSight los;
            // std::vector<std::pair<int, int>> static_cells_after_rot = los.getCellsOriginByPolyon_Optimized(A.i, A.j, shape_AC, map);
            // for (auto w : static_cells_after_rot)
            // {
            //     if (!map.CellOnGrid(w.first, w.second) || map.CellIsObstacle(w.first, w.second))
            //     {
            //         continue;
            //     }
            // }

            newNode.g = curNode.Parent->g + getCost(curNode.Parent->i, curNode.Parent->j, newNode.i, newNode.j) / curagent.mspeed;
            newNode.Parent = curNode.Parent;

            // 检查新的父节点是否与原始父节点不同，如果是，则重新计算旋转代价和成本。
            if (newNode.Parent->i != parent->i || newNode.Parent->j != parent->j)
            {

                angleNode = *newNode.Parent; // 此时angleNode的位置是祖父A的位置
                newNode.heading = calcHeading(*newNode.Parent, newNode);   // newNode.heading是A->C的朝向                          
                angleNode.g += getRCost(angleNode.heading, newNode.heading) + config->additionalwait;  // angleNode.heading是A的朝向，但是成本已经加入了转向成本
                newNode.g += getRCost(angleNode.heading, newNode.heading) + config->additionalwait;    // newNode.g 也加入了A->C的转向成本
                newNode.Parent = &angleNode; // newNode的父节点现在是angleNode（祖父A的位置）

                // // 这是检测 A'->A 是否与静态障碍物碰撞
                // std::set<std::pair<int, int>> rotated_cells = constraints->multiAngleSampleCells_DuringRot(A.i, A.j, angleNode.heading, newNode.heading, 15, shape_ori,  map);
                // for (auto w : rotated_cells)
                // {
                //     if (!map.CellOnGrid(w.first, w.second) || map.CellIsObstacle(w.first, w.second))
                //     {
                //         continue;
                //     }
                // }

                if (angleNode.g > angleNode.interval.end)
                    continue;
                // 再次寻找安全区间并创建新节点状态
                shape_cur.clear();
                std::vector<Location> shape_cur = s_temp.rotate_shape(ori, newNode.heading, shape_ori); 

                // double Rcost = getRCost(angleNode.heading, newNode.heading);
                // double Dcost = sqrt((curNode.i - angleNode.i) * (curNode.i - angleNode.i) + (curNode.j - angleNode.j) * (curNode.j - angleNode.j)) / curagent.mspeed;

                // bug出现在这里
                // 25.10.13 这里发现传shape形状传错了，应该传入shape_ori未旋转的形状
                // intervals = constraints->findIntervals(newNode, EAT, close, shape_cur, map); // 传入当前朝向的形状
                // 传入一个bool 标志，用于触发父节点迭代中，A->A'的原地旋转的碰撞检测
                intervals = constraints->findIntervals(newNode, EAT, close, shape_ori, map, true); // 传入当前朝向的形状

                for (unsigned int k = 0; k < intervals.size(); k++)
                {
                    newNode.interval = intervals[k];
                    newNode.Parent = parent->Parent;
                    newNode.g = EAT[k];
                    newNode.F = newNode.g + h_value;
                    // newNode.F = newNode.g + h_value + getRCost(newNode.heading, goalNode.heading);
                    successors.push_front(newNode);
                }
            }
            // std::cout << "test04" << std::endl;
        }
    }

    return successors;
}

Node AA_SIPP::findMin(int size)
{
    Node min;
    min.F = std::numeric_limits<double>::max();
    for (int i = 0; i < size; i++)
    {
        if (!open[i].empty() && open[i].begin()->F - CN_EPSILON < min.F)
        {
            if (fabs(open[i].begin()->F - min.F) < CN_EPSILON)
            {
                if (min.g < open[i].begin()->g)
                    min = *open[i].begin();
            }
            else
                min = *open[i].begin();
        }
    }
    return min;
}

void AA_SIPP::addOpen(Node &newNode)
{
    if (open[newNode.i].empty())
    {
        open[newNode.i].push_back(newNode);
        openSize++;
        return;
    }

    std::list<Node>::iterator iter, pos, delpos;
    bool posFound(false);
    pos = open[newNode.i].end();
    for (iter = open[newNode.i].begin(); iter != open[newNode.i].end(); ++iter)
    {
        if ((newNode.F - CN_EPSILON < iter->F) && !posFound)
        {
            if (fabs(iter->F - newNode.F) < CN_EPSILON)
            {
                if (newNode.g > iter->g)
                {
                    pos = iter;
                    posFound = true;
                }
            }
            else
            {
                pos = iter;
                posFound = true;
            }
        }

        if (iter->j == newNode.j && iter->interval.id == newNode.interval.id)
        {
            if ((iter->g - newNode.g + getRCost(iter->heading, newNode.heading)) < CN_EPSILON) // if existing state dominates new one
                return;
            if ((newNode.g - iter->g + getRCost(iter->heading, newNode.heading)) < CN_EPSILON) // if new state dominates the existing one
            {
                if (pos == iter)
                {
                    iter->F = newNode.F;
                    iter->g = newNode.g;
                    iter->interval = newNode.interval;
                    iter->Parent = newNode.Parent;
                    iter->heading = newNode.heading;
                    return;
                }
                delpos = iter;
                iter--;
                open[newNode.i].erase(delpos);
                openSize--;
            }
        }
    }
    open[newNode.i].insert(pos, newNode);
    openSize++;
    return;
}

// void AA_SIPP::addOpen(Node &newNode, const Map &map)
// {
//     // 检查 CLOSE 表中是否已存在一个更优或等优的节点状态。
//     // 一个状态由 (位置, 安全区间ID) 唯一确定。
//     auto range = close.equal_range(newNode.i * map.width + newNode.j);
//     for (auto it = range.first; it != range.second; ++it)
//     {
//         // 如果找到了同一个状态(位置和区间ID都相同)
//         if (it->second.interval.id == newNode.interval.id)
//         {
//             // 且CLOSE表中的节点g值更小（路径更优），则新节点被“支配”，无需加入OPEN表
//             if (it->second.g <= newNode.g)
//             {
//                 return;
//             }
//         }
//     }

//     // 如果没有被支配，则将新节点加入优先队列
//     open.push(newNode);
//     return;
// }

void AA_SIPP::setPriorities(const Task &task, const std::vector<std::vector<Location>> &shapes)
{
    current_priorities.clear();
    current_priorities.resize(task.getNumberOfAgents(), -1);
    if (config->initialprioritization == CN_IP_FIFO)
        for (int i = 0; i < task.getNumberOfAgents(); i++)
            current_priorities[i] = i;
    else if (config->initialprioritization != CN_IP_RANDOM && config->initialprioritization != CN_IP_SHAPE && config->initialprioritization != CN_IP_SOAP)
    {
        std::vector<double> dists(task.getNumberOfAgents(), -1);
        for (int i = 0; i < task.getNumberOfAgents(); i++)
            dists[i] = sqrt(pow(task.getAgent(i).start_i - task.getAgent(i).goal_i, 2) + pow(task.getAgent(i).start_j - task.getAgent(i).goal_j, 2));
        int k = task.getNumberOfAgents() - 1;
        while (k >= 0)
        {
            double mindist = CN_INFINITY;
            int min_i = -1;
            for (unsigned int i = 0; i < dists.size(); i++)
                if (mindist > dists[i])
                {
                    min_i = i;
                    mindist = dists[i];
                }
            if (config->initialprioritization == CN_IP_LONGESTF)
                current_priorities[k] = min_i;
            else
                current_priorities[task.getNumberOfAgents() - k - 1] = min_i;
            dists[min_i] = CN_INFINITY;
            k--;
        }
    }
    else if (config->initialprioritization == CN_IP_RANDOM) // random
    {
        for (int i = 0; i < task.getNumberOfAgents(); i++)
            current_priorities[i] = i;
        std::mt19937 g(rand());
        std::shuffle(current_priorities.begin(), current_priorities.end(), g);
    }
    else if (config->initialprioritization == CN_IP_SHAPE) // shape heuristic
    {
        // std::vector<int> shape_heuristic(task.getNumberOfAgents());
        // 建立 序号-SH 映射
        std::cout << "Using SHAPE HEURISTIC ~~~" << std::endl;
        // std::vector<double> SH(task.getNumberOfAgents());
        // std::vector<int> SH_idx(task.getNumberOfAgents()); // index of SH
        // for (int i = 0; i < task.getNumberOfAgents(); i++)
        // {
        //     double sh = 0.0, alpha = 0.5, beta = 0.5;
        //     double C_corner = 0.0, K_area = 0.0;

        //     C_corner = shapes[i].size();
        //     K_area = calculateSH_K(shapes[i]);
        //     SH[i] = alpha * C_corner + beta * K_area;
        //     SH_idx[i] = i;
        // }
        // // 对 SH 进行排序，获取SH值降序排序后的索引
        // // std::sort(SH_idx.begin(), SH_idx.end(), [&SH](int a, int b) { return SH[a] > SH[b]; });
        // std::sort(SH_idx.begin(), SH_idx.end(), [&SH](int a, int b)
        //           { return SH[a] > SH[b]; }); // 降序
        // for (int i = 0; i < task.getNumberOfAgents(); ++i)
        // {
        //     current_priorities[i] = SH_idx[i];
        //     // std::cout << "SH: " << SH[SH_idx[i]] << std::endl;
        //     int original_agent_index = SH_idx[i];
        //     double corresponding_sh_value = SH[original_agent_index];

        //     // std::cout << "排名 " << i << " -> 智能体 " << original_agent_index
        //     //         << " -> SH = " << corresponding_sh_value << std::endl;
        //     // std::cout << std::endl;
        // }
        std::vector<double> SH2(task.getNumberOfAgents());
        std::vector<int> SH2_idx(task.getNumberOfAgents()); // index of SH
        for (int i = 0; i < task.getNumberOfAgents(); i++)
        {
            double sh = 0.0;
            double P_perimeter = 0.0, A_area = 0.0;
            
            P_perimeter = calculateSH_P(shapes[i]);
            A_area = calculateSH_K(shapes[i]);
            SH2[i] =  (P_perimeter * P_perimeter / A_area) * sqrt(A_area / CN_PI);
            SH2_idx[i] = i;
        }
        std::sort(SH2_idx.begin(), SH2_idx.end(), [&SH2](int a, int b)
                  { return SH2[a] > SH2[b]; }); // 降序
        for (int i = 0; i < task.getNumberOfAgents(); ++i)
        {
            current_priorities[i] = SH2_idx[i];
            int original_agent_index = SH2_idx[i];
            double corresponding_sh_value = SH2[original_agent_index];
        }
    }

    else if (config->initialprioritization == CN_IP_SOAP) // shape heuristic
    {
        std::vector<double> SH2(task.getNumberOfAgents());
        std::vector<int> SH2_idx(task.getNumberOfAgents()); // index of SH
        for (int i = 0; i < task.getNumberOfAgents(); i++)
        {
            double sh = 0.0;
            double P_perimeter = 0.0, A_area = 0.0;
            
            P_perimeter = calculateSH_P(shapes[i]);
            A_area = calculateSH_K(shapes[i]);
            SH2[i] =  (P_perimeter * P_perimeter / A_area) * sqrt(A_area / CN_PI);
            SH2_idx[i] = i;
        }
        std::sort(SH2_idx.begin(), SH2_idx.end(), [&SH2](int a, int b)
                  { return SH2[a] > SH2[b]; }); // 降序
        for (int i = 0; i < task.getNumberOfAgents(); ++i)
        {
            current_priorities[i] = SH2_idx[i];
            int original_agent_index = SH2_idx[i];
            double corresponding_sh_value = SH2[original_agent_index];
        }
    }
}

double AA_SIPP::calculateSH_K(std::vector<Location> shape)
{
    // 通过鞋帶公式计算多邊形的面積
    // // 一個有效的多邊形至少需要3個頂點
    // if (shape.size() < 3) {
    //     return 0.0;
    // }

    double area_sum = 0.0;
    int n = shape.size();

    // 遍歷所有頂點，應用鞋帶公式
    // 公式: Area = 0.5 * |Σ(x_i * y_{i+1} - x_{i+1} * y_i)|
    for (int i = 0; i < n; ++i)
    {
        const Location &current_vertex = shape[i];
        // 使用模運算(%)來處理最後一個頂點與第一個頂點的配對，實現環繞
        const Location &next_vertex = shape[(i + 1) % n];

        area_sum += (current_vertex.x * next_vertex.y - next_vertex.x * current_vertex.y);
    }

    // 最後將總和除以2並取絕對值
    return 0.5 * std::abs(area_sum);
}

double AA_SIPP::calculateSH_P(std::vector<Location> shape)
{
    // 计算形状的周长
    double perimeter = 0.0;
    int n = shape.size();

    // 遍历所有顶点，计算相邻顶点之间的距离
    for (int i = 0; i < n; ++i)
    {
        const Location &current_vertex = shape[i];
        // 使用模运算(%)来处理最后一个顶点与第一个顶点的配对，实现环绕
        const Location &next_vertex = shape[(i + 1) % n];
        double distance = std::sqrt(std::pow(next_vertex.x - current_vertex.x, 2) + std::pow(next_vertex.y - current_vertex.y, 2));
        perimeter += distance;

    }
    return perimeter;
}

bool AA_SIPP::changePriorities(int bad_i)
{
    if (config->rescheduling == CN_RE_NO)
        return false;
    priorities.push_back(current_priorities);
    if (config->rescheduling == CN_RE_RULED) // rises the piority of the agent that can't find its path
    {
        for (auto it = current_priorities.begin(); it != current_priorities.end(); it++)
            if (*it == bad_i)
            {
                current_priorities.erase(it);
                current_priorities.insert(current_priorities.begin(), bad_i);
                break;
            }
        for (unsigned int i = 0; i < priorities.size(); i++)
            for (unsigned int j = 0; j < priorities[i].size(); j++)
            {
                if (j + 1 == priorities[i].size())
                    return false;
                if (current_priorities[j] != priorities[i][j])
                    break;
            }
        return true;
    }
    else // random
    {
        std::mt19937 g(rand());
        std::shuffle(current_priorities.begin(), current_priorities.end(), g);
        bool unique = false;
        int maxtries(1000000), tries(0);
        while (!unique && tries < maxtries)
        {
            tries++;
            for (unsigned int i = 0; i < priorities.size(); i++)
            {
                for (unsigned int j = 0; j < priorities[i].size(); j++)
                {
                    if (j + 1 == priorities[i].size())
                        unique = false;
                    if (current_priorities[j] != priorities[i][j])
                        break;
                }
                if (!unique)
                {
                    std::shuffle(current_priorities.begin(), current_priorities.end(), g);
                    break;
                }
            }
            unique = true;
        }
        return unique;
    }
}

SearchResult AA_SIPP::startSearch(Map &map, Task &task, DynamicObstacles &obstacles, Shape &shapes)
{

#ifdef __linux__
    timeval begin, end;
    gettimeofday(&begin, NULL);
#else
    LARGE_INTEGER begin, end, freq;
    QueryPerformanceCounter(&begin);
    QueryPerformanceFrequency(&freq);
#endif

    std::cout << "------ Start Search ------" << std::endl;
    std::cout << "Agent Num: " << task.getNumberOfAgents() << std::endl;
    std::cout << "Map Size - height: " << map.height << " *  width: " << map.width << std::endl;
    // map.printGrid(map.Grid);

    bool solution_found(false);
    int tries(0), bad_i(0);
    double timespent(0);
    priorities.clear();

    open.resize(map.height);
    std::vector<std::vector<Location>> shapes_all = shapes.shape_all;
    setPriorities(task, shapes_all); // 设置优先级，后续这里使用Shape Heuristic

    Shape shape_initial_heading;
    shape_initial_heading.id_all = shapes.id_all;

    // 处理初始角度形状
    for (int k = 0; k < task.getNumberOfAgents(); k++) // *要注意这里后续会不会有优先级导致的bug
    {
        // 切记，shape_initial_heading是初始角度、相对、旋转后的坐标
        // shapes是未旋转的、相对坐标
        task.getAgent(k).start_heading;
        Location start;
        start.x = task.getAgent(k).start_i;
        start.y = task.getAgent(k).start_j;
        Shape_Collide shape_temp;
        // shape_temp.getShapeInfo(start, shapes.shape_all[k], task.getAgent(k).size);
        Location ori = Location(0, 0);
        std::vector<Location> shape_rotate = shape_temp.rotate_shape(ori, task.getAgent(k).start_heading, shapes.shape_all[k]);
        shape_initial_heading.shape_all.push_back(shape_rotate);
    }

    do
    {
        constraints = new Constraints(map.width, map.height);
        for (int k = 0; k < obstacles.getNumberOfObstacles(); k++)
        {
            std::cout << "Moving Obstacle " << k << " size: " << obstacles.getSize(k) << std::endl;
            constraints->addShapeConstraints(obstacles.getSections(k), obstacles.getSize(k),
                                             obstacles.getMSpeed(k), shapes.shape_all[k], map);
        }
        sresult.pathInfo.clear();
        sresult.pathInfo.resize(task.getNumberOfAgents());
        sresult.agents = task.getNumberOfAgents();
        sresult.agentsSolved = 0;
        sresult.flowtime = 0;
        sresult.makespan = 0;

        // 为当前代理的起始位置添加初始约束。这个约束确保代理在起始位置有一个安全间隔，避免与其他代理或障碍物碰撞
        for (int k = 0; k < task.getNumberOfAgents(); k++)
        {
            curagent = task.getAgent(k); // agent结构体包含起点、终点、起终点朝向、速度角速度等信息；
            constraints->setParams(curagent.size, curagent.mspeed, curagent.rspeed, config->planforturns, config->inflatecollisionintervals);
            lineofsight.setShapeSize(curagent.size, shape_initial_heading.shape_all[k]);
            if (config->startsafeinterval > 0)
            {
                auto cells = lineofsight.getCellsOriginByPolyon_Optimized(curagent.start_i, curagent.start_j, shape_initial_heading.shape_all[k], map);
                double d = 0;
                for (auto p : shape_initial_heading.shape_all[k])
                {
                    double dis = p.x * p.x + p.y * p.y;
                    d = std::max(d, dis);
                }
                d = sqrt(d);
                curagent.size = d;
                // ！！这里用的是包络圆哦！！ 不过不是很重要，startConstraint比较保守
                constraints->addStartConstraint(curagent.start_i, curagent.start_j, config->startsafeinterval, cells, curagent.size);
            }
        }

        // 开始正式为每个智能体按顺序搜索路径！！
        for (unsigned int numOfCurAgent = 0; numOfCurAgent < task.getNumberOfAgents(); numOfCurAgent++)
        {
            // 当前Agent
            curagent = task.getAgent(current_priorities[numOfCurAgent]);

            // 根据智能体的大小、最大速度、旋转速度、规划转向的步数和碰撞间隔的膨胀系数设置约束参数。
            constraints->setParams(curagent.size, curagent.mspeed, curagent.rspeed, config->planforturns, config->inflatecollisionintervals);
            // 移除起始位置的约束，以确保代理可以从起始位置开始移动。
            lineofsight.setShapeSize(curagent.size, shape_initial_heading.shape_all[numOfCurAgent]);
            if (config->startsafeinterval > 0)
            {
                auto cells = lineofsight.getCells(curagent.start_i, curagent.start_j);
                constraints->removeStartConstraint(cells, curagent.start_i, curagent.start_j);
            }
            // 为当前智能体寻找路径。如果找到路径，则添加路径约束；
            auto s1 = std::chrono::high_resolution_clock::now();
            if (findPath(current_priorities[numOfCurAgent], map, shapes.shape_all[current_priorities[numOfCurAgent]], shape_initial_heading.shape_all[current_priorities[numOfCurAgent]]))
            {
                constraints->goal_heading = curagent.goal_heading;
                constraints->initial_heading = curagent.start_heading;
                constraints->addShapeConstraints(sresult.pathInfo[current_priorities[numOfCurAgent]].sections, curagent.size, curagent.mspeed, shapes.shape_all[current_priorities[numOfCurAgent]], map);
            }

            // 否则，记录无法找到路径的代理索引并退出循环。
            else
            {
                bad_i = current_priorities[numOfCurAgent];
                break;
            }
            auto e1 = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double, std::milli> duration1 = e1 - s1;
            std::cout << "Time for findPath using time: " << duration1.count() << " (ms)" << std::endl;
            // 检查是否所有代理都找到路径
            if (numOfCurAgent + 1 == task.getNumberOfAgents())
                solution_found = true;
        }

        delete constraints;
        tries++;
#ifdef __linux__
        gettimeofday(&end, NULL);
        timespent = (end.tv_sec - begin.tv_sec) + static_cast<double>(end.tv_usec - begin.tv_usec) / 1000000;
#else
        QueryPerformanceCounter(&end);
        timespent = static_cast<double long>(end.QuadPart - begin.QuadPart) / freq.QuadPart;
#endif
        if (timespent > config->timelimit)
            break;
    } while (changePriorities(bad_i) && !solution_found);

#ifdef __linux__
    gettimeofday(&end, NULL);
    sresult.runtime = (end.tv_sec - begin.tv_sec) + static_cast<double>(end.tv_usec - begin.tv_usec) / 1000000;
#else
    QueryPerformanceCounter(&end);
    sresult.runtime = static_cast<double long>(end.QuadPart - begin.QuadPart) / freq.QuadPart;
#endif
    sresult.tries = tries;
    if (sresult.pathfound)
    {
        std::vector<conflict> confs = CheckConflicts(task);
        for (unsigned int i = 0; i < confs.size(); i++)
            std::cout << confs[i].i << " " << confs[i].j << " " << confs[i].g << " " << confs[i].agent1 << " " << confs[i].agent2 << "\n";
    }

    std::cout << "------ End Search ------" << std::endl;
    return sresult;
}

Node AA_SIPP::resetParent(Node current, Node Parent, const Map &map)
{
    // 原函数是只判断了点-点连线是否可行，应该加入对形状的判断！
    // checkLine函数需要加工或者新增；

    if (Parent.Parent == nullptr || (current.i == Parent.Parent->i && current.j == Parent.Parent->j))
        return current;
    if (lineofsight.checkLine(Parent.Parent->i, Parent.Parent->j, current.i, current.j, map))
    {
        current.g = Parent.Parent->g + getCost(Parent.Parent->i, Parent.Parent->j, current.i, current.j) / curagent.mspeed;
        current.Parent = Parent.Parent;
    }
    return current;
}

Node AA_SIPP::resetShapeParent(Node current, Node Parent, std::vector<Location> &shape_ori, const Map &map)
{
    // 把这个函数改为bool的，然后在外部，对祖父节点进行原地旋转动作的检查；

    if (Parent.Parent == nullptr || (current.i == Parent.Parent->i && current.j == Parent.Parent->j))
        return current;

    // // 先测试连线上的连通性，如果连线上有障碍物，那么直接pass
    // if(!lineofsight.checkLine_Amanatides_Woo(Parent.Parent->i, Parent.Parent->j, current.i, current.j, map))
    // {
    //     return current;
    // }
    long long dx1 = (long long)Parent.i - Parent.Parent->i;
    long long dy1 = (long long)Parent.j - Parent.Parent->j;
    long long dx2 = (long long)current.i - Parent.i;
    long long dy2 = (long long)current.j - Parent.j;
    long long atLine = dx1 * dy2 - dx2 * dy1;
    if (std::abs(atLine) < 0.000001)
    {
        current.g = Parent.Parent->g + getCost(Parent.Parent->i, Parent.Parent->j, current.i, current.j) / curagent.mspeed;
        current.Parent = Parent.Parent;
        return current;
    }

    if (lineofsight.checkShapeLine(Parent.Parent->i, Parent.Parent->j, current.i, current.j, shape_ori, map))
    {
        current.g = Parent.Parent->g + getCost(Parent.Parent->i, Parent.Parent->j, current.i, current.j) / curagent.mspeed;
        current.Parent = Parent.Parent;
    }

    return current;
}

bool AA_SIPP::resetShapeParent_bool(Node current, Node Parent, std::vector<Location> &shape_ori, const Map &map)
{
    if (Parent.Parent == nullptr || (current.i == Parent.Parent->i && current.j == Parent.Parent->j))
        return false;

    // long long dx1 = (long long)Parent.i - Parent.Parent->i;
    // long long dy1 = (long long)Parent.j - Parent.Parent->j;
    // long long dx2 = (long long)current.i - Parent.i;
    // long long dy2 = (long long)current.j - Parent.j;
    // long long atLine = dx1 * dy2 - dx2 * dy1;
    // if (std::abs(atLine) < 0.000001)
    // {
    //     return true;
    // }

    // // 这一段发现一点作用都没有好像
    // // 不光是在resetParent时会发生原地旋转，正常的路径节点迭代时也会发生原地旋转，在这里进行检查是不全面的
    // if (Parent.Parent->Parent != nullptr)
    // {
    //     Node A = *Parent.Parent; // 祖父节点A
    //     Node C = current;         // 节点C
    //     double heading_AC = calcHeading(A, C); // A->C,也就是A'和C的朝向
    //     std::set<std::pair<int, int>> rotated_cells = constraints->multiAngleSampleCells_DuringRot(Parent.Parent->i, Parent.Parent->j, Parent.Parent->heading, heading_AC+1, 10, shape_ori,  map);
    //     for (auto w : rotated_cells)
    //     {
    //         if (!map.CellOnGrid(w.first, w.second) || map.CellIsObstacle(w.first, w.second))
    //         {
    //             return false;
    //         }
    //     }
    // }
    

    if (lineofsight.checkShapeLine(Parent.Parent->i, Parent.Parent->j, current.i, current.j, shape_ori, map))
    {
        return true;
    }

    return false;
}

bool AA_SIPP::findPath(unsigned int numOfCurAgent, const Map &map,
                       const std::vector<Location> &shape_ori, const std::vector<Location> &shape_initial_heading)
{

    std::cout << "--- Finding path for agent: " << numOfCurAgent << " ---" << std::endl;
    std::cout << "From: (" << curagent.start_i << ", " << curagent.start_j << ") to (" << curagent.goal_i << ", " << curagent.goal_j << ")" << std::endl;

#ifdef __linux__
    timeval begin, end;
    gettimeofday(&begin, NULL);
#else
    LARGE_INTEGER begin, end, freq;
    QueryPerformanceCounter(&begin);
    QueryPerformanceFrequency(&freq);
#endif
    close.clear();
    for (unsigned int i = 0; i < open.size(); i++)
        open[i].clear();
    ResultPathInfo resultPath;
    openSize = 0;
    // 初始化全图每个栅格的safe intervals
    constraints->resetSafeIntervals(map.width, map.height);
    // 更新当前智能体起点位置所占据的格子的safe intervals
    // constraints->updateShapeCellSafeIntervals({curagent.start_i, curagent.start_j}, shape_initial_heading, map);
    double round_radius = 0;
    for (const auto &s : shape_ori)
    {
        round_radius = std::max(round_radius, s.x * s.x + s.y * s.y);
    }
    round_radius = std::sqrt(round_radius);
    constraints->updateCellSafeIntervals_Round({curagent.start_i, curagent.start_j}, shape_initial_heading, map);
    // 创建起始节点和目标节点
    Node curNode(curagent.start_i, curagent.start_j, 0, 0), goalNode(curagent.goal_i, curagent.goal_j, CN_INFINITY, CN_INFINITY);
    Node startNode(curagent.start_i, curagent.start_j, 0, 0);
    curNode.F = getHValue(curNode.i, curNode.j);
    curNode.interval = constraints->getSafeInterval(curNode.i, curNode.j, 0);
    // 初始朝向角
    curNode.heading = curagent.start_heading;
    // 将起始节点加入open列表
    open[curNode.i].push_back(curNode);
    openSize++;

    // std::cout << "---测试区域---" << std::endl; 

    // // std::vector<Location> test_shape = {Location(0,2), Location(1,0), Location(0,-1), Location(-1,0)};
    // std::vector<Location> test_shape = {Location(0,2), Location(-1,0), Location(0,-1), Location(1,0)};
    // // std::vector<Location> test_shape = {Location(0,1), Location(1,-1), Location(-1,-1)};
    // // std::vector<Location> test_shape = {Location(1,1), Location(1,-1), Location(-1,-1), Location(-1,1)};
    // Shape_Collide s_temp;
    // // std::vector<std::pair<int,int>> cells1 = lineofsight.getCellsCrossedByPolyon(47,81,50,74,test_shape, map);
    // // Location p = Location(31,4);
    // // std::vector<Location> rot_shape1 = s_temp.rotate_shape(p, 60, test_shape);
    // // for (auto v : rot_shape1)
    // // {
    // //     std::cout << "Rotated Shape Point: (" << v.x << "," << v.y << ")" << std::endl;
    // // }

    // // std::vector<Location> test_shape = {Location(0,1), Location(-1,-1), Location(1,-1)};
    // // auto s1 = std::chrono::high_resolution_clock::now();
    // std::vector<std::pair<int,int>> cells1 = lineofsight.getCellsOriginByPolyon_Optimized(5,81,test_shape, map);
    // // std::vector<std::pair<int,int>> cells1 = lineofsight.getCellsCrossedByLine_Amanatides_Woo(0,0,2,1, map);
    // // std::vector<std::pair<int,int>> cells1 = lineofsight.getCellsCrossedByLine_Amanatides_Woo(4,4,6,9, map);
    // // std::set<std::pair<int, int>> cells1 = constraints->multiAngleSampleCells_DuringRot(59, 35, 143.1301, 0, 10, test_shape,  map);
    // // std::vector<std::pair<int,int>> cells1 = lineofsight.getCellsCrossedByPolyon(19,4,21,14,test_shape, map);
    // // auto e1 = std::chrono::high_resolution_clock::now();
    // // std::chrono::duration<double, std::milli> duration1 = e1 - s1;
    // // std::cout << "Time for int using time: " << duration1.count() << " (ms)" << std::endl;
    // for (auto p: cells1)
    // {
    //     std::cout << "(" << p.first << "," << p.second << ")"<< std::endl;
    // }

    // // auto s2 = std::chrono::high_resolution_clock::now();
    // // // std::vector<std::pair<int,int>> cells2 = lineofsight.getCellsOriginByPolyon_double(2.2,2.2,test_shape, map);
    // // std::vector<std::pair<int,int>> cells2 = lineofsight.getCellsCrossedByLine_double(0.1,0.2,2.2,1.1, map);
    // // auto e2 = std::chrono::high_resolution_clock::now();
    // // std::chrono::duration<double, std::milli> duration2 = e2 - s2;
    // // std::cout << "Time for double using time: " << duration2.count() << " (ms)" << std::endl;
    // // for (auto p: cells2)
    // //     {
    // //         std::cout << "(" << p.first << "," << p.second << ")"<< std::endl;
    // //     }
    // // std::vector<std::pair<int,int>> cells = lineofsight.getCellsCrossedByPolyon(1,1,4,4,test_shape, map);
    // // std::vector<std::pair<int,int>> cells = lineofsight.getCellsCrossedByLine_Amanatides_Woo(0,0,2,1, map);

    // // LineOfSight los(1.6);
    // // std::vector<std::pair<int,int>> cells = los.getCells(0,0);
    // // std::cout << "radius = 1.6"<< std::endl;
    // // for (auto p: cells)
    // //     {
    // //         std::cout << "(" << p.first << "," << p.second << ")"<< std::endl;
    // //     }

    // // int i1 = 18;
    // // int j1 = 63;
    // // int i2 = 12;
    // // int j2 = 56;

    // // bool aa = lineofsight.checkShapeLine(i1, j1, i2, j2, test_shape, map);
    // // std::cout << "checkShapeLine(" << i1 << ", " << j1 << ", " << i2 << ", " << j2 << ") = " << aa << std::endl;

    // // std::set<std::pair<int, int>> BB;

    // // BB =  lineofsight.getCellsOriginByPolyon(i1, j1, test_shape, map);
    // // std::cout << "getCellsOriginByPolyon() of (" << i1 << ", " << j1 << ")"<< std::endl;
    // // std::cout << "size: " << BB.size() << std::endl;
    // // for (auto it = BB.begin(); it != BB.end(); ++it)
    // // {
    // //     std::cout << it->first << " " << it->second << std::endl;
    // // }

    // // std::set<std::pair<int, int>> CC;
    // // CC = lineofsight.getCellsCrossedByPolyon(i1, j1, i2, j2, test_shape, map);
    // // std::cout << "getCellsCrossedByPolyon() of (" << i1 << ", " << j1 << ") to (" << i2 << ", " << j2 << ")"<< std::endl;
    // // std::cout << "size: " << CC.size() << std::endl;
    // // for (auto it = CC.begin(); it != CC.end(); ++it)
    // // {
    // //     std::cout << it->first << " " << it->second << std::endl;
    // // }

    // std::cout << "---测试区域结束---" << std::endl;
    // return true;
    // std::cout << "---测试区域结束---" << std::endl;

    // 主循环
    // auto s2 = std::chrono::high_resolution_clock::now();
    int count = 0;
    while (!stopCriterion(curNode, goalNode))
    {
        count++;
        // 找到open列表中成本最小的节点
        curNode = findMin(map.height);
        // std::cout << "curNode.F :" << curNode.F << std::endl;
        // 从open列表中移除当前节点
        open[curNode.i].pop_front();
        openSize--;
        // 将当前节点加入close列表
        close.insert({curNode.i * map.width + curNode.j, curNode});
        // 生成当前节点的所有合法后继节点 （核心）

        // // auto s5 = std::chrono::high_resolution_clock::now();
        // Node touch_goal = touchGoal(curNode, goalNode, map, shape_ori);
        // // auto e5 = std::chrono::high_resolution_clock::now();
        // // std::chrono::duration<double, std::milli> duration5 = e5 - s5;
        // // std::cout << "-- Time for touchGoal using time: " << duration5.count() << " (ms)" << std::endl;
        // if(touch_goal.i != -1)
        // {
        //     // addOpen(touch_goal);
        //     std::cout << "*Path shortcut found from (" << curNode.i << ", " << curNode.j << ")!" << std::endl;
        //     Node tempNode = touch_goal; // 将这个新发现的终点状态作为最终结果
        //     if (!stopCriterion(tempNode,goalNode))
        //         break;
        // }
        std::list<Node> Successors = findSuccessors(curNode, goalNode, map, shape_ori);

        for (Node s : Successors) // *这里要传入原始shape
        {
            addOpen(s);
        }
    }
    // auto e2 = std::chrono::high_resolution_clock::now();
    // std::chrono::duration<double, std::milli> duration2 = e2 - s2;
    // std::cout << "-- Time for main cycle using time: " << duration2.count() << " (ms)" << std::endl;
    std::cout << "count for whole search: [" << count << "]" << std::endl;

    // 是否已到达终点
    if (goalNode.g < CN_INFINITY)
    {
        // 根据结果生成路径
        makePrimaryPath(goalNode);

        // --- [调试代码开始]：检查生成的 hppath ---
    std::cout << "--- Agent " << curagent.id << " - hppath Debug ---" << std::endl;
    std::cout << "Path found for Agent " << curagent.id << " (Priority: " << numOfCurAgent << ")" << std::endl;
    std::cout << "Total Nodes in hppath: " << hppath.size() << std::endl;
    std::cout << "Path Length (g-value): " << goalNode.g << std::endl;

    for (size_t i = 0; i < hppath.size(); ++i) {
        const Node& node = hppath[i];
        std::cout << "Node [" << i << "]: ";
        std::cout << "Pos: (" << node.i << ", " << node.j << ") ";
        std::cout << "Time (g): " << node.g << " ";
        std::cout << "Heading: " << node.heading << " ";
        // 如果可行，最好能知道父节点，来确认回溯链是否完整
        if (node.Parent != nullptr) {
            std::cout << "Parent: (" << node.Parent->i << ", " << node.Parent->j << ")";
        } else {
            std::cout << "Parent: NULL (Start Node)";
        }
        std::cout << std::endl;

        // 额外的合法性检查（如果可能）：检查路径段是否穿过静态障碍物
        if (i > 0) {
            const Node& prev_node = hppath[i-1];
            // 静态障碍物检查是 los.checkShapeLine 的职责，这里我们只检查节点坐标
            // 假设 Map::CellIsObstacle(i, j) 可以访问
            if (map.CellIsObstacle(node.i, node.j)) {
                 std::cout << "!!! WARNING: Node [" << i << "] at (" << node.i << ", " << node.j << ") is in an OBSTACLE cell! !!!" << std::endl;
            }
        }
    }
    std::cout << "------------------------------------------" << std::endl;

    
#ifdef __linux__
        // 运行时间
        gettimeofday(&end, NULL);
        resultPath.runtime = (end.tv_sec - begin.tv_sec) + static_cast<double>(end.tv_usec - begin.tv_usec) / 1000000;
#else
        QueryPerformanceCounter(&end);
        resultPath.runtime = static_cast<double long>(end.QuadPart - begin.QuadPart) / freq.QuadPart;
#endif
        resultPath.sections = hppath;
        makeSecondaryPath(goalNode);
        resultPath.pathfound = true;
        resultPath.path = lppath;
        resultPath.pathlength = goalNode.g;
        sresult.pathfound = true;
        sresult.flowtime += goalNode.g;
        sresult.makespan = std::max(sresult.makespan, goalNode.g);
        sresult.pathInfo[numOfCurAgent] = resultPath;
        sresult.agentsSolved++;
        // 输出路径
        // std::cout<<"Path for agent "<<curagent.id<<" found!\n";
        // for (auto it = resultPath.path.begin(); it != resultPath.path.end(); it++)
        // {
        //     std::cout << "(" << it->i << ", " << it->j << ") ";
        // }
    }
    else
    {
#ifdef __linux__
        gettimeofday(&end, NULL);
        resultPath.runtime = (end.tv_sec - begin.tv_sec) + static_cast<double>(end.tv_usec - begin.tv_usec) / 1000000;
#else
        QueryPerformanceCounter(&end);
        resultPath.runtime = static_cast<double long>(end.QuadPart - begin.QuadPart) / freq.QuadPart;
#endif
        std::cout << "Path for agent " << curagent.id << " not found!\n";
        sresult.pathfound = false;
        resultPath.pathfound = false;
        resultPath.path.clear();
        resultPath.sections.clear();
        resultPath.pathlength = 0;
        sresult.pathInfo[numOfCurAgent] = resultPath;
    }
    return resultPath.pathfound;
}

std::vector<conflict> AA_SIPP::CheckConflicts(const Task &task)
{
    std::vector<conflict> conflicts(0);
    conflict conf;
    Node cur, check;
    std::vector<std::vector<conflict>> positions;
    positions.resize(sresult.agents);
    for (unsigned int i = 0; i < sresult.agents; i++)
    {
        if (!sresult.pathInfo[i].pathfound)
            continue;
        positions[i].resize(0);
        int k = 0;
        double part = 1;
        for (unsigned int j = 1; j < sresult.pathInfo[i].sections.size(); j++)
        {
            cur = sresult.pathInfo[i].sections[j];
            check = sresult.pathInfo[i].sections[j - 1];
            int di = cur.i - check.i;
            int dj = cur.j - check.j;
            double dist = (cur.g - check.g) * 10;
            int steps = (cur.g - check.g) * 10;
            if (dist - steps + part >= 1)
            {
                steps++;
                part = dist - steps;
            }
            else
                part += dist - steps;
            double stepi = double(di) / dist;
            double stepj = double(dj) / dist;
            double curg = double(k) * 0.1;
            double curi = check.i + (curg - check.g) * di / (cur.g - check.g);
            double curj = check.j + (curg - check.g) * dj / (cur.g - check.g);
            conf.i = curi;
            conf.j = curj;
            conf.g = curg;
            if (curg <= cur.g)
            {
                positions[i].push_back(conf);
                k++;
            }
            while (curg <= cur.g)
            {
                if (curg + 0.1 > cur.g)
                    break;
                curi += stepi;
                curj += stepj;
                curg += 0.1;
                conf.i = curi;
                conf.j = curj;
                conf.g = curg;
                positions[i].push_back(conf);
                k++;
            }
        }
        if (double(k - 1) * 0.1 < sresult.pathInfo[i].sections.back().g)
        {
            conf.i = sresult.pathInfo[i].sections.back().i;
            conf.j = sresult.pathInfo[i].sections.back().j;
            conf.g = sresult.pathInfo[i].sections.back().g;
            positions[i].push_back(conf);
        }
    }
    unsigned int max = 0;
    double sumsize = 0;
    for (unsigned int i = 0; i < positions.size(); i++)
        if (positions[i].size() > max)
            max = positions[i].size();
    for (unsigned int i = 0; i < sresult.agents; i++)
    {
        for (unsigned int k = 0; k < max; k++)
        {
            for (unsigned int j = i + 1; j < sresult.agents; j++)
            {
                if (!sresult.pathInfo[j].pathfound || !sresult.pathInfo[i].pathfound)
                    continue;
                sumsize = task.getAgent(i).size + task.getAgent(j).size;
                conflict a, b;
                if (positions[i].size() > k)
                    a = positions[i][k];
                else
                    a = positions[i].back();
                if (positions[j].size() > k)
                    b = positions[j][k];
                else
                    b = positions[j].back();
                if (sqrt((a.i - b.i) * (a.i - b.i) + (a.j - b.j) * (a.j - b.j)) + CN_EPSILON < sumsize)
                {
                    std::cout << i << " " << j << " " << a.i << " " << a.j << " " << b.i << " " << b.j << " " << sqrt((a.i - b.i) * (a.i - b.i) + (a.j - b.j) * (a.j - b.j)) << "\n";
                    conf.i = b.i;
                    conf.j = b.j;
                    conf.agent1 = i;
                    conf.agent2 = j;
                    conf.g = b.g;
                    conflicts.push_back(conf);
                }
            }
        }
    }
    return conflicts;
}

void AA_SIPP::makePrimaryPath(Node curNode)
{
    hppath.clear();
    hppath.shrink_to_fit();
    std::list<Node> path;
    path.push_front(curNode);
    if (curNode.Parent != nullptr)
    {
        curNode = *curNode.Parent;
        if (curNode.Parent != nullptr)
        {
            do
            {
                path.push_front(curNode);
                curNode = *curNode.Parent;
            } while (curNode.Parent != nullptr);
        }
        path.push_front(curNode);
    }
    for (auto it = path.begin(); it != path.end(); it++)
        hppath.push_back(*it);
    if (config->planforturns && curagent.goal_heading >= 0)
    {
        Node add = hppath.back();
        add.heading = curagent.goal_heading;
        hppath.back().g -= getRCost(hppath.back().heading, curagent.goal_heading);
        hppath.push_back(add);
    }
    for (unsigned int i = 1; i < hppath.size(); i++)
    {
        if ((hppath[i].g - (hppath[i - 1].g + getCost(hppath[i].i, hppath[i].j, hppath[i - 1].i, hppath[i - 1].j) / curagent.mspeed)) > CN_EPSILON)
        {
            Node add = hppath[i - 1];
            add.Parent = hppath[i].Parent;
            add.g = hppath[i].g - getCost(hppath[i].i, hppath[i].j, hppath[i - 1].i, hppath[i - 1].j) / curagent.mspeed;
            add.heading = hppath[i].heading;
            hppath.emplace(hppath.begin() + i, add);
            i++;
        }
    }
    if (config->planforturns && curagent.goal_heading >= 0)
        hppath.pop_back();
    return;
}

void AA_SIPP::makeSecondaryPath(Node curNode)
{
    lppath.clear();
    if (curNode.Parent != nullptr)
    {
        std::vector<Node> lineSegment;
        do
        {
            calculateLineSegment(lineSegment, *curNode.Parent, curNode);
            lppath.insert(lppath.begin(), ++lineSegment.begin(), lineSegment.end());
            curNode = *curNode.Parent;
        } while (curNode.Parent != nullptr);
        lppath.push_front(*lineSegment.begin());
    }
    else
        lppath.push_front(curNode);
}

void AA_SIPP::calculateLineSegment(std::vector<Node> &line, const Node &start, const Node &goal)
{
    int i1 = start.i;
    int i2 = goal.i;
    int j1 = start.j;
    int j2 = goal.j;

    int delta_i = std::abs(i1 - i2);
    int delta_j = std::abs(j1 - j2);
    int step_i = (i1 < i2 ? 1 : -1);
    int step_j = (j1 < j2 ? 1 : -1);
    int error = 0;
    int i = i1;
    int j = j1;
    if (delta_i > delta_j)
    {
        for (; i != i2; i += step_i)
        {
            line.push_back(Node(i, j));
            error += delta_j;
            if ((error << 1) > delta_i)
            {
                j += step_j;
                error -= delta_i;
            }
        }
    }
    else
    {
        for (; j != j2; j += step_j)
        {
            line.push_back(Node(i, j));
            error += delta_i;
            if ((error << 1) > delta_j)
            {
                i += step_i;
                error -= delta_j;
            }
        }
    }
    return;
}
