#include "mapf_ros/aaasipp/constraints.h"

Constraints::Constraints(int width, int height)
{
    safe_intervals.resize(height);
    for(int i = 0; i < height; i++)
    {
        safe_intervals[i].resize(width);
        for(int j = 0; j < width; j++)
        {
            safe_intervals[i][j].resize(0);
            // safe_intervals[i][j].push_back({0,CN_INFINITY});
            safe_intervals[i][j].emplace_back(0, CN_INFINITY);
        }
    }
    constraints.resize(height);
    for(int i = 0; i < height; i++)
    {
        constraints[i].resize(width);
        for(int j = 0; j < width; j++)
            constraints[i][j].resize(0);
    }
}

bool sort_function(std::pair<double, double> a, std::pair<double, double> b)
{
    return a.first < b.first;
}

double Constraints::minDist(Point A, Point C, Point D)
{
    int classA = A.classify(C, D);
    if(classA == 3)
        return sqrt(pow(A.i - C.i, 2) + pow(A.j - C.j, 2));
    else if(classA == 4)
        return sqrt(pow(A.i - D.i, 2) + pow(A.j - D.j, 2));
    else
        return fabs((C.i - D.i)*A.j + (D.j - C.j)*A.i + (C.j*D.i - D.j*C.i))/sqrt(pow(C.i - D.i, 2) + pow(C.j - D.j, 2));
}

void Constraints::resetSafeIntervals(int width, int height)
{
    safe_intervals.resize(height);
    for(int i = 0; i < height; i++)
    {
        safe_intervals[i].resize(width);
        for(int j = 0; j < width; j++)
        {
            safe_intervals[i][j].resize(0);
            // 将全图所有栅格的safe interval设置为(0, inf)
            safe_intervals[i][j].push_back({0,CN_INFINITY});
        }
    }
}

void Constraints::updateShapeCellSafeIntervals(std::pair<int, int> cell,  const std::vector<Location> &shape, const Map &map) // 更新cells的安全区间
{    
    // 传入的是shape_cur，沿着当前heading旋转的
    // std::cout << "Enter updateShapeCellSafeIntervals" <<std::endl;
    // 如果该agent起点位置有多个safe intervals，返回
    if(safe_intervals[cell.first][cell.second].size() > 1)
        return;
    // 获取所占据的单元格的相对坐标
    // double d = 0;
    // for (auto p: shape)
    // {
    //     double dis = p.x*p.x + p.y*p.y;
    //     d = std::max(d, dis);
    // }
    // d = sqrt(d);
    // // std::cout << "This agent  max radius is :" << d<<  std::endl;

    // agentsize = d;
    LineOfSight los(agentsize);

    // std::set<std::pair<int,int>> cells_shape = los.getCellsOriginByPolyon(cell.first, cell.second, shape ,map);
    std::vector<std::pair<int,int>> cells = los.getCellsOriginByPolyon_Optimized(cell.first, cell.second, shape ,map);
    // // 将cells_shape转化成vector
    // std::vector<std::pair<int,int>> cells;
    // for(auto it = cells_shape.begin(); it != cells_shape.end(); it++)
    // {
    //     // std::cout << "(" << it->first << ", " << it->second << ")" << std::endl;
    //     cells.push_back(std::pair<int,int>{it->first, it->second});
    // }
    std::vector<section> secs;
    // 遍历agent所占据的每个栅格，将这些栅格所有的不重复的constraints 添加到secs
    for(int k = 0; k < cells.size(); k++)
        for(int l = 0; l < constraints[cells[k].first][cells[k].second].size(); l++)
            if(std::find(secs.begin(), secs.end(), constraints[cells[k].first][cells[k].second][l]) == secs.end())
                {
                    secs.push_back(constraints[cells[k].first][cells[k].second][l]);
                    // std::cout << "A constraint in (" << cells[k].first << ", " << cells[k].second << ")" << std::endl;
                }

    Shape_Collide agentPoly ;
    Location shape_center(cell.first, cell.second);
    std::vector<Location> shape_abs(shape.size());
    for (int i = 0; i < shape.size(); i++)
        shape_abs[i] = shape[i] + shape_center;
    

    // -> Start 对每个约束计算距离多边形的安全距离、更新安全区间
    // 遍历secs中的每个约束，计算agent与该约束的安全距离，并根据这个距离更新安全区间。
    // secs是cells中包含的约束, 与agent肯定是相交或者接近相交
    
    agentPoly.getShapeInfo(shape_center, shape_abs, agentsize);

    // for(int k = 0; k < secs.size(); k++)
    // {     
    for(const auto& sec : secs)
    {
        // std::cout << "In updateShapeCellSafeIntervals() " <<std::endl;
        // section sec = secs[k];
        double radius = agentsize + sec.size; // 安全距离 = 该agent半径+对方半径 已修改为最远角点距离-即包络圆距离
        // i0j0是sec的起点 i1j1是sec的终点 i2j2是agent的坐标
        int i0(sec.i1), j0(sec.j1), i1(sec.i2), j1(sec.j2), i2(cell.first), j2(cell.second);
        SafeInterval interval; // 被检查的【不安全区间】
        double dist, mindist;
        Location Line_start(i0, j0), Line_end(i1, j1);
        // edit from here
        // 计算代理与约束之间的距离，并根据这个距离以及约束的运动信息来确定不安全的时间段，进而更新安全间隔
        if(i0 == i1 && j0 == j1 && i0 == i2 && j0 == j2) // 跟人家起点重合了吧
            mindist = 0;
        else
        {
            // mindist = minDist(Point(i2,j2), Point(i0,j0), Point(i1,j1));
            mindist = agentPoly.LinePolygonDistance( Line_start, Line_end, shape_abs );
            // std::cout << "mindist with sec " << Line_start << " " << Line_end << " is " <<mindist << std::endl;
        }
        if(mindist >= radius) // 最小距离大于半径之和，这段sec对于该agent是安全的，无需进行其他操作，跳过该sec；
            continue;
        if(mindist < 0)
        {
            // 说明线段端点在多边形内
            std::vector<Location> boundary_points;
            bool StartInPoly = agentPoly.isPointInPolygon(Line_start, shape_abs);
            bool EndInPoly = agentPoly.isPointInPolygon(Line_end, shape_abs);

            // 如果只有一个端点，寻找距离该端点最远的边界点，列为危险区间；
            if (StartInPoly && !EndInPoly) // 起点在多边形内，终点在多边形外
            {
                boundary_points.clear();
                // std::vector<Location> dis_boundary_points = findDisBoundaryPointsWithPolygon(Line_start, Line_end, shape_abs);
                boundary_points = agentPoly.find_dis_boundary_points(shape_abs, Line_start, Line_end, radius);
                if (boundary_points.size() == 0)
                {
                    interval.begin = sec.g1;
                    interval.end = sec.g2;
                }
                else
                {
                    Location far_point;
                    double far_dis = 0;
                    double dis_temp = 0;
                    for (auto point : boundary_points)
                    {
                        dis_temp = agentPoly.distance(Line_start, point);
                        if (dis_temp > far_dis)
                        {
                            far_dis = dis_temp;
                            far_point = point;
                        }
                    }
                    interval.begin = sec.g1;
                    interval.end = sec.g1 + far_dis/sec.mspeed;
                }
            }
            else if (!StartInPoly && EndInPoly) // 起点在多边形外，终点在多边形内
            {
                boundary_points.clear();
                // std::vector<Location> dis_boundary_points = findDisBoundaryPointsWithPolygon(Line_start, Line_end, shape_abs);
                boundary_points = agentPoly.find_dis_boundary_points(shape_abs, Line_start, Line_end, radius);
                if (boundary_points.size() == 0)
                {
                    interval.begin = sec.g1;
                    interval.end = sec.g2;
                }
                else{
                    Location far_point;
                    double far_dis = 0;
                    double dis_temp = 0;
                    for (auto point : boundary_points)
                    {
                        dis_temp = agentPoly.distance(Line_end, point);
                        if (dis_temp > far_dis)
                        {
                            far_dis = dis_temp;
                            far_point = point;
                        }
                    }
                    interval.begin = sec.g2 - far_dis/sec.mspeed;
                    interval.end = sec.g2;
                }
            }
            // 如果两个端点都在多边形内，整个sec作为危险区间；
            else if (StartInPoly && EndInPoly)
            {
                // 不安全区间设置为sec的整个时间
                interval.begin = sec.g1;
                interval.end = sec.g2;
            } 
        }
        else // 线段端点不在多边形内，最小距离小于半径之和
        {
            std::vector<Location> boundary_points;
            boundary_points = agentPoly.find_dis_boundary_points(shape_abs, Line_start, Line_end, radius);
            // 考虑线段起点终点是否与多边形在安全半径内，如果都在，则不安全区间为sec的整个时间；
            if (boundary_points.empty())
            {
                interval.begin = sec.g1;
                interval.end = sec.g2;
            }
            else if (boundary_points.size() == 1)
            {
                if (agentPoly.distance_point_to_polygon(Line_start, shape_abs) < radius)
                {
                    interval.begin = sec.g1;
                    interval.end = sec.g1 + agentPoly.distance(boundary_points[0], Line_start)/sec.mspeed;
                }
                else if (agentPoly.distance_point_to_polygon(Line_end, shape_abs) < radius)
                {
                    interval.begin = sec.g2 - agentPoly.distance(boundary_points[0], Line_end)/sec.mspeed;
                    interval.end = sec.g2;
                }
            }
            else if (boundary_points.size() >= 2)
            {
                double dis_start = agentPoly.distance_point_to_polygon(Line_start, shape_abs);
                double dis_end = agentPoly.distance_point_to_polygon(Line_end, shape_abs);

                if (dis_start < radius)
                {
                    // 找距离起点最远的边界点
                    double far_dis = 0;
                    double dis_temp = 0;

                    for (const auto& point : boundary_points)
                    {
                        far_dis = std::max(far_dis, agentPoly.distance(Line_start, point));
                    }
                    interval.begin = sec.g1;
                    interval.end = sec.g1 + far_dis/sec.mspeed;
                }
                else if (dis_end < radius)
                {
                    // 找距离终点最远的边界点
                    double far_dis = 0;
                    double dis_temp = 0;
                    for (auto point : boundary_points)
                    {
                        far_dis = std::max(far_dis, agentPoly.distance(Line_end, point));
                    }
                    interval.begin = sec.g2 - far_dis/sec.mspeed;
                    interval.end = sec.g2;
                }
                else {
                    // 找距离起点最近的边界点
                    //double的最大值
                    double near_disA = std::numeric_limits<double>::max() ; 
                    for (auto point : boundary_points)
                    {
                        near_disA = std::min(near_disA, agentPoly.distance(Line_start, point));
                    }
                    interval.begin = sec.g1 + near_disA/sec.mspeed;

                    // 找距离终点最近的边界点
                    double near_disB = std::numeric_limits<double>::max() ;
                    //double的最大值
                    for (auto point : boundary_points)
                    {
                        near_disB = std::min(near_disB, agentPoly.distance(Line_end, point));
                    }
                    interval.end = sec.g2 - near_disB/sec.mspeed;
                }
            }

        }

        // 从当前单元格的现有安全时间间隔中“减去”或“切割”掉新计算出的不安全时间间隔 interval。
        // 它遍历 safe_intervals[i2][j2]（存储了当前单元格的安全间隔的向量），
        // 并根据 interval 与现有安全间隔的重叠情况进行更新。
        for(unsigned int j = 0; j < safe_intervals[i2][j2].size(); j++)
        {
            // 当前cell的安全区间比sec计算得到的不安全区间起点更早终点更晚，即重叠了
            if(safe_intervals[i2][j2][j].begin < interval.begin + CN_EPSILON && safe_intervals[i2][j2][j].end + CN_EPSILON > interval.begin)
            {
                // 如果安全区间起点和不安全区间起点相同
                if(fabs(safe_intervals[i2][j2][j].begin - interval.begin) < CN_EPSILON)
                {
                    // 插入一个零长度的安全间隔，起点和终点都设为当前安全间隔的起点。
                    safe_intervals[i2][j2].insert(safe_intervals[i2][j2].begin() + j, SafeInterval(safe_intervals[i2][j2][j].begin,safe_intervals[i2][j2][j].begin));
                    j++; // 插入后，当前索引 j 需要递增，以指向原先的元素
                    if(safe_intervals[i2][j2][j].end < interval.end)
                    {
                        // 如果当前安全区间的结束时间 在 不安全区间的结束时间之前，则删除当前安全区间
                        safe_intervals[i2][j2].erase(safe_intervals[i2][j2].begin() + j);
                        j--; // Bug修复：确保不跳过下一个元素
                    } 
                    else
                        safe_intervals[i2][j2][j].begin = interval.end; // 否则，将当前安全区间的开始时间设置为不安全区间的结束时间
                }
                else if(safe_intervals[i2][j2][j].end < interval.end)
                    safe_intervals[i2][j2][j].end = interval.begin;
                else
                {
                    std::pair<double,double> new1, new2;
                    new1.first = safe_intervals[i2][j2][j].begin;
                    new1.second = interval.begin;
                    new2.first = interval.end;
                    new2.second = safe_intervals[i2][j2][j].end;
                    safe_intervals[i2][j2].erase(safe_intervals[i2][j2].begin() + j);
                    if(new2.first < CN_INFINITY)
                        safe_intervals[i2][j2].insert(safe_intervals[i2][j2].begin() + j, SafeInterval(new2.first, new2.second));
                    safe_intervals[i2][j2].insert(safe_intervals[i2][j2].begin() + j, SafeInterval(new1.first, new1.second));
                }
            }
            // 该cell的安全区间 在 不安全区间内
            else if(safe_intervals[i2][j2][j].begin > interval.begin - CN_EPSILON && safe_intervals[i2][j2][j].begin < interval.end)
            {
                // 安全区间起点与不安全区间起点相同（或非常接近）
                if(fabs(safe_intervals[i2][j2][j].begin - interval.begin) < CN_EPSILON)
                {
                    // 插入一个零长度的安全间隔 
                    safe_intervals[i2][j2].insert(safe_intervals[i2][j2].begin() + j, SafeInterval(safe_intervals[i2][j2][j].begin,safe_intervals[i2][j2][j].begin));
                    j++;
                }
                // 安全区间完全被不安全区间吞噬（或安全区间结束于不安全区间内部）
                if(safe_intervals[i2][j2][j].end < interval.end)
                {
                    safe_intervals[i2][j2].erase(safe_intervals[i2][j2].begin() + j); // 删除安全区间
                    j--; // Bug修复：确保不跳过下一个元素
                }
                else // 不安全区间只覆盖安全区间的前半部分
                {
                    safe_intervals[i2][j2][j].begin = interval.end; // 移除重叠的不安全部分
                }
            }
        }
        // 更新安全间隔的 ID
        for(unsigned int j = 0; j < safe_intervals[i2][j2].size(); j++)
            safe_intervals[i2][j2][j].id = j;
    }


    //    // 如果该单元格的安全区间已经被分割过，说明已经处理完毕，直接返回以避免重复计算
    // if (safe_intervals[cell.first][cell.second].size() > 1)
    //     return;

    // // --- 1. 准备工作：获取静态智能体A占据的栅格 ---
    // LineOfSight los(agentsize); // LineOfSight 实例
    // Shape_Collide sc_temp;     // Shape_Collide 实例
    // Location ori = {0, 0};
    
    // // 获取智能体A静止在 cell 位置时占据的栅格
    // // 注意：这里的 shape 应该是根据到达该 cell 时的朝向旋转过的
    // // 在 findPath 中调用时传入的是 shape_initial_heading，这是对的。
    // // 在 findSuccessors 中调用时传入的是 shape_cur，这也是对的。
    // std::vector<std::pair<int, int>> static_shape_cells = los.getCellsOriginByPolyon_Optimized(cell.first, cell.second, shape, map);

    // if (static_shape_cells.empty()) return; // 如果不占据任何栅格，则不会有碰撞

    // // --- 2. 收集所有可能相关的约束 ---
    // // 为了优化，我们先收集所有可能影响到这些静态栅格的约束
    // std::vector<section> relevant_constraints;
    // std::set<section> unique_constraints; // 使用 set 自动去重
    // for (const auto& occupied_cell : static_shape_cells)
    // {
    //     for (const auto& constraint : constraints[occupied_cell.first][occupied_cell.second])
    //     {
    //         unique_constraints.insert(constraint);
    //     }
    // }
    // for(const auto& sec : unique_constraints)
    // {
    //     relevant_constraints.push_back(sec);
    // }

    // // --- 3. 遍历每个约束，计算精确的不安全时间窗口 ---
    // for (const auto& sec : relevant_constraints)
    // {
    //     double t1 = sec.g1;
    //     double t2 = sec.g2;

    //     // 定义时间步长。一个好的选择是智能体移动半个栅格所需的时间
    //     double time_step = 0.5 / sec.mspeed; 
    //     if (sec.mspeed < CN_EPSILON) { // 处理速度为0的约束
    //          time_step = 1.0; // 给一个默认值
    //     }

    //     double t_unsafe_start = -1.0;
    //     double t_unsafe_end = -1.0;

    //     Vector2D B_start_pos(sec.i1, sec.j1);
    //     Vector2D VB;
    //     if (t2 - t1 > CN_EPSILON) {
    //         VB = Vector2D((sec.i2 - sec.i1) / (t2 - t1), (sec.j2 - sec.j1) / (t2 - t1));
    //     } else {
    //         VB = Vector2D(0, 0); // 约束本身是静止的
    //     }
        
    //     // 获取移动智能体B的旋转后形状
    //     double heading_B = calcHeading_constraints(sec.i1, sec.j1, sec.i2, sec.j2);
    //     std::vector<Location> shape_ = sec.shape_ori;
    //     std::vector<Location> shape_B_rotated = sc_temp.rotate_shape(ori, heading_B, shape_);

    //     // --- 4. 时间步进模拟 ---
    //     for (double t = t1; t <= t2; t += time_step)
    //     {
    //         Vector2D B_pos_t = B_start_pos + VB * (t - t1);
    //         std::vector<std::pair<int, int>> moving_shape_cells_t = los.getCellsOriginByPolyon_Optimized(
    //             round(B_pos_t.i), round(B_pos_t.j), shape_B_rotated, map);
            
    //         bool collision_detected = false;
    //         // 检查两个栅格集合是否有交集
    //         for (const auto& cell_A : static_shape_cells) {
    //             for (const auto& cell_B : moving_shape_cells_t) {
    //                 if (cell_A == cell_B) {
    //                     collision_detected = true;
    //                     break;
    //                 }
    //             }
    //             if (collision_detected) break;
    //         }

    //         if (collision_detected) {
    //             if (t_unsafe_start < 0) { // 记录第一次碰撞的时间
    //                 t_unsafe_start = t;
    //             }
    //             t_unsafe_end = t; // 不断更新最后一次碰撞的时间
    //         }
    //     }

    //     // --- 5. 如果找到了不安全区间，就裁剪 safe_intervals ---
    //     if (t_unsafe_start >= 0) {
    //         SafeInterval unsafe_interval;
    //         unsafe_interval.begin = t_unsafe_start;
    //         // 因为步进有误差，结束时间可能需要稍微延长一个步长以确保安全
    //         unsafe_interval.end = t_unsafe_end + time_step;
            
    //         // 使用和原版代码完全相同的逻辑来裁剪安全区间
    //         // 注意：这里的裁剪目标是 cell 本身的安全区间，而不是所有占据的栅格
    //         int i2 = cell.first;
    //         int j2 = cell.second;
    //         for (unsigned int j = 0; j < safe_intervals[i2][j2].size(); j++) {
    //             if (safe_intervals[i2][j2][j].begin < unsafe_interval.end && safe_intervals[i2][j2][j].end > unsafe_interval.begin) {
    //                 if (safe_intervals[i2][j2][j].begin >= unsafe_interval.begin) {
    //                     if (safe_intervals[i2][j2][j].end <= unsafe_interval.end) {
    //                         safe_intervals[i2][j2].erase(safe_intervals[i2][j2].begin() + j);
    //                         j--;
    //                     } else {
    //                         safe_intervals[i2][j2][j].begin = unsafe_interval.end;
    //                     }
    //                 } else {
    //                     if (safe_intervals[i2][j2][j].end <= unsafe_interval.end) {
    //                         safe_intervals[i2][j2][j].end = unsafe_interval.begin;
    //                     } else {
    //                         safe_intervals[i2][j2].insert(safe_intervals[i2][j2].begin() + j + 1, SafeInterval(unsafe_interval.end, safe_intervals[i2][j2][j].end));
    //                         safe_intervals[i2][j2][j].end = unsafe_interval.begin;
    //                     }
    //                 }
    //             }
    //         }
    //     }
    // }
    
    // // --- 6. 最后，为所有新生成的安全区间重新编号 ---
    // int i2 = cell.first;
    // int j2 = cell.second;
    // for (unsigned int j = 0; j < safe_intervals[i2][j2].size(); j++) {
    //     safe_intervals[i2][j2][j].id = j;
    // }
}

void Constraints::updateCellSafeIntervals_Round(std::pair<int, int> cell, const std::vector<Location> &shape, const Map &map) // 更新cells的安全区间
{
   // 如果该agent起点位置有多个safe intervals，返回
    if(safe_intervals[cell.first][cell.second].size() > 1)
        return;
  
    double addition_safeDis = 0.0; // 额外膨胀距离
    double round_radius = 0;
    for (const auto& s : shape)
    {
        round_radius = std::max(round_radius, s.x*s.x + s.y*s.y);
    }
    round_radius = std::sqrt(round_radius) ; 
    

    LineOfSight los(round_radius); // 25.09.27 输入agentSize半径    
    // std::vector<std::pair<int, int>> cells = los.getCellsOriginByPolyon_Optimized(cell.first, cell.second, shape, map); // agent所占据的栅格（相对坐标）
    std::vector<std::pair<int, int>> cells2 = los.getCells(cell.first, cell.second); // agent所占据的栅格（相对坐标）
    std::vector<std::pair<int, int>> cells;
    for (auto c: cells2)
    {
        if (map.CellOnGrid(c.first, c.second ))
        {
            cells.push_back(std::make_pair(c.first , c.second ));
            // std::cout << "(" << c.first << "," << c.second << ") " ;
        }       
    }
    
    std::vector<section> secs;
    // 遍历agent所占据的每个栅格，将这些栅格所有的不重复的constraints 添加到secs
    for(int k = 0; k < cells.size(); k++)
        for(int l = 0; l < constraints[cells[k].first][cells[k].second].size(); l++)
            if(std::find(secs.begin(), secs.end(), constraints[cells[k].first][cells[k].second][l]) == secs.end())
                secs.push_back(constraints[cells[k].first][cells[k].second][l]);
    
    // -> Start 对每个约束计算安全距离、更新安全区间
    // 遍历secs中的每个约束，计算agent与该约束的安全距离，并根据这个距离更新安全区间。
    // 25.09.25 修改为包络圆距离
    agentsize = round_radius; // 包络圆距离


    for(int k = 0; k < secs.size(); k++)
    {
        section sec = secs[k];

        double radius = 1.0 * (agentsize + sec.size + addition_safeDis); // 怀疑AA-SIPP的size是直径 -> 破案了，输入的agentSize是半径就行
        // i0j0是sec的起点 i1j1是sec的终点 i2j2是cell的坐标
        int i0(secs[k].i1), j0(secs[k].j1), i1(secs[k].i2), j1(secs[k].j2), i2(cell.first), j2(cell.second);
        SafeInterval interval; // 被检查的不安全区间
        double dist, mindist;
        // 计算代理与约束之间的距离，并根据这个距离以及约束的运动信息来确定不安全的时间段，进而更新安全间隔
        if(i0 == i1 && j0 == j1 && i0 == i2 && j0 == j2) // sec是一个在cell处原地等待的section
            mindist = 0;
        else
            mindist = minDist(Point(i2,j2), Point(i0,j0), Point(i1,j1));
        if(mindist >= radius) // 最小距离大于半径之和，这段sec对于该agent是安全的，无需进行其他操作，跳过该sec；
            continue;
        // 详细碰撞时间计算：
        Point point(i2,j2), p0(i0,j0), p1(i1,j1);
        int cls = point.classify(p0, p1); // 判断点 point (代理) 相对于线段 p0p1 (约束) 的位置关系（例如，在线段前方、后方、在线段上等）
        // 计算点 (i2,j2) 到直线 (i0,j0)-(i1,j1) 的距离。
        dist = fabs((i0 - i1)*j2 + (j1 - j0)*i2 + (j0*i1 - i0*j1))/sqrt(pow(i0 - i1, 2) + pow(j0 - j1, 2));
        // 距离平方
        int da = (i0 - i2)*(i0 - i2) + (j0 - j2)*(j0 - j2);
        int db = (i1 - i2)*(i1 - i2) + (j1 - j2)*(j1 - j2);
        // ha 是点 p0 到代理在sec上投影点的距离。size 是基于 radius 和 dist 计算出的一个“碰撞窗口”大小。
        double ha = sqrt(std::max(0.0, da - dist*dist));
        double size = sqrt(std::max(0.0, radius*radius - dist*dist));
        // 根据 cls 的值以及 da 和 db 与 radius*radius 的关系，精确计算出不安全的时间间隔 
        // 目的是找出代理何时会与移动的约束发生接触。sec.g1 和 sec.g2 可能代表约束开始和结束的时间
        if(cls == 3) 
        {
            // cell在sec延长线终点外侧，所以不安全区间终点可以加上一段时间
            // 其实这种 延长线的内外侧，应该是由于其他有体积的agent占据的边缘cells导致的
            interval.begin = sec.g1;
            interval.end = sec.g1 + (sqrt(radius*radius - dist*dist) - ha)/sec.mspeed;
        }
        else if(cls == 4)
        {
            // cell在sec延长线起点外侧，所以不安全区间起点可以减去一段时间

            // 25.10.16 注释掉了这行，改为了aa-sipp中的形式
            // 更换了之后 ，bug15好像修复了。。。。。。。。。。。。。。。。。。。。。。。。。
            // interval.begin = sec.g2 - size/sec.mspeed + sqrt(std::max(0.0, db - dist*dist))/sec.mspeed;
            // 原函数是下面这个
            interval.begin = sec.g2 - sqrt(radius*radius - dist*dist)/sec.mspeed + sqrt(db - dist*dist)/sec.mspeed;
            // std::cout << "-------" << std::endl;
            // std::cout << " size: " << size << " sqrt: " << sqrt(radius*radius - dist*dist)/sec.mspeed << std::endl;
            // std::cout << "sqrt interval.begin: " << interval.begin << std::endl;
            // std::cout << "ori " << sec.g2 - size/sec.mspeed + sqrt(std::max(0.0, db - dist*dist))/sec.mspeed << std::endl;
            interval.end = sec.g2;
        }
        else if(da < radius*radius) // 和sec起点的距离已经小于安全距离（该agent半径+对方半径）了
        {
            if(db < radius*radius) // 和sec终点的距离也小于安全距离了
            {
                // 不安全区间设置为sec的整个时间
                interval.begin = sec.g1;
                interval.end = sec.g2;
            }
            else
            {
                // 不安全区间终点减去一段时间
                double hb = sqrt(std::max(0.0, db - dist*dist));
                // double hb = sqrt(db - dist*dist); // 投影点到sec终点的距离 
                interval.begin = sec.g1;
                interval.end = sec.g2 - hb/sec.mspeed + size/sec.mspeed;
            }
        }
        else // 和sec起点的距离大于安全距离
        {
            if(db < radius*radius) // 和sec终点的距离小于安全距离
            {
                interval.begin = sec.g1 + ha/sec.mspeed - size/sec.mspeed;
                interval.end = sec.g2;
            }
            else // 和sec起点、终点都大于安全距离
            {
                interval.begin = sec.g1 + ha/sec.mspeed - size/sec.mspeed;
                interval.end = sec.g1 + ha/sec.mspeed + size/sec.mspeed;
            }
        }
        // 从当前单元格的现有安全时间间隔中“减去”或“切割”掉新计算出的不安全时间间隔 interval。
        // 它遍历 safe_intervals[i2][j2]（存储了当前单元格的安全间隔的向量），
        // 并根据 interval 与现有安全间隔的重叠情况进行更新。
        for(unsigned int j = 0; j < safe_intervals[i2][j2].size(); j++)
        {
            // 当前cell的安全区间比sec计算得到的不安全区间起点更早终点更晚，即重叠了
            if(safe_intervals[i2][j2][j].begin < interval.begin + CN_EPSILON && safe_intervals[i2][j2][j].end + CN_EPSILON > interval.begin)
            {
                // 如果安全区间起点和不安全区间起点相同
                if(fabs(safe_intervals[i2][j2][j].begin - interval.begin) < CN_EPSILON)
                {
                    // 插入一个零长度的安全间隔，起点和终点都设为当前安全间隔的起点。
                    safe_intervals[i2][j2].insert(safe_intervals[i2][j2].begin() + j, SafeInterval(safe_intervals[i2][j2][j].begin,safe_intervals[i2][j2][j].begin));
                    j++; // 插入后，当前索引 j 需要递增，以指向原先的元素
                    if(safe_intervals[i2][j2][j].end < interval.end) // 如果当前安全区间的结束时间 在 不安全区间的结束时间之前，则删除当前安全区间
                        {
                            safe_intervals[i2][j2].erase(safe_intervals[i2][j2].begin() + j);
                        }
                    else
                        safe_intervals[i2][j2][j].begin = interval.end; // 否则，将当前安全区间的开始时间设置为不安全区间的结束时间
                }
                else if(safe_intervals[i2][j2][j].end < interval.end)
                    safe_intervals[i2][j2][j].end = interval.begin;
                else
                {
                    std::pair<double,double> new1, new2;
                    new1.first = safe_intervals[i2][j2][j].begin;
                    new1.second = interval.begin;
                    new2.first = interval.end;
                    new2.second = safe_intervals[i2][j2][j].end;
                    safe_intervals[i2][j2].erase(safe_intervals[i2][j2].begin() + j);
                    if(new2.first < CN_INFINITY)
                        safe_intervals[i2][j2].insert(safe_intervals[i2][j2].begin() + j, SafeInterval(new2.first, new2.second));
                    safe_intervals[i2][j2].insert(safe_intervals[i2][j2].begin() + j, SafeInterval(new1.first, new1.second));
                }
            }
            // 该cell的安全区间 在 不安全区间内
            else if(safe_intervals[i2][j2][j].begin > interval.begin - CN_EPSILON && safe_intervals[i2][j2][j].begin < interval.end)
            {
                // 安全区间起点与不安全区间起点相同（或非常接近）
                if(fabs(safe_intervals[i2][j2][j].begin - interval.begin) < CN_EPSILON)
                {
                    // 插入一个零长度的安全间隔 
                    safe_intervals[i2][j2].insert(safe_intervals[i2][j2].begin() + j, SafeInterval(safe_intervals[i2][j2][j].begin,safe_intervals[i2][j2][j].begin));
                    j++;
                }
                // 安全区间完全被不安全区间吞噬（或安全区间结束于不安全区间内部）
                if(safe_intervals[i2][j2][j].end < interval.end)
                {
                    safe_intervals[i2][j2].erase(safe_intervals[i2][j2].begin() + j); // 删除安全区间
                }
                else // 不安全区间只覆盖安全区间的前半部分
                {
                    safe_intervals[i2][j2][j].begin = interval.end; // 移除重叠的不安全部分
                }
            }
        }
        // 更新安全间隔的 ID
        for(unsigned int j = 0; j < safe_intervals[i2][j2].size(); j++)
            safe_intervals[i2][j2][j].id = j;
    }
}

void Constraints::updateCellSafeIntervals(std::pair<int, int> cell) // 更新cells的安全区间
{
    // 如果该agent起点位置有多个safe intervals，返回
    if(safe_intervals[cell.first][cell.second].size() > 1)
        return;
    // 获取所占据的单元格的相对坐标
    LineOfSight los(agentsize);
    std::vector<std::pair<int, int>> cells = los.getCells(cell.first, cell.second); // agent所占据的栅格（相对坐标）
    std::vector<section> secs;
    // 遍历agent所占据的每个栅格，将这些栅格所有的不重复的constraints 添加到secs
    for(int k = 0; k < cells.size(); k++)
        for(int l = 0; l < constraints[cells[k].first][cells[k].second].size(); l++)
            if(std::find(secs.begin(), secs.end(), constraints[cells[k].first][cells[k].second][l]) == secs.end())
                secs.push_back(constraints[cells[k].first][cells[k].second][l]);

    // -> Start 对每个约束计算安全距离、更新安全区间
    // 遍历secs中的每个约束，计算agent与该约束的安全距离，并根据这个距离更新安全区间。
    for(int k = 0; k < secs.size(); k++)
    {
        section sec = secs[k];
        double radius = agentsize + sec.size; // 安全距离 = 该agent半径+对方半径
        
        // i0j0是sec的起点 i1j1是sec的终点 i2j2是cell的坐标
        int i0(secs[k].i1), j0(secs[k].j1), i1(secs[k].i2), j1(secs[k].j2), i2(cell.first), j2(cell.second);
        SafeInterval interval; // 被检查的不安全区间
        double dist, mindist;
        // 计算代理与约束之间的距离，并根据这个距离以及约束的运动信息来确定不安全的时间段，进而更新安全间隔
        if(i0 == i1 && j0 == j1 && i0 == i2 && j0 == j2) // sec是一个在cell处原地等待的section
            mindist = 0;
        else
            mindist = minDist(Point(i2,j2), Point(i0,j0), Point(i1,j1));
        if(mindist >= radius) // 最小距离大于半径之和，这段sec对于该agent是安全的，无需进行其他操作，跳过该sec；
            continue;

        // 详细碰撞时间计算：
        Point point(i2,j2), p0(i0,j0), p1(i1,j1);
        int cls = point.classify(p0, p1); // 判断点 point (代理) 相对于线段 p0p1 (约束) 的位置关系（例如，在线段前方、后方、在线段上等）
        // 计算点 (i2,j2) 到直线 (i0,j0)-(i1,j1) 的距离。
        dist = fabs((i0 - i1)*j2 + (j1 - j0)*i2 + (j0*i1 - i0*j1))/sqrt(pow(i0 - i1, 2) + pow(j0 - j1, 2));
        // 距离平方
        int da = (i0 - i2)*(i0 - i2) + (j0 - j2)*(j0 - j2);
        int db = (i1 - i2)*(i1 - i2) + (j1 - j2)*(j1 - j2);
        // ha 是点 p0 到代理在sec上投影点的距离。size 是基于 radius 和 dist 计算出的一个“碰撞窗口”大小。
        double ha = sqrt(da - dist*dist);
        double size = sqrt(radius*radius - dist*dist);
        // 根据 cls 的值以及 da 和 db 与 radius*radius 的关系，精确计算出不安全的时间间隔 
        // 目的是找出代理何时会与移动的约束发生接触。sec.g1 和 sec.g2 可能代表约束开始和结束的时间
        if(cls == 3) 
        {
            // cell在sec延长线终点外侧，所以不安全区间终点可以加上一段时间
            // 其实这种 延长线的内外侧，应该是由于其他有体积的agent占据的边缘cells导致的
            interval.begin = sec.g1;
            interval.end = sec.g1 + (sqrt(radius*radius - dist*dist) - ha)/sec.mspeed;
        }
        else if(cls == 4)
        {
            // cell在sec延长线起点外侧，所以不安全区间起点可以减去一段时间
            interval.begin = sec.g2 - sqrt(radius*radius - dist*dist)/sec.mspeed + sqrt(db - dist*dist)/sec.mspeed;
            interval.end = sec.g2;
        }
        else if(da < radius*radius) // 和sec起点的距离已经小于安全距离（该agent半径+对方半径）了
        {
            if(db < radius*radius) // 和sec终点的距离也小于安全距离了
            {
                // 不安全区间设置为sec的整个时间
                interval.begin = sec.g1;
                interval.end = sec.g2;
            }
            else
            {
                // 不安全区间终点减去一段时间
                double hb = sqrt(db - dist*dist); // 投影点到sec终点的距离 
                interval.begin = sec.g1;
                interval.end = sec.g2 - hb/sec.mspeed + size/sec.mspeed;
            }
        }
        else // 和sec起点的距离大于安全距离
        {
            if(db < radius*radius) // 和sec终点的距离小于安全距离
            {
                interval.begin = sec.g1 + ha/sec.mspeed - size/sec.mspeed;
                interval.end = sec.g2;
            }
            else // 和sec起点、终点都大于安全距离
            {
                interval.begin = sec.g1 + ha/sec.mspeed - size/sec.mspeed;
                interval.end = sec.g1 + ha/sec.mspeed + size/sec.mspeed;
            }
        }
        // 从当前单元格的现有安全时间间隔中“减去”或“切割”掉新计算出的不安全时间间隔 interval。
        // 它遍历 safe_intervals[i2][j2]（存储了当前单元格的安全间隔的向量），
        // 并根据 interval 与现有安全间隔的重叠情况进行更新。
        for(unsigned int j = 0; j < safe_intervals[i2][j2].size(); j++)
        {
            // 当前cell的安全区间比sec计算得到的不安全区间起点更早终点更晚，即重叠了
            if(safe_intervals[i2][j2][j].begin < interval.begin + CN_EPSILON && safe_intervals[i2][j2][j].end + CN_EPSILON > interval.begin)
            {
                // 如果安全区间起点和不安全区间起点相同
                if(fabs(safe_intervals[i2][j2][j].begin - interval.begin) < CN_EPSILON)
                {
                    // 插入一个零长度的安全间隔，起点和终点都设为当前安全间隔的起点。
                    safe_intervals[i2][j2].insert(safe_intervals[i2][j2].begin() + j, SafeInterval(safe_intervals[i2][j2][j].begin,safe_intervals[i2][j2][j].begin));
                    j++; // 插入后，当前索引 j 需要递增，以指向原先的元素
                    if(safe_intervals[i2][j2][j].end < interval.end) // 如果当前安全区间的结束时间 在 不安全区间的结束时间之前，则删除当前安全区间
                        safe_intervals[i2][j2].erase(safe_intervals[i2][j2].begin() + j);
                    else
                        safe_intervals[i2][j2][j].begin = interval.end; // 否则，将当前安全区间的开始时间设置为不安全区间的结束时间
                }
                else if(safe_intervals[i2][j2][j].end < interval.end)
                    safe_intervals[i2][j2][j].end = interval.begin;
                else
                {
                    std::pair<double,double> new1, new2;
                    new1.first = safe_intervals[i2][j2][j].begin;
                    new1.second = interval.begin;
                    new2.first = interval.end;
                    new2.second = safe_intervals[i2][j2][j].end;
                    safe_intervals[i2][j2].erase(safe_intervals[i2][j2].begin() + j);
                    if(new2.first < CN_INFINITY)
                        safe_intervals[i2][j2].insert(safe_intervals[i2][j2].begin() + j, SafeInterval(new2.first, new2.second));
                    safe_intervals[i2][j2].insert(safe_intervals[i2][j2].begin() + j, SafeInterval(new1.first, new1.second));
                }
            }
            // 该cell的安全区间 在 不安全区间内
            else if(safe_intervals[i2][j2][j].begin > interval.begin - CN_EPSILON && safe_intervals[i2][j2][j].begin < interval.end)
            {
                // 安全区间起点与不安全区间起点相同（或非常接近）
                if(fabs(safe_intervals[i2][j2][j].begin - interval.begin) < CN_EPSILON)
                {
                    // 插入一个零长度的安全间隔 
                    safe_intervals[i2][j2].insert(safe_intervals[i2][j2].begin() + j, SafeInterval(safe_intervals[i2][j2][j].begin,safe_intervals[i2][j2][j].begin));
                    j++;
                }
                // 安全区间完全被不安全区间吞噬（或安全区间结束于不安全区间内部）
                if(safe_intervals[i2][j2][j].end < interval.end)
                {
                    safe_intervals[i2][j2].erase(safe_intervals[i2][j2].begin() + j); // 删除安全区间
                }
                else // 不安全区间只覆盖安全区间的前半部分
                {
                    safe_intervals[i2][j2][j].begin = interval.end; // 移除重叠的不安全部分
                }
            }
        }
        // 更新安全间隔的 ID
        for(unsigned int j = 0; j < safe_intervals[i2][j2].size(); j++)
            safe_intervals[i2][j2][j].id = j;
    }
}

std::vector<SafeInterval> Constraints::getSafeIntervals(Node curNode, const std::unordered_multimap<int, Node> &close, int w)
{
    std::vector<SafeInterval> intervals(0);
    auto range = close.equal_range(curNode.i*w + curNode.j);
    for(unsigned int i = 0; i < safe_intervals[curNode.i][curNode.j].size(); i++)
        if(safe_intervals[curNode.i][curNode.j][i].end >= curNode.g //確保當前安全間隔的結束時間必須晚於或等於理論上最早到達這個節點的時間
                && safe_intervals[curNode.i][curNode.j][i].begin <= (curNode.Parent->interval.end + curNode.g - curNode.Parent->g)) //檢查當前安全間隔是否與父節點的安全間隔在時間上相交
        {
            bool has = false;
            for(auto it = range.first; it != range.second; it++) //遍歷所有位於同一柵格位置且已在 close 列表中的節點
                if(it->second.interval.begin == safe_intervals[curNode.i][curNode.j][i].begin)
                if((it->second.g + tweight*fabs(curNode.heading - it->second.heading)/(180*rspeed)) - curNode.g < CN_EPSILON)//take into account turning cost
                {
                    has = true;
                    break;
                }
            if(!has)
                intervals.push_back(safe_intervals[curNode.i][curNode.j][i]);
        }
    return intervals;
}

std::vector<SafeInterval> Constraints::getSafeIntervals(Node curNode)
{
    return safe_intervals[curNode.i][curNode.j];
}

void Constraints::addStartConstraint(int i, int j, int size, std::vector<std::pair<int, int> > cells, double agentsize)
{
    section sec(i, j, i, j, 0, size);
    sec.size = agentsize;
    for(auto cell: cells)
        // 将start占据的位置生成一个sec，加入到start位置的constraints中；
        constraints[cell.first][cell.second].insert(constraints[cell.first][cell.second].begin(),sec);
    return;
}

void Constraints::removeStartConstraint(std::vector<std::pair<int, int> > cells, int start_i, int start_j)
{
    for(auto cell: cells)
        for(size_t k = 0; k < constraints[cell.first][cell.second].size(); k++)
            if(constraints[cell.first][cell.second][k].i1 == start_i && constraints[cell.first][cell.second][k].j1 == start_j && constraints[cell.first][cell.second][k].g1 < CN_EPSILON)
            {
                constraints[cell.first][cell.second].erase(constraints[cell.first][cell.second].begin() + k);
                k--;
            }
    return;
}

void Constraints::addConstraints(const std::vector<Node> &sections, double size, double mspeed, const Map &map)
{
    std::vector<std::pair<int,int>> cells;
    LineOfSight los(size);
    section sec(sections.back(), sections.back());
    sec.g2 = CN_INFINITY;
    sec.size = size;
    sec.mspeed = mspeed;
    cells = los.getCellsCrossedByLine(sec.i1, sec.j1, sec.i2, sec.j2, map);
    for(auto cell: cells)
        constraints[cell.first][cell.second].push_back(sec);
    if(sec.g1 == 0)
        for(auto cell: cells)
            safe_intervals[cell.first][cell.second].clear();
    for(unsigned int a = 1; a < sections.size(); a++)
    {
        cells = los.getCellsCrossedByLine(sections[a-1].i, sections[a-1].j, sections[a].i, sections[a].j, map);
        sec = section(sections[a-1], sections[a]);
        sec.size = size;
        sec.mspeed = mspeed;
        for(unsigned int i = 0; i < cells.size(); i++)
            constraints[cells[i].first][cells[i].second].push_back(sec);
        /*if(a+1 == sections.size())
            updateSafeIntervals(cells,sec,true);
        else
            updateSafeIntervals(cells,sec,false);*/
    }
}

void Constraints::addShapeConstraints(const std::vector<Node> &sections, double size, double mspeed, const std::vector<Location> &shape, const Map &map)
{
    // 取最远角点距离作为size
    double d = 0;
    for (auto p: shape)
    {
        double dis = p.x*p.x + p.y*p.y;
        d = std::max(d, dis);
    }
    d = sqrt(d);

    // 首先处理终点sec
    section sec(sections.back(), sections.back());
    sec.size = d;
    size = d;
    // shape传入的是初始未旋转的相对坐标，因为要在后续内部计算沿着sec的形状
    std::vector<std::pair<int,int>> cells_goal;
    LineOfSight los(size);
    sec.g2 = CN_INFINITY;
    sec.mspeed = mspeed;

    Shape_Collide s_temp;
    Location ori = Location(0, 0);
    std::vector<Location> shape_ori = shape;
    std::vector<Location> shape_goal = s_temp.rotate_shape(ori, goal_heading, shape_ori);
    sec.shape_ori = shape; // 

    sec.end_heading = goal_heading;
    double s_heading = 0.0;
    if (sections.size() >=2)
    {
        s_heading = sections[sections.size()-2].heading;
        sec.start_heading = s_heading;
    }
    // sec.start_heading = goal_heading; // 终点sec的start_heading和end_heading相同
    // std::cout << "last heading is " << sec.start_heading << " to " << sec.end_heading << std::endl;
    
    // cells = los.getCellsCrossedByPolyon(sec.i1, sec.j1, sec.i2, sec.j2, shape_goal, map); // 终点处计算goal_heading所占据的栅格～
    cells_goal = los.getCellsOriginByPolyon_Optimized(sec.i1, sec.j1, shape_goal, map); // 终点处计算goal_heading所占据的栅格～
    // std::cout << "Goal heading: " << goal_heading << " at (" << sec.i1 << "," << sec.j1 << ") occupies cells: " << std::endl;
    for(auto cell: cells_goal)
    {
        constraints[cell.first][cell.second].push_back(sec);
        // std::cout << "Add goal constraint at (" << cell.first << "," << cell.second << ")" << std::endl;
    }
        
    if(sec.g1 == 0)
        for(auto cell: cells_goal)
            safe_intervals[cell.first][cell.second].clear();

    for(unsigned int a = 1; a < sections.size(); a++)
    {
        double start_heading, end_heading;
        if (a == 1)
        {
            start_heading = initial_heading;
            end_heading = calcHeading_constraints(sections[a].i, sections[a].j, sections[a+1].i, sections[a+1].j);;
        }

        // *** 如果是原地旋转的栅格，可以根据前后段的sec，计算出start_heading和end_heading，然后计算旋转过程中所占据的栅格，加入constraints
        if (sections[a-1].i == sections[a].i && sections[a-1].j == sections[a].j && a>=2) // 原地旋转栅格
        {
            if (a == sections.size()-1)
            {
                start_heading = calcHeading_constraints(sections[a-2].i, sections[a-2].j, sections[a-1].i, sections[a-1].j);
                end_heading = goal_heading;
            }
            else{
                start_heading = calcHeading_constraints(sections[a-2].i, sections[a-2].j, sections[a-1].i, sections[a-1].j);
                end_heading = calcHeading_constraints(sections[a].i, sections[a].j, sections[a+1].i, sections[a+1].j);
            }
            
            if (start_heading == end_heading) // 类似原地停止等待
            {
                std::vector<Location> shape_rot = s_temp.rotate_shape(ori, start_heading, shape_ori);
                std::vector<std::pair<int,int>> cells_waiting = los.getCellsOriginByPolyon_Optimized(sections[a-1].i, sections[a-1].j, shape_rot, map);
                sec = section(sections[a-1], sections[a]);
                sec.size = d;
                sec.mspeed = mspeed;
                sec.shape_ori = shape;
                sec.start_heading = start_heading;
                sec.end_heading = end_heading;
                // std::cout << "Waiting at (" << sections[a-1].i << "," << sections[a-1].j << ") with heading " << start_heading << std::endl;
                for(auto cell: cells_waiting)
                    constraints[cell.first][cell.second].push_back(sec);
                continue;
            }

            // ***多角度採樣疊加-计算旋转过程中的占据栅格***
            std::set<std::pair<int, int>> rotated_cells_union;
            double angle_step = 10.0; // 每15度採樣一次

            // std::cout << "Rotation at (" << sections[a-1].i << "," << sections[a-1].j << ") from " << start_heading << " to " << end_heading << std::endl;
            std::vector<Location> shape_rot = s_temp.rotate_shape(ori, start_heading, shape_ori);
            std::vector<Location> shape_rot_end = s_temp.rotate_shape(ori, end_heading, shape_ori);

            std::vector<std::pair<int,int>> cells_before = los.getCellsOriginByPolyon_Optimized(sections[a-1].i, sections[a-1].j, shape_rot, map);
            std::vector<std::pair<int,int>> cells_after = los.getCellsOriginByPolyon_Optimized(sections[a].i, sections[a].j, shape_rot_end, map);
            Shape_Collide sc_temp;

            double diff = end_heading - start_heading;
            if (diff > 180.0) diff -= 360.0;
            if (diff < -180.0) diff += 360.0;
            for (double angle = 0; std::abs(angle) <= std::abs(diff); angle += std::copysign(angle_step, diff)) {
                double current_heading = start_heading + angle;
                std::vector<Location> shape_rotated = sc_temp.rotate_shape(ori, current_heading, shape_ori);
                std::vector<std::pair<int, int>> cells_at_angle = los.getCellsOriginByPolyon_Optimized(sections[a-1].i, sections[a-1].j, shape_rotated, map);
                rotated_cells_union.insert(cells_at_angle.begin(), cells_at_angle.end());
            }
            
            // 不要忘了結束角度
            std::vector<Location> shape_end = sc_temp.rotate_shape(ori, end_heading, shape_ori);
            std::vector<std::pair<int, int>> cells_end = los.getCellsOriginByPolyon_Optimized(sections[a].i, sections[a].j, shape_end, map);
            rotated_cells_union.insert(cells_end.begin(), cells_end.end());

            sec = section(sections[a-1], sections[a]);
            sec.size = d;
            sec.mspeed = mspeed;
            sec.shape_ori = shape;
            sec.start_heading = start_heading;
            sec.end_heading = end_heading;
            for(const auto& cell : rotated_cells_union)
                constraints[cell.first][cell.second].push_back(sec);

            continue; // 繼續下一個 section
        }
        
        std::vector<std::pair<int,int>> cells_section = los.getCellsCrossedByPolyon(sections[a-1].i, sections[a-1].j, sections[a].i, sections[a].j, shape, map);
        double sec_heading = calcHeading_constraints(sections[a-1].i, sections[a-1].j, sections[a].i, sections[a].j);
        sec = section(sections[a-1], sections[a]);
        sec.size = d;
        sec.mspeed = mspeed;
        sec.shape_ori = shape;
        sec.start_heading = sec_heading;
        sec.end_heading = sec_heading;
        // std::cout << "Moving from (" << sections[a-1].i << "," << sections[a-1].j << ") to (" << sections[a].i << "," << sections[a].j << ") with heading " << sec_heading << std::endl;
        // sec.start_heading = calcHeading_constraints(sections[a-1].i, sections[a-1].j, sections[a].i, sections[a].j);
        for(auto cell: cells_section)
            constraints[cell.first][cell.second].push_back(sec);
        /*if(a+1 == sections.size())
            updateSafeIntervals(cells,sec,true);
        else
            updateSafeIntervals(cells,sec,false);*/
           
    }
}

std::vector<SafeInterval> Constraints::findIntervals(Node curNode, std::vector<double> &EAT, const std::unordered_multimap<int, Node> &close, const std::vector<Location> &shape, const Map &map, bool is_resetPar)
{
    // std::cout << "In findIntervals" << std::endl;
    // 这个curNode 就是外层调用时的 newNode，即新子节点， 煞笔原作者变量名称滥用
    std::vector<SafeInterval> curNodeIntervals = getSafeIntervals(curNode, close, map.width);
    if(curNodeIntervals.empty())
        return curNodeIntervals;
    EAT.clear();

    LineOfSight los(agentsize);
    // *** 这里之前curNode.Parent->i, curNode.Parent->j, curNode.i, curNode.j顺序写错了；***
    std::vector<std::pair<int,int>> cells_shape = los.getCellsCrossedByPolyon(curNode.Parent->i, curNode.Parent->j, curNode.i, curNode.j, shape, map); // shape_cur

    std::vector<std::pair<int,int>> cells_occupied;
    // std::cout << "from " << curNode.i << ", " << curNode.j << " to " << curNode.Parent->i << ", " << curNode.Parent->j << std::endl;
    for(auto it = cells_shape.begin(); it != cells_shape.end(); it++)
    {
        // std::cout << "(" << it->first << ", " << it->second << ")" << std::endl;
        cells_occupied.push_back(std::pair<int,int>{it->first, it->second});
    }

    std::vector<section> sections(0);
    section sec; 
    for(unsigned int i = 0; i < cells_occupied.size(); i++)
        for(unsigned int j = 0; j < constraints[cells_occupied[i].first][cells_occupied[i].second].size(); j++)
        {
            sec = constraints[cells_occupied[i].first][cells_occupied[i].second][j];
            if(sec.g2 < curNode.Parent->g || sec.g1 > (curNode.Parent->interval.end + curNode.g - curNode.Parent->g))
            // if(sec.g2 < curNode.Parent->g )
                continue;
            if(std::find(sections.begin(), sections.end(), sec) == sections.end())
                sections.push_back(sec);
        }
    auto range = close.equal_range(curNode.i*map.width + curNode.j);

    for(unsigned int i=0; i<curNodeIntervals.size(); i++)
    {
        SafeInterval cur_interval(curNodeIntervals[i]);
        if(cur_interval.begin < curNode.g)
            cur_interval.begin = curNode.g;
        double startTimeA = curNode.Parent->g;
        if(cur_interval.begin > startTimeA + curNode.g - curNode.Parent->g)
            startTimeA = cur_interval.begin - curNode.g + curNode.Parent->g;
        unsigned int j = 0;
        bool goal_collision;
        while(j < sections.size())
        {
            goal_collision = false;

            if(hasShapeCollision(curNode, startTimeA, sections[j], goal_collision, shape, map, is_resetPar))
            {
                double offset = 1.0;
                startTimeA += offset;
                cur_interval.begin += offset;
                j = 0;//start to check all constraints again, because time has changed
                // 这个 startTimeA > curNode.Parent->interval.end 的判断条件应该就是避免当前节点开始时间超出父节点区间结束时间
                if(goal_collision || cur_interval.begin > cur_interval.end || startTimeA > curNode.Parent->interval.end)
                {
                    curNodeIntervals.erase(curNodeIntervals.begin() + i);
                    i--;
                    break;
                }
            }
            else
            {
                j++;
            }   
        }
        if(j == sections.size())
        {
            bool has = false;
            for(auto rit = range.first; rit != range.second; rit++)
                if(rit->second.interval.begin == curNodeIntervals[i].begin)
                if((rit->second.g + tweight*fabs(curNode.heading - rit->second.heading)/(180*rspeed) - cur_interval.begin) < CN_EPSILON)//take into account turning cost
                {
                    has = true;
                    curNodeIntervals.erase(curNodeIntervals.begin()+i);
                    i--;
                    break;
                }
            if(!has)
                EAT.push_back(cur_interval.begin);
        }
    }
    return curNodeIntervals;
}

// std::vector<SafeInterval> Constraints::findIntervals_Improve(Node curNode, Node angleNode, double Rcost,double Dcost, std::vector<double> &EAT, const std::unordered_multimap<int, Node> &close, const std::vector<Location> &shape, const Map &map)
// {
//     // 这个curNode 就是外层调用时的 newNode， 煞笔原作者变量名称滥用
//     std::vector<SafeInterval> curNodeIntervals = getSafeIntervals(curNode, close, map.width);
//     if(curNodeIntervals.empty())
//         return curNodeIntervals;
//     EAT.clear();
//     LineOfSight los(agentsize);
    
//     // std::vector<std::pair<int,int>> cells = los.getCellsCrossedByLine(curNode.i, curNode.j, curNode.Parent->i, curNode.Parent->j, map);
    
//     // 这里改用了整个形状在移动中占据的栅格，时间确实有增加哦
//     std::vector<std::pair<int,int>> cells_shape = los.getCellsCrossedByPolyon(curNode.i, curNode.j, curNode.Parent->i, curNode.Parent->j, shape, map); // shape_cur
    
//     std::vector<std::pair<int,int>> cells;
//     // std::cout << "from " << curNode.i << ", " << curNode.j << " to " << curNode.Parent->i << ", " << curNode.Parent->j << std::endl;
//     for(auto it = cells_shape.begin(); it != cells_shape.end(); it++)
//     {
//         // std::cout << "(" << it->first << ", " << it->second << ")" << std::endl;
//         cells.push_back(std::pair<int,int>{it->first, it->second});
//     }
    
//     std::vector<section> sections(0);
//     section sec;
//     for(unsigned int i = 0; i < cells.size(); i++)
//         for(unsigned int j = 0; j < constraints[cells[i].first][cells[i].second].size(); j++)
//         {
//             sec = constraints[cells[i].first][cells[i].second][j];
//             if(sec.g2 < curNode.Parent->g || sec.g1 > (curNode.Parent->interval.end + curNode.g - curNode.Parent->g))
//                 continue;
//             if(std::find(sections.begin(), sections.end(), sec) == sections.end())
//                 sections.push_back(sec);
//         }
//     auto range = close.equal_range(curNode.i*map.width + curNode.j);

//     for(unsigned int i=0; i<curNodeIntervals.size(); i++)
//     {
//         SafeInterval cur_interval(curNodeIntervals[i]);
//         if(cur_interval.begin < curNode.g)
//             cur_interval.begin = curNode.g;
//         double startTimeA = curNode.Parent->g;
//         if(cur_interval.begin > startTimeA + curNode.g - curNode.Parent->g)
//             startTimeA = cur_interval.begin - curNode.g + curNode.Parent->g;
//         unsigned int j = 0;
//         bool goal_collision;

//         bool interval_discarded = false; // 用于标记当前区间是否应被丢弃

//         while(j < sections.size())
//         {
//             goal_collision = false;

//             // ！*需要将此函数改为两个多边形的碰撞形式；
//             // if(hasCollision(curNode, startTimeA, sections[j], goal_collision))
//             // 这里其实应该加上 相应的（加了offset的）父节点->angleNode 的碰撞检测，看看需要多传入什么参数？
//             if(hasShapeCollision(curNode, startTimeA, sections[j], goal_collision, shape, map))
//             {
//                 double offset = 1.0;
//                 startTimeA += offset;
//                 cur_interval.begin += offset;
//                 j = 0;//start to check all constraints again, because time has changed
//                 if(goal_collision || cur_interval.begin > cur_interval.end || startTimeA > curNode.Parent->interval.end)
//                 {
//                     curNodeIntervals.erase(curNodeIntervals.begin() + i);
//                     i--;
//                     break;
//                 }
                
//                 // 取消了在startTimeA之前（也就是curNode->angleNode原地旋转期间）的等待，尽快去寻找别的路径
//                 // 且curNode->angleNode原地旋转期间，没有加入碰撞检测
//                 // curNodeIntervals.erase(curNodeIntervals.begin() + i);
//                 // i--;
//                 // break;

//             }
//             else
//                 j++;
//         }
//         if(j == sections.size())
//         {
//             bool has = false;
//             for(auto rit = range.first; rit != range.second; rit++)
//                 if(rit->second.interval.begin == curNodeIntervals[i].begin)
//                 if((rit->second.g + tweight*fabs(curNode.heading - rit->second.heading)/(180*rspeed) - cur_interval.begin) < CN_EPSILON)//take into account turning cost
//                 {
//                     has = true;
//                     curNodeIntervals.erase(curNodeIntervals.begin()+i);
//                     i--;
//                     break;
//                 }
//             if(!has)
//                 EAT.push_back(cur_interval.begin);
//         }
//     }
//     return curNodeIntervals;
// }

bool Constraints::hasCollision(const Node &curNode, double startTimeA, const section &constraint, bool &goal_collision)
{
    double endTimeA(startTimeA + curNode.g - curNode.Parent->g), startTimeB(constraint.g1), endTimeB(constraint.g2);
    if(startTimeA > endTimeB || startTimeB > endTimeA)
        return false;
    Vector2D A(curNode.Parent->i,curNode.Parent->j);
    Vector2D VA((curNode.i - curNode.Parent->i)/(curNode.g - curNode.Parent->g), (curNode.j - curNode.Parent->j)/(curNode.g - curNode.Parent->g));
    Vector2D B(constraint.i1, constraint.j1);
    Vector2D VB((constraint.i2 - constraint.i1)/(constraint.g2 - constraint.g1), (constraint.j2 - constraint.j1)/(constraint.g2 - constraint.g1));
    if(startTimeB > startTimeA)
    {
      // Move A to the same time instant as B
      A += VA*(startTimeB-startTimeA);
      startTimeA=startTimeB;
    }
    else if(startTimeB < startTimeA)
    {
      B += VB*(startTimeA - startTimeB);
      startTimeB = startTimeA;
    }

    // 这个 r 即为 综合碰撞半径
    double r(constraint.size + agentsize + inflateintervals); //combined radius
    // double r = 2;
    
    Vector2D w(B - A);
    double c(w*w - r*r);
    if(c < 0)
    {
        if(constraint.g2 == CN_INFINITY)
            goal_collision = true;
        return true;
    } // Agents are currently colliding

    // Use the quadratic formula to detect nearest collision (if any)
    Vector2D v(VA - VB);
    double a(v*v);
    double b(w*v);

    double dscr(b*b - a*c);
    if(dscr <= 0)
        return false;

    double ctime = (b - sqrt(dscr))/a;
    if(ctime > -CN_EPSILON && ctime < std::min(endTimeB,endTimeA) - startTimeA + CN_EPSILON)
    {
        if(constraint.g2 == CN_INFINITY)
            goal_collision = true;
        return true;
    }
    else
        return false;
}


bool Constraints::hasShapeCollision(const Node &curNode, double startTimeA, const section &constraint, bool &goal_collision, const vector<Location> &shapeA, const Map &map, bool is_resetPar)
{
    // std::cout<< " Enter hasShapeCollision" << std::endl;
    // *如果有一个agent的section是原地旋转，那么容易在下面的相对运动检测中存在漏洞，需要fix
    
    double endTimeA(startTimeA + curNode.g - curNode.Parent->g), startTimeB(constraint.g1), endTimeB(constraint.g2);
    double startTimeA_ = startTimeA;
    double startTimeB_ = startTimeB;
    if(startTimeA > endTimeB || startTimeB > endTimeA)
        return false;
    Vector2D A(curNode.Parent->i,curNode.Parent->j);
    Vector2D VA((curNode.i - curNode.Parent->i)/(curNode.g - curNode.Parent->g), (curNode.j - curNode.Parent->j)/(curNode.g - curNode.Parent->g));
    Vector2D B(constraint.i1, constraint.j1);
    Vector2D VB((constraint.i2 - constraint.i1)/(constraint.g2 - constraint.g1), (constraint.j2 - constraint.j1)/(constraint.g2 - constraint.g1));
    if(startTimeB > startTimeA)
    {
      // Move A to the same time instant as B
      A += VA*(startTimeB-startTimeA);
      startTimeA=startTimeB;
    }
    else if(startTimeB < startTimeA)
    {
      B += VB*(startTimeA - startTimeB);
      startTimeB = startTimeA;
    }

    // A一定是后序的，因为是当前智能体，B一定是前序的，因为是已有的constraint

    double headingA = curNode.heading;
    // double headingA = calcHeading_constraints(curNode.Parent->i, curNode.Parent->j, curNode.i, curNode.j);
    // double headingB = calcHeading_constraints(constraint.i1, constraint.j1, constraint.i2, constraint.j2); 
    LineOfSight los;
    Shape_Collide sc_temp;
    Location ori = {0, 0};
    std::vector<Location> shapeA_ori = shapeA;
    std::vector<Location> shapeB_ori = constraint.shape_ori;
    // std::vector<Location> shape_headingA = sc_temp.rotate_shape(ori, headingA, shapeA_ori);
    std::vector<Location> shape_headingB = sc_temp.rotate_shape(ori, constraint.start_heading, shapeB_ori);
    std::vector<Location> shape_headingA1 = sc_temp.rotate_shape(ori, curNode.Parent->heading, shapeA_ori);
    std::vector<Location> shape_headingA2 = sc_temp.rotate_shape(ori, curNode.heading, shapeA_ori);

    std::vector<std::pair<int, int>> cells_A_all; // Anyway, A占据的栅格；
    std::vector<std::pair<int, int>> cells_B_all; // B占据的栅格；

    std::set<std::pair<int, int>> rotated_cells_union_A;
    double angle_step = 10.0; // 每15度採樣一次

    // 如果A运动
    if (std::abs(VA.i) > CN_EPSILON || std::abs(VA.j) > CN_EPSILON)
    {
        double endTime = std::min(endTimeA, endTimeB);
        Vector2D A_end = A + VA*(endTime - startTimeA);
        cells_A_all = los.getCellsCrossedByPolyon(
            round(A.i), round(A.j), 
            round(A_end.i), round(A_end.j), 
            shape_headingA2, map);
        // cells_A_all = los.getCellsCrossedByPolyon_double(
        //     A.i, A.j, 
        //     A_end.i, A_end.j, 
        //     shape_headingA2, map);
        
    }
    else // 如果A原地
    {
        if (std::abs(curNode.heading - curNode.Parent->heading) < CN_EPSILON) // A原地等待，起终点角度一致；
        {
            cells_A_all = los.getCellsOriginByPolyon_Optimized(curNode.i, curNode.j, shape_headingA1, map);
        }
        else  // A原地旋转，起终点角度不一致；
        {   
            // ---------- 多角度采样 ---------
            // std::cout <<"Agent A rotating at (" << curNode.i << "," << curNode.j << ") from " << curNode.Parent->heading << " to " << curNode.heading << std::endl;
            std::set<std::pair<int, int>> rotated_cells_unio_A = multiAngleSampleCells_DuringRot(round(A.i), round(A.j), curNode.Parent->heading, curNode.heading, angle_step, shapeA_ori,  map);
            // 将set转换为vector
            cells_A_all.assign(rotated_cells_union_A.begin(), rotated_cells_union_A.end());
            // for (auto cellA : cells_A_all)
            // {
            //     std::cout << "A occupies cell (" << cellA.first << "," << cellA.second << ")" << std::endl;
            // }
        }

    }

    // 如果B运动
    if (std::abs(VB.i) > CN_EPSILON || std::abs(VB.j) > CN_EPSILON)
    {
        double endTime = std::min(endTimeA, endTimeB);
        Vector2D B_end = B + VB*(endTime - startTimeB);
        cells_B_all = los.getCellsCrossedByPolyon(
            round(B.i), round(B.j), 
            round(B_end.i), round(B_end.j), 
            shape_headingB, map);
        // cells_B_all = los.getCellsCrossedByPolyon_double(
        //     B.i, B.j, 
        //     B_end.i, B_end.j, 
        //     shape_headingB, map);
    }
    else{
        if (std::abs(constraint.start_heading - constraint.end_heading) < CN_EPSILON) // B原地等待，起终点角度一致；
        {
            cells_B_all = los.getCellsOriginByPolyon_Optimized(constraint.i1, constraint.j1, shape_headingB, map);
        }
        else  // B原地旋转，起终点角度不一致；
        {
            std::set<std::pair<int, int>> rotated_cells_union_B = multiAngleSampleCells_DuringRot(round(B.i), round(B.j), constraint.start_heading, constraint.end_heading, angle_step, shapeB_ori,  map);
            cells_B_all.assign(rotated_cells_union_B.begin(), rotated_cells_union_B.end());
        }
    }

    for (auto cellA : cells_A_all)
    {
        // if (!map.CellOnGrid(cellA.first, cellA.second) || map.CellIsObstacle(cellA.first, cellA.second))
        // {
        //     goal_collision = true;
        //     return true;
        // }
        for (auto cellB : cells_B_all)
        {
            if (cellA == cellB)
            {
                if(constraint.g2 == CN_INFINITY  )
                {
                    // std::cout << " Goal collision detected at cell (" << cellA.first << "," << cellA.second << ")" << std::endl;
                    // std::cout << "  A from (" << curNode.Parent->i << "," << curNode.Parent->j << ") to (" << curNode.i << "," << curNode.j << ") with heading " << headingA << std::endl;
                    goal_collision = true;
                }
                    
                return true;
            }
        }
    }

    return false;
}

std::set<std::pair<int, int>> Constraints::multiAngleSampleCells_DuringRot(int x, int y, double start_heading, double end_heading, double angle_step, const std::vector<Location> &shape_ori, const Map &map)
{
    std::set<std::pair<int, int>> rotated_cells_unio;
    double diff = end_heading - start_heading;
    Shape_Collide sc_temp;
    LineOfSight los;
    Location ori = {0, 0};
    std::vector<Location> shape_ = shape_ori;
    // double diff = end_heading - start_heading;
    if (diff > 180.0) diff -= 360.0;
    if (diff < -180.0) diff += 360.0;
    double angle_diff = std::abs(diff);

    for (double angle = 0; std::abs(angle) <= std::abs(diff); angle += std::copysign(angle_step, diff)) {
        double current_heading = start_heading + angle;
        std::vector<Location> shape_rotated = sc_temp.rotate_shape(ori, current_heading, shape_);
        std::vector<std::pair<int, int>> cells_at_angle = los.getCellsOriginByPolyon_Optimized(x, y, shape_rotated, map);
        rotated_cells_unio.insert(cells_at_angle.begin(), cells_at_angle.end());
    }
    
    // 不要忘了結束角度
    std::vector<Location> shape_end = sc_temp.rotate_shape(ori, end_heading, shape_);
    std::vector<std::pair<int, int>> cells_end = los.getCellsOriginByPolyon_Optimized(x, y, shape_end, map);
    rotated_cells_unio.insert(cells_end.begin(), cells_end.end());

    return rotated_cells_unio;
}

bool Constraints::hasCollisionDuringWait(const Node &static_node, double wait_startTime, double wait_endTime, 
                                          const section &constraint, const std::vector<Location> &shape_static, const Map &map)
{
    double moving_startTime = constraint.g1;
    double moving_endTime = constraint.g2;

    // 1. 检查时间区间是否有重叠
    if (wait_startTime >= moving_endTime || moving_startTime >= wait_endTime) {
        return false;
    }

    // 2. 确定共同的时间窗口
    double common_startTime = std::max(wait_startTime, moving_startTime);
    double common_endTime = std::min(wait_endTime, moving_endTime);

    // 3. 获取静止智能体占据的栅格
    // 注意：这里的朝向应该是它到达等待点时的朝向，即 static_node.heading
    Shape_Collide sc_temp;
    Location ori = {0, 0};
    std::vector<Location> shape_ = shape_static;
    std::vector<Location> rotated_shape_static = sc_temp.rotate_shape(ori, static_node.heading, shape_);
    LineOfSight los;
    std::vector<std::pair<int, int>> cells_static = los.getCellsOriginByPolyon_Optimized(static_node.i, static_node.j, rotated_shape_static, map);
    
    // 如果静止物体没有占据任何格子，则不可能碰撞
    if (cells_static.empty()) {
        return false;
    }

    // 4. 计算在共同时间窗口内，移动智能体的起点和终点
    Vector2D B_start_pos(constraint.i1, constraint.j1);
    Vector2D VB((constraint.i2 - constraint.i1) / (constraint.g2 - constraint.g1), (constraint.j2 - constraint.j1) / (constraint.g2 - constraint.g1));
    
    Vector2D B_common_start_pos = B_start_pos + VB * (common_startTime - moving_startTime);
    Vector2D B_common_end_pos = B_start_pos + VB * (common_endTime - moving_startTime);
    
    // 5. 获取移动智能体在共同时间窗口内扫掠过的栅格
    double heading_moving = calcHeading_constraints(constraint.i1, constraint.j1, constraint.i2, constraint.j2);
    std::vector<Location> shape_temp = constraint.shape_ori;
    std::vector<Location> rotated_shape_moving = sc_temp.rotate_shape(ori, heading_moving, shape_temp);

    std::vector<std::pair<int, int>> cells_moving_swept = los.getCellsCrossedByPolyon(
        round(B_common_start_pos.i), round(B_common_start_pos.j),
        round(B_common_end_pos.i), round(B_common_end_pos.j),
        rotated_shape_moving, map
    );

    // 6. 检查栅格集合是否有交集 (这是一个可以优化的部分，例如用哈希集合)
    // 为了简单和正确，先用双重循环
    for (const auto& static_cell : cells_static) {
        for (const auto& moving_cell : cells_moving_swept) {
            if (static_cell == moving_cell) {
                return true; // 发现碰撞
            }
        }
    }

    return false; // 没有碰撞
}

double Constraints::calcHeading_constraints(int i1, int j1, int i2, int j2)
{
    double heading = atan2(i2 - i1, j2 - j1)*180/CN_PI;
    if(heading < 0)
        heading += 360;
    return heading;
}


