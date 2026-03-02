/* (c) 2017. Andreychuk A.
 * This class implements line-of-sight function for a variable size of agent.
 * It also has a method for checking cell's traversability.
 * For its work is needed the size of agent and a map container that has 'cellIsObstacle' and 'cellOnGrid' methods.
 * If it is not possible to give the permission to access the grid, the one can use 'getCellsCrossedByLine' method.
 * It doesn't use grid and returns a set of all cells(as pairs of coordinates) that are crossed by an agent moving along a line.
 */

#ifndef LINEOFSIGHT_H
#define LINEOFSIGHT_H
#include "gl_const.h"

#define CN_OBSTACLE 1

#include <vector>
#include <math.h>
#include <algorithm>
#include <set>
#include "shape_collide.hpp"
#include <chrono>

// A helper struct to store edge information for the scanline algorithm
struct EdgeEntry {
    double x_intersect; // The x-coordinate where the edge intersects the current scanline
    double inv_slope;   // The inverse slope (dx/dy) of the edge
    int y_max;          // The maximum y-coordinate for this edge

    // For sorting edges in the Active Edge Table by their x_intersect
    bool operator<(const EdgeEntry& other) const {
        return x_intersect < other.x_intersect;
    }
};

class LineOfSight
{
public:
    LineOfSight(double agentSize = 0.5)
    {
        this->agentSize = agentSize;
        int add_x, add_y, num = agentSize + 0.5 - CN_EPSILON;
        // int add_x, add_y, num = agentSize + 1.5 - CN_EPSILON;
        for(int x = -num; x <= +num; x++)
            for(int y = -num; y <= +num; y++)
            {
                add_x = x != 0 ? 1 : 0;
                add_y = y != 0 ? 1 : 0;
                if((pow(2*abs(x) - add_x, 2) + pow(2*abs(y) - add_y, 2)) < pow(2*agentSize, 2))
                    cells.push_back({x, y});
            }
        if(cells.empty())
            cells.push_back({0,0});
    }

    void setSize(double agentSize) 
    {
        // 计算一个圆形代理（agentSize）所占用的单元格的相对坐标，存储在cells中
        // 后续应该需要制作一个多边形所占用的格子的函数；
        this->agentSize = agentSize;
        int add_x, add_y, num = agentSize + 0.5 - CN_EPSILON;
        cells.clear();
        for(int x = -num; x <= +num; x++)
            for(int y = -num; y <= +num; y++)
            {
                add_x = x != 0 ? 1 : 0;
                add_y = y != 0 ? 1 : 0;
                if((pow(2*abs(x) - add_x, 2) + pow(2*abs(y) - add_y, 2)) < pow(2*agentSize, 2))
                    cells.push_back({x, y});
            }
        if(cells.empty())
            cells.push_back({0,0});
    }

    void setShapeSize(double agentSize, const std::vector<Location> &shape) 
    {
        // 计算一个多边形的代理（agentSize）所占用的单元格的【相对】坐标，存储在cells中
        this->agentSize = agentSize;
        cells.clear();
        std::set<std::pair<int, int>> cells_temp = getCellsOriginByPolyon_Ori(shape);
        for (const auto& c : cells_temp) {
            cells.push_back({c.first, c.second});
        }
        if(cells.empty())
            cells.push_back({0,0});
    }

    double calcHeading_lineofsight(double i1, double j1, double i2, double j2)
    {
        // 航向角的计算基于二维平面直角坐标系，且 x 轴正向为 0 度，顺时针方向角度增加
        // double heading = acos((j2 - j1)/sqrt((i2 - i1)*(i2 - i1) + (j2 - j1)*(j2 - j1)))*180/PI;
        // if(i1 < i2)
        //     heading = 360 - heading;
        // double heading = atan2(j2 - j1, i2 - i1)*180/PI;
        // if(heading < 0)
        //     heading += 360;
        double heading = atan2(i2 - i1, j2 - j1)*180/CN_PI;
        if(heading < 0)
            heading += 360;
        return heading;
    }

    // 辅助函数：用于计算点集凸包的叉积
    // > 0: p1->p2->p3 是逆时针（左转）
    // < 0: p1->p2->p3 是顺时针（右转）
    // = 0: 三点共线
    double cross_product(const Location& p1, const Location& p2, const Location& p3) {
        return (p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x);
    }

    // 辅助函数：使用Monotone Chain算法计算点集的凸包
    std::vector<Location> calculateConvexHull(std::vector<Location>& points) {
        if (points.size() <= 3) {
            return points;
        }

        // 按x坐标排序，如果x相同则按y坐标排序
        std::sort(points.begin(), points.end(), [](const Location& a, const Location& b) {
            return a.x < b.x || (a.x == b.x && a.y < b.y);
        });

        std::vector<Location> upper_hull;
        std::vector<Location> lower_hull;

        for (const auto& p : points) {
            while (lower_hull.size() >= 2 && cross_product(lower_hull[lower_hull.size()-2], lower_hull.back(), p) <= 0.01) { // 使用 0.01 代替 0 作为容差判断是否共线
                lower_hull.pop_back();
            }
            lower_hull.push_back(p);
        }

        for (int i = points.size() - 1; i >= 0; --i) {
            const auto& p = points[i];
            while (upper_hull.size() >= 2 && cross_product(upper_hull[upper_hull.size()-2], upper_hull.back(), p) <= 0.01) {
                upper_hull.pop_back();
            }
            upper_hull.push_back(p);
        }

        std::vector<Location> convex_hull = lower_hull;
        // 移除重复的起始点和终点
        for (size_t i = 1; i < upper_hull.size() - 1; ++i) {
            convex_hull.push_back(upper_hull[i]);
        }
        
        return convex_hull;
    }

    // /**
    //  * @brief 快速检测一个多边形的顶点序列是否构成凸多边形
    //  * @details 该函数通过遍历所有顶点，检查由相邻三点构成的转角方向是否一致。
    //  * 它利用叉积来判断转向，时间复杂度为O(V)，其中V是顶点数。
    //  * @param shape 描述多边形形状的、按顺序排列的顶点列表（相对坐标或绝对坐标均可）
    //  * @return true 如果多边形是凸的或共线的，否则返回 false
    //  */
    bool isConvex(const std::vector<Location>& shape) {
        size_t n = shape.size();
        if (n < 3) {
            // 少于3个点无法构成多边形，可视为退化情况
            return true; 
        }

        bool got_negative = false;
        bool got_positive = false;

        for (size_t i = 0; i < n; ++i) {
            const Location& p1 = shape[i];
            const Location& p2 = shape[(i + 1) % n];
            const Location& p3 = shape[(i + 2) % n];

            long long cp = cross_product(p1, p2, p3);

            if (cp < 0) {
                got_negative = true;
            }
            if (cp > 0) {
                got_positive = true;
            }

            // 如果同时出现了正负叉积，说明既有左转又有右转，必定是非凸的
            if (got_negative && got_positive) {
                return false;
            }
        }

        // 如果遍历完成，只出现了一种转向（或全是共线），则是凸的
        return true;
    }

    template <class T>
    std::vector<std::pair<int, int>> getCellsCrossedByPolyon(int x1, int y1, int x2, int y2, const std::vector<Location> &shape_ori, const T &map)
    {
        // FORTH METHOD : 混合方法
        // --- 智能调度中心 ---
        // 1. 检查原始形状是否为凸
        if (isConvex(shape_ori)) {
            // 2. 如果是凸多边形，使用速度最快的“凸包法”
            // （此处直接粘贴凸包法的完整实现）
            // 如果是原地旋转或停留，直接栅格化当前位置的多边形
            if (x1 == x2 && y1 == y2) {
                // 注意：原地旋转也需要考虑朝向，但此处简化为栅格化原始形状
                // 一个完整的实现可能需要传入旋转后的形状
                // return getCellsOriginByPolyon(x1, y1, shape_ori, map);
                return getCellsOriginByPolyon_Optimized(x1, y1, shape_ori, map);
            }

            // --- 凸包法 ---

            // 1. 获取起点和终点处，旋转后形状的绝对顶点坐标
            double heading = calcHeading_lineofsight(x1, y1, x2, y2); 
            Shape_Collide sc_temp; 
            Location ori = {0, 0};
            std::vector<Location> shape_temp = shape_ori;
            std::vector<Location> shape_heading = sc_temp.rotate_shape(ori, heading, shape_temp);

            std::vector<Location> all_vertices;
            all_vertices.reserve(shape_ori.size() * 2);

            // 添加起点处多边形的顶点
            for (const auto& p_rel : shape_heading) {
                all_vertices.push_back(p_rel + Location(x1, y1));
            }
            // 添加终点处多边形的顶点
            for (const auto& p_rel : shape_heading) {
                all_vertices.push_back(p_rel + Location(x2, y2));
            }
            // std::cout << "All Vertices:" << std::endl;
            // for (const auto& p : all_vertices) {
            //     std::cout << "(" << p.x << ", " << p.y << ")" << std::endl;
            // }  

            // 2. 计算这些顶点的凸包，形成一个包裹整个移动轨迹的大多边形
            std::vector<Location> swept_hull_abs = calculateConvexHull(all_vertices);

            // for (const auto& p : swept_hull_abs) {
            //     std::cout << "Convex Hull Point: (" << p.x << ", " << p.y << ")" << std::endl;
            // }

            // 将凸包的绝对坐标转换为相对于其自身包围盒左上角的相对坐标，以便复用getCellsOriginByPolyon
            if (swept_hull_abs.empty()) {
                return {};
            }
            int hull_min_x = swept_hull_abs[0].x;
            int hull_min_y = swept_hull_abs[0].y;
            for(const auto& p : swept_hull_abs) {
                hull_min_x = std::min(hull_min_x, static_cast<int>(p.x) );
                hull_min_y = std::min(hull_min_y, static_cast<int>(p.y) );
            }
            std::vector<Location> swept_hull_rel;
            for(const auto& p_abs : swept_hull_abs) {
                swept_hull_rel.push_back({p_abs.x - hull_min_x, p_abs.y - hull_min_y});
                // std::cout << "Relative Hull Point: (" << p_abs.x - hull_min_x << ", " << p_abs.y - hull_min_y << ")" << std::endl;
            }

            // 3. 调用我们已优化好的栅格化函数，对这个新的“扫掠多边形”进行一次光栅化
            // std::cout << "hull_min_x: " << hull_min_x << ", hull_min_y: " << hull_min_y << std::endl;
            // std::vector<std::pair<int, int>> result_cells = getCellsOriginByPolyon_Optimized(hull_min_x, hull_min_y, swept_hull_rel, map);
            // for (const auto& cell : result_cells) {
            //     std::cout << "Final Cell: (" << cell.first << ", " << cell.second << ")" << std::endl;
            // }
            return getCellsOriginByPolyon_Optimized(hull_min_x, hull_min_y, swept_hull_rel, map);

        } else {
            if (x1 == x2 && y1 == y2) {
                return getCellsOriginByPolyon_Optimized(x1, y1, shape_ori, map);
            }

            double heading = calcHeading_lineofsight(x1, y1, x2, y2);
            Shape_Collide sc_temp;
            Location ori = {0, 0};
            std::vector<Location> shape_temp = shape_ori;
            std::vector<Location> shape_heading = sc_temp.rotate_shape(ori, heading, shape_temp);

            std::vector<std::pair<int, int>> cells1_vec = getCellsOriginByPolyon_Optimized(x1, y1, shape_heading, map);
            if (cells1_vec.empty()) {
                return {};
            }

            // --- 优化: 高效的RLE转换算法 ---
            std::map<int, std::vector<std::pair<int, int>>> shape_rle;
            if (!cells1_vec.empty()) {
                int current_y = cells1_vec[0].second;
                int run_start_x = cells1_vec[0].first;
                for (size_t i = 1; i < cells1_vec.size(); ++i) {
                    // 如果当前点与上一个点在同一行且X坐标连续，则继续当前行程
                    if (cells1_vec[i].second == current_y && cells1_vec[i].first == cells1_vec[i-1].first + 1) {
                        continue;
                    } else {
                        // 否则，结束上一个行程并记录
                        shape_rle[current_y].push_back({run_start_x, cells1_vec[i-1].first});
                        // 开始新的行程
                        current_y = cells1_vec[i].second;
                        run_start_x = cells1_vec[i].first;
                    }
                }
                // 不要忘记记录最后一个行程
                shape_rle[current_y].push_back({run_start_x, cells1_vec.back().first});
            }

            std::vector<std::pair<int, int>> cells2 = getCellsCrossedByLine_Amanatides_Woo(x1, y1, x2, y2, map);
            if (cells2.empty()) {
                return cells1_vec;
            }

            int shape_min_x = cells1_vec.front().first;
            int shape_max_x = cells1_vec.back().first;
            int shape_min_y = cells1_vec.front().second;
            // 获取 shape_max_y 需要遍历，但RLE转换后就不需要这个值了
            
            int path_min_dx = 0, path_max_dx = 0, path_min_dy = 0, path_max_dy = 0;
            for (const auto& cell : cells2) {
                int dx = cell.first - x1;
                int dy = cell.second - y1;
                path_min_dx = std::min(path_min_dx, dx);
                path_max_dx = std::max(path_max_dx, dx);
                path_min_dy = std::min(path_min_dy, dy);
                path_max_dy = std::max(path_max_dy, dy);
            }
            
            int final_min_x = shape_min_x + path_min_dx;
            int final_max_x = shape_max_x + path_max_dx;
            int final_min_y = shape_rle.begin()->first + path_min_dy; // 使用RLE map的第一个y
            int final_max_y = shape_rle.rbegin()->first + path_max_dy; // 使用RLE map的最后一个y

            int grid_width = final_max_x - final_min_x + 1;
            int grid_height = final_max_y - final_min_y + 1;
            std::vector<std::vector<bool>> result_grid(grid_height, std::vector<bool>(grid_width, false));

            for (const auto& path_cell : cells2) {
                int dx = path_cell.first - x1;
                int dy = path_cell.second - y1;

                for (const auto& pair : shape_rle) {
                    int y_orig = pair.first;
                    const auto& runs = pair.second;
                    for (const auto& run : runs) {
                        int target_y = y_orig + dy - final_min_y;

                        if (target_y >= 0 && target_y < grid_height) {
                            // 微优化: 预先裁剪x的范围
                            int target_start_x = run.first + dx - final_min_x;
                            int target_end_x = run.second + dx - final_min_x;
                            
                            int start_x = std::max(0, target_start_x);
                            int end_x = std::min(grid_width - 1, target_end_x);

                            for (int x = start_x; x <= end_x; ++x) {
                                result_grid[target_y][x] = true;
                            }
                        }
                    }
                }
            }

            std::vector<std::pair<int, int>> whole_move_cells;
            whole_move_cells.reserve(grid_width * grid_height / 2);
            for (int y_idx = 0; y_idx < grid_height; ++y_idx) {
                for (int x_idx = 0; x_idx < grid_width; ++x_idx) {
                    if (result_grid[y_idx][x_idx]) {
                        whole_move_cells.push_back({x_idx + final_min_x, y_idx + final_min_y});
                    }
                }
            }
            
            // 此处的去重是必要的，因为不同中心的“盖章”可能会有重叠区域
            std::sort(whole_move_cells.begin(), whole_move_cells.end());
            whole_move_cells.erase(std::unique(whole_move_cells.begin(), whole_move_cells.end()), whole_move_cells.end());
            
            return whole_move_cells;
        }
    }


    template <class T>
    std::vector<std::pair<int, int>> getCellsCrossedByPolyon_double(double x1, double y1, double x2, double y2, const std::vector<Location> &shape_ori, const T &map)
    {
        // FORTH METHOD : 混合方法
        // --- 智能调度中心 ---
        // 1. 先验知识检测：检查原始形状是否为凸
        if (isConvex(shape_ori)) {
            // 2. 如果是凸多边形，使用速度最快的“凸包法”
            // （此处直接粘贴凸包法的完整实现）
            // 如果是原地旋转或停留，直接栅格化当前位置的多边形
            if (x1 == x2 && y1 == y2) {
                // 注意：原地旋转也需要考虑朝向，但此处简化为栅格化原始形状
                // 一个完整的实现可能需要传入旋转后的形状
                // return getCellsOriginByPolyon(x1, y1, shape_ori, map);
                return getCellsOriginByPolyon_double(x1, y1, shape_ori, map);
            }

            // --- 正确且高效的凸包法 ---

            // 1. 获取起点和终点处，旋转后形状的绝对顶点坐标
            double heading = calcHeading_lineofsight(x1, y1, x2, y2); // 假设此函数已存在
            Shape_Collide sc_temp; // 假设此类已存在
            Location ori = {0, 0};
            std::vector<Location> shape_temp = shape_ori;
            std::vector<Location> shape_heading = sc_temp.rotate_shape(ori, heading, shape_temp);

            std::vector<Location> all_vertices;
            all_vertices.reserve(shape_ori.size() * 2);

            // 添加起点处多边形的顶点
            for (const auto& p_rel : shape_heading) {
                all_vertices.push_back(p_rel + Location(x1, y1));
            }
            // 添加终点处多边形的顶点
            for (const auto& p_rel : shape_heading) {
                all_vertices.push_back(p_rel + Location(x2, y2));
            }

            // 2. 计算这些顶点的凸包，形成一个包裹整个移动轨迹的大多边形
            std::vector<Location> swept_hull_abs = calculateConvexHull(all_vertices);

            // 将凸包的绝对坐标转换为相对于其自身包围盒左上角的相对坐标，以便复用getCellsOriginByPolyon
            if (swept_hull_abs.empty()) {
                return {};
            }
            double hull_min_x = swept_hull_abs[0].x;
            double hull_min_y = swept_hull_abs[0].y;
            for(const auto& p : swept_hull_abs) {
                hull_min_x = std::min(hull_min_x, p.x );
                hull_min_y = std::min(hull_min_y, p.y );
            }
            std::vector<Location> swept_hull_rel;
            for(const auto& p_abs : swept_hull_abs) {
                swept_hull_rel.push_back({p_abs.x - hull_min_x, p_abs.y - hull_min_y});
            }

            // 3. 调用我们已优化好的栅格化函数，对这个新的“扫掠多边形”进行一次光栅化
            return getCellsOriginByPolyon_double(hull_min_x, hull_min_y, swept_hull_rel, map);

        } else {
            if (x1 == x2 && y1 == y2) {
                return getCellsOriginByPolyon_double(x1, y1, shape_ori, map);
            }

            double heading = calcHeading_lineofsight(x1, y1, x2, y2);
            Shape_Collide sc_temp;
            Location ori = {0, 0};
            std::vector<Location> shape_temp = shape_ori;
            std::vector<Location> shape_heading = sc_temp.rotate_shape(ori, heading, shape_temp);

            std::vector<std::pair<int, int>> cells1_vec = getCellsOriginByPolyon_double(x1, y1, shape_heading, map);
            if (cells1_vec.empty()) {
                return {};
            }

            // --- 优化: 高效的RLE转换算法 ---
            std::map<int, std::vector<std::pair<int, int>>> shape_rle;
            if (!cells1_vec.empty()) {
                int current_y = cells1_vec[0].second;
                int run_start_x = cells1_vec[0].first;
                for (size_t i = 1; i < cells1_vec.size(); ++i) {
                    // 如果当前点与上一个点在同一行且X坐标连续，则继续当前行程
                    if (cells1_vec[i].second == current_y && cells1_vec[i].first == cells1_vec[i-1].first + 1) {
                        continue;
                    } else {
                        // 否则，结束上一个行程并记录
                        shape_rle[current_y].push_back({run_start_x, cells1_vec[i-1].first});
                        // 开始新的行程
                        current_y = cells1_vec[i].second;
                        run_start_x = cells1_vec[i].first;
                    }
                }
                // 不要忘记记录最后一个行程
                shape_rle[current_y].push_back({run_start_x, cells1_vec.back().first});
            }

            std::vector<std::pair<int, int>> cells2 = getCellsCrossedByLine_double(x1, y1, x2, y2, map);
            if (cells2.empty()) {
                return cells1_vec;
            }

            int shape_min_x = cells1_vec.front().first;
            int shape_max_x = cells1_vec.back().first;
            int shape_min_y = cells1_vec.front().second;
            // 获取 shape_max_y 需要遍历，但RLE转换后就不需要这个值了
            
            int path_min_dx = 0, path_max_dx = 0, path_min_dy = 0, path_max_dy = 0;
            for (const auto& cell : cells2) {
                int dx = cell.first - x1;
                int dy = cell.second - y1;
                path_min_dx = std::min(path_min_dx, dx);
                path_max_dx = std::max(path_max_dx, dx);
                path_min_dy = std::min(path_min_dy, dy);
                path_max_dy = std::max(path_max_dy, dy);
            }
            
            int final_min_x = shape_min_x + path_min_dx;
            int final_max_x = shape_max_x + path_max_dx;
            int final_min_y = shape_rle.begin()->first + path_min_dy; // 使用RLE map的第一个y
            int final_max_y = shape_rle.rbegin()->first + path_max_dy; // 使用RLE map的最后一个y

            int grid_width = final_max_x - final_min_x + 1;
            int grid_height = final_max_y - final_min_y + 1;
            std::vector<std::vector<bool>> result_grid(grid_height, std::vector<bool>(grid_width, false));

            for (const auto& path_cell : cells2) {
                int dx = path_cell.first - x1;
                int dy = path_cell.second - y1;

                for (const auto& pair : shape_rle) {
                    int y_orig = pair.first;
                    const auto& runs = pair.second;
                    for (const auto& run : runs) {
                        int target_y = y_orig + dy - final_min_y;

                        if (target_y >= 0 && target_y < grid_height) {
                            // 微优化: 预先裁剪x的范围
                            int target_start_x = run.first + dx - final_min_x;
                            int target_end_x = run.second + dx - final_min_x;
                            
                            int start_x = std::max(0, target_start_x);
                            int end_x = std::min(grid_width - 1, target_end_x);

                            for (int x = start_x; x <= end_x; ++x) {
                                result_grid[target_y][x] = true;
                            }
                        }
                    }
                }
            }

            std::vector<std::pair<int, int>> whole_move_cells;
            whole_move_cells.reserve(grid_width * grid_height / 2);
            for (int y_idx = 0; y_idx < grid_height; ++y_idx) {
                for (int x_idx = 0; x_idx < grid_width; ++x_idx) {
                    if (result_grid[y_idx][x_idx]) {
                        whole_move_cells.push_back({x_idx + final_min_x, y_idx + final_min_y});
                    }
                }
            }
            
            // 此处的去重是必要的，因为不同中心的“盖章”可能会有重叠区域
            std::sort(whole_move_cells.begin(), whole_move_cells.end());
            whole_move_cells.erase(std::unique(whole_move_cells.begin(), whole_move_cells.end()), whole_move_cells.end());
            
            return whole_move_cells;
        }
    }


    std::set<std::pair<int, int>> getCellsOriginByPolyon_Ori(const std::vector<Location> &shape)
    {
        // 只获取相对占据栅格且不考虑可通行性的简单版本
        std::set<std::pair<int, int>> polygonCells;
    int numVertices = shape.size();

    if (numVertices < 3) {
        return polygonCells; // A polygon needs at least 3 vertices
    }

    // Calculate bounding box for scanlines in relative coordinates
    int minX_rel = shape[0].x;
    int maxX_rel = shape[0].x;
    int minY_rel = shape[0].y;
    int maxY_rel = shape[0].y;

    for (int i = 0; i < numVertices; ++i) {
        // Update min/max for bounding box
        minX_rel = std::min(minX_rel, static_cast<int>(shape[i].x));
        maxX_rel = std::max(maxX_rel, static_cast<int>(shape[i].x));
        minY_rel = std::min(minY_rel, static_cast<int>(shape[i].y));
        maxY_rel = std::max(maxY_rel, static_cast<int>(shape[i].y));

        // 1. Add cells on the polygon's boundary (relative coordinates)
        const Location& p1_rel = shape[i];
        const Location& p2_rel = shape[(i + 1) % numVertices]; // Connect last to first

        // 25.10.20 这里从 double -> int 未经处理会有精度损失
        std::vector<std::pair<int, int>> lineCells = getCellsCrossedByLine_Ori(round(p1_rel.x), round(p1_rel.y), round(p2_rel.x), round(p2_rel.y));
        for (const auto& cell : lineCells) {
            polygonCells.insert(cell);
        }
    }

    // 2. Scanline fill using the Even-Odd Rule (relative coordinates)
    for (int y = minY_rel; y <= maxY_rel; ++y) {
        std::vector<double> intersections_x; // Store x-coordinates of intersections (can be float)

        for (int i = 0; i < numVertices; ++i) {
            const Location& p1_rel = shape[i];
            const Location& p2_rel = shape[(i + 1) % numVertices];

            // Normalize points so pA has the smaller Y-coordinate
            const Location* pA = &p1_rel;
            const Location* pB = &p2_rel;
            if (pA->y > pB->y) {
                std::swap(pA, pB);
            }

            // Check if the scanline 'y' crosses the edge (pA, pB)
            // Condition for Even-Odd rule: y is within the Y-range of the edge,
            // and strictly less than the upper Y to avoid double-counting horizontal edges
            // or vertices that are "peaks" or "valleys"
            if (y >= pA->y && y < pB->y) {
                // Exclude perfectly horizontal segments as they don't change parity
                if (pA->y != pB->y) {
                    double intersectX = pA->x + static_cast<double>(y - pA->y) * (pB->x - pA->x) / (pB->y - pA->y);
                    intersections_x.push_back(intersectX);
                }
            }
        }

        std::sort(intersections_x.begin(), intersections_x.end());

        // Fill cells between pairs of intersections
        for (size_t i = 0; i + 1 < intersections_x.size(); i += 2) {
            // Round floating-point intersections to integer grid coordinates
            // We use round for consistency with Bresenham's line algorithm.
            int startX = static_cast<int>(std::round(intersections_x[i]));
            int endX = static_cast<int>(std::round(intersections_x[i+1]));

            // Ensure startX <= endX (can be flipped due to rounding)
            if (startX > endX) {
                std::swap(startX, endX);
            }

            for (int x = startX; x <= endX; ++x) {
                polygonCells.insert({x, y});
            }
        }
    }
    return polygonCells;
    }

    template <class T>
    void getCellsCrossedByLine_AW2(int x1, int y1, int x2, int y2, std::set<std::pair<int, int>>& cells_ori, const T &map)
    {
        int current_x = x1;
        int current_y = y1;

        const int dx = x2 - x1;
        const int dy = y2 - y1;

        const int step_x = (dx > 0) ? 1 : -1;
        const int step_y = (dy > 0) ? 1 : -1;

        // 处理垂直和水平的特殊情况，以避免除以零
        if (dx == 0) {
            for (int y = y1; y != y2 + step_y; y += step_y) {
                if (map.CellOnGrid(x1, y)) 
                    cells_ori.insert({x1, y});
            }
            return ;
        }
        if (dy == 0) {
            for (int x = x1; x != x2 + step_x; x += step_x) {
                if (map.CellOnGrid(x, y1)) 
                    cells_ori.insert({x, y1});
            }
            return ;
        }

        // tDeltaX/Y: 沿x/y方向前进一个单位栅格，t值需要增加多少
        const double tDeltaX = std::abs(1.0 / static_cast<double>(dx));
        const double tDeltaY = std::abs(1.0 / static_cast<double>(dy));

        // tMaxX/Y: t值到达下一个x/y方向栅格边界的值
        double tMaxX, tMaxY;

        // 计算初始的tMaxX和tMaxY
        if (step_x > 0) {
            tMaxX = (static_cast<double>(x1 + 1) - (x1 + 0.5)) * tDeltaX;
        } else {
            tMaxX = ((x1 + 0.5) - static_cast<double>(x1)) * tDeltaX;
        }
        if (step_y > 0) {
            tMaxY = (static_cast<double>(y1 + 1) - (y1 + 0.5)) * tDeltaY;
        } else {
            tMaxY = ((y1 + 0.5) - static_cast<double>(y1)) * tDeltaY;
        }

        while (true) {
            if (map.CellOnGrid(current_x, current_y)) {
                cells_ori.insert({current_x, current_y});
            }

            if (current_x == x2 && current_y == y2) {
                break;
            }

            // 核心逻辑：比较到达下一个x边界和y边界的t值
            // 哪个t值小，就先朝哪个方向移动
            if (tMaxX < tMaxY) {
                tMaxX += tDeltaX;
                current_x += step_x;
            } else {
                tMaxY += tDeltaY;
                current_y += step_y;
            }
        }
        return;
    }

    // 辅助函数：使用一个健壮的Bresenham算法实现来获取线段上的所有栅格
    template <class T>
    void getCellsOnLine_Optimized(int x1, int y1, int x2, int y2, std::set<std::pair<int, int>>& cells_ori, const T &map)
    {
        bool is_steep = std::abs(y2 - y1) > std::abs(x2 - x1);
        if (is_steep) {
            std::swap(x1, y1);
            std::swap(x2, y2);
        }

        if (x1 > x2) {
            std::swap(x1, x2);
            std::swap(y1, y2);
        }

        int dx = x2 - x1;
        int dy = std::abs(y2 - y1);
        int err = dx / 2;
        int y_step = (y1 < y2) ? 1 : -1;
        int y = y1;

        for (int x = x1; x <= x2; ++x) {
            if (is_steep) {
                if (map.CellOnGrid(y, x)) {
                    cells_ori.insert({y, x});
                }
            } else {
                if (map.CellOnGrid(x, y)) {
                    cells_ori.insert({x, y});
                }
            }

            err -= dy;
            if (err < 0) {
                y += y_step;
                err += dx;
            }
        }
    }



        // 改进后的版本，将结果存入 vector 以提高效率，并优化了循环内的分支
    template <class T>
    void getCellsOnLine_Optimized_Improved(int x1, int y1, int x2, int y2, std::vector<std::pair<int, int>>& cells_out, const T &map)
    {
        bool is_steep = std::abs(y2 - y1) > std::abs(x2 - x1);
        if (is_steep) {
            std::swap(x1, y1);
            std::swap(x2, y2);
        }

        if (x1 > x2) {
            std::swap(x1, x2);
            std::swap(y1, y2);
        }

        int dx = x2 - x1;
        int dy = std::abs(y2 - y1);
        int err = dx / 2;
        int y_step = (y1 < y2) ? 1 : -1;
        int y = y1;

        if (is_steep) { // 将 is_steep 判断移出循环
            for (int x = x1; x <= x2; ++x) {
                // 25.10.22 取消了所占据的栅格是否可通行、是否在地图内的判断
                // if (map.CellOnGrid(y, x)) {
                    cells_out.push_back({y, x}); // 使用 push_back，效率更高
                // }
                err -= dy;
                if (err < 0) {
                    y += y_step;
                    err += dx;
                }
            }
        } else {
            for (int x = x1; x <= x2; ++x) {
                // 25.10.22 取消了所占据的栅格是否可通行、是否在地图内的判断
                // if (map.CellOnGrid(x, y)) {
                    cells_out.push_back({x, y}); // 使用 push_back，效率更高
                // }
                err -= dy;
                if (err < 0) {
                    y += y_step;
                    err += dx;
                }
            }
        }
    }


    template <class T>
    std::set<std::pair<int, int>> getCellsOriginByPolyon(int x1, int y1, const std::vector<Location> &shape, const T &map)
    {
        // FORTH METHOD : 边所占据的栅格+AET方法填充
        std::set<std::pair<int, int>> polygonCells;
        // std::vector<std::pair<int, int>> polygonCells;
        if (shape.size() < 3) {
            return {};
            }
        
        // --- 准备工作：计算绝对坐标和Y范围 ---
        std::vector<Location> absolute_shape;
        absolute_shape.reserve(shape.size());
        int minY = std::numeric_limits<int>::max();
        int maxY = std::numeric_limits<int>::min();

        for (const auto& p_rel : shape) {
            Location p_abs = p_rel + Location(x1, y1);
            absolute_shape.push_back(p_abs);
            minY = std::min(minY, static_cast<int>(p_abs.y));
            maxY = std::max(maxY, static_cast<int>(p_abs.y));
        }

        std::vector<std::pair<int, int>> boundaryCellsVec;
        boundaryCellsVec.reserve(absolute_shape.size() * 10); // 预分配一些内存以减少重分配
        // --- 第一阶段：精确绘制所有边界 ---
        for (size_t i = 0; i < absolute_shape.size(); ++i) {
            const auto& p1 = absolute_shape[i];
            const auto& p2 = absolute_shape[(i + 1) % absolute_shape.size()];
            // getCellsOnLine_Optimized(p1.x, p1.y, p2.x, p2.y, polygonCells, map);
            // 25.10.20 这里从 double -> int 未经处理会有精度损失
            // getCellsOnLine_Optimized_Improved(round(p1.x), round(p1.y), round(p2.x), round(p2.y), boundaryCellsVec, map);

            // 25.10.22 修改为 Amanatides-Woo 算法
            boundaryCellsVec = getCellsCrossedByLine_Amanatides_Woo(round(p1.x), round(p1.y), round(p2.x), round(p2.y), map);
        }
        // 将所有边界点一次性插入set
        polygonCells.insert(boundaryCellsVec.begin(), boundaryCellsVec.end());

        // --- 第二阶段：使用AET填充内部区域 ---
        std::map<int, std::vector<EdgeEntry>> edgeTable;
        for (size_t i = 0; i < absolute_shape.size(); ++i) {
            Location p1 = absolute_shape[i];
            Location p2 = absolute_shape[(i + 1) % absolute_shape.size()];

            // 非水平边才需要加入边表进行扫描线填充
            if (p1.y != p2.y) {
                if (p1.y > p2.y) std::swap(p1, p2);
                EdgeEntry edge;
                edge.inv_slope = static_cast<double>(p2.x - p1.x) / (p2.y - p1.y);
                edge.x_intersect = static_cast<double>(p1.x);
                edge.y_max = p2.y;
                edgeTable[p1.y].push_back(edge);
            }
        }

        std::vector<EdgeEntry> activeEdgeTable;
        for (int y = minY; y < maxY; ++y) { // 扫描从minY到maxY-1的“缝隙”
            // 将在当前扫描线y开始的新边从ET添加到AET
            if (edgeTable.count(y)) {
                activeEdgeTable.insert(activeEdgeTable.end(), edgeTable.at(y).begin(), edgeTable.at(y).end());
            }

            // 移除已经完成使命的边
            activeEdgeTable.erase(
                std::remove_if(activeEdgeTable.begin(), activeEdgeTable.end(),
                            [y](const EdgeEntry& edge) { return edge.y_max <= y; }),
                activeEdgeTable.end()
            );
            
            // 排序并填充内部
            std::sort(activeEdgeTable.begin(), activeEdgeTable.end());

            if (activeEdgeTable.size() >= 2) {
                for (size_t i = 0; i < activeEdgeTable.size(); i += 2) {
                    if(i + 1 < activeEdgeTable.size()){
                        // 使用ceil和floor来处理浮点数，确保覆盖所有内部栅格
                        int startX = static_cast<int>(std::ceil(activeEdgeTable[i].x_intersect));
                        int endX = static_cast<int>(std::floor(activeEdgeTable[i + 1].x_intersect));

                        for (int x = startX; x <= endX; ++x) {
                            // 25.10.22 取消了所占据的栅格是否可通行、是否在地图内的判断
                            // if (map.CellOnGrid(x, y) && map.CellIsTraversable(x, y)) {
                            // if (map.CellOnGrid(x, y)) {
                                polygonCells.insert({x, y});
                            // }
                        }
                    }
                }
            }
            // 更新AET中所有边的x坐标，为下一条扫描线做准备
            for (auto& edge : activeEdgeTable) {
                edge.x_intersect += edge.inv_slope;
            }
        }
        return polygonCells;
    }


    template <class T>
// 优化: 返回 vector 以获得更好的性能。调用者如果确实需要，可以自行转换为 set。
std::vector<std::pair<int, int>> getCellsOriginByPolyon_Optimized(int x1, int y1, const std::vector<Location> &shape, const T &map)
{
    if (shape.size() < 3) {
        return {};
    }

    // 优化: 在整个函数处理过程中使用 vector。
    std::vector<std::pair<int, int>> polygonCells;

    // --- 准备工作: 计算绝对坐标和Y轴范围 ---
    std::vector<Location> absolute_shape;
    absolute_shape.reserve(shape.size());
    int minY = std::numeric_limits<int>::max();
    int maxY = std::numeric_limits<int>::min();

    int minX = std::numeric_limits<int>::max();
    int maxX = std::numeric_limits<int>::min();

    for (const auto& p_rel : shape) {
        Location p_abs = p_rel + Location(x1, y1);
        absolute_shape.push_back(p_abs);
        minY = std::min(minY, static_cast<int>(std::floor(p_abs.y)));
        maxY = std::max(maxY, static_cast<int>(std::ceil(p_abs.y)));
        minX = std::min(minX, static_cast<int>(std::floor(p_abs.x)));
        maxX = std::max(maxX, static_cast<int>(std::ceil(p_abs.x)));
    }
    
    // 优化: 为结果 vector 预分配内存。
    // 这是一种启发式策略，但可以有效避免运行时的多次内存重分配。
    polygonCells.reserve((size_t)( (maxX - minX + 1) * (maxY - minY + 1) * 0.5 ));


    // --- 阶段一: 精确绘制所有边界 ---
    std::vector<std::pair<int, int>> boundaryCellsVec;
    boundaryCellsVec.reserve(absolute_shape.size() * 10);
    for (size_t i = 0; i < absolute_shape.size(); ++i) {
        const auto& p1 = absolute_shape[i];
        const auto& p2 = absolute_shape[(i + 1) % absolute_shape.size()];
        
        // 25.10.20 这里从 double -> int 未经处理会有精度损失
        // getCellsOnLine_Optimized_Improved(p1.x, p1.y, p2.x, p2.y, boundaryCellsVec, map);
        // getCellsOnLine_Optimized_Improved(round(p1.x), round(p1.y), round(p2.x), round(p2.y), boundaryCellsVec, map);

        // 25.10.22 修改为 Amanatides-Woo 算法
        std::vector<std::pair<int, int>> cells_temp = getCellsCrossedByLine_Amanatides_Woo(round(p1.x), round(p1.y), round(p2.x), round(p2.y), map);
        // std::cout<< "Line From (" << round(p1.x) << ", " << round(p1.y) << ") to (" << round(p2.x) << ", " << round(p2.y) << ") " << std::endl;
        for (const auto& cell : cells_temp) {
            
            // std::cout<< "Boundary Cell: (" << cell.first << ", " << cell.second << ")" << std::endl;
            boundaryCellsVec.push_back(cell);
        }
    }
    // 将所有边界点一次性插入到我们的主 vector 中。
    polygonCells.insert(polygonCells.end(), boundaryCellsVec.begin(), boundaryCellsVec.end());

    // --- 阶段二: 使用 AET 填充内部区域 ---
    std::map<int, std::vector<EdgeEntry>> edgeTable;
    for (size_t i = 0; i < absolute_shape.size(); ++i) {
        Location p1 = absolute_shape[i];
        Location p2 = absolute_shape[(i + 1) % absolute_shape.size()];

        if (static_cast<int>(p1.y) != static_cast<int>(p2.y)) { // 只处理非水平边
            if (p1.y > p2.y) std::swap(p1, p2);
            EdgeEntry edge;
            edge.inv_slope = (p2.x - p1.x) / (p2.y - p1.y);
            // 对起始交点x坐标进行微调，以处理边起点恰好落在扫描线上的情况
            if (static_cast<int>(p1.y) == p1.y) {
                 edge.x_intersect = p1.x;
            } else {
                 edge.x_intersect = p1.x + edge.inv_slope * (std::ceil(p1.y) - p1.y);
            }
            edge.y_max = static_cast<int>(std::ceil(p2.y));
            edgeTable[static_cast<int>(std::ceil(p1.y))].push_back(edge);
        }
    }

    std::vector<EdgeEntry> activeEdgeTable;
    activeEdgeTable.reserve(shape.size());

    for (int y = minY; y < maxY; ++y) {
        // 优化: 添加新边时维持AET有序，避免完全重排。
        if (edgeTable.count(y)) {
            for(const auto& new_edge : edgeTable.at(y)) {
                // 找到正确的排序位置并插入
                auto it = std::lower_bound(activeEdgeTable.begin(), activeEdgeTable.end(), new_edge);
                activeEdgeTable.insert(it, new_edge);
            }
        }

        // 移除已经结束的边
        activeEdgeTable.erase(
            std::remove_if(activeEdgeTable.begin(), activeEdgeTable.end(),
                        [y](const EdgeEntry& edge) { return edge.y_max <= y; }),
            activeEdgeTable.end()
        );
        
        // 优化: 不再需要 std::sort()！

        // 在成对的边之间填充颜色
        for (size_t i = 0; i + 1 < activeEdgeTable.size(); i += 2) {
            int startX = static_cast<int>(std::ceil(activeEdgeTable[i].x_intersect));
            int endX = static_cast<int>(std::floor(activeEdgeTable[i + 1].x_intersect));

            for (int x = startX; x <= endX; ++x) {
                // 25.10.22 取消了所占据的栅格是否可通行、是否在地图内的判断
                // if (map.CellOnGrid(x, y) && map.CellIsTraversable(x, y)) {
                    // 优化: 高效地 push_back 到 vector。
                    polygonCells.push_back({x, y});
                // }
            }
        }
    
        // 为下一条扫描线更新边的 x 坐标
        for (auto& edge : activeEdgeTable) {
            edge.x_intersect += edge.inv_slope;
        }
    }

    // 优化: 在函数末尾高效地去重。
    std::sort(polygonCells.begin(), polygonCells.end());
    polygonCells.erase(std::unique(polygonCells.begin(), polygonCells.end()), polygonCells.end());

    return polygonCells;
}

template <class T>
std::vector<std::pair<int, int>> getCellsOriginByPolyon_double(double x1, double y1, const std::vector<Location> &shape, const T &map)
{
    if (shape.size() < 3) {
        return {};
    }

    std::vector<std::pair<int, int>> polygonCells;

    // --- 準備工作: 計算絕對座標和Y軸範圍 (您的這部分是正確的) ---
    std::vector<Location> absolute_shape;
    absolute_shape.reserve(shape.size());
    int minY = std::numeric_limits<int>::max();
    int maxY = std::numeric_limits<int>::min();
    int minX = std::numeric_limits<int>::max();
    int maxX = std::numeric_limits<int>::min();

    for (const auto& p_rel : shape) {
        Location p_abs = p_rel + Location(x1, y1);
        absolute_shape.push_back(p_abs);
        minY = std::min(minY, static_cast<int>(std::floor(p_abs.y)));
        maxY = std::max(maxY, static_cast<int>(std::ceil(p_abs.y)));
        minX = std::min(minX, static_cast<int>(std::floor(p_abs.x)));
        maxX = std::max(maxX, static_cast<int>(std::ceil(p_abs.x)));
    }
    
    polygonCells.reserve((size_t)((maxX - minX + 1) * (maxY - minY + 1) * 0.5));

    // --- 階段一: 精確繪製所有边界 ---
    // 【修正 #1】: 使用能夠處理 double 座標的 getCellsCrossedByLine_double 函數
    for (size_t i = 0; i < absolute_shape.size(); ++i) {
        const auto& p1 = absolute_shape[i];
        const auto& p2 = absolute_shape[(i + 1) % absolute_shape.size()];
        
        // 呼叫正確的 double 版本線段光柵化函數
        std::vector<std::pair<int, int>> boundarySegment = getCellsCrossedByLine_double(p1.x, p1.y, p2.x, p2.y, map);
        polygonCells.insert(polygonCells.end(), boundarySegment.begin(), boundarySegment.end());
    }

    // --- 階段二: 使用 AET 填充內部區域 ---
    std::map<int, std::vector<EdgeEntry>> edgeTable;
    for (size_t i = 0; i < absolute_shape.size(); ++i) {
        Location p1 = absolute_shape[i];
        Location p2 = absolute_shape[(i + 1) % absolute_shape.size()];

        // 【修正 #2】: 使用更穩健的方式判斷非水平邊
        constexpr double EPSILON = 1e-9;
        if (std::abs(p1.y - p2.y) > EPSILON) { // 只處理非嚴格水平的邊
            if (p1.y > p2.y) std::swap(p1, p2);
            EdgeEntry edge;
            edge.inv_slope = (p2.x - p1.x) / (p2.y - p1.y);
            
            // 您的這段 x_intersect 計算邏輯是正確且巧妙的
            // 它計算的是邊與第一條掃描線 ceil(p1.y) 的交點
            edge.x_intersect = p1.x + edge.inv_slope * (std::ceil(p1.y) - p1.y);
            
            edge.y_max = static_cast<int>(std::ceil(p2.y));
            edgeTable[static_cast<int>(std::ceil(p1.y))].push_back(edge);
        }
    }

    // AET 處理迴圈 (您的這部分邏輯是正確的)
    std::vector<EdgeEntry> activeEdgeTable;
    activeEdgeTable.reserve(shape.size());

    for (int y = minY; y < maxY; ++y) {
        if (edgeTable.count(y)) {
            for(const auto& new_edge : edgeTable.at(y)) {
                auto it = std::lower_bound(activeEdgeTable.begin(), activeEdgeTable.end(), new_edge);
                activeEdgeTable.insert(it, new_edge);
            }
        }

        activeEdgeTable.erase(
            std::remove_if(activeEdgeTable.begin(), activeEdgeTable.end(),
                           [y](const EdgeEntry& edge) { return edge.y_max <= y; }),
            activeEdgeTable.end()
        );
        
        for (size_t i = 0; i + 1 < activeEdgeTable.size(); i += 2) {
            int startX = static_cast<int>(std::ceil(activeEdgeTable[i].x_intersect));
            int endX = static_cast<int>(std::floor(activeEdgeTable[i + 1].x_intersect));

            for (int x = startX; x <= endX; ++x) {
                // 假設 map 檢查是需要的
                if (map.CellOnGrid(x, y) && map.CellIsTraversable(x, y)) {
                    polygonCells.push_back({x, y});
                }
            }
        }
    
        for (auto& edge : activeEdgeTable) {
            edge.x_intersect += edge.inv_slope;
        }
    }

    // 在函數末尾高效地去重
    std::sort(polygonCells.begin(), polygonCells.end());
    polygonCells.erase(std::unique(polygonCells.begin(), polygonCells.end()), polygonCells.end());

    return polygonCells;
}





    template <class T>
    std::vector<std::pair<int, int>> getCellsCrossedByLine_Amanatides_Woo(int x1, int y1, int x2, int y2, const T &map)
    {
        std::vector<std::pair<int, int>> lineCells;

        int current_x = x1;
        int current_y = y1;

        const int dx = x2 - x1;
        const int dy = y2 - y1;

        const int step_x = (dx > 0) ? 1 : -1;
        const int step_y = (dy > 0) ? 1 : -1;

        // 处理垂直和水平的特殊情况，以避免除以零
        if (dx == 0) {
            for (int y = y1; y != y2 + step_y; y += step_y) {
                // if (map.CellOnGrid(x1, y))  // 25.10.23 取消了所占据的栅格是否可通行、是否在地图内的判断
                lineCells.push_back({x1, y});
            }
            return lineCells;
        }
        if (dy == 0) {
            for (int x = x1; x != x2 + step_x; x += step_x) {
                // if (map.CellOnGrid(x, y1)) // 25.10.23 取消了所占据的栅格是否可通行、是否在地图内的判断
                lineCells.push_back({x, y1});
            }
            return lineCells;
        }

        // tDeltaX/Y: 沿x/y方向前进一个单位栅格，t值需要增加多少
        const double tDeltaX = std::abs(1.0 / static_cast<double>(dx));
        const double tDeltaY = std::abs(1.0 / static_cast<double>(dy));

        // tMaxX/Y: t值到达下一个x/y方向栅格边界的值
        double tMaxX, tMaxY;

        // 计算初始的tMaxX和tMaxY
        if (step_x > 0) {
            tMaxX = (static_cast<double>(x1 + 1) - (x1 + 0.5)) * tDeltaX;
        } else {
            tMaxX = ((x1 + 0.5) - static_cast<double>(x1)) * tDeltaX;
        }
        if (step_y > 0) {
            tMaxY = (static_cast<double>(y1 + 1) - (y1 + 0.5)) * tDeltaY;
        } else {
            tMaxY = ((y1 + 0.5) - static_cast<double>(y1)) * tDeltaY;
        }

        while (true) {

            // 25.10.22 取消了所占据的栅格是否可通行、是否在地图内的判断
            // if (map.CellOnGrid(current_x, current_y)) {
                lineCells.push_back({current_x, current_y});
            // }

            if (current_x == x2 && current_y == y2) {
                break;
            }

            // 核心逻辑：比较到达下一个x边界和y边界的t值
            // 哪个t值小，就先朝哪个方向移动
            if (tMaxX < tMaxY) {
                tMaxX += tDeltaX;
                current_x += step_x;
            } else {
                tMaxY += tDeltaY;
                current_y += step_y;
            }
        }

        return lineCells;
    }

    template <class T>
    std::vector<std::pair<int, int>> getCellsCrossedByLine_double(double x1, double y1, double x2, double y2, const T &map)
    {
        std::vector<std::pair<int, int>> lineCells;

        // --- 修改部分 ---
        // 透過 floor 函數取得起點和終點所在的儲存格座標
        int current_x = static_cast<int>(std::floor(x1));
        int current_y = static_cast<int>(std::floor(y1));
        const int end_x = static_cast<int>(std::floor(x2));
        const int end_y = static_cast<int>(std::floor(y2));

        // dx 和 dy 現在是 double 類型
        const double dx = x2 - x1;
        const double dy = y2 - y1;

        // step_x/y 的計算邏輯保持不變
        const int step_x = (dx >= 0) ? 1 : -1;
        const int step_y = (dy >= 0) ? 1 : -1;

        // --- 修改部分 ---
        // 處理 dx 或 dy 為 0 的情況，避免除以零。
        // 如果 dx 為 0，tDeltaX 設為最大值，這樣在主循環中 tMaxX < tMaxY 永遠為 false，只會沿 y 軸移動。
        const double tDeltaX = (dx == 0) ? std::numeric_limits<double>::max() : std::abs(1.0 / dx);
        const double tDeltaY = (dy == 0) ? std::numeric_limits<double>::max() : std::abs(1.0 / dy);

        // --- 修改部分 ---
        // 計算從浮點數起點 (x1, y1) 到達第一個 x 和 y 栅格邊界所需的 t 值。
        // 這一步是從 int 版本到 double 版本最核心的修改。
        double tMaxX, tMaxY;
        if (dx != 0) {
            if (step_x > 0) {
                // 從 x1 到下一個垂直網格線 (floor(x1) + 1) 的距離
                tMaxX = (std::floor(x1) + 1.0 - x1) * tDeltaX;
            } else {
                // 從 x1 到前一個垂直網格線 (floor(x1)) 的距離
                tMaxX = (x1 - std::floor(x1)) * tDeltaX;
            }
        } else {
            tMaxX = std::numeric_limits<double>::max();
        }

        if (dy != 0) {
            if (step_y > 0) {
                // 從 y1 到下一個水平網格線 (floor(y1) + 1) 的距離
                tMaxY = (std::floor(y1) + 1.0 - y1) * tDeltaY;
            } else {
                // 從 y1 到前一個水平網格線 (floor(y1)) 的距離
                tMaxY = (y1 - std::floor(y1)) * tDeltaY;
            }
        } else {
            tMaxY = std::numeric_limits<double>::max();
        }

        // 主循環，直到到達終點儲存格
        while (true) {
            // 將當前儲存格加入列表
            if (map.CellOnGrid(current_x, current_y)) {
                lineCells.push_back({current_x, current_y});
            }

            // --- 修改部分 ---
            // 迴圈的終止條件變為當前儲存格是否為終點儲存格
            if (current_x == end_x && current_y == end_y) {
                break;
            }

            // 核心邏輯不變：比較到達下一個 x 邊界和 y 邊界的 t 值
            // 哪個 t 值小，就先朝哪個方向移動
            if (tMaxX < tMaxY) {
                tMaxX += tDeltaX;
                current_x += step_x;
            } else {
                tMaxY += tDeltaY;
                current_y += step_y;
            }
        }

        return lineCells;
    }

    template <class T>
    std::vector<std::pair<int, int>> getCellsCrossedByLine(int x1, int y1, int x2, int y2, const T &map)
    {
        // 这个考虑了agentSize，需要一个不考虑agentSize的版本
        // std::cout << "Calculating cells crossed by line from (" << x1 << ", " << y1 << ") to (" << x2 << ", " << y2 << ")" << std::endl;
        // std::cout << "agentSize is :" << agentSize << std::endl;
        std::vector<std::pair<int, int>> lineCells(0);
        // 输出全部cells
        // std::cout << "All the Cells of now are :"  << std::endl;
        // for(auto cell:cells)
        // {
        //     std::cout << cell.first << " " << cell.second << std::endl;
        // }
            
        if(x1 == x2 && y1 == y2)
        {
            for(auto cell:cells)
                lineCells.push_back({x1+cell.first, y1+cell.second});
            return lineCells;
        }
        int delta_x = std::abs(x1 - x2);
        int delta_y = std::abs(y1 - y2);
        if((delta_x >= delta_y && x1 > x2) || (delta_y > delta_x && y1 > y2))
        {
            std::swap(x1, x2);
            std::swap(y1, y2);
        }
        int step_x = (x1 < x2 ? 1 : -1);
        int step_y = (y1 < y2 ? 1 : -1);
        int error = 0, x = x1, y = y1;
        int k, num;
        std::pair<int, int> add;
        int gap = agentSize*sqrt(pow(delta_x, 2) + pow(delta_y, 2)) + double(delta_x + delta_y)/2 - CN_EPSILON;

        if(delta_x >= delta_y)
        {
            int extraCheck = agentSize*delta_y/sqrt(pow(delta_x, 2) + pow(delta_y, 2)) + 0.5 - CN_EPSILON;
            for(int n = 1; n <= extraCheck; n++)
            {
                error += delta_y;
                num = (gap - error)/delta_x;
                for(k = 1; k <= num; k++)
                    lineCells.push_back({x1 - n*step_x, y1 + k*step_y});
                for(k = 1; k <= num; k++)
                    lineCells.push_back({x2 + n*step_x, y2 - k*step_y});
            }
            error = 0;
            for(x = x1; x != x2 + step_x; x+=step_x)
            {
                lineCells.push_back({x, y});
                if(x < x2 - extraCheck)
                {
                    num = (gap + error)/delta_x;
                    for(k = 1; k <= num; k++)
                        lineCells.push_back({x, y + k*step_y});
                }
                if(x > x1 + extraCheck)
                {
                    num = (gap - error)/delta_x;
                    for(k = 1; k <= num; k++)
                        lineCells.push_back({x, y - k*step_y});
                }
                error += delta_y;
                if((error<<1) > delta_x)
                {
                    y += step_y;
                    error -= delta_x;
                }
            }
        }
        else
        {
            int extraCheck = agentSize*delta_x/sqrt(pow(delta_x, 2) + pow(delta_y, 2)) + 0.5 - CN_EPSILON;
            for(int n = 1; n <= extraCheck; n++)
            {
                error += delta_x;
                num = (gap - error)/delta_y;
                for(k = 1; k <= num; k++)
                    lineCells.push_back({x1 + k*step_x, y1 - n*step_y});
                for(k = 1; k <= num; k++)
                    lineCells.push_back({x2 - k*step_x, y2 + n*step_y});
            }
            error = 0;
            for(y = y1; y != y2 + step_y; y += step_y)
            {
                lineCells.push_back({x, y});
                if(y < y2 - extraCheck)
                {
                    num = (gap + error)/delta_y;
                    for(k = 1; k <= num; k++)
                        lineCells.push_back({x + k*step_x, y});
                }
                if(y > y1 + extraCheck)
                {
                    num = (gap - error)/delta_y;
                    for(k = 1; k <= num; k++)
                        lineCells.push_back({x - k*step_x, y});
                }
                error += delta_x;
                if((error<<1) > delta_y)
                {
                    x += step_x;
                    error -= delta_y;
                }
            }
        }
        for(k = 0; k < cells.size(); k++)
        {
            add = {x1 + cells[k].first, y1 + cells[k].second};
            if(std::find(lineCells.begin(), lineCells.end(), add) == lineCells.end())
                lineCells.push_back(add);
            add = {x2 + cells[k].first, y2 + cells[k].second};
            if(std::find(lineCells.begin(), lineCells.end(), add) == lineCells.end())
                lineCells.push_back(add);
        }

        for(auto it = lineCells.begin(); it != lineCells.end(); it++)
            if(!map.CellOnGrid(it->first, it->second))
            {
                lineCells.erase(it);
                it = lineCells.begin();
            }
        // std::cout << "Line : [" << x1 << ", " << y1 << "] -> [" << x2 << ", " << y2 << "]" << std::endl;
        // for(auto cell:lineCells)
        //     std::cout << "Line-Cell : [" << cell.first << ", " << cell.second << "]" << std::endl;
        return lineCells;
    }
    //returns all cells that are affected by agent during moving along a line

    std::vector<std::pair<int, int>> getCellsCrossedByLine_Ori(int x1, int y1, int x2, int y2)
    {
         std::set<std::pair<int, int>> cellSet;

    // 如果起点和终点是同一个点，则只返回该点的坐标。
    if (x1 == x2 && y1 == y2)
    {
        return {{x1, y1}};
    }

    // 为了简化后续循环，通过交换端点来“标准化”线的方向。
    int delta_x = std::abs(x1 - x2);
    int delta_y = std::abs(y1 - y2);
    if ((delta_x >= delta_y && x1 > x2) || (delta_y > delta_x && y1 > y2))
    {
        std::swap(x1, x2);
        std::swap(y1, y2);
    }

    int step_x = (x1 < x2 ? 1 : (x1 > x2 ? -1 : 0));
    int step_y = (y1 < y2 ? 1 : (y1 > y2 ? -1 : 0));

    double line_length = std::hypot(static_cast<double>(delta_x), static_cast<double>(delta_y));
    int gap = agentSize * line_length + static_cast<double>(delta_x + delta_y) / 2 - CN_EPSILON;

    int error = 0;
    int x = x1, y = y1;

    if (delta_x >= delta_y) // 线段更偏向水平
    {
        int extraCheck = agentSize * delta_y / line_length + 0.5 - CN_EPSILON;
        for (int n = 1; n <= extraCheck; ++n)
        {
            error += delta_y;
            int num = (gap - error) / delta_x;
            for (int k = 1; k <= num; ++k)
            {
                cellSet.insert({x1 - n * step_x, y1 + k * step_y});
                cellSet.insert({x2 + n * step_x, y2 - k * step_y});
            }
        }
        error = 0;
        for (x = x1; x != x2 + step_x; x += step_x)
        {
            cellSet.insert({x, y});
            if (x < x2 - extraCheck)
            {
                int num = (gap + error) / delta_x;
                for (int k = 1; k <= num; ++k) cellSet.insert({x, y + k * step_y});
            }
            if (x > x1 + extraCheck)
            {
                int num = (gap - error) / delta_x;
                for (int k = 1; k <= num; ++k) cellSet.insert({x, y - k * step_y});
            }
            error += delta_y;
            if ((error << 1) > delta_x)
            {
                y += step_y;
                error -= delta_x;
            }
        }
    }
    else // 线段更偏向垂直
    {
        int extraCheck = agentSize * delta_x / line_length + 0.5 - CN_EPSILON;
        for (int n = 1; n <= extraCheck; ++n)
        {
            error += delta_x;
            int num = (gap - error) / delta_y;
            for (int k = 1; k <= num; ++k)
            {
                cellSet.insert({x1 + k * step_x, y1 - n * step_y});
                cellSet.insert({x2 - k * step_x, y2 + n * step_y});
            }
        }
        error = 0;
        for (y = y1; y != y2 + step_y; y += step_y)
        {
            cellSet.insert({x, y});
            if (y < y2 - extraCheck)
            {
                int num = (gap + error) / delta_y;
                for (int k = 1; k <= num; ++k) cellSet.insert({x + k * step_x, y});
            }
            if (y > y1 + extraCheck)
            {
                int num = (gap - error) / delta_y;
                for (int k = 1; k <= num; ++k) cellSet.insert({x - k * step_x, y});
            }
            error += delta_x;
            if ((error << 1) > delta_y)
            {
                x += step_x;
                error -= delta_y;
            }
        }
    }
    // 将最终的、不重复的栅格坐标集合从 set 转换为 vector 并返回。
    std::vector<std::pair<int, int>> lineCells(cellSet.begin(), cellSet.end());
    // std::cout << "Line : [" << x1 << ", " << y1 << "] -> [" << x2 << ", " << y2 << "]" << std::endl;
    // for(auto cell:lineCells)
    //     std::cout << "L-Cell : [" << cell.first << ", " << cell.second << "]" << std::endl;
    return lineCells;
    }

    template <class T>
    bool checkTraversability(int x, int y, const T &map)
    {
        // 这里需要刷新一下cells，以当前的航向角，不过其实也不用吧，在上一步应该都检查过了
        for(int k = 0; k < cells.size(); k++)
            if(!map.CellOnGrid(x + cells[k].first, y + cells[k].second) || map.CellIsObstacle(x + cells[k].first, y + cells[k].second))
                return false;
        return true;
    }
    //checks traversability of all cells affected by agent's body

    template <class T>
    bool checkLine_Amanatides_Woo(int x1, int y1, int x2, int y2, const T &map)
    {
        std::vector<std::pair<int, int>> lineCells = getCellsCrossedByLine_Amanatides_Woo(x1, y1, x2, y2, map);
        for (auto w : lineCells)
        {
            if (map.CellOnGrid(w.first, w.second))
            {
                if (map.CellIsObstacle(w.first, w.second))
                {                  
                    return false;
                }
                    
            }
        }
        return true;
    }

    template <class T>
    bool checkShapeLine(int x1, int y1, int x2, int y2, std::vector<Location> &shape_ori, const T &map)
    {

        // double heading_for_move = acos((y2 - y1)/sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2)))*180/PI;
        // double heading_for_move =calcHeading_lineofsight(x1, y1, x2, y2);
        // if(x1 < x2)
        //     heading_for_move = 360 - heading_for_move;
        // 检查在上一步和下一步，能否以该航向角容纳
        // Shape_Collide s_temp;
        // Location ori = Location(0,0);
        // std::vector<Location> shape_relative;
        std::vector<Location> shape_temp = shape_ori;
        // shape_relative = s_temp.rotate_shape(ori, heading_for_move, shape_ori);
        std::vector<std::pair<int, int>> whole_cells = getCellsCrossedByPolyon(x1,y1,x2,y2,shape_temp,map);
        
        for (auto w : whole_cells)
        {
            if (!map.CellOnGrid(w.first, w.second) || map.CellIsObstacle(w.first, w.second))
            {
                return false;
            }
        }
        // std::cout << "From (" << x1 << ", " << y1 << ") to (" << x2 << ", " << y2 << ") Can be a Parent." << std::endl;
        return true;
    }

    template <class T>
    bool checkLine(int x1, int y1, int x2, int y2, const T &map)
    {
        //if(!checkTraversability(x1, y1) || !checkTraversability(x2, y2)) //additional check of start and goal traversability,
        //    return false;                                                //it can be removed if they are already checked

        int delta_x = std::abs(x1 - x2);
        int delta_y = std::abs(y1 - y2);
        if((delta_x > delta_y && x1 > x2) || (delta_y >= delta_x && y1 > y2))
        {
            std::swap(x1, x2);
            std::swap(y1, y2);
        }
        int step_x = (x1 < x2 ? 1 : -1);
        int step_y = (y1 < y2 ? 1 : -1);
        int error = 0, x = x1, y = y1;
        int gap = agentSize*sqrt(pow(delta_x, 2) + pow(delta_y, 2)) + double(delta_x + delta_y)/2 - CN_EPSILON;
        int k, num;

        if(delta_x > delta_y)
        {
            int extraCheck = agentSize*delta_y/sqrt(pow(delta_x, 2) + pow(delta_y, 2)) + 0.5 - CN_EPSILON;
            for(int n = 1; n <= extraCheck; n++)
            {
                error += delta_y;
                num = (gap - error)/delta_x;
                for(k = 1; k <= num; k++)
                    if(map.CellOnGrid(x1 - n*step_x, y1 + k*step_y))
                        if(map.CellIsObstacle(x1 - n*step_x, y1 + k*step_y))
                            return false;
                for(k = 1; k <= num; k++)
                    if(map.CellOnGrid(x2 + n*step_x, y2 - k*step_y))
                        if(map.CellIsObstacle(x2 + n*step_x, y2 - k*step_y))
                            return false;
            }
            error = 0;
            for(x = x1; x != x2 + step_x; x+=step_x)
            {
                if(map.CellIsObstacle(x, y))
                    return false;
                if(x < x2 - extraCheck)
                {
                    num = (gap + error)/delta_x;
                    for(k = 1; k <= num; k++)
                        if(map.CellIsObstacle(x, y + k*step_y))
                            return false;
                }
                if(x > x1 + extraCheck)
                {
                    num = (gap - error)/delta_x;
                    for(k = 1; k <= num; k++)
                        if(map.CellIsObstacle(x, y - k*step_y))
                            return false;
                }
                error += delta_y;
                if((error<<1) > delta_x)
                {
                    y += step_y;
                    error -= delta_x;
                }
            }
        }
        else
        {
            int extraCheck = agentSize*delta_x/sqrt(pow(delta_x, 2) + pow(delta_y, 2)) + 0.5 - CN_EPSILON;
            for(int n = 1; n <= extraCheck; n++)
            {
                error += delta_x;
                num = (gap - error)/delta_y;
                for(k = 1; k <= num; k++)
                    if(map.CellOnGrid(x1 + k*step_x, y1 - n*step_y))
                        if(map.CellIsObstacle(x1 + k*step_x, y1 - n*step_y))
                            return false;
                for(k = 1; k <= num; k++)
                    if(map.CellOnGrid(x2 - k*step_x, y2 + n*step_y))
                        if(map.CellIsObstacle(x2 - k*step_x, y2 + n*step_y))
                            return false;
            }
            error = 0;
            for(y = y1; y != y2 + step_y; y += step_y)
            {
                if(map.CellIsObstacle(x, y))
                    return false;
                if(y < y2 - extraCheck)
                {
                    num = (gap + error)/delta_y;
                    for(k = 1; k <= num; k++)
                        if(map.CellIsObstacle(x + k*step_x, y))
                            return false;
                }
                if(y > y1 + extraCheck)
                {
                    num = (gap - error)/delta_y;
                    for(k = 1; k <= num; k++)
                        if(map.CellIsObstacle(x - k*step_x, y))
                            return false;
                }
                error += delta_x;
                if((error<<1) > delta_y)
                {
                    x += step_x;
                    error -= delta_y;
                }
            }
        }
        return true;
    }
    //checks line-of-sight between a line
    std::vector<std::pair<int, int>> getCells(int i, int j)
    {
        std::vector<std::pair<int, int>> cells;
        for(int k=0; k<this->cells.size(); k++)
        
            cells.push_back({i+this->cells[k].first,j+this->cells[k].second});
        return cells;
    }
    // std::vector<std::pair<int, int>> getCells(int i, int j, const Map &map)
    // {
    //     std::vector<std::pair<int, int>> cells;
    //     for(int k=0; k<this->cells.size(); k++)
    //     {
    //         int x = i + this->cells[k].first;
    //         int y = j + this->cells[k].second;
    //         if (map.CellIsObstacle(x,y) || !map.CellOnGrid(x,y))
    //             continue;
    //         cells.push_back({x,y});
    //     }
    //     return cells;
    // }
private:
    double agentSize;
    std::vector<std::pair<int, int>> cells; //cells that are affected by agent's body
};

#endif // LINEOFSIGHT_H



    // template <class T>
    // std::set<std::pair<int, int>> getCellsOriginByPolyon(int x1, int y1, const std::vector<Location> &shape, const T &map)
    // {
        // OLD METHOD 👇
    //     // std::cout << "In getCellsOriginByPolyon Current Shape is :" << std::endl;
    //     // for (const auto &loc : shape) {
    //     //     std::cout << "(" << loc.x << ", " << loc.y << ")"<< std::endl;
    //     // }
    //     // 要正确处理凹多边形，扫描线算法需要引入奇偶规则（Even-Odd Rule）
    //     // 对于扫描线上的任意一点，从该点向一个固定方向（例如，向右）引一条射线。
    //     // 如果这条射线与多边形的边相交的次数是奇数，则该点在多边形内部；如果是偶数，则在多边形外部。
    //     std::set<std::pair<int, int>> polygonCells;
    //      // 1. 获取多边形边界上的所有栅格单元
    //     int numVertices = shape.size();
    //     if (numVertices < 3) {
    //         // 多边形至少需要3个顶点
    //         // std::cout << "This Shape is not a Polygon." << std::endl;
    //         return polygonCells;
    //     }

    //     int minY = shape[0].y + y1;  // 绝对坐标
    //     int maxY = shape[0].y + y1;

    //     for (int i = 0; i < numVertices; ++i) {
    //         const Location& p1 = shape[i] + Location(x1, y1); // 绝对坐标
    //         const Location& p2 = shape[(i + 1) % numVertices] + Location(x1, y1); // 连接最后一个点到第一个点形成闭合多边形

    //         // 更新 minY 和 maxY
    //         minY = std::min(std::min(minY, static_cast<int>(p1.y)), static_cast<int>(p2.y));
    //         maxY = std::max(std::max(maxY, static_cast<int>(p1.y)), static_cast<int>(p2.y));

    //         // 获取线段经过的栅格单元并添加到集合中
    //         std::vector<std::pair<int, int>> lineCells = getCellsCrossedByLine(p1.x, p1.y, p2.x, p2.y, map);
    //         for (const auto& cell : lineCells) {
    //             polygonCells.insert( std::pair<int, int>(cell.first, cell.second) );
    //         }
            
    //     }
    //     // 输出lineCells
    //     // std::cout << "All the cells crossed by the edge of Poly are :" <<  std::endl;
    //     // for (const auto& c : polygonCells) {
    //     //     std::cout << "(" << c.first << ", " << c.second << ") " <<  std::endl;
    //     // }
    //     // --- 2. 扫描填充多边形内部 (核心修改部分) ---
    // for (int y = minY; y <= maxY; ++y) {
    //     std::vector<int> intersections; // 存储当前扫描线与多边形边的交点的 x 坐标

    //     for (int i = 0; i < numVertices; ++i) {
    //         const Location& p1_rel = shape[i];
    //         const Location& p2_rel = shape[(i + 1) % numVertices];

    //         // 计算绝对坐标
    //         Location p1_abs = p1_rel + Location(x1, y1);
    //         Location p2_abs = p2_rel + Location(x1, y1);

    //         // 检查线段是否与当前扫描线相交
    //         // 垂直方向上跨越扫描线
    //         bool p1_below = p1_abs.y < y;
    //         bool p2_below = p2_abs.y < y;
    //         bool p1_above = p1_abs.y > y;
    //         bool p2_above = p2_abs.y > y;
    //         bool p1_on = p1_abs.y == y;
    //         bool p2_on = p2_abs.y == y;


    //         if ((p1_below && p2_above) || (p2_below && p1_above)) { // 线段穿过扫描线 (严格跨越)
    //             // 排除水平线段，以及顶点恰好在扫描线上的情况 (避免重复计算或误判)
    //             // 确保线段的y坐标范围包含y，并且不是完全水平
    //             if (p1_abs.y != p2_abs.y) {
    //                 double intersectX = p1_abs.x + (double)(y - p1_abs.y) * (p2_abs.x - p1_abs.x) / (p2_abs.y - p1_abs.y);
    //                 intersections.push_back(static_cast<int>(std::round(intersectX)));
    //             }
    //         } else if ((p1_on && p2_above) || (p2_on && p1_above)) { // 顶点在扫描线上，且另一个顶点在上方
    //             // 只添加下方顶点在扫描线上的情况，避免重复添加相同的交点
    //             // 考虑从下向上扫描，或者只添加左侧的交点
    //             if (p1_abs.y == y && p2_abs.y > y) {
    //                  intersections.push_back(p1_abs.x);
    //             } else if (p2_abs.y == y && p1_abs.y > y) {
    //                  intersections.push_back(p2_abs.x);
    //             }
    //         }
    //         // 注意：对于水平线段（p1.y == p2.y == y），我们通常不将它们作为交点
    //         // 因为它们不改变奇偶性。它们的内部填充由之前的 getCellsCrossedByLine 覆盖。
    //         // 除非它们是多边形最上方或最下方的水平边，且需要精确捕捉
    //         // 但是为了奇偶规则的简单性，通常只考虑非水平且跨越扫描线的边
    //     }

    //     std::sort(intersections.begin(), intersections.end());

    //     // 使用奇偶规则填充
    //     for (size_t i = 0; i < intersections.size(); ++i) {
    //         if (i + 1 < intersections.size()) {
    //             // 跳过重复的交点，这通常发生在顶点恰好在扫描线上时
    //             if (intersections[i] == intersections[i+1]) {
    //                 i++; // 跳过下一个重复的
    //                 continue;
    //             }
    //         }

    //         // 奇数个交点表示进入多边形，偶数个表示离开多边形
    //         // 所以我们总是从一个交点开始填充，到下一个交点结束
    //         // 这仍然是两两配对，但现在交点的选择会更严格
    //         if (i % 2 == 0) { // 如果是偶数索引的交点 (第一个，第三个，等等)
    //             int startX = intersections[i];
    //             int endX = (i + 1 < intersections.size()) ? intersections[i+1] : startX; // 防止越界

    //             if (startX > endX) std::swap(startX, endX);

    //             for (int x = startX; x <= endX; ++x) {
    //                 if (map.CellOnGrid(x, y) && map.CellIsTraversable(x, y)) {
    //                     polygonCells.insert({x, y});
    //                 }
    //             }
    //         }
    //     }
    // }
    //     // // 输出polygonCells
    //     // std::cout << "Agent Shape is: " << std::endl;
    //     // // 输出shape
    //     // for (const auto& point : shape) {
    //     //     std::cout << "Point: (" << point.x << ", " << point.y << ")" << std::endl;
    //     // }
    //     // std::cout << "Polygon Cells is: " << std::endl;
    //     // for (const auto& cell : polygonCells) {
    //     //     std::cout << "Cell: (" << cell.first << ", " << cell.second << ")" << std::endl;
    //     // }
    //     return polygonCells;
    // }



    //     template <class T>
    // std::vector<std::pair<int, int>> getCellsCrossedByPolyon(int x1, int y1, int x2, int y2, const std::vector<Location> &shape_ori, const T &map)
    // {
    //     // FORTH METHOD : 混合方法
    //     // --- 智能调度中心 ---
    //     // 1. 先验知识检测：检查原始形状是否为凸
    //     if (isConvex(shape_ori)) {
    //         // 2. 如果是凸多边形，使用速度最快的“凸包法”
    //         // （此处直接粘贴凸包法的完整实现）
    //         // 如果是原地旋转或停留，直接栅格化当前位置的多边形
    //         if (x1 == x2 && y1 == y2) {
    //             // 注意：原地旋转也需要考虑朝向，但此处简化为栅格化原始形状
    //             // 一个完整的实现可能需要传入旋转后的形状
    //             // return getCellsOriginByPolyon(x1, y1, shape_ori, map);
    //             return getCellsOriginByPolyon_Optimized(x1, y1, shape_ori, map);
    //         }

    //         // --- 正确且高效的凸包法 ---

    //         // 1. 获取起点和终点处，旋转后形状的绝对顶点坐标
    //         double heading = calcHeading_lineofsight(x1, y1, x2, y2); // 假设此函数已存在
    //         Shape_Collide sc_temp; // 假设此类已存在
    //         Location ori = {0, 0};
    //         std::vector<Location> shape_temp = shape_ori;
    //         std::vector<Location> shape_heading = sc_temp.rotate_shape(ori, heading, shape_temp);

    //         std::vector<Location> all_vertices;
    //         all_vertices.reserve(shape_ori.size() * 2);

    //         // 添加起点处多边形的顶点
    //         for (const auto& p_rel : shape_heading) {
    //             all_vertices.push_back(p_rel + Location(x1, y1));
    //         }
    //         // 添加终点处多边形的顶点
    //         for (const auto& p_rel : shape_heading) {
    //             all_vertices.push_back(p_rel + Location(x2, y2));
    //         }

    //         // 2. 计算这些顶点的凸包，形成一个包裹整个移动轨迹的大多边形
    //         std::vector<Location> swept_hull_abs = calculateConvexHull(all_vertices);

    //         // 将凸包的绝对坐标转换为相对于其自身包围盒左上角的相对坐标，以便复用getCellsOriginByPolyon
    //         if (swept_hull_abs.empty()) {
    //             return {};
    //         }
    //         int hull_min_x = swept_hull_abs[0].x;
    //         int hull_min_y = swept_hull_abs[0].y;
    //         for(const auto& p : swept_hull_abs) {
    //             hull_min_x = std::min(hull_min_x, static_cast<int>(p.x) );
    //             hull_min_y = std::min(hull_min_y, static_cast<int>(p.y) );
    //         }
    //         std::vector<Location> swept_hull_rel;
    //         for(const auto& p_abs : swept_hull_abs) {
    //             swept_hull_rel.push_back({p_abs.x - hull_min_x, p_abs.y - hull_min_y});
    //         }

    //         // 3. 调用我们已优化好的栅格化函数，对这个新的“扫掠多边形”进行一次光栅化
    //         return getCellsOriginByPolyon_Optimized(hull_min_x, hull_min_y, swept_hull_rel, map);

    //     } else {
    //         // 3. 如果是非凸多边形，使用能保证结果精确性的“RLE法”
    //          // THIRD METHOD ： 行程编码（RLE）+ 批量盖章
    //         if (x1 == x2 && y1 == y2) {
    //             return getCellsOriginByPolyon_Optimized(x1, y1, shape_ori, map);
    //         }

    //         // --- 准备工作 ---
    //         double heading = calcHeading_lineofsight(x1, y1, x2, y2);
    //         Shape_Collide sc_temp;
    //         Location ori = {0, 0};
    //         std::vector<Location> shape_temp = shape_ori;
    //         std::vector<Location> shape_heading = sc_temp.rotate_shape(ori, heading, shape_temp);

    //         // 1. 获取初始形状的栅格，并进行行程编码 (RLE)
    //         std::vector<std::pair<int, int>> cells1_set = getCellsOriginByPolyon_Optimized(x1, y1, shape_heading, map);
    //         if (cells1_set.empty()) {
    //             return {};
    //         }

    //         std::map<int, std::vector<std::pair<int, int>>> shape_rle;
    //         int shape_min_x = cells1_set.begin()->first, shape_max_x = cells1_set.begin()->first;
    //         int shape_min_y = cells1_set.begin()->second, shape_max_y = cells1_set.begin()->second;

    //         for (const auto& cell : cells1_set) {
    //             shape_min_x = std::min(shape_min_x, cell.first);
    //             shape_max_x = std::max(shape_max_x, cell.first);
    //             shape_min_y = std::min(shape_min_y, cell.second);
    //             shape_max_y = std::max(shape_max_y, cell.second);
    //         }

    //         for (int y = shape_min_y; y <= shape_max_y; ++y) {
    //             bool in_run = false;
    //             int run_start_x = 0;
    //             for (int x = shape_min_x; x <= shape_max_x + 1; ++x) { // +1 to terminate last run
    //                 // if (cells1_set.count({x, y})) {
    //                 if (std::binary_search(cells1_set.begin(), cells1_set.end(), std::make_pair(x, y)))
    //                 {
    //                     if (!in_run) {
    //                         run_start_x = x;
    //                         in_run = true;
    //                     }
    //                 } else {
    //                     if (in_run) {
    //                         shape_rle[y].push_back({run_start_x, x - 1});
    //                         in_run = false;
    //                     }
    //                 }
    //             }
    //         }

    //         // 2. 获取中心点移动路径
    //         std::vector<std::pair<int, int>> cells2 = getCellsCrossedByLine_Amanatides_Woo(x1, y1, x2, y2, map);
    //         if (cells2.empty()) {
    //             return cells1_set;
    //         }

    //         // 3. 计算最终区域的包围盒，并创建布尔网格
    //         int path_min_dx = 0, path_max_dx = 0, path_min_dy = 0, path_max_dy = 0;
    //         for (const auto& cell : cells2) {
    //             int dx = cell.first - x1;
    //             int dy = cell.second - y1;
    //             path_min_dx = std::min(path_min_dx, dx);
    //             path_max_dx = std::max(path_max_dx, dx);
    //             path_min_dy = std::min(path_min_dy, dy);
    //             path_max_dy = std::max(path_max_dy, dy);
    //         }
            
    //         int final_min_x = shape_min_x + path_min_dx;
    //         int final_max_x = shape_max_x + path_max_dx;
    //         int final_min_y = shape_min_y + path_min_dy;
    //         int final_max_y = shape_max_y + path_max_dy;

    //         int grid_width = final_max_x - final_min_x + 1;
    //         int grid_height = final_max_y - final_min_y + 1;
    //         std::vector<std::vector<bool>> result_grid(grid_height, std::vector<bool>(grid_width, false));

    //         // 4. 遍历路径，将RLE编码的形状批量“盖章”到布尔网格上
    //         for (const auto& path_cell : cells2) {
    //             int dx = path_cell.first - x1;
    //             int dy = path_cell.second - y1;

    //             // for (auto const& [y_orig, runs] : shape_rle) {
    //             for (const auto& pair : shape_rle) {
    //                 int y_orig = pair.first;
    //                 const auto& runs = pair.second;
    //                 for (const auto& run : runs) {
    //                     int target_y = y_orig + dy - final_min_y;
    //                     int target_start_x = run.first + dx - final_min_x;
    //                     int target_end_x = run.second + dx - final_min_x;

    //                     if (target_y >= 0 && target_y < grid_height) {
    //                         for (int x = target_start_x; x <= target_end_x; ++x) {
    //                             if (x >= 0 && x < grid_width) {
    //                                 result_grid[target_y][x] = true;
    //                             }
    //                         }
    //                     }
    //                 }
    //             }
    //         }

    //         // 5. 从布尔网格收集结果
    //         std::vector<std::pair<int, int>> whole_move_cells;
    //         for (int y_idx = 0; y_idx < grid_height; ++y_idx) {
    //             for (int x_idx = 0; x_idx < grid_width; ++x_idx) {
    //                 if (result_grid[y_idx][x_idx]) {
    //                     // whole_move_cells.insert({x_idx + final_min_x, y_idx + final_min_y});
    //                     whole_move_cells.push_back({x_idx + final_min_x, y_idx + final_min_y});
    //                 }
    //             }
    //         }
    //         // whole_move_cells去重
    //         std::sort(whole_move_cells.begin(), whole_move_cells.end());
    //         whole_move_cells.erase(std::unique(whole_move_cells.begin(), whole_move_cells.end()), whole_move_cells.end());
    //         return whole_move_cells;
    //     }
    // }