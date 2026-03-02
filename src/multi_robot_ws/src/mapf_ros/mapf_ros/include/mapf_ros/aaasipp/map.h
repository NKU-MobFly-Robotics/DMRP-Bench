#ifndef MAP_H
#define MAP_H

#include <iostream>
#include <string>
#include <algorithm>
#include <sstream>
#include <vector>

#include <queue>
#include <cmath>
#include <limits>
// #include <iomanip> 

#include "structs.h"
#include "tinyxml2.h"
#include "gl_const.h"
#include "lineofsight.h"
#include "shape_collide.hpp"

// // 一个简单的结构体，用于存储点的二维坐标
// struct Point2 {
//     int r, c; // r: row (行), c: column (列)

//     // 定义一个静态方法来表示一个“无限远”或无效的点
//     static Point2 infinity() {
//         return {-1, -1};
//     }

//     // 检查点坐标是否有效
//     bool isValid() const {
//         return r != -1;
//     }
// };

// // 内联函数，用于快速计算两个点之间欧式距离的平方
// // 使用距离的平方进行比较可以避免在中间步骤进行开方运算，从而提高效率
// inline double distSq(const Point2& p1, const Point2& p2) {
//     if (!p1.isValid() || !p2.isValid()) {
//         return std::numeric_limits<double>::max();
//     }
//     double dr = static_cast<double>(p1.r) - p2.r;
//     double dc = static_cast<double>(p1.c) - p2.c;
//     return dr * dr + dc * dc;
// }

class Map
{
public:
    std::vector<std::vector<int>> Grid;
    unsigned int height, width, obs_num;

public:
    Map();
    ~Map();
    bool getMap(const char* FileName);
    bool CellIsTraversable (int i, int j) const;
    bool CellOnGrid (int i, int j) const;
    bool CellIsObstacle(int i, int j) const;
    int  getValue(int i, int j) const;
    void printGrid(std::vector<std::vector<int>> grid);
    std::vector<std::vector<Location>> nearestObs();
    std::vector<Node> getValidMoves(int i, int j, int k, double size) const;
    double getDis(int i, int j, int i_next, int j_next) const;
    double calcHeading_A(int i1, int j1, int i2, int j2) const;
    std::vector<Node> getShapeValidMoves(int i, int j, int k, double size, double heading, std::vector<Location> shape) const;
    std::set<std::pair<int, int>> Map_multiAngleSampleCells_DuringRot(int x, int y, double start_heading, double end_heading, double angle_step, 
                                                                    const std::vector<Location> &shape_ori) const;
};

#endif
