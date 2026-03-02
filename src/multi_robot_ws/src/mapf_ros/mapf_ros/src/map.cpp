#include "mapf_ros/aaasipp/map.h"

#include <iomanip>

using namespace tinyxml2;



Map::Map()
{
    height = 0;
    width = 0;
    obs_num = 0;
}
Map::~Map()
{	
    Grid.clear();
}

bool Map::getMap(const char* FileName)
{
    XMLDocument doc;
    if(doc.LoadFile(FileName) != XMLError::XML_SUCCESS)
    {
        std::cout << "Error openning input XML file."<<std::endl;
        return false;
    }

    XMLElement *root = nullptr;
    root = doc.FirstChildElement(CNS_TAG_ROOT);
    if (!root)
    {
        std::cout << "No '"<<CNS_TAG_ROOT<<"' element found in XML map-file."<<std::endl;
        return false;
    }

    XMLElement *map = root->FirstChildElement(CNS_TAG_MAP);
    if (!map)
    {
        std::cout << "No '"<<CNS_TAG_MAP<<"' element found in XML file."<<std::endl;
        return false;
    }

    XMLElement *grid = map->FirstChildElement(CNS_TAG_GRID);
    if (!grid)
    {
        std::cout << "No '"<<CNS_TAG_GRID<<"' element found in XML file."<<std::endl;
        return false;
    }
    height = grid->IntAttribute(CNS_TAG_ATTR_HEIGHT);
    if(height <= 0)
    {
        std::cout<<"Wrong value of "<<CNS_TAG_ATTR_HEIGHT<<" attribute. It should be >0.\n";
        return false;
    }
    width = grid->IntAttribute(CNS_TAG_ATTR_WIDTH);
    if(width <= 0)
    {
        std::cout<<"Wrong value of "<<CNS_TAG_ATTR_WIDTH<<" attribute. It should be >0.\n";
        return false;
    }
    XMLElement *row = grid->FirstChildElement(CNS_TAG_ROW);
    Grid.resize(height);
    for(int i = 0; i < height; i++)
        Grid[i].resize(width, 0);

    std::string value;
    const char* rowtext;
    std::stringstream stream;
    for(int i = 0; i < height; i++)
    {
        if (!row)
        {
            std::cout << "Not enough '" << CNS_TAG_ROW << "' in '" << CNS_TAG_GRID << "' given." << std::endl;
            return false;
        }

        rowtext = row->GetText();
        unsigned int k = 0;
        value = "";
        int j = 0;

        for(k = 0; k < strlen(rowtext); k++)
        {
            if (rowtext[k] == ' ')
            {
                stream << value;
                stream >> Grid[i][j];
                stream.clear();
                stream.str("");
                value = "";
                j++;
            }
            else
            {
                value += rowtext[k];
            }
        }
        stream << value;
        stream >> Grid[i][j];
        stream.clear();
        stream.str("");

        if (j < width-1)
        {
            std::cout << "Not enough cells in '" << CNS_TAG_ROW << "' " << i << " given." << std::endl;
            return false;
        }
        row = row->NextSiblingElement(CNS_TAG_ROW);

        // 遍历Grid
        for (int m = 0; m < width; m++)
        {
            if (Grid[i][m] != 0)
                obs_num++;
        }
    }
    std::cout << "Obs num is: " << obs_num << std::endl;
    return true;
}


bool Map::CellIsTraversable(int i, int j) const
{
    return (Grid[i][j] == 0);
}

bool Map::CellIsObstacle(int i, int j) const
{
    return (Grid[i][j] != 0);
}

bool Map::CellOnGrid(int i, int j) const
{
    return (i < height && i >= 0 && j < width && j >= 0);
    // return (i < height-1 && i > 1 && j < width-1 && j > 1);
}

int Map::getValue(int i, int j) const
{
    if(i < 0 || i >= height)
        return -1;
    if(j < 0 || j >= width)
        return -1;

    return Grid[i][j];
}

std::vector<Node> Map::getShapeValidMoves(int i, int j, int k, double size, double heading, std::vector<Location> shape_ori) const
{
   LineOfSight los;
   los.setShapeSize(size, shape_ori);
   std::vector<Node> moves;
   if(k == 2)
       moves = {Node(0,1,1.0),   Node(1,0,1.0),         Node(-1,0,1.0), Node(0,-1,1.0)};
   else if(k == 3)
       moves = {Node(0,1,1.0),   Node(1,1,sqrt(2.0)),   Node(1,0,1.0),  Node(1,-1,sqrt(2.0)),
                Node(0,-1,1.0),  Node(-1,-1,sqrt(2.0)), Node(-1,0,1.0), Node(-1,1,sqrt(2.0))};
   else if(k == 4)
       moves = {Node(0,1,1.0),          Node(1,1,sqrt(2.0)),    Node(1,0,1.0),          Node(1,-1,sqrt(2.0)),
                Node(0,-1,1.0),         Node(-1,-1,sqrt(2.0)),  Node(-1,0,1.0),         Node(-1,1,sqrt(2.0)),
                Node(1,2,sqrt(5.0)),    Node(2,1,sqrt(5.0)),    Node(2,-1,sqrt(5.0)),   Node(1,-2,sqrt(5.0)),
                Node(-1,-2,sqrt(5.0)),  Node(-2,-1,sqrt(5.0)),  Node(-2,1,sqrt(5.0)),   Node(-1,2,sqrt(5.0))};
   else
       moves = {Node(0,1,1.0),          Node(1,1,sqrt(2.0)),    Node(1,0,1.0),          Node(1,-1,sqrt(2.0)),
                Node(0,-1,1.0),         Node(-1,-1,sqrt(2.0)),  Node(-1,0,1.0),         Node(-1,1,sqrt(2.0)),
                Node(1,2,sqrt(5.0)),    Node(2,1,sqrt(5.0)),    Node(2,-1,sqrt(5.0)),   Node(1,-2,sqrt(5.0)),
                Node(-1,-2,sqrt(5.0)),  Node(-2,-1,sqrt(5.0)),  Node(-2,1,sqrt(5.0)),   Node(-1,2,sqrt(5.0)),
                Node(1,3,sqrt(10.0)),   Node(2,3,sqrt(13.0)),   Node(3,2,sqrt(13.0)),   Node(3,1,sqrt(10.0)),
                Node(3,-1,sqrt(10.0)),  Node(3,-2,sqrt(13.0)),  Node(2,-3,sqrt(13.0)),  Node(1,-3,sqrt(10.0)),
                Node(-1,-3,sqrt(10.0)), Node(-2,-3,sqrt(13.0)), Node(-3,-2,sqrt(13.0)), Node(-3,-1,sqrt(10.0)),
                Node(-3,1,sqrt(10.0)),  Node(-3,2,sqrt(13.0)),  Node(-2,3,sqrt(13.0)),  Node(-1,3,sqrt(10.0))};
   std::vector<bool> valid(moves.size(), true);
   for(int k = 0; k < moves.size(); k++)
   {
        // 计算上一步到这一步的航向
        // std::cout << "This step :(" << i << "," << j << ") -> (" << i + moves[k].i << "," << j + moves[k].j << ")" << std::endl;
        int i2 = i + moves[k].i;
        int j2 = j + moves[k].j;
        double heading_for_move = calcHeading_A(i, j, i2, j2);
        // 检查在上一步和下一步，能否以该航向角容纳
        Shape_Collide shape_temp;
        std::vector<Location> shape_before = shape_ori;
        bool valid_shape = true;
        double angle_step = 10.0;

        // 检查父节点->子节点，父节点的原地旋转是否与障碍物碰撞
        if (std::abs(heading - heading_for_move) > 0.01 && obs_num > 1)
        {
            // 这里 end_heading -1.0 是为了 break-tie, 防止函数算错 0-180度 应该占据的栅格
            std::set<std::pair<int, int>> cells_during_rot =  Map_multiAngleSampleCells_DuringRot(i, j, heading, heading_for_move -1.0 , angle_step, shape_before);
            // std::cout << "cells_during_rot size: " << cells_during_rot.size() << std::endl;
            for(auto cell : cells_during_rot)
            {
                if (CellOnGrid(cell.first, cell.second)) // 注意 此时的cell.first, cell.second 可能不在地图内，不能直接交给CellIsObstacle() 判断，会超出边ERROR
                {
                    if(CellIsObstacle(cell.first, cell.second))
                    {
                        valid[k] = false;
                        break;
                    }
                }
            }
        }
        if(!valid[k])
            continue;
        // 检查父节点->子节点，父节点的原地旋转是否与障碍物碰撞


        bool shape_line_move = los.checkShapeLine(i, j, i + moves[k].i, j + moves[k].j, shape_before, *this);
        // std::cout << "checkShapeLine is valid? :" << shape_line_move << std::endl;

        // 这里是不是也要检查原地旋转是不是会导致碰撞

        if(!CellOnGrid(i + moves[k].i, j + moves[k].j) || CellIsObstacle(i + moves[k].i, j + moves[k].j)
                || !shape_line_move)
        {
            // std::cout << "B -> This move is not OK (" << i << "," << j << ") -> (" << i + moves[k].i << "," << j + moves[k].j << ")" << std::endl;
            valid[k] = false;
        }
           
   }
       
   std::vector<Node> v_moves = {};
   for(int k = 0; k < valid.size(); k++)
   {
       if(valid[k])
       {
            // std::cout << "valid[k] is : " << valid[k] << std::endl;
            v_moves.push_back(moves[k]);
       }
   }
   return v_moves;
}

std::set<std::pair<int, int>> Map::Map_multiAngleSampleCells_DuringRot(int x, int y, double start_heading, double end_heading, double angle_step, const std::vector<Location> &shape_ori) const
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
        std::vector<std::pair<int, int>> cells_at_angle = los.getCellsOriginByPolyon_Optimized(x, y, shape_rotated, *this);
        rotated_cells_unio.insert(cells_at_angle.begin(), cells_at_angle.end());
    }
    
    // 不要忘了結束角度
    std::vector<Location> shape_end = sc_temp.rotate_shape(ori, end_heading, shape_);
    std::vector<std::pair<int, int>> cells_end = los.getCellsOriginByPolyon_Optimized(x, y, shape_end, *this);
    rotated_cells_unio.insert(cells_end.begin(), cells_end.end());

    return rotated_cells_unio;
}

double Map::calcHeading_A(int i1, int j1, int i2, int j2) const
{
    double heading = atan2(i2 - i1, j2 - j1)*180/CN_PI;
    if(heading < 0)
        heading += 360;
    return heading;
}

double Map::getDis(int a_i, int a_j, int b_i, int b_j) const
{
    return sqrt((a_i - b_i) * (a_i - b_i) + (a_j - b_j) * (a_j - b_j));
}

std::vector<Node> Map::getValidMoves(int i, int j, int k, double size) const
{
   LineOfSight los;
   los.setSize(size);
   std::vector<Node> moves;
   if(k == 2)
       moves = {Node(0,1,1.0),   Node(1,0,1.0),         Node(-1,0,1.0), Node(0,-1,1.0)};
   else if(k == 3)
       moves = {Node(0,1,1.0),   Node(1,1,sqrt(2.0)),   Node(1,0,1.0),  Node(1,-1,sqrt(2.0)),
                Node(0,-1,1.0),  Node(-1,-1,sqrt(2.0)), Node(-1,0,1.0), Node(-1,1,sqrt(2.0))};
   else if(k == 4)
       moves = {Node(0,1,1.0),          Node(1,1,sqrt(2.0)),    Node(1,0,1.0),          Node(1,-1,sqrt(2.0)),
                Node(0,-1,1.0),         Node(-1,-1,sqrt(2.0)),  Node(-1,0,1.0),         Node(-1,1,sqrt(2.0)),
                Node(1,2,sqrt(5.0)),    Node(2,1,sqrt(5.0)),    Node(2,-1,sqrt(5.0)),   Node(1,-2,sqrt(5.0)),
                Node(-1,-2,sqrt(5.0)),  Node(-2,-1,sqrt(5.0)),  Node(-2,1,sqrt(5.0)),   Node(-1,2,sqrt(5.0))};
   else
       moves = {Node(0,1,1.0),          Node(1,1,sqrt(2.0)),    Node(1,0,1.0),          Node(1,-1,sqrt(2.0)),
                Node(0,-1,1.0),         Node(-1,-1,sqrt(2.0)),  Node(-1,0,1.0),         Node(-1,1,sqrt(2.0)),
                Node(1,2,sqrt(5.0)),    Node(2,1,sqrt(5.0)),    Node(2,-1,sqrt(5.0)),   Node(1,-2,sqrt(5.0)),
                Node(-1,-2,sqrt(5.0)),  Node(-2,-1,sqrt(5.0)),  Node(-2,1,sqrt(5.0)),   Node(-1,2,sqrt(5.0)),
                Node(1,3,sqrt(10.0)),   Node(2,3,sqrt(13.0)),   Node(3,2,sqrt(13.0)),   Node(3,1,sqrt(10.0)),
                Node(3,-1,sqrt(10.0)),  Node(3,-2,sqrt(13.0)),  Node(2,-3,sqrt(13.0)),  Node(1,-3,sqrt(10.0)),
                Node(-1,-3,sqrt(10.0)), Node(-2,-3,sqrt(13.0)), Node(-3,-2,sqrt(13.0)), Node(-3,-1,sqrt(10.0)),
                Node(-3,1,sqrt(10.0)),  Node(-3,2,sqrt(13.0)),  Node(-2,3,sqrt(13.0)),  Node(-1,3,sqrt(10.0))};
   std::vector<bool> valid(moves.size(), true);
   for(int k = 0; k < moves.size(); k++)
       if(!CellOnGrid(i + moves[k].i, j + moves[k].j) || CellIsObstacle(i + moves[k].i, j + moves[k].j)
               || !los.checkLine(i, j, i + moves[k].i, j + moves[k].j, *this))
           valid[k] = false;
   std::vector<Node> v_moves = {};
   for(int k = 0; k < valid.size(); k++)
       if(valid[k])
           v_moves.push_back(moves[k]);
   return v_moves;
}

void Map::printGrid(std::vector<std::vector<int>> grid)
{
    std::cout << std::fixed << std::setprecision(1);
    std::cout << "--- Girds is : ---" << std::endl;
    for (int i = 0; i < grid.size(); i++)
    {
        for (int j = 0; j < grid[0].size(); j++)
        {
            std::cout << grid[i][j] << " ";
        }
        std::cout << std::endl;
    }
}

std::vector<std::vector<Location>> Map::nearestObs()
{
    // 初始化一个与地图大小相同的二维数组
    // std::vector<std::vector<double>> static_field(height, std::vector<double>(width, INT_MAX));

    if (Grid.empty() || Grid[0].empty()) {
        return {};
    }

    const int m = Grid.size();
    const int n = Grid[0].size();

    // 辅助网格，用于存储每个点最近的障碍物的坐标
    std::vector<std::vector<Location>> nearestObstacle(m, std::vector<Location>(n));

    // 1. 初始化
    // 障碍物点指向自身，可通行点指向一个无效点
    for (int i = 0; i < m; ++i) {
        for (int j = 0; j < n; ++j) {
            if (Grid[i][j] == 1) {
                nearestObstacle[i][j] = {i, j};
            } else {
                nearestObstacle[i][j] = Location::infinity();
            }
        }
    }

    // 2. 第一遍扫描 (正向：从左上到右下)
    for (int i = 0; i < m; ++i) {
        for (int j = 0; j < n; ++j) {
            Location currentPoint = {i, j};
            double minSq = distSq(currentPoint, nearestObstacle[i][j]);

            // 检查4个已经处理过的邻居：上, 左, 左上, 右上
            Location candidates[4];
            int count = 0;
            if (i > 0) candidates[count++] = nearestObstacle[i - 1][j];
            if (j > 0) candidates[count++] = nearestObstacle[i][j - 1];
            if (i > 0 && j > 0) candidates[count++] = nearestObstacle[i - 1][j - 1];
            if (i > 0 && j < n - 1) candidates[count++] = nearestObstacle[i - 1][j + 1];

            for(int k=0; k<count; ++k) {
                double dSq = distSq(currentPoint, candidates[k]);
                if (dSq < minSq) {
                    minSq = dSq;
                    nearestObstacle[i][j] = candidates[k];
                }
            }
        }
    }

    // 3. 第二遍扫描 (反向：从右下到左上)
    for (int i = m - 1; i >= 0; --i) {
        for (int j = n - 1; j >= 0; --j) {
            Location currentPoint = {i, j};
            double minSq = distSq(currentPoint, nearestObstacle[i][j]);

            // 检查4个已经处理过的邻居：下, 右, 右下, 左下
            Location candidates[4];
            int count = 0;
            if (i < m - 1) candidates[count++] = nearestObstacle[i + 1][j];
            if (j < n - 1) candidates[count++] = nearestObstacle[i][j + 1];
            if (i < m - 1 && j < n - 1) candidates[count++] = nearestObstacle[i + 1][j + 1];
            if (i < m - 1 && j > 0) candidates[count++] = nearestObstacle[i + 1][j - 1];
            
            for(int k=0; k<count; ++k) {
                double dSq = distSq(currentPoint, candidates[k]);
                if (dSq < minSq) {
                    minSq = dSq;
                    nearestObstacle[i][j] = candidates[k];
                }
            }
        }
    }

    // 4. 最终计算
    // std::vector<std::vector<double>> result(m, std::vector<double>(n));
    // for (int i = 0; i < m; ++i) {
    //     for (int j = 0; j < n; ++j) {
    //         if (Grid[i][j] == 1) {
    //             result[i][j] = -1.0;
    //         } else {
    //             Location obs = nearestObstacle[i][j];
    //             if (!obs.isValid()) {
    //                 // 如果地图上没有任何障碍物
    //                 result[i][j] = std::numeric_limits<double>::max();
    //             } else {
    //                 result[i][j] = std::sqrt(distSq({i, j}, obs));
    //             }
    //         }
    //     }
    // }

    // return result;
    
    return nearestObstacle;
}
