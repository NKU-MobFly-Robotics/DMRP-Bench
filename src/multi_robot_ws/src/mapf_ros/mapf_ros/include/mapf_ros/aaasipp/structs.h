#ifndef STRUCTS_H
#define STRUCTS_H
#include "gl_const.h"
#include <utility>
#include <vector>
#include <string>
#include "shape_collide.hpp"
#include <tuple>


struct conflict
{
    int agent1;
    int agent2;
    int sec1;
    int sec2;
    double i;
    double j;
    double g;
};


struct Agent
{
    std::string id;
    int start_i;
    int start_j;
    double start_heading;
    int goal_i;
    int goal_j;
    double goal_heading;
    double size;
    double rspeed;
    double mspeed;
    Agent(){ start_i = -1; start_j = -1; goal_i = -1; goal_j = -1;
             size = CN_DEFAULT_SIZE; mspeed = CN_DEFAULT_MSPEED; rspeed = CN_DEFAULT_RSPEED;
             start_heading = CN_DEFAULT_SHEADING; goal_heading = CN_DEFAULT_GHEADING; }
};

struct constraint
{
    double i;
    double j;
    double g;
    bool goal;
};

struct movement
{
    double g;
    int p_dir;
    int s_dir;
};

struct SafeInterval
{
    double begin;
    double end;
    int id;
    SafeInterval(double begin_=0, double end_=CN_INFINITY, int id_=0):begin(begin_), end(end_), id(id_) {}
};

struct Node
{
    Node(int _i=-1, int _j=-1, double _g=-1, double _F=-1):i(_i),j(_j),g(_g),F(_F),Parent(nullptr){}
    ~Node(){ Parent = nullptr; }
    int     i, j;
    double  size;
    double  g;
    double  F;
    double  heading;
    Node*   Parent;
    SafeInterval interval;

    // bool operator>(const Node& other) const
    // {
    //     if (std::abs(F - other.F) > CN_EPSILON)
    //     {
    //         return F > other.F;
    //     }
    //     // F值相同时进行Tie-breaking(打破僵局)。
    //     // 根据您原始 findMin 的逻辑，F值相同时，g值大的节点有更高优先级。
    //     // 所以如果 this->g < other.g，则 this 的优先级更低，返回 true。
    //     return g < other.g;
    // }
};

struct obstacle
{
    std::string id;
    double size;
    double mspeed;
    std::vector<Node> sections;
    obstacle(){ id = -1; size = CN_DEFAULT_SIZE; mspeed = CN_DEFAULT_MSPEED; }
};

struct section
{
    section(int _i1=-1, int _j1=-1, int _i2=-1, int _j2=-1, double _g1=-1, double _g2=-1)
        :i1(_i1), j1(_j1), i2(_i2), j2(_j2), g1(_g1), g2(_g2){}
    section(const Node &a, const Node &b):i1(a.i), j1(a.j), i2(b.i), j2(b.j), g1(a.g), g2(b.g){}
    int i1;
    int j1;
    int i2;
    int j2;
    double size;
    double g1;
    double g2;//is needed for goal and wait actions
    double mspeed;
    std::vector<Location> shape_ori; 
    double start_heading; // 该段起点的朝向
    double end_heading;   // 该段终点的朝向
    bool operator == (const section &comp) const {return (i1 == comp.i1 && j1 == comp.j1 && g1 == comp.g1);}

};
    inline bool operator<(const section& a, const section& b) {
    // 按顺序比较关键成员，直到找到一个不相等的为止
    // 这创建了一个唯一的、稳定的排序
    return std::tie(a.g1, a.g2, a.i1, a.j1, a.i2, a.j2) <
           std::tie(b.g1, b.g2, b.i1, b.j1, b.i2, b.j2);
}

class Vector2D {
  public:
    Vector2D(double _i = 0.0, double _j = 0.0):i(_i),j(_j){}
    double i, j;

    inline Vector2D operator +(const Vector2D &vec) { return Vector2D(i + vec.i, j + vec.j); }
    inline Vector2D operator -(const Vector2D &vec) { return Vector2D(i - vec.i, j - vec.j); }
    inline Vector2D operator -() { return Vector2D(-i,-j); }
    inline Vector2D operator /(const double &num) { return Vector2D(i/num, j/num); }
    inline Vector2D operator *(const double &num) { return Vector2D(i*num, j*num); }
    inline double operator *(const Vector2D &vec){ return i*vec.i + j*vec.j; }
    inline void operator +=(const Vector2D &vec) { i += vec.i; j += vec.j; }
    inline void operator -=(const Vector2D &vec) { i -= vec.i; j -= vec.j; }
};

class Point {
public:
    double i;
    double j;

    Point(double _i = 0.0, double _j = 0.0):i (_i), j (_j){}
    Point operator-(Point &p){return Point(i - p.i, j - p.j);}
    int operator== (Point &p){return (i == p.i) && (j == p.j);}
    int classify(Point &pO, Point &p1)
    {
        Point p2 = *this;
        Point a = p1 - pO;
        Point b = p2 - pO;
        double sa = a.i * b.j - b.i * a.j;
        if (sa > 0.0)
            return 1;//LEFT;
        if (sa < 0.0)
            return 2;//RIGHT;
        if ((a.i * b.i < 0.0) || (a.j * b.j < 0.0)) // p2 和 p1 在 pO 的不同侧
            return 3;//BEHIND;
        if ((a.i*a.i + a.j*a.j) < (b.i*b.i + b.j*b.j)) // p2 和 pO 在 p1 的不同侧
            return 4;//BEYOND;
        if (pO == p2)
            return 5;//ORIGIN; // 在起点
        if (p1 == p2)
            return 6;//DESTINATION; // 在终点
        return 7;//BETWEEN; // p2 在线段 pO 到 p1 之间（不包括起点和终点
    }
};
#endif
