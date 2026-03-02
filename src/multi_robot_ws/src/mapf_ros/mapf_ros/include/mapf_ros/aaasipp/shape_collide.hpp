// 此函数用于检测不同的形状是否发生碰撞
// 关于形状的定义如下：
// center: 形状的中心点坐标
// shape points: 形状的顶点坐标，多个points可以连接组成一个多边形
// safe radius: 形状的安全半径，用于判断两个形状是否发生碰撞
// 形状的类型有：点，圆，线段，多边形
// 点：由一个center坐标点表示
// 圆：由一个center坐标点和一个safe radius半径表示
// 线段：由一个center坐标点和一个point坐标点表示，此时线段的一个端点是center，
// 如果想更精确的描述一条线段，可以使用两个point坐标点作为两个端点，center作为中间点表示，但会增加计算量
// 多边形：center作为多边形中心坐标（不一定非在质心），由多个point坐标点组成，多个points可以连接组成一个多边形。
// 多边形的points序列应该按逆时针或者顺时针给出；
#pragma once

#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include <limits>

#include <boost/functional/hash.hpp>
#include <boost/program_options.hpp>


using namespace std;

const double EPS = 1e-9;

struct Location {  // 用来表示坐标，也可以用来表示向量
  Location(double x, double y) : x(x), y(y) {}
  double x;
  double y;

  Location() : x(0), y(0) {}

  bool operator<(const Location& other) const {
    return std::tie(x, y) < std::tie(other.x, other.y);
  }

  bool operator==(const Location& other) const {
    return std::tie(x, y) == std::tie(other.x, other.y);
  }

  static Location infinity(){
    return Location(-1,-1);
  }

  bool isValid() const {
    return x != -1;
  }

  friend std::ostream& operator<<(std::ostream& os, const Location& c) {
    return os << "(" << c.x << "," << c.y << ")";
  }
  Location operator+(const Location& other) const {
    return Location(x + other.x, y + other.y);
  }
  Location operator-(const Location& other) const {
    return Location(x - other.x, y - other.y);
  }
  Location operator*(double scalar) const {
    return Location(x * scalar, y * scalar);
  }

};

// 内联函数，用于快速计算两个点之间欧式距离的平方
// 使用距离的平方进行比较可以避免在中间步骤进行开方运算，从而提高效率
inline double distSq(const Location& p1, const Location& p2) {
    if (!p1.isValid() || !p2.isValid()) {
        return std::numeric_limits<double>::max();
    }
    double dr = static_cast<double>(p1.x) - p2.x;
    double dc = static_cast<double>(p1.y) - p2.y;
    return dr * dr + dc * dc;
}

namespace std {
template <>
struct hash<Location> {
  size_t operator()(const Location& s) const {
    size_t seed = 0;
    boost::hash_combine(seed, s.x);
    boost::hash_combine(seed, s.y);
    return seed;
  }
};
}  // namespace std



class Shape_Collide {
public:
    Shape_Collide() {}
    virtual ~Shape_Collide() {}
    // virtual void run() = 0;

    void getShapeInfo(Location center, std::vector<Location> points, double radius) {
        m_center = center;
        m_points = points;
        m_radius = radius;
        // std::cout << "Shape info get. " << std::endl;
    }

    bool collisionDetection(Shape_Collide& other) {
        // std::cout << "Collision detection begin. " << std::endl;
        int Apoints_num = m_points.size();
        int Bpoints_num = other.m_points.size();
        bool collide = false;

        // 两个形状均为点
        if (Apoints_num == 0 && Bpoints_num == 0) {
          collide = PointsCheck(m_center, other.m_center, m_radius, other.m_radius);
        }

        // 一个为直线，一个为点
        if ( (Apoints_num == 1 && Bpoints_num == 0) || (Apoints_num == 0 && Bpoints_num == 1) ) {
          if (Apoints_num == 1) {
            collide = PointLineCheck(m_center, m_points[0], other.m_center, m_radius, other.m_radius);
          } else {
            collide = PointLineCheck(other.m_center, other.m_points[0], m_center, other.m_radius, m_radius);
          }
        }
        
        // 两个形状均为线段
        if (Apoints_num == 1 && Bpoints_num == 1) {
          collide = LinesCheck(m_center, m_points[0], other.m_center, other.m_points[0], m_radius, other.m_radius);
        }

        // 点和多边形
        if ( (Apoints_num == 0 && Bpoints_num > 1) || (Apoints_num > 1 && Bpoints_num == 0) ) {
          if (Apoints_num == 0) {
            collide = PointPolygonCheck(m_center, other.m_points, m_radius, other.m_radius);
          } else {
            collide = PointPolygonCheck(other.m_center, m_points, other.m_radius, m_radius);
          }
        }

        // 线段和多边形
        if ( (Apoints_num == 1 && Bpoints_num > 1) || (Apoints_num > 1 && Bpoints_num == 1) ) {
          if (Apoints_num == 1) {
            collide = LinePolygonCheck(m_center, m_points[0], other.m_points, m_radius, other.m_radius);
          } else {
            collide = LinePolygonCheck(other.m_center, other.m_points[0], m_points, other.m_radius, m_radius);
          }
        }


        // 多边形之间
        // if (Apoints_num > 1 && Bpoints_num > 1) {
        //     collide = PolygonsCheck_Simple(m_points, other.m_points, m_radius, other.m_radius);
        // }

        if (Apoints_num > 1 && Bpoints_num > 1) {
            collide = PolygonsCheck_SAT(m_points, other.m_points, m_radius, other.m_radius);
        }

        

        return collide;
    }

    

    bool PointsCheck(Location& Acenter, Location& Bcenter, double Aradius, double Bradius) {
        // std::cout << "Points check begin. " << std::endl;
        double dis = distance(Acenter, Bcenter);
        if (dis <= Aradius + Bradius) {
            return true;
        } else {
            return false;
        }
    }

    bool PointLineCheck(Location& Acenter, Location& Apoint, Location& Bcenter, double Aradius, double Bradius) {
        // std::cout << "Point line check begin. " << std::endl;
        double dis = PLdistance(Bcenter, Acenter, Apoint);
        if (dis <= Aradius + Bradius) {
            return true;
        } else {
            return false;
        } 
    }

    bool LinesCheck(Location& Acenter, Location& Apoint, Location& Bcenter, Location& Bpoint, double Aradius, double Bradius) {
        // std::cout << "Lines check begin. " << std::endl;

        double dis = LLdistance(Acenter, Apoint, Bcenter, Bpoint);
        // std::cout << "LLdistance: " << dis << std::endl;
        if (dis <= Aradius + Bradius) {
            return true;
        } else {
            return false;
        }    
    }

    bool PointPolygonCheck(Location& Acenter, std::vector<Location>& Bpoints, double Aradius, double Bradius) {
        // std::cout << "Point polygon check begin. " << std::endl;
        double dis = PointPolygonDistance(Acenter, Bpoints);
        // std::cout << "PointPolydistance: " << dis << std::endl;
        if (dis <= Aradius + Bradius) {
            return true;
        } else {
            return false;
        }
    }

    bool LinePolygonCheck(Location& Acenter, Location& Apoint, std::vector<Location>& Bpoints, double Aradius, double Bradius) {
        // std::cout << "Line polygon check begin. " << std::endl;
        double dis = LinePolygonDistance(Acenter, Apoint, Bpoints);
        if (dis <= Aradius + Bradius) {
            return true;
        } else {
            return false;
        }
    }

    bool PolygonsCheck_Simple(std::vector<Location>& Apoints, std::vector<Location>& Bpoints, double Aradius, double Bradius) {
        // std::cout << "Polygons check begin. " << std::endl;
        // 此方法为简单的，逐个遍历边，得到最近距离，但该方法计算量较大；
        // 更高级的SAT、GJK方法待实现；
        double dis = PolygonsDistance_Simple(Apoints, Bpoints);
        if (dis <= Aradius + Bradius) {
            return true;
        } else {
            return false;
        }
    }

    bool PolygonsCheck_SAT(std::vector<Location>& Apoints, std::vector<Location>& Bpoints, double Aradius, double Bradius) {
        // std::cout << "Polygons check begin. " << std::endl;
        // SAT方法没法直接获得两个多边形之间的最小距离；
        // 可以通过计算它们在所有分离轴上的距离，取最小距离作为最近距离。
        double dis = PolygonsDistance_SAT(Apoints, Bpoints);
        if (dis <= Aradius + Bradius) {
            return true;
        } else {
            return false;
        }
    }

    double distance(Location A, Location B) {
    return std::sqrt((B.x - A.x) * (B.x - A.x) + (B.y - A.y) * (B.y - A.y));
    }

    double dotProduct(Location A, Location B) {
        return A.x * B.x + A.y * B.y;
    }

    // 向量长度的平方
    double norm_sq(const Location& v) {
        return dotProduct(v, v);
    }
    
    
    double crossProduct(Location A, Location B) {
        return A.x * B.y - A.y * B.x;
    }

    // 向量的模长
    double magnitude(const Location& v) {
        return std::sqrt(v.x * v.x + v.y * v.y);
    }

    // 将向量归一化
    Location normalize(const Location& v) {
        double len = magnitude(v);
        return Location(v.x / len, v.y / len);
    }

    bool on_segment(const Location& p, const Location& a, const Location& b) {
        double dist_ap = distance(a, p);
        double dist_pb = distance(p, b);
        double dist_ab = distance(a, b);
        return std::abs(dist_ap + dist_pb - dist_ab) < EPS;
    }

    // 计算两条线段 (a, b) 和 (c, d) 的交点
    Location segment_intersection(const Location& a, const Location& b, const Location& c, const Location& d) {
        Location ab = b - a;
        Location cd = d - c;
        double det = crossProduct(ab, cd);

        if (std::abs(det) < EPS) {
            // 平行或共线，此简化版本不处理共线重叠情况
            return Location::infinity();
        }

        double t = crossProduct(c - a, cd) / det;
        double u = crossProduct(c - a, ab) / det;

        if (t >= -EPS && t <= 1.0 + EPS && u >= -EPS && u <= 1.0 + EPS) {
            return a + ab * t;
        }

        return Location::infinity();
    }

    // 判断顶点在多边形中是否为凸顶点
    bool is_convex_vertex(const Location& prev, const Location& current, const Location& next) {
        Location v1 = current - prev;
        Location v2 = next - current;
        // 假设多边形顶点是逆时针顺序，正叉积表示左转，是凸顶点
        return crossProduct(v1, v2) > 0;
    }

    // 计算直线与圆的交点
    std::vector<Location> line_circle_intersection(const Location& p1, const Location& p2, const Location& circle_center, double radius) {
        std::vector<Location> points;
        Location d = p2 - p1;
        Location f = p1 - circle_center;

        double a = dotProduct(d, d);
        double b = 2 * dotProduct(f, d);
        double c = dotProduct(f, f) - radius * radius;

        double discriminant = b * b - 4 * a * c;

        if (discriminant < -EPS) {
            return points;
        } 
        
        discriminant = (discriminant < 0) ? 0 : std::sqrt(discriminant);
        double t1 = (-b - discriminant) / (2 * a);
        double t2 = (-b + discriminant) / (2 * a);

        if (t1 >= -EPS && t1 <= 1.0 + EPS) {
            points.push_back(p1 + d * t1);
        }
        if (t2 >= -EPS && t2 <= 1.0 + EPS) {
            if (points.empty() || distance(points.back(), p1 + d * t2) > EPS) {
                points.push_back(p1 + d * t2);
            }
        }
        return points;
    }

    double LLdistance(Location& Acenter, Location& Apoint, Location& Bcenter, Location& Bpoint) {
        double d1 = crossProduct(Apoint - Acenter, Bcenter - Acenter);
        double d2 = crossProduct(Apoint - Acenter, Bpoint - Acenter);
        double d3 = crossProduct(Bpoint - Bcenter, Acenter - Bcenter);
        double d4 = crossProduct(Bpoint - Bcenter, Apoint - Bcenter);

        if (((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) && ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0))) {
            Location dir1 = Apoint - Acenter;
            Location dir2 = Bpoint - Bcenter;
            double len1 = sqrt(dir1.x * dir1.x + dir1.y * dir1.y);
            double len2 = sqrt(dir2.x * dir2.x + dir2.y * dir2.y);
            dir1.x /= len1;
            dir1.y /= len1;
            dir2.x /= len2;
            dir2.y /= len2;

            double cosTheta = dotProduct(dir1, dir2);
            if (fabs(cosTheta) > 1.0) {
                cosTheta = (cosTheta > 0) ? 1.0 : -1.0;
            }
            double theta = acos(cosTheta);

            if (theta > std::numeric_limits<double>::epsilon()) {
                double sinTheta = sqrt(1.0 - cosTheta * cosTheta);
                double len1Perp = len1 / sinTheta;
                double len2Perp = len2 / sinTheta;

                double minLen = std::min(len1Perp, len2Perp);
                double maxLen = std::max(len1Perp, len2Perp);

                if (minLen < std::numeric_limits<double>::epsilon()) {
                    return 0.0;
                }
                else {
                    double t = dotProduct(Bcenter - Acenter, dir2);
                    if (t < 0) {
                        t = 0;
                    }
                    else if (t > len2) {
                        t = len2;
                    }
                    Location closestPoint = Acenter + Location(dir1.x * t, dir1.y * t);
                    return PLdistance(closestPoint, Bcenter, Bpoint);
                }
            }
            else {
                return std::abs(d1) / len1;
            }
        }
        else {
            double d = std::min(PLdistance(Bcenter, Acenter, Apoint), PLdistance(Bpoint, Acenter, Apoint));
            d = std::min(d, PLdistance(Acenter, Bcenter, Bpoint));
            d = std::min(d, PLdistance(Apoint, Bcenter, Bpoint));
            return d;
        }
    
    }

    double PLdistance(Location& Bcenter, Location& Acenter, Location& Apoint) { 
        double length = distance(Acenter, Apoint);
        if (length < std::numeric_limits<double>::epsilon()) {
            return distance(Bcenter, Acenter);
        }
        double t = std::max(0.0, std::min(1.0, dotProduct(Bcenter - Acenter, Apoint - Acenter) / (length * length)));
        Location projection = Acenter + (Apoint - Acenter) * t;
        return distance(Bcenter, projection);
    }

    double PointPolygonDistance(Location& Acenter, std::vector<Location>& Bpoints) {

        // 先判断点是否在多边形内
        bool inPoly = false;
        inPoly = isPointInPolygon(Acenter, Bpoints);

        if (inPoly) {
            return -1.0;    // 点在多边形内，不再计算精确距离；
        }

        // 如果不在，则计算到最近边的距离
        int n = Bpoints.size();
        double min_dis = std::numeric_limits<double>::max();
        for (int i = 0, j = n -1; i < n; j = i++) {
            Location current = Bpoints[i];
            Location next = Bpoints[j];
            min_dis = std::min(min_dis, PLdistance(Acenter, current, next));
        }
        return min_dis;        
    }

    double LinePolygonDistance(Location& Acenter, Location& Apoint, std::vector<Location>& Bpoints) {
        // 先判断点是否在多边形内
        // 这里面Apoint和Bpoint使用的都是绝对坐标
        bool inPoly = false;
        inPoly = isPointInPolygon(Acenter, Bpoints) || isPointInPolygon(Apoint, Bpoints);
        if (inPoly) {
            return -1.0;    // 线段端点在多边形内，不再计算精确距离；
        } else {
            // 线段端点不在多边形内，则计算到最近边的距离
            int n = Bpoints.size();
            double min_dis = std::numeric_limits<double>::max();
            for (int i = 0, j = n -1; i < n; j = i++) {
                Location current = Bpoints[i];
                Location next = Bpoints[j];
                min_dis = std::min(min_dis, LLdistance(Acenter, Apoint, current, next));
            }
            return min_dis;            
        }
    }

    double PolygonsDistance_Simple(std::vector<Location>& Apoints, std::vector<Location>& Bpoints) {
        // 先判断点是否在多边形内
        bool inPoly = false;
        for (int i = 0; i < Apoints.size(); i++) {
            inPoly = isPointInPolygon(Apoints[i], Bpoints);
            if (inPoly) {
                return -1.0;    // 点在多边形内，不再计算精确距离；
            }
        }
        for (int i = 0; i < Bpoints.size(); i++) {
            inPoly = isPointInPolygon(Bpoints[i], Apoints);
            if (inPoly) {
                return -1.0;    // 点在多边形内，不再计算精确距离；
            }
        }
            int m = Apoints.size();
            // int n = Bpoints.size();
            double min_dis = std::numeric_limits<double>::max();
            for (int i = 0, j = m -1; i < m; j = i++) {
                Location A_curr = Apoints[i];
                Location A_next = Apoints[j];
                min_dis = std::min(min_dis, LinePolygonDistance(A_curr, A_next, Bpoints));
            }
            return min_dis;
    }

    double PolygonsDistance_SAT(std::vector<Location>& Apoints, std::vector<Location>& Bpoints) {
        // 检查第一个多边形的轴
        for (size_t i = 0; i < Apoints.size(); ++i) {
            const Location& p1 = Apoints[i];
            const Location& p2 = Apoints[(i + 1) % Apoints.size()];
            Location edge = p2 - p1;
            Location axis(-edge.y, edge.x); // 直接计算轴，无需存储

            auto proj1 = project(Apoints, axis);   // const引用
            auto proj2 = project(Bpoints, axis);

            if (proj1.second < proj2.first || proj2.second < proj1.first) {
                // 找到了分离轴，表示不碰撞。
                // 此时可以返回一个大于0的数表示不碰撞。
                // 注意：SAT 本身不直接计算最小距离，它只判断是否重叠。
                // 您之前的 getIntervalDistance 实现是正确的，但为了纯粹的速度，
                // 我们可以简化为直接判断。
                return 1.0; // 返回一个正值，表示不碰撞
        }
        }

        // 检查第二个多边形的轴
        for (size_t i = 0; i < Bpoints.size(); ++i) {
            const Location& p1 = Bpoints[i];
            const Location& p2 = Bpoints[(i + 1) % Bpoints.size()];
            Location edge = p2 - p1;
            Location axis(-edge.y, edge.x);

            auto proj1 = project(Apoints, axis);
            auto proj2 = project(Bpoints, axis);

            if (proj1.second < proj2.first || proj2.second < proj1.first) {
                return 1.0; // 返回一个正值，表示不碰撞
            }
        }

        // 所有轴都检查过，没有找到分离轴，表示碰撞
        return -1.0;
    }

    std::vector<Location> getAxes(const std::vector<Location>& polygon) {
        std::vector<Location> axes;
        int n = polygon.size();
        for (int i = 0; i < n; ++i) {
            Location p1 = polygon[i];
            Location p2 = polygon[(i + 1) % n];
            Location edge = p2 - p1;
            Location normal(-edge.y, edge.x);
            normal = normalize(normal);
            axes.push_back(normal);
        }
        return axes;
    }

    // 将多边形投影到某个轴上，返回投影的最小值和最大值
    std::pair<double, double> project(const std::vector<Location>& polygon, const Location& axis) {
        double min = dotProduct(axis, polygon[0] - Location(0, 0));
        double max = min;
        for (size_t i = 1; i < polygon.size(); ++i) {
            double projection = dotProduct(axis, polygon[i] - Location(0, 0));
            if (projection < min) min = projection;
            if (projection > max) max = projection;
        }
        return {min, max};
    }

    // [重要]判断两个投影区间是否重叠，如果不重叠则返回间隔距离（正值表示不重叠，负值表示重叠）
    double getIntervalDistance(const std::pair<double, double>& a, const std::pair<double, double>& b) {
        if (a.second < b.first) {
            return a.second - b.first;
        } else if (b.second < a.first) {
            return b.second - a.first;
        } else {
            return 0.0; // 重叠
        }
    }

    bool isPointInPolygon(Location& Acenter, std::vector<Location>& Bpoints) {
        bool ans = false;
        int n = Bpoints.size();
        for (int i = 0, j = n -1; i < n; j = i++) {
            const Location& current = Bpoints[i];
            const Location& next = Bpoints[j];
            if ((current.y == Acenter.y && current.x == Acenter.x) || 
                (next.y == Acenter.y && next.x == Acenter.x)){
                    return true; // 点在多边形顶点
            }
            // 射线法判断点是否在多边形内
            if ((current.y > Acenter.y) != (next.y > Acenter.y) &&
                (Acenter.x < (next.x - current.x) * (Acenter.y - current.y) / (next.y - current.y) + current.x)) {
                ans = !ans;
            }
        }
        return ans;
    }

    // [新增] 计算点 p 到线段 (a, b) 的最短距离的平方
    double distance_sq_point_to_segment(const Location& p, const Location& a, const Location& b) {
        Location ab = b - a;
        Location ap = p - a;
        double l2 = norm_sq(ab);
        if (l2 == 0.0) return norm_sq(ap);
        double t = std::max(0.0, std::min(1.0, dotProduct(ap, ab) / l2));
        Location projection = a + ab * t;
        return norm_sq(p - projection);
    }

    // [新增] 计算点 p 到整个多边形的最短距离->绝对值
    double distance_point_to_polygon(const Location& p, const std::vector<Location>& polygon) {
        double min_dist_sq = std::numeric_limits<double>::max();
        for (size_t i = 0; i < polygon.size(); ++i) {
            min_dist_sq = std::min(min_dist_sq, distance_sq_point_to_segment(p, polygon[i], polygon[(i + 1) % polygon.size()]));
        }
        return std::sqrt(min_dist_sq);
    }


    std::vector<Location> find_dis_boundary_points(
        const std::vector<Location>& shape_abs, 
        const Location& line_start, 
        const Location& line_end, 
        double safe_dis) 
    {
        std::vector<Location> shape_abs_temp = shape_abs;
        std::vector<Location> result_points;
        // 1. 基本验证
        if (safe_dis <= 0 || shape_abs.size() < 3) {
            return result_points;
        }

        // 2. 找到线段与多边形边界的所有交点
        std::vector<Location> intersection_points;
        for (size_t i = 0; i < shape_abs.size(); ++i) {
            const Location& p1 = shape_abs[i];
            const Location& p2 = shape_abs[(i + 1) % shape_abs.size()];
            Location intersection = segment_intersection(line_start, line_end, p1, p2);
            if (intersection.isValid()) {
                intersection_points.push_back(intersection);
            }
        }

        // 3. 构建并排序所有分割点，形成子线段
        std::vector<Location> segment_delimiters;
        segment_delimiters.push_back(line_start);
        segment_delimiters.insert(segment_delimiters.end(), intersection_points.begin(), intersection_points.end());
        segment_delimiters.push_back(line_end);

        std::sort(segment_delimiters.begin(), segment_delimiters.end(), [&](const Location& a, const Location& b) {
            return norm_sq(a - line_start) < norm_sq(b - line_start);
        });
        segment_delimiters.erase(std::unique(segment_delimiters.begin(), segment_delimiters.end()), segment_delimiters.end());
        
        // 4. 遍历所有子线段，找出外部线段并计算目标点
        for (size_t i = 0; i < segment_delimiters.size() - 1; ++i) {
            Location seg_start = segment_delimiters[i];
            Location seg_end = segment_delimiters[i+1];
            
            if (distance(seg_start, seg_end) < EPS) continue;

            // 通过检查子线段的中点来准确判断其是否在多边形外部
            Location midpoint = seg_start * 0.5 + seg_end * 0.5;
            if (isPointInPolygon(midpoint, shape_abs_temp)) {
                continue; // 跳过内部线段
            }
            
            // --- 此时，我们已确认 [seg_start, seg_end] 是一段外部线段 ---
            
            // 5. 在当前外部线段上寻找与安全边界的所有交点 (候选点)
            std::vector<Location> candidates;

            // a) 与多边形平移后的“直边”求交
            for (size_t j = 0; j < shape_abs.size(); ++j) {
                const Location& p1 = shape_abs[j];
                const Location& p2 = shape_abs[(j + 1) % shape_abs.size()];
                Location edge_vec = p2 - p1;
                Location normal(edge_vec.y, -edge_vec.x); 
                normal = normalize(normal);
                Location p1_offset = p1 + normal * safe_dis;
                Location p2_offset = p2 + normal * safe_dis;
                
                Location candidate = segment_intersection(seg_start, seg_end, p1_offset, p2_offset);
                if (candidate.isValid()) {
                    candidates.push_back(candidate);
                }
            }
            
            // b) 与多边形凸顶点的“圆角”求交
            for (size_t j = 0; j < shape_abs.size(); ++j) {
                const Location& p_prev = shape_abs[(j + shape_abs.size() - 1) % shape_abs.size()];
                const Location& p_curr = shape_abs[j];
                const Location& p_next = shape_abs[(j + 1) % shape_abs.size()];

                if (is_convex_vertex(p_prev, p_curr, p_next)) {
                    auto circle_intersections = line_circle_intersection(seg_start, seg_end, p_curr, safe_dis);
                    
                    for (const auto& pt : circle_intersections) {
                        // [最终修正方案]
                        // 一个点要成为圆角上的有效边界点，必须满足一个根本条件：
                        // 该顶点必须是整个多边形距离这个点最近的部分。
                        // 我们用 distance_point_to_polygon 函数来做这个终极验证。
                        double actual_min_dist = distance_point_to_polygon(pt, shape_abs);

                        if (std::abs(actual_min_dist - safe_dis) < EPS) {
                            candidates.push_back(pt);
                        }
                    }
                }
            }

            // 6. 收集所有在当前线段上的有效候选点
            for (const auto& cand : candidates) {
                if (on_segment(cand, seg_start, seg_end)) {
                    result_points.push_back(cand);
                }
            }
        }
        
        // 7. 统一排序和去重，确保结果的唯一性和顺序性
        std::sort(result_points.begin(), result_points.end(), [&](const Location& a, const Location& b) {
            return norm_sq(a - line_start) < norm_sq(b - line_start);
        });
        
        result_points.erase(std::unique(result_points.begin(), result_points.end()), result_points.end());
        
        return result_points;
    }

    std::vector<Location> rotate_shape (Location &center, double heading, std::vector<Location> &shape) {
        // 以x轴正方向为0度，顺时针为正
        std::vector<Location> shape_rotated;
        for (auto point : shape) {
            shape_rotated.push_back(rotate_point(center, heading, point));
        }
        return shape_rotated;
    }

    Location rotate_point(Location &center, double heading, Location &point) {
        // 1. 将角度从度转换为弧度，因为 cmath 函数使用弧度
        // M_PI 是 cmath 中定义的常量 π
        double angle_rad = heading * M_PI / 180.0;

        // 2. 将 point 坐标平移，使其相对于 center（旋转中心）
        double p_relative_x = point.x ;//- center.x;
        double p_relative_y = point.y ;//- center.y;

        // 3. 应用顺时针旋转公式
        // 对于顺时针旋转 theta 角度:
        // x' = x*cos(theta) + y*sin(theta)
        // y' = -x*sin(theta) + y*cos(theta)
        double rotated_x = p_relative_x * cos(angle_rad) + p_relative_y * sin(angle_rad);
        double rotated_y = -p_relative_x * sin(angle_rad) + p_relative_y * cos(angle_rad);

        // 4. 将旋转后的点平移回原来的坐标系 -> 输出相对坐标
        double final_x = rotated_x + center.x;
        double final_y = rotated_y + center.y;
        if (abs(std::round(final_x) - final_x) < EPS) {
            final_x = std::round(final_x);
        }
        if (abs(std::round(final_y) - final_y) < EPS) {
            final_y = std::round(final_y);
        }
        
        // 5. 返回计算出的新坐标
        return Location(final_x, final_y);
    }
        
    


private:
    Location m_center;
    std::vector<Location> m_points;
    double m_radius;
    

};


// 两线段是否相交
        // int x1 = Acenter.x;
        // int y1 = Acenter.y;
        // int x2 = Apoint.x;
        // int y2 = Apoint.y;
        // int x3 = Bcenter.x;
        // int y3 = Bcenter.y;
        // int x4 = Bpoint.x;
        // int y4 = Bpoint.y;
        // double t1 = ( (x4-x3)*(y1-y3) - (y4-y3)*(x1-x3) ) / ( (y4-y3)*(x2-x1) - (x4-x3)*(y2-y1) );
        // double t2 = ( (x2-x1)*(y1-y3) - (y2-y1)*(x1-x3) ) / ( (y4-y3)*(x2-x1) - (x4-x3)*(y2-y1) );
        // if (t1 >= 0 && t1 <= 1 && t2 >= 0 && t2 <= 1) {
        //     // std::cout << "Lines check result: Yes" << std::endl;
        //     return true;
        // } else {
        //     // std::cout << "Lines check result: No" << std::endl;
        //     return false;
        // }