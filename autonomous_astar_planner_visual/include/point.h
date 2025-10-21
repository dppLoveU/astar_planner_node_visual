#ifndef POINT_H
#define POINT_H

#include <functional>
#include <tuple>

struct Point {
    int x, y;
    
    Point(int x = 0, int y = 0) : x(x), y(y) {}
    
    // 重载==运算符
    bool operator==(const Point& other) const {
        return x == other.x && y == other.y;
    }
    
    // 重载!=运算符
    bool operator!=(const Point& other) const {
        return !(*this == other);
    }
    
    // 重载<运算符，用于排序
    bool operator<(const Point& other) const {
        return std::tie(x, y) < std::tie(other.x, other.y);
    }
};

// 哈希函数特化
namespace std {
    template<>
    struct hash<Point> {
        size_t operator()(const Point& p) const {
            return hash<int>()(p.x) ^ (hash<int>()(p.y) << 1);
        }
    };
}

#endif
