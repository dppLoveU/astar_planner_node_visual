#ifndef ASTAR_PLANNER_H
#define ASTAR_PLANNER_H

#include "grid_map.h"
#include "point.h"
#include <memory>
#include <vector>
#include <string>
#include <unordered_map>

// 前向声明
struct Node;

// 搜索过程数据记录
struct SearchStep {
    std::vector<Point> open_set;
    std::vector<Point> closed_set;
    std::vector<Point> current_path;
    Point current_node;
};

class AStarPlanner {
private:
    std::shared_ptr<GridMap> map_;
    std::vector<SearchStep> search_history_;
    Point start_;
    Point goal_;
    std::unordered_map<Point, Point> came_from_;  // 存储Point到Point的映射
    
public:
    AStarPlanner(std::shared_ptr<GridMap> map);
    std::vector<Point> findPath(const Point& start, const Point& goal);
    
    // 获取搜索历史数据，用于Python可视化
    const std::vector<SearchStep>& getSearchHistory() const { return search_history_; }
    void clearSearchHistory() { search_history_.clear(); }
    
    // 导出数据到文件，供Python读取
    void exportSearchData(const std::string& filename, const Point& start, const Point& goal, const std::vector<Point>& path) const;
    
private:
    double euclideanHeuristic(const Point& a, const Point& b) const;
    double manhattanHeuristic(const Point& a, const Point& b) const;
    std::vector<Point> getNeighbors(const Point& current) const;
    double getMoveCost(const Point& from, const Point& to) const;
    std::vector<Point> reconstructPath(std::shared_ptr<Node> goal_node);
    
    // 记录搜索步骤
    void recordSearchStep(const std::vector<std::pair<double, Point>>& open_set, 
                         const std::unordered_map<Point, double>& visited_costs,
                         Point current_node);
};

#endif
