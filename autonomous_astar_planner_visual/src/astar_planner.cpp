#include "astar_planner.h"
#include <queue>
#include <unordered_map>
#include <cmath>
#include <algorithm>
#include <memory>
#include <fstream>
#include <iostream>
#include <functional>

// Node结构体定义
struct Node {
    Point position;
    std::shared_ptr<Node> parent;
    double g_cost;
    double h_cost;
    double f_cost;
    
    Node(Point pos, std::shared_ptr<Node> parent = nullptr, 
         double g = 0, double h = 0)
        : position(pos), parent(parent), g_cost(g), h_cost(h), f_cost(g + h) {}
    
    // 重载>运算符，用于优先队列
    bool operator>(const Node& other) const {
        return f_cost > other.f_cost;
    }
};

AStarPlanner::AStarPlanner(std::shared_ptr<GridMap> map) : map_(map) {}

double AStarPlanner::euclideanHeuristic(const Point& a, const Point& b) const {
    return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
}

double AStarPlanner::manhattanHeuristic(const Point& a, const Point& b) const {
    return std::abs(a.x - b.x) + std::abs(a.y - b.y);
}

std::vector<Point> AStarPlanner::getNeighbors(const Point& current) const {
    std::vector<Point> neighbors;
    
    // 8方向移动向量
    const std::vector<Point> directions = {
        {0, 1}, {0, -1}, {-1, 0}, {1, 0},
        {-1, 1}, {1, 1}, {-1, -1}, {1, -1}
    };
    
    for (const auto& dir : directions) {
        Point neighbor(current.x + dir.x, current.y + dir.y);
        if (map_->isTraversable(neighbor)) {
            neighbors.push_back(neighbor);
        }
    }
    
    return neighbors;
}

double AStarPlanner::getMoveCost(const Point& from, const Point& to) const {
    int dx = std::abs(from.x - to.x);
    int dy = std::abs(from.y - to.y);
    
    if (dx == 1 && dy == 1) {
        return 1.414;  // 对角线代价
    } else {
        return 1.0;    // 直线移动代价
    }
}

std::vector<Point> AStarPlanner::reconstructPath(std::shared_ptr<Node> goal_node) {
    std::vector<Point> path;
    
    // 使用came_from_重建路径
    Point current = goal_node->position;
    while (current != start_) {
        path.push_back(current);
        current = came_from_[current];
    }
    path.push_back(start_);
    
    std::reverse(path.begin(), path.end());
    return path;
}

void AStarPlanner::recordSearchStep(const std::vector<std::pair<double, Point>>& open_set, 
                                   const std::unordered_map<Point, double>& visited_costs,
                                   Point current_node) {
    SearchStep step;
    step.current_node = current_node;
    
    // 记录开放集合
    for (const auto& item : open_set) {
        step.open_set.push_back(item.second);
    }
    
    // 记录关闭集合
    for (const auto& item : visited_costs) {
        step.closed_set.push_back(item.first);
    }
    
    // 重建当前路径（使用came_from_）
    if (came_from_.find(current_node) != came_from_.end()) {
        Point temp = current_node;
        while (temp != start_ && came_from_.find(temp) != came_from_.end()) {
            step.current_path.push_back(temp);
            temp = came_from_[temp];
        }
        std::reverse(step.current_path.begin(), step.current_path.end());
    }
    
    search_history_.push_back(step);
}

std::vector<Point> AStarPlanner::findPath(const Point& start, const Point& goal) {
    // 清空历史记录和之前的状态
    clearSearchHistory();
    came_from_.clear();
    start_ = start;
    goal_ = goal;
    
    if (!map_->isTraversable(start) || !map_->isTraversable(goal)) {
        std::cout << "Start or goal position is not traversable!" << std::endl;
        return {};
    }
    
    if (start == goal) {
        return {start};
    }
    
    // 定义节点比较函数
    auto nodeCompare = [](const std::shared_ptr<Node>& a, const std::shared_ptr<Node>& b) {
        return a->f_cost > b->f_cost;
    };
    
    // 使用priority_queue
    std::priority_queue<
        std::shared_ptr<Node>,
        std::vector<std::shared_ptr<Node>>,
        decltype(nodeCompare)
    > open_list(nodeCompare);
    
    std::unordered_map<Point, double> visited_costs;
    
    auto start_node = std::make_shared<Node>(start);
    start_node->h_cost = euclideanHeuristic(start, goal);
    start_node->f_cost = start_node->g_cost + start_node->h_cost;
    
    open_list.push(start_node);
    visited_costs[start] = 0.0;
    came_from_[start] = start;  // 起点指向自己
    
    // 记录初始状态
    std::vector<std::pair<double, Point>> current_open_set;
    current_open_set.push_back(std::make_pair(start_node->f_cost, start));
    recordSearchStep(current_open_set, visited_costs, start);
    
    int iterations = 0;
    const int MAX_ITERATIONS = 10000;
    
    while (!open_list.empty() && iterations < MAX_ITERATIONS) {
        iterations++;
        
        auto current_node = open_list.top();
        open_list.pop();
        
        // 更新当前开放集合
        current_open_set.clear();
        
        // 复制开放列表来获取所有节点
        auto open_list_copy = open_list;
        std::vector<std::shared_ptr<Node>> temp_nodes;
        while (!open_list_copy.empty()) {
            temp_nodes.push_back(open_list_copy.top());
            open_list_copy.pop();
        }
        
        for (const auto& node : temp_nodes) {
            current_open_set.push_back(std::make_pair(node->f_cost, node->position));
        }
        
        if (current_node->position == goal) {
            std::cout << "Path found! Iterations: " << iterations << std::endl;
            auto path = reconstructPath(current_node);
            
            // 记录最终步骤
            recordSearchStep(current_open_set, visited_costs, goal);
            return path;
        }
        
        for (const auto& neighbor_pos : getNeighbors(current_node->position)) {
            double new_g_cost = current_node->g_cost + 
                              getMoveCost(current_node->position, neighbor_pos);
            
            auto visited_it = visited_costs.find(neighbor_pos);
            if (visited_it == visited_costs.end() || new_g_cost < visited_it->second) {
                visited_costs[neighbor_pos] = new_g_cost;
                
                double h_cost = euclideanHeuristic(neighbor_pos, goal);
                auto neighbor_node = std::make_shared<Node>(
                    neighbor_pos, current_node, new_g_cost, h_cost);
                
                open_list.push(neighbor_node);
                came_from_[neighbor_pos] = current_node->position;  // 存储Point而不是Node
            }
        }
        
        // 记录每一步的搜索状态
        recordSearchStep(current_open_set, visited_costs, current_node->position);
    }
    
    std::cout << "Path not found after " << iterations << " iterations!" << std::endl;
    return {};
}

void AStarPlanner::exportSearchData(const std::string& filename, 
                                   const Point& start, 
                                   const Point& goal, 
                                   const std::vector<Point>& path) const {
    std::ofstream file(filename);
    
    file << "{" << std::endl;
    file << "  \"map_size\": [" << map_->getWidth() << ", " << map_->getHeight() << "]," << std::endl;
    file << "  \"start\": [" << start.x << ", " << start.y << "]," << std::endl;
    file << "  \"goal\": [" << goal.x << ", " << goal.y << "]," << std::endl;
    
    // 导出障碍物
    file << "  \"obstacles\": [";
    bool first_obstacle = true;
    for (int y = 0; y < map_->getHeight(); ++y) {
        for (int x = 0; x < map_->getWidth(); ++x) {
            if (!map_->isTraversable(x, y)) {
                if (!first_obstacle) file << ", ";
                file << "[" << x << ", " << y << "]";
                first_obstacle = false;
            }
        }
    }
    file << "]," << std::endl;
    
    // 导出最终路径
    file << "  \"final_path\": [";
    for (size_t i = 0; i < path.size(); ++i) {
        if (i > 0) file << ", ";
        file << "[" << path[i].x << ", " << path[i].y << "]";
    }
    file << "]," << std::endl;
    
    // 导出搜索历史
    file << "  \"search_history\": [" << std::endl;
    for (size_t step_idx = 0; step_idx < search_history_.size(); ++step_idx) {
        const auto& step = search_history_[step_idx];
        file << "    {" << std::endl;
        file << "      \"step\": " << step_idx << "," << std::endl;
        file << "      \"current_node\": [" << step.current_node.x << ", " << step.current_node.y << "]," << std::endl;
        
        // 开放集合
        file << "  \"open_set\": [";
        for (size_t i = 0; i < step.open_set.size(); ++i) {
            if (i > 0) file << ", ";
            file << "[" << step.open_set[i].x << ", " << step.open_set[i].y << "]";
        }
        file << "]," << std::endl;
        
        // 关闭集合
        file << "  \"closed_set\": [";
        for (size_t i = 0; i < step.closed_set.size(); ++i) {
            if (i > 0) file << ", ";
            file << "[" << step.closed_set[i].x << ", " << step.closed_set[i].y << "]";
        }
        file << "]," << std::endl;
        
        // 当前路径
        file << "  \"current_path\": [";
        for (size_t i = 0; i < step.current_path.size(); ++i) {
            if (i > 0) file << ", ";
            file << "[" << step.current_path[i].x << ", " << step.current_path[i].y << "]";
        }
        file << "]" << std::endl;
        
        file << "    }";
        if (step_idx < search_history_.size() - 1) file << ",";
        file << std::endl;
    }
    file << "  ]" << std::endl;
    file << "}" << std::endl;
    
    file.close();
    std::cout << "Search data exported to: " << filename << std::endl;
}
