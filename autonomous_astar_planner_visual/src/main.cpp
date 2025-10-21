#include "grid_map.h"
#include "astar_planner.h"
#include <memory>
#include <iostream>

class AutonomousPathPlanner {
private:
    std::shared_ptr<GridMap> environment_map_;
    std::unique_ptr<AStarPlanner> path_planner_;
    
public:
    AutonomousPathPlanner(int width, int height) {
        environment_map_ = std::make_shared<GridMap>(width, height);
        path_planner_ = std::make_unique<AStarPlanner>(environment_map_);
        setupTestObstacles();
    }
    
    void setupTestObstacles() {
        for (int i = 5; i < 15; ++i) {
            environment_map_->setObstacle(i, 8);
        }
        
        for (int i = 3; i < 12; ++i) {
            environment_map_->setObstacle(10, i);
        }
        
        environment_map_->setObstacle(3, 3);
        environment_map_->setObstacle(7, 12);
        environment_map_->setObstacle(15, 5);
    }
    
    std::vector<Point> planRoute(const Point& current_pose, const Point& destination, 
                                bool export_data = true) {
        std::cout << "Planning route from (" << current_pose.x << ", " << current_pose.y 
                  << ") to (" << destination.x << ", " << destination.y << ")" << std::endl;
        
        auto path = path_planner_->findPath(current_pose, destination);
        
        if (!path.empty()) {
            std::cout << "Path length: " << path.size() << " points" << std::endl;
            environment_map_->printMap(current_pose, destination, path);
            
            // 导出数据供Python可视化
            if (export_data) {
                path_planner_->exportSearchData("astar_search_data.json", 
                                               current_pose, destination, path);
            }
        }
        
        return path;
    }
};

int main() {
    std::cout << "=== Autonomous Vehicle A* Path Planner ===" << std::endl;
    
    AutonomousPathPlanner auto_planner(20, 15);
    
    Point vehicle_position(2, 2);
    Point parking_spot(18, 13);
    
    auto planned_path = auto_planner.planRoute(vehicle_position, parking_spot);
    
    if (planned_path.empty()) {
        std::cout << "Failed to find a path!" << std::endl;
    } else {
        std::cout << "Path planning successful!" << std::endl;
        std::cout << "Run the Python visualizer to see the animation!" << std::endl;
    }
    
    return 0;
}
