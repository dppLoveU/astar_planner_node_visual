#include "grid_map.h"
#include <algorithm>

GridMap::GridMap(int width, int height) : width_(width), height_(height) {
    grid_.resize(height, std::vector<bool>(width, true));
}

void GridMap::setObstacle(int x, int y) {
    if (isValid(x, y)) {
        grid_[y][x] = false;
    }
}

void GridMap::clearObstacle(int x, int y) {
    if (isValid(x, y)) {
        grid_[y][x] = true;
    }
}

bool GridMap::isTraversable(int x, int y) const {
    return isValid(x, y) && grid_[y][x];
}

bool GridMap::isTraversable(const Point& p) const {
    return isTraversable(p.x, p.y);
}

bool GridMap::isValid(int x, int y) const {
    return x >= 0 && x < width_ && y >= 0 && y < height_;
}

bool GridMap::isValid(const Point& p) const {
    return isValid(p.x, p.y);
}

int GridMap::getWidth() const { return width_; }
int GridMap::getHeight() const { return height_; }

void GridMap::printMap(const Point& start, const Point& goal, 
                      const std::vector<Point>& path) const {
    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            Point current(x, y);
            if (current == start) {
                std::cout << "S ";
            } else if (current == goal) {
                std::cout << "G ";
            } else if (std::find(path.begin(), path.end(), current) != path.end()) {
                std::cout << "* ";
            } else {
                std::cout << (grid_[y][x] ? ". " : "X ");
            }
        }
        std::cout << std::endl;
    }
}