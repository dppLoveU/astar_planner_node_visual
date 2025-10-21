#ifndef GRID_MAP_H
#define GRID_MAP_H

#include "point.h"
#include <vector>
#include <iostream>

class GridMap {
private:
    int width_, height_;
    std::vector<std::vector<bool>> grid_;
    
public:
    GridMap(int width, int height);
    void setObstacle(int x, int y);
    void clearObstacle(int x, int y);
    bool isTraversable(int x, int y) const;
    bool isTraversable(const Point& p) const;
    bool isValid(int x, int y) const;
    bool isValid(const Point& p) const;
    int getWidth() const;
    int getHeight() const;
    void printMap(const Point& start, const Point& goal, 
                  const std::vector<Point>& path = {}) const;
};

#endif