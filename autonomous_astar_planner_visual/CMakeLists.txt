cmake_minimum_required(VERSION 3.10)
project(AutonomousAStarPlanner)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 包含目录
include_directories(include)

# 添加可执行文件
add_executable(astar_planner
    src/grid_map.cpp
    src/astar_planner.cpp
    src/main.cpp
)

# 设置编译器选项
target_compile_options(astar_planner PRIVATE -Wall -Wextra)
