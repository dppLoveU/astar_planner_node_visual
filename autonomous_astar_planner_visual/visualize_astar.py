import json
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation
import numpy as np

class CppAStarVisualizer:
    def __init__(self, data_file):
        """
        从C++导出的数据文件初始化可视化器
        
        Args:
            data_file: C++导出的JSON数据文件路径
        """
        with open(data_file, 'r') as f:
            self.data = json.load(f)
        
        self.map_width = self.data['map_size'][0]
        self.map_height = self.data['map_size'][1]
        self.start = tuple(self.data['start'])
        self.goal = tuple(self.data['goal'])
        self.obstacles = [tuple(obs) for obs in self.data['obstacles']]
        self.final_path = [tuple(point) for point in self.data['final_path']]
        self.search_history = self.data['search_history']
        
        self.current_step = 0
        self.total_steps = len(self.search_history)
        
        # 设置matplotlib
        self.fig, self.ax = plt.subplots(figsize=(12, 8))
        self.setup_plot()
        
    def setup_plot(self):
        """设置绘图参数"""
        self.ax.set_xlim(-0.5, self.map_width - 0.5)
        self.ax.set_ylim(-0.5, self.map_height - 0.5)
        self.ax.set_aspect('equal')
        self.ax.grid(True, alpha=0.3)
        self.ax.set_xticks(np.arange(0, self.map_width, 1))
        self.ax.set_yticks(np.arange(0, self.map_height, 1))
        self.ax.set_title('C++ A* Path Planning Visualization', fontsize=14, fontweight='bold')
        
    def draw_static_elements(self):
        """绘制静态元素（障碍物、起点、终点）"""
        # 绘制障碍物
        for x, y in self.obstacles:
            rect = patches.Rectangle((x-0.5, y-0.5), 1, 1, 
                                   facecolor='red', alpha=0.7, label='Obstacle')
            self.ax.add_patch(rect)
        
        # 绘制起点
        start_rect = patches.Rectangle((self.start[0]-0.5, self.start[1]-0.5), 1, 1, 
                                     facecolor='green', alpha=0.8, label='Start')
        self.ax.add_patch(start_rect)
        self.ax.text(self.start[0], self.start[1], 'S', 
                    ha='center', va='center', fontweight='bold', fontsize=12)
        
        # 绘制终点
        goal_rect = patches.Rectangle((self.goal[0]-0.5, self.goal[1]-0.5), 1, 1, 
                                    facecolor='blue', alpha=0.8, label='Goal')
        self.ax.add_patch(goal_rect)
        self.ax.text(self.goal[0], self.goal[1], 'G', 
                    ha='center', va='center', fontweight='bold', fontsize=12)
    
    def update_animation(self, frame):
        """更新动画帧"""
        self.ax.clear()
        self.setup_plot()
        self.draw_static_elements()
        
        if frame < self.total_steps:
            step_data = self.search_history[frame]
            
            # 绘制开放集合（黄色）
            for x, y in step_data['open_set']:
                rect = patches.Rectangle((x-0.5, y-0.5), 1, 1, 
                                       facecolor='yellow', alpha=0.5, label='Open Set')
                self.ax.add_patch(rect)
            
            # 绘制关闭集合（橙色）
            for x, y in step_data['closed_set']:
                if (x, y) != self.start and (x, y) != self.goal:
                    rect = patches.Rectangle((x-0.5, y-0.5), 1, 1, 
                                           facecolor='orange', alpha=0.5, label='Closed Set')
                    self.ax.add_patch(rect)
            
            # 绘制当前节点（红色边框）
            current_x, current_y = step_data['current_node']
            if (current_x, current_y) != self.start and (current_x, current_y) != self.goal:
                rect = patches.Rectangle((current_x-0.5, current_y-0.5), 1, 1, 
                                       fill=False, edgecolor='red', linewidth=3)
                self.ax.add_patch(rect)
            
            # 绘制当前路径
            current_path = step_data['current_path']
            if len(current_path) > 1:
                x_coords = [point[0] for point in current_path]
                y_coords = [point[1] for point in current_path]
                self.ax.plot(x_coords, y_coords, 'purple', linewidth=2, alpha=0.7, label='Current Path')
            
            # 添加信息文本
            info_text = f"Step: {frame + 1}/{self.total_steps}\n"
            info_text += f"Open Set: {len(step_data['open_set'])}\n"
            info_text += f"Closed Set: {len(step_data['closed_set'])}"
            
            self.ax.text(0.02, 0.98, info_text, transform=self.ax.transAxes,
                        verticalalignment='top', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
        
        # 在最后一帧绘制最终路径
        if frame == self.total_steps - 1 and self.final_path:
            x_coords = [point[0] for point in self.final_path]
            y_coords = [point[1] for point in self.final_path]
            self.ax.plot(x_coords, y_coords, 'green', linewidth=3, label='Final Path')
            
            # 标记路径点
            for i, (x, y) in enumerate(self.final_path):
                if (x, y) != self.start and (x, y) != self.goal:
                    self.ax.text(x, y, str(i), ha='center', va='center', 
                                fontsize=8, color='darkgreen')
        
        # 添加图例（只添加一次）
        if frame == 0:
            handles, labels = self.ax.get_legend_handles_labels()
            by_label = dict(zip(labels, handles))
            self.ax.legend(by_label.values(), by_label.keys(), 
                          bbox_to_anchor=(1.05, 1), loc='upper left')
    
    def animate_search(self, save_gif=False):
        """运行动画"""
        print(f"Starting animation with {self.total_steps} steps...")
        print(f"Map size: {self.map_width}x{self.map_height}")
        print(f"Start: {self.start}, Goal: {self.goal}")
        print(f"Obstacles: {len(self.obstacles)}")
        print(f"Final path length: {len(self.final_path)}")
        
        anim = FuncAnimation(self.fig, self.update_animation, 
                           frames=self.total_steps, interval=100, repeat=False)
        
        if save_gif:
            anim.save('cpp_astar_animation.gif', writer='pillow', fps=10)
            print("Animation saved as cpp_astar_animation.gif")
        
        plt.tight_layout()
        plt.show()
    
    def show_final_result(self):
        """只显示最终结果"""
        self.ax.clear()
        self.setup_plot()
        self.draw_static_elements()
        
        # 绘制最终路径
        if self.final_path:
            x_coords = [point[0] for point in self.final_path]
            y_coords = [point[1] for point in self.final_path]
            self.ax.plot(x_coords, y_coords, 'green', linewidth=3, label='Final Path')
            
            # 标记路径点
            for i, (x, y) in enumerate(self.final_path):
                if (x, y) != self.start and (x, y) != self.goal:
                    self.ax.plot(x, y, 'go', markersize=8, alpha=0.6)
                    self.ax.text(x, y, str(i), ha='center', va='center', 
                                fontsize=8, color='darkgreen')
        
        plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
        plt.tight_layout()
        plt.show()

def main():
    """主函数"""
    data_file = "astar_search_data.json"
    
    try:
        visualizer = CppAStarVisualizer(data_file)
        
        print("=== C++ A* Path Planning Visualizer ===")
        print("1. Show search animation")
        print("2. Show final result only")
        
        choice = input("Enter your choice (1 or 2): ").strip()
        
        if choice == "1":
            save_gif = input("Save as GIF? (y/n): ").strip().lower() == 'y'
            visualizer.animate_search(save_gif=save_gif)
        elif choice == "2":
            visualizer.show_final_result()
        else:
            print("Invalid choice. Showing final result.")
            visualizer.show_final_result()
            
    except FileNotFoundError:
        print(f"Error: Data file '{data_file}' not found.")
        print("Please run the C++ A* planner first to generate the data file.")
    except json.JSONDecodeError:
        print(f"Error: Invalid JSON format in '{data_file}'.")
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    main()