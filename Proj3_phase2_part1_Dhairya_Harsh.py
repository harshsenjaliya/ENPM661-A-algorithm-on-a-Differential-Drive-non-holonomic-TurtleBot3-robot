import numpy as np
import math
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import heapq
from math import dist
import matplotlib.patches as patches
import time
import rclpy
from geometry_msgs.msg import Twist

class Plotter:
    def __init__(self):
        self.figure, self.axes = plt.subplots()
        self.axes.set(xlim=(0, 6000), ylim=(0, 2000))
        self.obstacle1 = plt.Circle((4000, 1100), 500, fill=True, color='red')
        self.obstacle2 = patches.Rectangle((1500, 750), 150, 1250, color='red')
        self.obstacle3 = patches.Rectangle((2500, 0), 150, 1250, color='red')  
        self.axes.set_aspect('equal')
        self.axes.add_artist(self.obstacle1)
        self.axes.add_artist(self.obstacle2)
        self.axes.add_patch(self.obstacle3)
        self.explored_scatter = None
        self.path_line = None

    def plot_start_and_goal(self, start_node, goal_node):
        self.axes.plot(start_node.x, start_node.y, "Dg")
        self.axes.plot(goal_node.x, goal_node.y, "Dg")

    def update_plot(self, Nodes_list, Path_list):
        if self.explored_scatter is not None:
            self.explored_scatter.remove()
        x_values, y_values = zip(*Nodes_list)
        self.explored_scatter = self.axes.scatter(x_values, y_values, color='yellow')

        if self.path_line is not None:
            self.path_line.remove()
        if Path_list:
            path_x, path_y = zip(*Path_list)
            self.path_line, = self.axes.plot(path_x, path_y, '-b')

    def plot_backtrack_path(self, backtrack_path):
        if self.path_line is not None:
            self.path_line.remove()
        if backtrack_path:
            backtrack_x, backtrack_y = zip(*backtrack_path)
            self.path_line, = self.axes.plot(backtrack_x, backtrack_y, '-g')
        plt.show()


