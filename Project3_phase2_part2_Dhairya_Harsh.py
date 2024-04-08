#!/usr/bin/env python3

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
        self.obstacle1 = plt.Circle((4200, 1200), (600), fill=True, color='red')
        self.obstacle2 = patches.Rectangle((1500, 1000), 250, 1000, color='red')
        self.obstacle3 = patches.Rectangle((2500, 0), 250, 1000, color='red')  
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

class Node:
    def __init__(self, x, y, parent, current_theta, change_theta, UL, UR, c2c, c2g, total_cost):
        self.x = x
        self.y = y
        self.parent = parent
        self.current_theta = current_theta
        self.change_theta = change_theta
        self.UL = UL
        self.UR = UR
        self.c2c = c2c
        self.c2g = c2g
        self.total_cost = total_cost

    def __lt__(self, other):
        return self.total_cost < other.total_cost

def plot_curve(Xi, Yi, Thetai, UL, UR, c, plot, Nodes_list, Path_list):
    t = 0
    r = 33 
    L = 287  
    dt = 0.1
    cost = 0
    X_end = Xi
    Y_end = Yi
    Theta_end = math.pi * Thetai / 180

    while t < 1:
        t = t + dt
        X_start = X_end
        Y_start = Y_end
        X_end += r * 0.5 * (UL + UR) * math.cos(Theta_end) * dt
        Y_end += r * 0.5 * (UL + UR) * math.sin(Theta_end) * dt
        Theta_end += (r / L) * (UR - UL) * dt

        if valid_move(X_end, Y_end, r, c):
            if plot == 0:
                Nodes_list.append((X_end, Y_end))
                Path_list.append((X_start, Y_start))
            if plot == 1:
                plt.plot([X_start, X_end], [Y_start, Y_end], color="red")
        else:
            return None

    Theta_end = 180 * (Theta_end) / math.pi
    return [X_end, Y_end, Theta_end, cost, Nodes_list, Path_list]

def key(node):
    key = 1000 * node.x + 111 * node.y
    return key
    
def is_point_inside_rectangle(x, y, vertices):
    x_min = min(vertices[0][0], vertices[1][0], vertices[2][0], vertices[3][0])
    x_max = max(vertices[0][0], vertices[1][0], vertices[2][0], vertices[3][0])
    y_min = min(vertices[0][1], vertices[1][1], vertices[2][1], vertices[3][1])
    y_max = max(vertices[0][1], vertices[1][1], vertices[2][1], vertices[3][1])
    return x_min <= x <= x_max and y_min <= y <= y_max

def is_point_inside_circle(x, y, center_x, center_y, diameter):
    radius = diameter / 2.0 
    distance = math.sqrt((x - center_x) ** 2 + (y - center_y) ** 2)
    return distance <= radius

def obstacle_space_check(x, y, robot_radius, clearance):
   
    rectangle1_buffer_vts = [(1500 - (robot_radius + clearance), 2000), (1750 + (robot_radius + clearance), 2000), (1750 + (robot_radius + clearance), 1000 - (robot_radius + clearance)), (1500 - (robot_radius + clearance), 1000 - (robot_radius + clearance))]
    rectangle2_buffer_vts = [(2500 - (robot_radius + clearance), 1000 + (robot_radius + clearance)), (2750 + (robot_radius + clearance), 1000 - (robot_radius + clearance)), (2750 + (robot_radius + clearance), 0), (2500 - (robot_radius + clearance), 0)]

    rect1_buffer = is_point_inside_rectangle(x,y, rectangle1_buffer_vts)
    rect2_buffer = is_point_inside_rectangle(x, y, rectangle2_buffer_vts)
    circ_buffer = is_point_inside_circle(x, y, 4200, 1300, 1200 + 2*(robot_radius + clearance))
    
    if rect1_buffer or rect2_buffer or circ_buffer:
        return True
    
    if x <= (robot_radius + clearance) or y >= 2000 - (robot_radius + clearance) or x >= 6000 - (robot_radius + clearance) or y <= (robot_radius + clearance):
        return True

    return False
    
def valid_move(x, y, r, c):
    if obstacle_space_check(x, y, r, c):
        return False
    else:
        return True

def Astar(start_node, goal_node, rpm1, rpm2, radius, clearance):

    if check_goal(start_node, goal_node):
        return 1, None, None
    
    start_node = start_node
    start_node_id = key(start_node)
    goal_node = goal_node

    Nodes_list = [] 
    Path_list = []  

    closed_node = {} 
    open_node = {} 
    
    open_node[start_node_id] = start_node 

    priority_list = [] 
    
    moves = [[rpm1, 0], 
             [0, rpm1], 
             [rpm1, rpm1], 
             [0, rpm2], 
             [rpm2, 0], 
             [rpm2, rpm2], 
             [rpm1, rpm2],
             [rpm2, rpm1]]

    heapq.heappush(priority_list, [start_node.total_cost, start_node])

    while (len(priority_list) != 0):

        current_nodes = (heapq.heappop(priority_list))[1]
        current_id = key(current_nodes)

        if check_goal(current_nodes, goal_node):
            goal_node.parent = current_nodes.parent
            goal_node.total_cost = current_nodes.total_cost
            print("Goal Node found")
            return 1, Nodes_list, Path_list
        
        if current_id in closed_node:  
            continue
        else:
            closed_node[current_id] = current_nodes
        
        del open_node[current_id]
        
        for move in moves:
            action = plot_curve(current_nodes.x, current_nodes.y, current_nodes.current_theta, move[0], move[1],
                            clearance, 0, Nodes_list, Path_list)
           
            if (action != None):
                angle = action[2]
                
                theta_lim = 30
                x = (round(action[0] * 10) / 10)
                y = (round(action[1] * 10) / 10)
                theta = (round(angle / theta_lim) * theta_lim)
                
                current_theta = current_nodes.change_theta - theta
                c2g = dist((x,y), (goal_node.x, goal_node.y))
                new_node = Node(x, y, current_nodes, theta, current_theta, move[0], move[1], current_nodes.c2c+action[3], c2g, current_nodes.c2c+action[3]+c2g)

                new_node_id = key(new_node)
                
                if not valid_move(new_node.x, new_node.y, radius, clearance):
                    continue
                elif new_node_id in closed_node:
                    continue

                if new_node_id in open_node:
                    if new_node.total_cost < open_node[new_node_id].total_cost:
                        open_node[new_node_id].total_cost = new_node.total_cost
                        open_node[new_node_id].parent = new_node
       
                else:
                    open_node[new_node_id] = new_node
                    heapq.heappush(priority_list, [ open_node[new_node_id].total_cost, open_node[new_node_id]])
            
    return 0, Nodes_list, Path_list

def check_goal(current, goal):
    dt = dist((current.x, current.y), (goal.x, goal.y))

    if dt < 100:
        return True
    else:
        return False

def valid_orientation(theta):
    if (theta % 30 == 0):
        return True
    else:
        return False

def back_track(goal_node):  
    x_path = []
    y_path = []
    theta_path = []
    rpm1_path = []
    rpm2_path = []

    parent_node = goal_node.parent

    x_path.append(goal_node.x)
    y_path.append(goal_node.y)
    theta_path.append(goal_node.current_theta)
    rpm1_path.append(goal_node.UL)
    rpm2_path.append(goal_node.UR)
    

    while parent_node != -1:
        x_path.append(parent_node.x)
        y_path.append(parent_node.y)
        theta_path.append(parent_node.current_theta)
        rpm1_path.append(parent_node.UL)
        rpm2_path.append(parent_node.UR)
        parent_node = parent_node.parent

    x_path.reverse()
    y_path.reverse()
    theta_path.reverse()
    rpm1_path.reverse()
    rpm2_path.reverse()

    x = np.asarray(x_path)
    y = np.asarray(y_path)
    theta = np.array(theta_path)
    rpm1_final = np.array(rpm1_path)
    rpm2_final = np.array(rpm2_path)

    return x, y, theta, rpm1_final, rpm2_final

def euclidean_distance(point1, point2):
    return math.sqrt((point2[0] - point1[0]) * 2 + (point2[1] - point1[1]) * 2)

def calculate_twist(rpm1,rpm2,theta):
    r = 0.33
    L = 0.287
    linear_x = []
    angular_z = []
    for i in range(len(rpm1)):
        linear_x.append((r/2) * ((rpm1[i]*(2*math.pi/60))+(rpm2[i]*(2*math.pi/60))))
        angular_z.append((r/L) * ((rpm2[i]*(2*math.pi/60))-(rpm1[i]*(2*math.pi/60))))
    return linear_x, angular_z

def publish_twist_messages(linear_x, angular_z):
    rclpy.init()
    node = rclpy.create_node('publisher')

    publisher = node.create_publisher(Twist, 'cmd_vel', 10)
    msg = Twist()

    for linear, angular in zip(linear_x, angular_z):
        msg.linear.x = linear
        msg.angular.z = angular
        publisher.publish(msg)
        node.get_logger().info("Published twist message: Linear=%.2f, Angular=%.2f" % (msg.linear.x, msg.angular.z))
        time.sleep(1)
    rclpy.shutdown()

def main():
    plotter = Plotter()
    radius = 230
    clearance = 5
    RPM1 = 10
    RPM2 = 5
    start_x = 500
    start_y =  1000

    goal_x = float(input("Enter X coordinate of Goal (in mm): "))
    goal_y = float(input("Enter Y coordinate of Goal (in mm): "))

    if not valid_move(goal_x, goal_y, radius, clearance):
        print("Invalid start or goal node, or in obstacle space")
        exit(-1)

    start_theta = 0

    timer_start = time.time()

    c2g = dist((start_x, start_y), (goal_x, goal_y))
    total_cost = c2g

    start_node = Node(start_x, start_y, -1, start_theta, 0, 0, 0, 0, c2g, total_cost)
    goal_node = Node(goal_x, goal_y, -1, 0, 0, 0, 0, c2g, 0, total_cost)

    flag, Nodes_list, Path_list = Astar(start_node, goal_node, RPM1, RPM2, radius, clearance)

    if flag == 1:
        x_path, y_path, theta_path, rpm1, rpm2 = back_track(goal_node)
    else:
        print("Path not found")
        exit(-1)

    plotter.plot_start_and_goal(start_node, goal_node)
    plotter.update_plot(Nodes_list, Path_list) 
    plotter.axes.plot(x_path, y_path, '-r') 
    
    timer_stop = time.time()
    runtime = timer_stop - timer_start
    plt.show()
    
    print("Total Runtime: ", runtime)
    print("Total Cost", total_cost)

    linear_x, angular_z = calculate_twist(rpm1, rpm2, theta_path)
    publish_twist_messages(linear_x, angular_z)

if __name__ == '__main__':
    main()
