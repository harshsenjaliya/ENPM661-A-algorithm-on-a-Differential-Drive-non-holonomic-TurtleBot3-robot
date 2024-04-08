
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
    r = 40 #Robot Wheel Radius
    L = 160 # Distance vetween the wheels of the robot. 
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

def Astar(start_node, goal_node, rpm1, rpm2, radius, clearance):

    # Check if the goal node is reached 
    if check_goal(start_node, goal_node):
        return 1, None, None
    
    start_node = start_node
    start_node_id = key(start_node)
    goal_node = goal_node

    Nodes_list = []  # List to store all the explored nodes
    Path_list = []  # List to store the final path from start to goal node

    closed_node = {}  # Dictionary to store all the closed nodes
    open_node = {}  # Dictionary to store all the open nodes
    
    open_node[start_node_id] = start_node   # Add the start node to the open nodes dictionary

    priority_list = []  # Priority queue to store nodes based on their total cost
    
    # All the possible moves of the robot
    moves = [[rpm1, 0], 
             [0, rpm1], 
             [rpm1, rpm1], 
             [0, rpm2], 
             [rpm2, 0], 
             [rpm2, rpm2], 
             [rpm1, rpm2],
             [rpm2, rpm1]]

    # Push the start node into the priority queue with its total cost
    heapq.heappush(priority_list, [start_node.total_cost, start_node])

    while (len(priority_list) != 0):

        # Pop the node with the minimum cost from the priority queue
        current_nodes = (heapq.heappop(priority_list))[1]
        current_id = key(current_nodes)

        # Check if the popped node is the goal node
        if check_goal(current_nodes, goal_node):
            goal_node.parent = current_nodes.parent
            goal_node.total_cost = current_nodes.total_cost
            print("Goal Node found")
            return 1, Nodes_list, Path_list
        
        # Add the popped node to the closed nodes dictionary
        if current_id in closed_node:  
            continue
        else:
            closed_node[current_id] = current_nodes
        
        del open_node[current_id]
        
        # Loop through all the possible moves
        for move in moves:
            action = plot_curve(current_nodes.x, current_nodes.y, current_nodes.current_theta, move[0], move[1],
                            clearance, 0, Nodes_list, Path_list)
           
            # Check if the move is valid
            if (action != None):
                angle = action[2]
                
                # Round off the coordinates and the angle to nearest integer
                theta_lim = 30
                x = (round(action[0] * 10) / 10)
                y = (round(action[1] * 10) / 10)
                theta = (round(angle / theta_lim) * theta_lim)
                
                # Calculate the new orientation and the cost to move to the new node
                current_theta = current_nodes.change_theta - theta
                c2g = dist((x,y), (goal_node.x, goal_node.y))
                new_node = Node(x, y, current_nodes, theta, current_theta, move[0], move[1], current_nodes.c2c+action[3], c2g, current_nodes.c2c+action[3]+c2g)

                new_node_id = key(new_node)
                
                # Check if the new node is valid and has not already been visited
                if not valid_move(new_node.x, new_node.y, radius, clearance):
                    continue
                elif new_node_id in closed_node:
                    continue

                # Update the node information if it already exists in the open list
                if new_node_id in open_node:
                    if new_node.total_cost < open_node[new_node_id].total_cost:
                        open_node[new_node_id].total_cost = new_node.total_cost
                        open_node[new_node_id].parent = new_node

                # Add the new node to the open list if it doesn't already exist        
                else:
                    open_node[new_node_id] = new_node
                    heapq.heappush(priority_list, [ open_node[new_node_id].total_cost, open_node[new_node_id]])
            
    return 0, Nodes_list, Path_list


def obstacle_space_check(x, y, radius, clearance):
    total_space = radius + clearance

    obstacle1 = ((np.square(x - 4000)) + (np.square(y - 1100)) <= np.square(500 + total_space))
    obstacle2 = (x >= 1500 - total_space) and (x <= 1625 + total_space) and (y >= 750 - total_space)
    obstacle3 = (x >= 2500 - total_space) and (x <= 2625 + total_space) and (y <= 1250 + total_space)
 
    border1 = (x <= 0 + total_space)     
    border2 = (x >= 5990 - total_space)
    border3 = (y <= 0 + total_space)
    border4 = (y >= 1990 - total_space)

    if obstacle1 or obstacle2 or obstacle3 or border1 or border2 or border3 or border4:
        return True
    else:
        return False
    

def valid_move(x, y, r, c):
    if obstacle_space_check(x, y, r, c):
        return False
    else:
        return True


def check_goal(current, goal):
    dt = dist((current.x, current.y), (goal.x, goal.y))

    if dt < 100:
        return True
    else:
        return False


def valid_orientation(theta):
    if theta==0 or theta ==30 or theta==60 or theta==90 :
        return theta
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
    r = 40
    L = 16
    linear_x = []
    angular_z = []
    for i in range(len(rpm1)):
        linear_x.append((r/2) * ((rpm1[i]*(2*math.pi/60))+(rpm2[i]*(2*math.pi/60))*(math.cos(theta[i]*(math.pi/180)))))
        angular_z.append((r/L) * ((rpm1[i]*(2*math.pi/60))-(rpm2[i]*(2*math.pi/60))))
    return linear_x, angular_z

def calculate_twist2(x_path, y_path):
        distance = []
        angle = []
        linear_x = []
        angular_z = []
        for i in range(len(x_path)):
            distance.append(math.sqrt(((x_path[i+1] - x_path[i]))**2 + ((y_path[i + 1] - y_path[i]))**2))
            linear_x.append(distance[i] / 10)  # Adjust linear velocity as needed
            angle.append(math.atan2(y_path[i + 1] - y_path[i], x_path[i + 1] - x_path[i]))
            angular_z.append(angle[i] / 2)  # Adjust angular velocity as needed
            return linear_x, angular_z, distance

def publish_twist_messages(linear_x, angular_z):
    rclpy.init()
    node = rclpy.create_node('publisher')

    publisher = node.create_publisher(Twist, 'cmd_vel', 10)
    msg = Twist()

    # Calculate the time interval based on the desired frequency
    interval = 1

    while rclpy.ok():
        start_time = time.time()

        for linear, angular in zip(linear_x, angular_z):
            msg.linear.x = linear
            msg.angular.z = angular
            publisher.publish(msg)
            node.get_logger().info("Published twist message: Linear=%.2f, Angular=%.2f" % (msg.linear.x, msg.angular.z))

            # Calculate the time elapsed since the start of the loop
            elapsed_time = time.time() - start_time

            # If there's remaining time in the interval, wait
            if elapsed_time < interval:
                time.sleep(interval - elapsed_time)

    rclpy.shutdown()       
    
def main():
    plotter = Plotter()
    robot_radius = 100

    clearance = float(input("Enter clearance of robot (in mm): "))
    print("Enter RPM in RPM")
    RPM1 = int(input("Enter High RPM: "))
    RPM2 = int(input("Enter Low RPM: "))

    start_x = float(input("Enter Start X coordinate (in mm): "))
    start_y = float(input("Enter start Y coordinate (in mm): "))

    goal_x = float(input("Enter X coordinate of Goal (in mm): "))
    goal_y = float(input("Enter Y coordinate of Goal (in mm): "))

    if not valid_move(start_x, start_y, robot_radius, clearance) or not valid_move(goal_x, goal_y, robot_radius, clearance):
        print("Invalid start or goal node, or in obstacle space")
        exit(-1)

    start_theta = int(input("Enter orientation of the robot at start node: "))
    if not valid_orientation(start_theta):
        print("Orientation has to be a multiple of 30")
        exit(-1)

    timer_start = time.time()

    c2g = dist((start_x, start_y), (goal_x, goal_y))
    total_cost = c2g

    start_node = Node(start_x, start_y, -1, start_theta, 0, 0, 0, 0, c2g, total_cost)
    goal_node = Node(goal_x, goal_y, -1, 0, 0, 0, 0, c2g, 0, total_cost)

    flag, Nodes_list, Path_list = Astar(start_node, goal_node, RPM1, RPM2, robot_radius, clearance)

    if flag == 1:
        x_path, y_path, theta_path, rpm1, rpm2 = back_track(goal_node)
    else:
        print("Path not found")
        exit(-1)

    plotter.plot_start_and_goal(start_node, goal_node) 
    def update(frame):
        plotter.update_plot(Nodes_list[:frame+1], Path_list)
    
    anim = FuncAnimation(plotter.figure, update, frames=len(Nodes_list), interval=50)
    plotter.axes.plot(x_path, y_path, '-r') 
    
    timer_stop = time.time()
    runtime = timer_stop - timer_start
    plt.show()
    
    print("Total Runtime: ", runtime)

    linear_x, angular_z = calculate_twist(rpm1, rpm2, theta_path)
    print(len(linear_x), len(angular_z), "Lengths from Calcualte Twist 1")
    print(linear_x, angular_z)
    linx, angz, distance = calculate_twist2(x_path, y_path)
    print(len(linx), len(angz), len(distance), "Lengths form Calculate Twist 2")
    print(linx, angz, distance)
    print(len(x_path), x_path)
    publish_twist_messages(linear_x, angular_z)

if __name__ == '__main__':
    main()
