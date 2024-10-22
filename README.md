# ENPM661 A*-algorithm on a Differential Drive Non-Holonomic TurtleBot3 Robot

This repository implements the A* search algorithm for path planning on a differential drive, non-holonomic TurtleBot3 robot. The project is divided into two parts: the first part handles the 2D implementation of the A* algorithm using Python, while the second part visualizes the path in Gazebo using ROS2.

## Table of Contents
- [Part 1: 2D Implementation](#part-1-2d-implementation)
  - [Description](#description)
  - [Dependencies](#dependencies)
  - [Instructions](#instructions)
  - [Sample Input](#sample-input)
- [Part 2: Gazebo Visualization](#part-2-gazebo-visualization)
  - [Dependencies](#dependencies-1)
  - [Instructions](#instructions-1)
  - [Sample Input](#sample-input-1)
- [Output](#output)
  - [Part 1 Video](#part-1-video)
  - [Part 2 Video](#part-2-video)

---

## Part 1: 2D Implementation

### Description
This part of the project implements the A* algorithm for path planning on a 2D plane for a differential drive, non-holonomic TurtleBot3 robot. The algorithm computes the optimal path from a start point to a goal point, taking into account the robot's configuration and environment constraints.

### Dependencies
- `numpy`
- `math`
- `matplotlib`
- `heapq`

### Instructions

To run the 2D implementation, follow these steps:

1. Clone the repository:
   `git clone https://github.com/harshsenaliya4433/ENPM661-A-algorithm-on-a-Differential-Drive-non-holonomic-TurtleBot3-robot.git`

2. Navigate to the project directory:
   `cd ENPM661-A-algorithm-on-a-Differential-Drive-non-holonomic-TurtleBot3-robot`

3. Run the Python script:
   `python3 Proj3_phase2_part1_Dhairya_Harsh.py`

4. Follow the terminal prompts to input the required parameters such as the start and goal coordinates, robot clearance, and RPM values.

5. The robot’s path will be visualized using `matplotlib` in a 2D plot.

### Sample Input
`start_x = 500`
`start_y = 1000`
`clearance = 5`
`goal_x = 5750`
`goal_y = 1000`
`rpm1 = 10`
`rpm2 = 5`
`goal_orientation = 0`

---

## Part 2: Gazebo Visualization

### Description
This part visualizes the path planning results from Part 1 using Gazebo in ROS2. The TurtleBot3 robot navigates the environment based on the path computed by the A* algorithm.

### Dependencies
- ROS2 (Galactic or later)
- TurtleBot3 packages
- Gazebo

### Instructions

To run the Gazebo visualization, follow these steps:

1. Set up your ROS2 Catkin workspace:
   `mkdir -p ~/project3_ws/src`
   `cd ~/project3_ws/src`

2. Clone the repository:
   `git clone https://github.com/harshsenaliya4433/ENPM661-A-algorithm-on-a-Differential-Drive-non-holonomic-TurtleBot3-robot.git`

3. Source ROS2:
   `source /opt/ros/galactic/setup.bash`

4. Build the workspace:
   `cd ~/project3_ws`
   `colcon build --packages-select turtlebot3_project3`

5. Source the workspace to ensure ROS2 recognizes the package:
   `source install/setup.bash`

6. Launch the Gazebo environment:
   `ros2 launch turtlebot3_project3 competition_world.launch.py`

7. In a new terminal window, source the workspace again:
   `source install/setup.bash`

8. Run the script to initiate the simulation:
   `ros2 run turtlebot3_project3 Proj3_phase2_part2_Dhairya_Harsh.py`

### Sample Input
`goal_x = 5750`
`goal_y = 1000`

After closing the 2D plot, the simulation in Gazebo will begin. The robot will follow the planned path.

To exit Gazebo, press `Ctrl+C` in the terminal.

---

## Output

### Part 1 Video
Watch the video demo for the 2D implementation:
[![Part 1 Video](https://img.youtube.com/vi/BYHxcvdin9k/maxresdefault.jpg)](https://youtu.be/BYHxcvdin9k)

### Part 2 Video
Watch the video demo for the Gazebo visualization:
[![Part 2 Video](https://img.youtube.com/vi/RQ4x9f2U4tc/maxresdefault.jpg)](https://youtu.be/RQ4x9f2U4tc)

---

Feel free to reach out if you encounter any issues or have questions regarding the project.
