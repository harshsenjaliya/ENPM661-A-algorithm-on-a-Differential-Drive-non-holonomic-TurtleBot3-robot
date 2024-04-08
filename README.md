# ENPM661-A-algorithm-on-a-Differential-Drive-non-holonomic-TurtleBot3-robot

## Part 1: 2D Implementation

### Description
This part of the project implements the A* search algorithm for path planning on a 2D plane for a differential drive non-holonomic TurtleBot3 robot. The implementation is done in Python.

### Dependencies
- numpy
- math
- matplotlib
- heapq

### Instructions
To run the 2D implementation, follow these steps:
1. Clone the repository:
'''bash git clone https://github.com/

2. Navigate to the directory where the repository is cloned.
'''css cd Path-Planning-for-a-differential-drive-robot-using-A-Star-Search-Algorithm

3. Run the Python script 'Part01.py' with the following command:
python3 Phase1.py

4. Follow the prompts in the terminal to input the required parameters.

5. The path will be visualized in a plot after a few seconds.


## Part 2: 2D Implementation
### Instructions
To run the Gazebo visualization, follow these steps:

1. Copy and paste the a_star_turtlebot package into the src folder of your Catkin workspace.
2. Run the following commands in the terminal:
   '''bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
roslaunch a_star_turtlebot proj.launch

3. Follow the prompts in the terminal to input the required parameters.
4. The robot will start moving towards the goal node.
5. To exit Gazebo, press 'Ctrl+C' in the terminal.
   
