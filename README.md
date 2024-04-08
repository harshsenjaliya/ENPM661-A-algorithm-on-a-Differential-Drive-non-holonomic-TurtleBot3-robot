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
```git clone https://github.com/harshsenaliya4433/ENPM661-A-algorithm-on-a-Differential-Drive-non-holonomic-TurtleBot3-robot.git```

3. Navigate to the directory where the repository is cloned.
```cd Path-Planning-for-a-differential-drive-robot-using-A-Star-Search-Algorithm```

4. Run the Python script 'Part01.py' with the following command:
```python3 Proj3_phase2_part1_Dhairya_Harsh.py```

5. Follow the prompts in the terminal to input the required parameters.

6. The path will be visualized in a plot after a few seconds.


## Part 2: 2D Implementation
### Instructions
To run the Gazebo visualization, follow these steps:

1. Copy and paste the a_star_turtlebot package into the src folder of your Catkin workspace.
2. Run the following commands in the terminal:
Create a workpace
```mkdir -p project3_ws/src```
```cd ~/project3_ws/src```

Clone the reposiory
```git clone https://github.com/harshsenaliya4433/ENPM661-A-algorithm-on-a-Differential-Drive-non-holonomic-TurtleBot3-robot.git```

Source ROS 
```source /opt/ros/galactic/setup.bash```

Build the workspace
```cd ~\project3_ws```
```colcon build --packages-select turtlebot3_project3```

Source ROS (Package will be identified)
```source install/setup.bash```

Run Code to Test
Launch Environment
```ros2 launch turtlebot3_project3 competition_world.launch.py```

3. Opem the new terminal window and Source workspace using command:
```source install/setup.bash```
4. Execute following command:
```ros2 run turtlebot3_project3 Proj3_phase2_part2_Dhairya_Harsh.py```
5. After closing the plot simulation in Gazebo will begin. 
6. To exit Gazebo, press 'Ctrl+C' in the terminal.
   
