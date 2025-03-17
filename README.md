# Solve Maze - TurtleBot3 Maze Solving and Mapping

## Project Overview
This project uses **TurtleBot3 in the Gazebo simulation environment** to solve and map a maze. TurtleBot3 receives **laser scan measurements** and publishes **velocity commands** to navigate through the maze. Two primary functionalities are provided:
- **Maze Solving:** Wall Following algorithm for autonomous navigation.
- **Maze Mapping:** Real-time environment mapping using laser scan data.

Two nodes run concurrently:
- **Solver node (`my_solver`)**: Guides TurtleBot3 through the maze using Wall Following.
- **Mapper node (`my_mapper`)**: Generates a real-time map from laser scans using OpenCV.

## Installation and Setup

### 1. Clone Necessary Repositories
```bash
cd ~/robotlar_ws/src
git clone https://gitlab.com/blm6191_2425b_tai/blm6191/micromouse_maze.git
```

### 2. Set Environment Variables
```bash
cd ~/robotlar_ws
catkin_make
source ~/.bashrc

echo "export TURTLEBOT3_MODEL=waffle" >> ~/.bashrc
echo "export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:~/robotlar_ws/src/" >> ~/.bashrc
source ~/.bashrc
```

## Building the Solve Maze Package

### CMakeLists.txt Adjustments
Ensure your `CMakeLists.txt` contains:
```cmake
add_executable(my_solver src/solver.cpp)
target_link_libraries(my_solver ${catkin_LIBRARIES})

add_executable(my_mapper src/mapper.cpp)
target_link_libraries(my_mapper ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
```

### 3. Compile the Package
```bash
cd ~/robotlar_ws
catkin_make
source devel/setup.bash
```

## Running Both Nodes Concurrently

Create a launch file (`solver_mapper.launch`) in your package's `launch` folder:
```xml
<launch>
  <include file="$(find micromouse_maze)/launch/micromouse_maze4.launch" />

  <node pkg="solve_maze" type="my_solver" name="solver" output="screen" />
  <node pkg="solve_maze" type="my_mapper" name="mapper" output="screen" />
</launch>
```

### Launch Both Nodes
```bash
roslaunch solve_maze solver_mapper.launch
```

## Running Nodes Individually:

**Solver only:**
```bash
roslaunch micromouse_maze micromouse_maze4.launch
rosrun solve_maze my_solver
```

**Mapper only (manual control or joystick required):**
```bash
roslaunch micromouse_maze micromouse_maze4.launch
rosrun solve_maze my_mapper
```

## Adjusting Simulation Speed
You can speed up the Gazebo simulation **by 10 times** using:
```bash
gz physics -u 10000
```

## Camera Adjustment
Adjust Gazebo camera **relative to TurtleBot3** for clearer simulation visualization.

## Shutting Down the Simulation
To cleanly terminate all simulation processes:
```bash
rosnode kill -a
pkill -9 gzserver
pkill -9 gzclient
pkill -9 roscore
```

## Results
- **Solver:** Robot navigates the maze autonomously via Wall Following algorithm.
- **Mapper:** Real-time visualization and mapping of the maze environment.


---

This README addresses both **Question 1 (Maze Solving)** and **Question 2 (Maze Mapping)** requirements.

