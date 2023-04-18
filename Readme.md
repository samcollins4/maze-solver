
# LAB 1: Wall Follower
- Revolve Around a given Box 
- Start and End Location will be the same
- Code Should be Independent of upon Box Size 
- Teleop is not allowed 


## Steps for lab 1:
It is a good idea to make sure your code is working before applying motion.

 1. Print the range of the laser scan and figure out the array index you want to use to detect the obstacles. 
 2. Create an algorithm for wall following using the laser scan. (Reference for the algorithm [Wall Follower Algorithm](https://www.theconstructsim.com/wall-follower-algorithm/), You don't have to specifically implement this, this is just for your reference.)
 3. Verify algorithm on turtlebot using print statements and manually moving objects around the robot.
 4. Finally, integrate motion in your algorithm.

### Run Code
#### 1. Installing Git 
```
sudo apt install git-all
```
#### 2. Clone Package

    cd ./ros_ws/src
    git clone https://github.com/nvnmangla/Maze-Solver-450.git

#### 3. Build Package
```
cd ./ros_ws
colcon build
```

#### 4. Run Node ( Modify as Needed )
```
ros2 run wall_follower follow 
```


### Visualization in RVIZ
Use this to observe what the robot is picking up in it's surrounding
In new terminal/tab,
```
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_bringup rviz2.launch.py 
```

### Simulation on Gazebo
This can be used when you don't have the physical robot with you. You can use this to continue to work on your algorithm

In new terminal/tab,
```
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_gazebo empty_world.launch.py 
```


## Start and Stop (Only for starter code)
- Move Forward 'm'
- Move Back 'b'
- Stop 'Any other Key'
