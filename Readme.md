# LAB 0
## Wall Follower 
In the next session, you will be working on Wall Follower.

### Task: 

- Revolve Around a given Box 
- Start and End Location will be the same
- Code Should be Independent of upon Box Size 
- Teleop is not allowed 



### Resources:
Simulation:

Gazebo:

```
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_gazebo empty_world.launch.py 

```
RVIZ:

In new Terminal

```
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_bringup rviz2.launch.py 

```
### Helper Code:

A helper code is given in wall_follower package

Installing Git 

```
sudo apt install git-all
```


Download Using
```
cd <your ros_ws>/src
git clone git@github.com:nvnmangla/Maze-Solver-450.git
cd <your ros_ws>
colcon build 
```
Running Node ( Modify as Needed )

```
ros2 run wall_follower follow 

```