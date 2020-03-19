# FASTER: Fast and Safe Trajectory Planner for Flights in Unknown Environments #

Code used for the paper **FASTER: Fast and Safe Trajectory Planner for Flights in Unknown Environments** (IROS 2019) ([pdf](https://arxiv.org/abs/1903.03558), [video](https://www.youtube.com/watch?v=gwV0YRs5IWs))

```
@inproceedings{tordesillas2019faster,
  title={{FASTER}: Fast and Safe Trajectory Planner for Flights in Unknown Environments},
  author={Tordesillas, Jesus and Lopez, Brett T and How, Jonathan P},
  booktitle={2019 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  year={2019},
  organization={IEEE}
}

```

UAV               |  Ground Robot           | 
:-------------------------:|:-------------------------:|
![](./faster/imgs/uav_sim.gif)       |  ![](./faster/imgs/gr_sim.gif)  |  
:-------------------------:|:-------------------------:|
![](./faster/imgs/uav_hw.gif)       |  ![](./faster/imgs/gr_hw.gif)  |  


## General Setup
Install [Gurobi](https://www.gurobi.com/) (you can test your installation typing `gurobi.sh` in the terminal).

Create a workspace, clone this repo and its dependencies, and compile the workspace:
```
mkdir ws && cd ws && mkdir src && cd src
git clone https://github.com/mit-acl/faster.git
wstool init
wstool merge ./faster/faster/install/faster.rosinstall

```

### Instructions to use FASTER with an aerial robot:

Compile the code:

```
wstool update -j8
cd ..
catkin config -DCMAKE_BUILD_TYPE=Release
catkin build
```

And finally open 5 terminals and execute these commands:
```
roslaunch acl_sim start_world.launch
roslaunch acl_sim perfect_tracker_and_sim.launch
roslaunch global_mapper_ros global_mapper_node.launch
roslaunch faster faster_interface.launch
roslaunch faster faster.launch
```
Now you can click `Takeoff` in the GUI, and then, in RVIZ, press `G` (or click the option `2D Nav Goal` on the top bar of RVIZ) and click any goal for the drone. 

### Instructions to use FASTER with a ground robot:

Install the following dependencies:
```
sudo apt-get install ros-kinetic-robot-localization ros-kinetic-lms1xx ros-kinetic-interactive-marker-twist-server ros-kinetic-hector-gazebo-plugins ros-kinetic-move-base ros-kinetic-ros-control ros-kinetic-ros-controllers ros-kinetic-pointgrey-camera-description ros-kinetic-hardware-interface ros-kinetic-message-to-tf ros-kinetic-gazebo-ros-control
```
Then download the ground_robot-specific packages and compile the repo:

```
wstool merge ./faster/faster/install/faster_ground_robot.rosinstall
wstool update -j8
cd ..
catkin config -DCMAKE_BUILD_TYPE=Release
catkin build
```

Then, in `faster.yaml`, change these parameters:
```
drone_radius: 0.5

v_max: 1.4   #[m/s]  
a_max: 1.4   #[m/s2] 
j_max: 5.0   #[m/s3]

is_ground_robot: true  
```

And finally open 4 terminals and execute these commands
```
roslaunch faster ground_robot.launch
roslaunch global_mapper_ros global_mapper_node.launch quad:=JA01
roslaunch faster faster_interface.launch quad:=JA01 is_ground_robot:=true
roslaunch faster faster.launch quad:=JA01
```

Now you can click `Takeoff` in the GUI, and then, in RVIZ, press `G` (or click the option `2D Nav Goal` on the top bar of RVIZ) and click any goal for the ground robot. 



## Architecture:


![](./faster/imgs/diagram.png) 

Note that for the case of a ground robot, a real simulation is provided (using the `multi_jackal` package)


## Credits:
This package uses code from the [JPS3D](https://github.com/KumarRobotics/jps3d) and [DecompROS](https://github.com/sikang/DecompROS) repos (included in the `thirdparty` folder), so credit to them as well. 





