## Instructions
Install [Gurobi](https://www.gurobi.com/) (you can test your installation typing `gurobi.sh` in the terminal).

Create a workspace, clone this repo and its dependencies, and compile the workspace:
```
mkdir ws && cd ws && mkdir src && cd src
git clone https://github.com/mit-acl/faster.git
wstool init
wstool merge ./faster/faster/install/faster.rosinstall
wstool update -j8
cd ..
catkin config -DCMAKE_BUILD_TYPE=Release
catkin build
```

And finally open 5 terminals and execute these commands:
```
roslaunch acl_sim start_world.launch
roslaunch faster perfect_tracker_and_sim.launch
roslaunch global_mapper_ros global_mapper_node.launch
roslaunch faster faster_interface.launch
roslaunch faster faster.launch
```
Now you can click `Takeoff` in the GUI, and then, in RVIZ, press `G` (or click the option `2D Nav Goal` on the top bar of RVIZ) and click any goal for the drone. 

## Architecture:







