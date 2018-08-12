# MRSL Jump Point Search Planning Library v1.0
[![wercker status](https://app.wercker.com/status/880ab5feaff25f0483e5f2c4f834b8c0/s/master "wercker status")](https://app.wercker.com/project/byKey/880ab5feaff25f0483e5f2c4f834b8c0)
- - -
Jump Point Search for path planning in both 2D and 3D environments. Original jump point seach algorithm is proposed in ["D. Harabor and A. Grastien. Online Graph Pruning for Pathfinding on Grid Maps. In National Conference on Artificial Intelligence (AAAI), 2011"](https://www.aaai.org/ocs/index.php/AAAI/AAAI11/paper/download/3761/4007). The 3D version is proposed in ["S. Liu, M. Watterson, K. Mohta, K. Sun, S. Bhattacharya, C.J. Taylor and V. Kumar. Planning Dynamically Feasible Trajectories for Quadrotors using Safe Flight Corridors in 3-D Complex Environments. ICRA 2017"](http://ieeexplore.ieee.org/abstract/document/7839930/).

The distance map planner (`DMPlanner`) is also included in this repo. `DMPlanner` inflate a artificial potential field around obstacles and plan a safer path within a certain region. More detials are refered in the latter section.

## Installation
#### Required:
 - Eigen3
 - yaml-cpp

Simply run following commands to install dependancy:
```bash
$ sudo apt update
$ sudo apt install -y libeigen3-dev libyaml-cpp-dev libboost-dev cmake
```

#### A) Simple cmake
```bash
$ mkdir build && cd build && cmake .. && make -j4
```

#### B) Using CATKIN
```bash
$ mv jps3d ~/catkin_ws/src
$ cd ~/catkin_ws & catkin_make_isolated -DCMAKE_BUILD_TYPE=Release
```

#### CTest
Run following command in the `build` folder for testing the executables:
```bash
$ make test
```

If everything works, you should see the results as:
```bash
Running tests...
Test project /home/sikang/thesis_ws/src/packages/jps3d/build
    Start 1: test_planner_2d
1/3 Test #1: test_planner_2d ..................   Passed    0.95 sec
    Start 2: test_planner_3d
2/3 Test #2: test_planner_3d ..................   Passed    0.00 sec
    Start 3: test_distance_map_planner_2d
3/3 Test #3: test_distance_map_planner_2d .....   Passed    1.26 sec

100% tests passed, 0 tests failed out of 3

Total Test time (real) =   2.22 sec
```

#### Include in other projects
Note that in other repository, add following commands in `CMakeLists.txt` in order to correctly link `jps3d`:
```sh
find_package(jps3d REQUIRED)
include_directories(${JPS3D_INCLUDE_DIRS})
...
add_executable(test_xxx src/test_xxx.cpp)
target_link_libraries(test_xxx ${JPS3D_LIBRARIES})
```

Two libs will be installed: the standard `jps_lib` and a variation `dmp_lib`.

## JPS Usage
To start a simple `JPS` planning thread:
```c++
JPSPlanner2D planner(false); // Declare a 2D planner
planner.setMapUtil(map_util); // Set collision checking function
planner.updateMap(); // Set map, must be called before plan
bool valid = planner.plan(start, goal, 1); // Plan from start to goal with heuristic weight 1, using JPS
```

First, the collision checking util must be loaded as:
```c++
planner.setMapUtil(MAP_UTIL_PTR); // Set collision checking function
```
The `MAP_UTIL_PTR` can be either `JPS::OCCMapUtil` for 2D or `JPS::VoxelMapUtil` for 3D. It can be confusing to set up this util, see the example code for more details.

Second, call the function `updateMap()` to allocate the internal map:
```
planner.updateMap(); // Set map, must be called before plan
```

Finally, call the function `plan` to plan a path from `start` to `goal`. The third input is the heuristic weight, the forth input indicates whether planning with `JPS` or `A*`.
By default, the forth input is set to be `true`, means the `JPS` is the default back-end. To use normal `A*`:
```c++
bool valid_astar = planner.plan(start, goal, 1, false); // Plan from start to goal with heuristic weight 1, using A*
```

Tow planners are provided for 2D and 3D map:
 - ```JPSPlanner2D```
 - ```JPSPlanner3D```

#### Planning Example
An example code for a 2D map is given in [`test/test_planner_2d.cpp`](https://github.com/sikang/jps3d/blob/master/test/test_planner_2d.cpp),
in which we plan from start to goal using both ```A*``` and ```JPS```.
The results are plotted in [corridor.png](https://github.com/sikang/jps3d/blob/master/data/corridor.png).
Green path is from ```A*```, red path is from ```JPS```. Even though they are using two different routes and `JPS` is much faster, the distance/cost of two paths is the same.
In other words, `JPS` guarantees the optimality but saves a significant amount of computation time.

![Visualization](./data/corridor.png)
```bash
$ ./build/test_planner_2d ../data/corridor.yaml
start: 2.5  -2
goal:  35 2.5
origin:  0 -5
dim: 799 199
resolution: 0.05
JPS Planner takes: 5.000000 ms
JPS Path Distance: 35.109545
JPS Planner takes: 5.000000 ms
AStar Planner takes: 62.000000 ms
AStar Path Distance: 35.109545
```

An example in 3D map is presented in [`test/test_planner_3d.cpp`](https://github.com/sikang/jps3d/blob/master/test/test_planner_3d.cpp) with the yaml `data/simple3d.yaml`.

##### Mapping Example
To generate map in `yaml` format which can be loaded directly in the test node, a simple executable file [`test/create_map.cpp`](https://github.com/sikang/jps3d/blob/master/test/create_map.cpp) is used.
User can easily change the location of blocks in the source code.

## DMP Usage
As mentioned before, `DMPlanner` stands for distance map planner which utilizes the artificial potential field to find a safer local path around a given path for the robot to navigate.
The key feature of this planner is its ability to push the path away from obstacles as much as possible. An example is given in the following figure
[example_dmp.png](https://github.com/sikang/jps3d/blob/master/data/example_dmp.png), where red path comes from `JPS` which is always attached to obstacles and blue path is derived from `DMP` which is much safer.

![Visualization](./data/example_dmp.png)

The code for generating this figure is given in [`test/test_distance_map_2d.cpp`](https://github.com/sikang/jps3d/blob/master/test/test_distance_map_planner_2d.cpp).
```bash
$ ./build/test_distance_map_planner_2d ../data/corridor.yaml
start: 2.5  -2
goal:  35 2.5
origin:  0 -5
dim: 799 199
resolution: 0.05
JPS Planner takes: 7.000000 ms
JPS Path Distance: 35.109545
DMP Planner takes: 104.000000 ms
DMP Path Distance: 37.062964
```

## Doxygen
For more details, please refer to [Doxygen](https://sikang.github.io/jps3d).

