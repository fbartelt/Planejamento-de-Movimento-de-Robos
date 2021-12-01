# EEE935 - Tópicos Especiais em Sinais e Sistemas: Planejamento de Movimento de Robôs

## Contents

- [Overview](#overview)
- [Usage](#usage)
  - [Tangent Bug](#tangent-bug)
  - [Path Following](#path-following)
  - [Potential Function](#potential-function)
  - [Wave-Front](#wave-front)
- [References](#references)

## Overview

This repository is just a compilation of each assignment from the 2021/2 EEE935 class. It includes the following motion planning algorithms using ROS + Python: [Tangent Bug](./catkin_ws/src/tangent_bug/), a [Path Following](./catkin_ws/src/path_following/), a simple [Potential Function](./catkin_ws/src/potential_function/) and [Wave-Front](./catkin_ws/src/wavefront/). For each one of these, the robot is non-holonomic and a velocity controller is used with feedback linearization. Stage simulator is used for every algorithm. Every algorithm was implemented as given in *Choset, Lynch, and Hutchinson*[^1].

## Usage

The python scripts for Tangent Bug, Potential Function, and Wave-Front algorithms assume the existence of two robots in Stage simulator/.world file, the first, green square, robot being the robot to be controlled and the second, orange square, a representation of the goal. So if any other map is used and it does not have two objects in this specific order, then the rospy Publishers and Subscribers for each one of these python scripts should be changed.

Path Following and Wave-Front behavior will not change if only the map is changed as they have internal parameters in respect to the default map. The path followed by the first algorithm is defined in its python script and for Wave-Front there are some Stage parameters variables to be changed in its python script as well as its configuration space grid.

### Tangent Bug

### Path Following

By default it is assumed that the map is [empty.world](./catkin_ws/src/path_following/worlds/empty.world). If any other map is used, the user must edit its launch file, if you are not going to run it manually.

The default map implies that the path to be followed is a Rhodonea curve given with 6 petals.

#### Launch File

It's possible to run the [Path Following](./catkin_ws/src/path_following/) algorithm using its [launch file](./catkin_ws/src/path_following/launch/path_following.launch).

Simply run in terminal:

```zsh
roslaunch path_following path_following.launch
```

#### Running Manually

In one terminal run:

```zsh
roscd path_following
rosrun stage_ros stageros worlds/empty.world 
```

In another terminal run:

```zsh
rosrun path_following path_following.py
```

### Potential Function

By default it is assumed that the map is [maze.world](./catkin_ws/src/potential_function/worlds/maze.world). If any other map is used, the user must edit its launch file, if you are not going to run it manually.

Running with the default map and parameters implies that the goal, the orange square, is set to (5, 2) and the initial robot position is set to (-2, -4). When this goal is reached, it's possible to move the robot with the mouse and rerun the algorithm, such that the robot can converge to the *same* goal from a different initial position.

#### Launch File

It's possible to run the [Potential Function](./catkin_ws/src/potential_function/) algorithm using its [launch file](./catkin_ws/src/potential_function/launch/potential_function.launch). There is a optional argument `goal` that corresponds to the coordinates of the goal in meters as `"x_goal y_goal"`, by default `goal:="5 2"`.

To run *with default parameters*, simply run in terminal:

```zsh
roslaunch potential_function potential_function.launch
```

To run with *different parameters*:

```zsh
roslaunch potential_function potential_function.launch goal:="-6 -2"
```

#### Running Manually

In one terminal run:

```zsh
roscd potential_function
rosrun stage_ros stageros worlds/maze.world 
```

In another terminal run:

```zsh
rosrun potential_function potential_function.py x y n 
```

where `x` is the goal x-coordinate, `y` is the goal y-coordinate.

### Wave-Front

The [Wave-Front](./catkin_ws/src/wavefront/) algorithm depends on a pre-made grid that corresponds to the configuration space of a map. This grid is a binary matrix, where 0s correspond to free space and 1s correspond to obstacles. It then computes the planner grid as in *Choset, Lynch, and Hutchinson*[^1], using 4 or 8 point connectivity. Robot motion is then based on the generated planner grid.

By default it is assumed that the map is [maze.world](./catkin_ws/src/wavefront/worlds/maze.world) and its respective configuration space grid [grid1.npy](./catkin_ws/src/wavefront/worlds/grid1.npy). If any other map is used, the user must edit [wavefront.py](./catkin_ws/src/wavefront/scripts/wavefront.py) with the respective Stage parameters and configuration space grid (.npy file). If the launch file is used, the user should change its .world file.

To create configuration space grids for other maps, you can use the [map_expander.py](./catkin_ws/src/wavefront/scripts/map_expander.py) script changing the parameters as necessary.

The robot's initial position **must** be set on `wavefront.py`. When this algorithm is run with the default map the *green* square represents the robot and the *orange* square represents the goal at the default position, i.e. (5, 2).

#### Launch File

It's possible to run the [Wave-Front](./catkin_ws/src/wavefront/) planner using its [launch file](./catkin_ws/src/wavefront/launch/wavefront.launch). There are two optional arguments `goal` and `neighbors`. `goal` are the coordinates of the goal in meters as `"x_goal y_goal"`, and `neighbors` is the number of connectivity points to use when creating the Wave-Front grid, its value can be either **4** or **8** (if any other integer is passed, 8 is assumed).

The default arguments are `goal:="5 2"` and `neighbors:="4"`.

To run *with default parameters*, simply run in terminal:

```zsh
roslaunch wavefront wavefront.launch
```

To run with *different parameters*:

```zsh
roslaunch wavefront wavefront.launch goal:="-6 -2" neighbors:="8"
```

#### Running Manually

In one terminal run:

```zsh
roscd wavefront
rosrun stage_ros stageros worlds/maze.world 
```

In another terminal run:

```zsh
rosrun wavefront wavefront.py x y n 
```

where `x` is the goal x-coordinate, `y` is the goal y-coordinate, and `n` is the number of neighbors.

## References

[^1]: Howie Choset, K. M. Lynch, and S. Hutchinson, Principles of robot motion: theory, algorithms, and implementations. Cambridge, Mass. Bradford, 2005.
‌
