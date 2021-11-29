# EEE935 - Tópicos Especiais em Sinais e Sistemas: Planejamento de Movimento de Robôs

## Table of Contents

- [Usage](#usage)
  - [Tangent Bug](#tangent-bug)
  - [Path Following](#path-following)
  - [Potential Function](#potential-function)
  - [Wave-Front](#wave-front)
  
## Usage

### Tangent Bug

### Path Following

### Potential Function

### Wave-Front

The [Wave-Front](./catkin_ws/src/wavefront/) algorithm depends on a pre-made grid that corresponds to the configuration space of a map. This grid is a binary matrix, where 0s correspond to free space and 1s correspond to obstacles. It then computes the planner grid as in *Choset, Lynch and Hutchinson*[^1], using 4 or 8 point connectivity. Robot motion is then based on the generated planner grid.

By default it is assumed that the map to be used is [maze.world](./catkin_ws/src/wavefront/worlds/maze.world) and its respective configuration space grid [grid1.npy](./catkin_ws/src/wavefront/worlds/grid1.npy). If any other map is to be used, the user must edit [wavefront.py](./catkin_ws/src/wavefront/scripts/wavefront.py) with the respective Stage parameters and configuration space grid (.npy file). If the launch file is to be used, the user should change its .world file.

To create configuration space grids for other maps, you can use the [map_expander.py](./catkin_ws/src/wavefront/scripts/map_expander.py) script changing the parameters as necessary.

The robot's initial position **must** be set on `wavefront.py`. When this algorithm is run with the default map the *green* square represents the robot and the *orange* square represents the goal at default position, i.e. (5, 2).

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
rosrun stage_ros stageros worlds/maze.world 
```

In another terminal run:

```zsh
rosrun wavefront wavefront.py x y n 
```

where `x` is the goal x-coordinate, `y` is the goal y-coordinate and `n` the number of neighbors.

[^1]: Howie Choset, K. M. Lynch, and S. Hutchinson, Principles of robot motion: theory, algoritms, and implementations. Cambridge, Mass. Bradford, 2005.
‌
