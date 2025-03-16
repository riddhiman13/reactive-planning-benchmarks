# Reactive Planning Benchmarks

## Requirements:
These nodes are developed and tested on Ubuntu 20.04.6 (Focal Fossa), ROS Noetic and dependent on [panda_moveit_config](https://github.com/moveit/panda_moveit_config) package.

## Installation:
Clone both packages to your workspace and then build them using:
```
catkin build
```


## Packages Description:

### [scene_creation](scene_creation):
This package is responsible for loading the robot embodiment (in this case, only the Franka Panda; future updates will include other embodiments) and scene obstacles using a [yaml file](scene_creation/config/example_scene.yaml) (currently, static obstacles only are parsed).

Scene initialisation also includes the planning pipeline initialisation i.e (OMPL, STOMP, ..., etc). To start a specific pipeline, launch its corresponding launch file:

<div align="center">
  
|Pipeline     | Launch File                     |
|:-----------:|:-------------------------------:|
|OMPL         |ompl_create_scene.launch         |
|STOMP        |stomp_create_scene.launch        |

</div>

**For Example:**
```
roslaunch scene_creation ompl_create_scene.launch
```

----

The scene obstacles yaml file is passed in a parameter in the last line of the corresponding **create_scene** launch file, as shown. You can change the scene by specifying which file path you want. Please make sure to keep the same structure of the obstacle description to have proper yaml parsing.

```xml
<!-- Select the path to the wanted scene -->
<param name="scene_file" type="string" value="$(find scene_creation)/config/example_scene.yaml" />
```
----
#### Important Note:
***Scene files also include the start and goal position of the motion planning algorithm.***

----

### [planners](planners):
This package is responsible for planning a collision-free trajectory given the robot, the scene obstacles and the start and goal configuration based on the [OMPL planners](https://ompl.kavrakilab.org/planners.html) and [STOMP Planner](https://wiki.ros.org/stomp_motion_planner)

Currently developed planners are:
<div align="center">
  
|Planner      | Launch File       |
|:-----------:|:-----------------:|
|RRT          |RRT.launch         |
|RRT Connect  |RRTConnect.launch |
|RRT Star     |RRTstar.launch    |
|PRM          |PRM.launch         |
|Lazy PRM     |LazyPRM.launch     |
|EST          |EST.launch         |
|KPIECE       |KPIECE.launch      |
|BKPIECE      |BKPIECE.launch     |
|LBKPIECE     |LBKPIECE.launch    |
|STOMP        |STOMP.launch    |

</div>

----
To launch a specific planner use the following command after sourcing your workspace:
```
roslaunch planners <your launch file>
```
**For example**
```
roslaunch planners RRT.launch
```

----
## Next Steps:
1. Implement CHOMP and Motion planning in microseconds.
2. Update scene parsing to visualize dynamic obstacles
3. Implement an independent package for metrics evaluation
