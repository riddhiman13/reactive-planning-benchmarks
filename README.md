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

Scene initialisation also includes the planning pipeline initialisation i.e. (OMPL, STOMP, CHOMP). To start a specific pipeline, launch its corresponding launch file:

<div align="center">
  
|Pipeline     | Launch File                     |
|:-----------:|:-------------------------------:|
|OMPL         |ompl_create_scene.launch         |
|STOMP        |stomp_create_scene.launch        |
|CHOMP        |chomp_create_scene.launch        |

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
This package is responsible for planning a collision-free trajectory given the robot, the scene obstacles and the start and goal configuration based on the [OMPL planners](https://ompl.kavrakilab.org/planners.html), [STOMP Planner](https://wiki.ros.org/stomp_motion_planner) and [CHOMP Planner](https://wiki.ros.org/chomp_motion_planner)

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
|CHOMP        |CHOMP.launch    |

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

## Important Notes:
### Note 1:
In case CHOMP fails on almost every plan please add the following line in the [chomp_planning.yaml](https://github.com/moveit/panda_moveit_config/blob/noetic-devel/config/chomp_planning.yaml) file in the [panda_moveit_config](https://github.com/moveit/panda_moveit_config) package.
```yaml
collision_checker: FCL
```
This changes the hybrid collision checking method with the FCL method. Please note that CHOMP isn't perfectly tuned for clustered environments such as the given [example_scene](scene_creation/config/example_scene.yaml) and some intense tuning might be needed to have a functional trajectory. This is still an open issue in Moveit ROS1 as discussed [here](https://github.com/moveit/moveit/issues/305).

----

## Next Steps:
1. Update scene parsing to visualize dynamic obstacles
2. Implement an independent package for metrics evaluation
