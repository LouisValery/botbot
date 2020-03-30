# Zedbot navigation tutorial

This package provides the params and launch files needed for the navigation tutorial. At the end of this tutorial you will be able to generate a dynamic costmap on which your robot will navigate autonomously.

This tutorial uses the zed camera associated to the RTABmap package to generate a global costmap. It also uses the ROS navigation stack (mainly move_base) to generate a path to reach the required target on the map. The ZED data are also used to generate a local costmap in order to perform obstacle detection. A local path, as close as possible from the global path, is generated to avoid the new obstacles.

## Requirements
You must have successfuly followed the ["From Turtlebot3 Burger to Zedbot"]() tutorial. 
Make sure you are able to compile your ROS workspace and to start your ZED.

## Tutorial

(Remote PC) 
```bash
$ roscore
```

(Jetson nano) 
```bash
$ roslaunch zedbot_navigation main.launch camera_model:=zed
```

(Remote nano) 
```bash
$ roslaunch zedbot_navigation rviz.launch
```

## Debug tools and complementary informations

This tutorial uses basicly tree main nodes: zed_node, rtabmap and move_base.

- zed_node publishes on several topics the ZED information (images, pointcloud, odom...)

- RTABmap generates a costmap thanks to the information from the ZED node (image rgb, image depth, camera information and odometry). RTABmap can generte its own odometry but we prefere to use the ZED odometry insofar as it has been computed for this specific sensor, so is more accurate. 

- move_base is the main node of the ROS navigation stack. Its purpose is to make your robot reach the given taget. It generates a path to be followed thanks to the RTABmap cost map. This path is named global_path. It also uses a local costmap directly generated from the ZED pointcloud in order to avoid the dynamic obstacles of its environment (walking people, other robots ...). 
![](../../readme_images/navigation_schema.png)


### Tf tree
The TF tree shows you who send tf information and how. In our case,  zed_node generates the TF between /map and /odom and between  /odom and /base_footprint.
Note that the odom (tf between /odom and /base_footprint) is provided by both the ZED and the turtlebot. As the ZED odom is more reliable, the tf information published by turtlebot3_core as been remapped in order to not take turtlebot's odom into account.


![](../../readme_images/tutorial_navigation_tf_tree.png)

### Node Graphe

This is the node Graphe you should obtain from RQT. It is closed to the general structure schema, but a bit more specific. It is a good debug tool if your project does not use a Turtlebot but an other robot. In that case the turtlebot_core node would be replaced by a similare one, able to send command to the wheels and to publish some tf (wheels position thanks to the encoders for instance).
![](../../readme_images/tutorial_navigation_node_graphe.png)

### RTABmap Tunning
To get a costmap as accurate as possible you need to modify some parameters. You can do that by editing rtabmap.yaml, common.yaml and zed.yaml. Some of them are discribed her:

zed.yaml parameters:

- max_depth and min_depth defined the max and min distance from the ZED where pixels are libelized with a depth. Do not chose a max_depth to high to limit noise (8m is a good setting). Set  as small as possible (0.3m). RTABmap tend to clear the unknown area around the camera (between 0 and min_depth). This behaviour results in the clearing of actual obstacles that are not seen by the camera because to closed.

RTABmap parameters:

- MinGroundHeight and MaxGroundHeight define the interval where the ground is relative to the camera. The camera is at altitude 0 so  MinGroundHeight and MaxGroundHeight should be negative.
    
- MaxObstacleHeight: This parameter is used to clear the objects that are higher than your robot (the robot could go under). MaxObstacleHeight must be greater than your robot's hight (your robot's hight + 0.3 m for instance). 

- FootprintHeight, FootprintLength and FootprintWidth define the dimensions of your robot

