# Occupany grid generation

Mobile robotics is often associated to autonomous navigation. In many case you need to generate your own map, called occupancy map or costmap, on which your robot will navigate. This tutorial explains how to generate such a map with RTABmap using the zed data. It uses the zedbot_global_costmap package of the Zedbot repository (no need of a Zedbot for this tutorial).
 You will be explained how to build to kind of cost map: a static costmap or a dynamic costmap. A static costmap is generated before the robot navigation and need to be saved somewhere. A dynamic costmap is generating during the navigation, the main advantage is that it is much more resilient to mobile obstacles and robot's drift, but it require a strong computation power.  

![](../../readme_images/tutorial_static_costmap.png)

This tutorial is divided in two part:
- Static costmap generation
	- SVO file generation
	- Static costmap construction using RTABmap

- Dynamic costmap

##Requirements
You need ROS installed on your computer (this tuto has been tested with ROS melodic).
You need the ZED SDK and the ZED ROS wrapper properly installed.
Clone the [zedbot repository]() in your ROS workspace.


## Static costmap
### SVO file generation
An SVO file is a file used to reccord all the data from the ZED during a choosen time. It allows you to re-run an algorithme using each time the same data. 
The first part of this tutorial consists in creating such a SVO file, in which you will record "a video" that capture the environment you plan to map.

How to generate a SVO:
- To start recording your SVO

```bash
$ cd /usr/local/zed/tools/
$ ./ZED\ Explorer 
```

From here you can record your environment to be mapped. You need to simulate a robot behaviour:
- always keep same height
- always keep same inclinaison
A good way to do is to record directly from your robot. You can also put your ZED and your PC on a rolling stuff (chair/ table ...) 

Please save your file in .../zedbot/zedbot_demo/zedbot_global_costmap/svo

### Static costmap construction using RTABmap

Run 
```bash
$ roslaunch zedbot_global_costmap zed_rtabmap.launch svo_file:=YOUR_SVO_PATH/FILE_NAME.svo
```

Or modify svo_file parameter in zed_rtabmap.launch and run:
 ```bash
$ roslaunch zedbot_global_costmap zed_rtabmap.launch
```




### Record my map
```bash
rosrun map_server map_saver map:=/zed/grid_prob_map -f ~/map
```


### Next steps

You now have a global cost map that can be used for navigation. You can read our Navigation tutorial to learn how to use the ROS navigation stack with the ZED.


## Dynamique costmap

Run (make sure the svo_file param is empty):
```bash
$ roslaunch zedbot_global_costmap zed_rtabmap.launch
```

Several costmap are published on:
-/zed/proj_map
-/zed/scan_map
-/zed/map

## RTABmap Tunning
To get a costmap as accurate as possible you need to modify some parameters. You can do that by editing rtabmap.yaml, common.yaml and zed.yaml. Some of them are discribed her:

zed.yaml parameters:
	- max_depth and min_depth defined the max and min distance from the ZED where pixels are libelized with a depth. Do not chose a max_depth to high to limit noise (8m is a good setting). Set  as small as possible (0.3m). RTABmap tend to clear the unknown area around the camera (between 0 and min_depth). This behaviour results in the clearing of actual obstacles that are not seen by the camera because to closed.

RTABmap parameters:

- MinGroundHeight and MaxGroundHeight define the interval where the ground is relative to the camera. The camera is at altitude 0 so  MinGroundHeight and MaxGroundHeight should be negative.
    
- MaxObstacleHeight: This parameter is used to clear the objects that are higher than your robot (the robot could go under).  MaxObstacleHeight must be greater than your robot's hight (your robot's hight + 0.3 m for instance). 

- FootprintHeight, FootprintLength and FootprintWidth define the dimensions of your robot

