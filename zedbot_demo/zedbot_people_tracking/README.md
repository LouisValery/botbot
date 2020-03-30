# Zedbot People Tracking tutorial (ZED2 only)

This tutorial shows you how to use the ZED2 camera in order to perform people tracking. Two solution are provided. The simpliest one detect someone to track and goes in his direction. The second one detect someone to track and send goals to move_base in order simultaneously reach the target and avoid potential obstacles on the path. Both of these solutions use the ZED object detection module to detect the target.

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
$ roslaunch zedbot_people_tracking main.launch
```

(Remote PC) 
```bash
$ roslaunch zedbot_people_tracking rviz.launch
```


## Debug tools and complementary informations
This tutorial uses basicly tree main nodes: zed_node, people_tracking and move_base.

- zed_node publishes on several topics the ZED information (images, pointcloud, odom...). In this tutorial, odom and object_detection are the most important topics.

- people_tracking subscribe to the object detection topic from zed_node (/zed/zed_node/obj_det/objects topic). It chose a target to be tracked and send either a goal to move_base (if tracking_with_move_base is set to true) or send velocity cmd to the /cmd_vel topic to go in target direction (if tracking_with_move_base is set to false). By default tracking_with_move_base = true. 

- (only if tracking_with_move_base = true). Move_base is the main node of the ROS navigation stack. Its purpose is to make your robot reach the given taget. It generates a global path and a local path in order to safely reach the goal that has been given by the people_tracking node. Move_base is supposed to obtain a global costmap and sensor data (pointcloud, laser scan) as input. Move_base build a local costmap from the data information (cf figure below). The global path is supposed to be computed from the global map, and the local path from the local costmap. In this tutorial the global costmap given to move_base is an empty map, meaning that no obstacle information are given to move_base to build the global path. Thus the global path will always be a straigth line between the robot and the target. The local path uses the ZED pointcloud to avoid the local obstacles on this line. 

<img src="/ros/images/tutorial_people_tracking_schema.png" style="max-width:1000px"/>

### Tf tree
The TF tree shows you who send tf information and how. In our case,  zed_node generates the TF between /map and /odom and between  /odom and /base_footprint.
Note that the odom (tf between /odom and /base_footprint) is provided by both the ZED and the turtlebot. As the ZED odom is more reliable, the tf information published by turtlebot3_core as been remapped in order to not take turtlebot's odom into account.


<img src="/ros/images/tutorial_people_tracking_tf_tree.png" style="max-width:1000px"/>

### Node Graphe

This is the node Graphe you should obtain from RQT. It is closed to the general structure schema, but a bit more specific. It is a good debug tool if your project does not use a Turtlebot but an other robot. In that case the turtlebot_core node would be replaced by a similare one, able to send command to the wheels and to publish some tf (wheels position thanks to the encoders for instance).
<img src="/ros/images/tutorial_people_tracking_node_graphe.png" style="max-width:1000px"/>


## The code explained
The people tracking tutorial is mainly based on the people_tracking node (people_tracking.cpp).

A subscriber subscribe to the object detection topic througth a callback function. When the node is started, the first action is to find a target. The chose_target function chose between the detected people the closest personn, in range [m_min_detection_distance, m_max_detection_distance]. When the target is chosen, it will be tracked by id (m_target_id).  

Each time the callback function is called, meaning each time object_detection topic is published by the zed_node, we look for m_target_id in the target list, and get its position relative to the camera (so the robot). The position is then used either to send a goal to move base (track_with_move_base function) or to go in target direction by commanding directly the motors througth the /cmd_vel topic (track_with_basic_cmd function).

If the target is detected closer than m_target_robot_min_dist(default 1m) the robot stop moving but keep tracking target by angle. 