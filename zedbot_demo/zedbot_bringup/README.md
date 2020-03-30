# Zedbot bringup

This package is not directly associated to a tutorial. It provides basic launch and config files to run start the robot.
The main launch file is the start_zedbot.launch file. It runs the turtlebot3_core, the zed_node (zed node wrapper) and a robot_state_publisher. By running this node you obtain a full rviz representation of the robot and an access to the topics associated to the zed and the turtlebot. You also can control your robot using RQT (robot_steering).

## The Zed Node
This node starts the zed and provides all the associated topics: rgb image, depth image, pointcould, odometry, object detection...

## The turtlebot node (turtlebot3_core)
It communicates with the openCR card, in particular, it subscribes the commands to the wheels (cmd_cel topic) that are then sent to the openCR card.
It subscribe the wheels position (tf), the robot odom (the ZED odom is prefered to this one, because more reliable), and the imu

## Robot state publishers
Two robot_state_publisher are runed:
- The first one publishes tf information about the robot position. It is based on the turtlebot3's urdf 
- The second robot state publisher publishes tf information about the ZED camera. It takes into account  the zed rotation (pitch) relative to the robot thanks to the ZED IMU sensor.
