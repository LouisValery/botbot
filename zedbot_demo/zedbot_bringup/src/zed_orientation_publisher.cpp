#include "ros/ros.h"

#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Quaternion.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <sstream>
/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */

//Global variable
double roll, pitch, yaw;
tf2::Quaternion my_zed_orientation_q;


void cameraOrientationCallback(const sensor_msgs::Imu& imu_msg)
{
    tf2::convert(imu_msg.orientation , my_zed_orientation_q); // conversion from geometry_msgs/Quaternion to tf2/LinearMath/Quaternion.h
    tf2::Matrix3x3(my_zed_orientation_q).getRPY(roll, pitch, yaw);
}


int main(int argc, char **argv)
{

    ros::init(argc, argv, "zed_orientation_publisher");
    ros::NodeHandle nh;

    ros::Publisher zed_orientation_publisher = nh.advertise<sensor_msgs::JointState>("zed_orientation_joint_states", 1000);
    ros::Subscriber imu_subscriber;
    ros::Rate loop_rate(10);

    sensor_msgs::JointState joint_states;
    int joint_total_number = 1;
    std::string camera_model, zed_model, zedm_model, zed2_model;
    zed_model = "zed";
    zedm_model = "zedm";
    zed2_model = "zed2";

    nh.param("/zed/zed_node/general/camera_model", camera_model, zed_model); //default zed

    //if zed2 or zedm, /zed/zed_node/imu/data exists and can be subscribed, otherwise, orientation, will be a null vector
    if ((camera_model == zed2_model) || (camera_model == zedm_model)){
        imu_subscriber = nh.subscribe("/zed/zed_node/imu/data", 1000, cameraOrientationCallback);
    }


    joint_states.name.resize(joint_total_number);
    joint_states.position.resize(joint_total_number);
    joint_states.velocity.resize(joint_total_number);
    joint_states.effort.resize(joint_total_number);

    joint_states.header.frame_id = "base_link";
    joint_states.name[0] = "camera_center_joint";
    joint_states.position[0] = pitch;
    joint_states.velocity[0] = 0.0;
    joint_states.effort[0] = 0.0;

    while (ros::ok())
    {
        joint_states.position[0] = pitch;
        joint_states.header.stamp = ros::Time::now();

        zed_orientation_publisher.publish(joint_states);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}