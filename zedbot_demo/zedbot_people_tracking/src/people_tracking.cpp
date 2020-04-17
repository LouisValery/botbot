///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2018, STEREOLABS.
//
// All rights reserved.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////

/**
 * This tutorial demonstrates how to receive the list of detected objects from a ZED node
 * from the ZED node
 */
#include <math.h>   //tanh, arctan
#include <typeinfo>  //for 'typeid' to work

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <string>

#include <zed_interfaces/ObjectStamped.h>
#include <zed_interfaces/Objects.h>

#include <tf2/LinearMath/Quaternion.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
// Sl IOT
#include <sl_iot/IoTCloud.hpp>
#include <csignal>
// time
#include <iostream>
#include <chrono>
#include <ctime> 

using namespace sl_iot;
using namespace std;
using json = sl_iot::json;

/**
 * Subscriber callbacks. The argument of the callback is a constant pointer to the received message
 */

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class PeopleTracking
{
public:
    PeopleTracking() : m_action_client("move_base", true)
    {
        //remote control 
        m_remote_control_enabled = false;
	    
        //target status param
        m_target_is_chosen = false; 
        m_target_id = -1;
        m_target_is_lost_id = -1;
        m_distance_from_camera = -1;
        m_current_state = "UNKNOWN";
        
        //target choice param
        m_nh.param("/people_tracking/min_detection_distance", m_min_detection_distance, 0.7);
        m_nh.param("/people_tracking/max_detection_distance", m_max_detection_distance, 4.0);

        //target tracking param
        m_nh.param("/people_tracking/tracking_with_move_base", m_tracking_with_move_base, true); //default true if param not found

        m_nh.param("/people_tracking/target_robot_min_dist", m_target_robot_min_dist, 1.0);
        m_nh.param("/people_tracking/max_robot_speed", m_max_robot_speed, 0.22);

        m_Kp_speed = 0.3;  
        m_Kp_angle = 2.0;

        m_anglular_tolerance = 0.2; //if alignement <=0.2 rad, no correction (about 12 degree)
        // Subscrber and publisher
        m_subObjList = m_nh.subscribe("/zed/zed_node/obj_det/objects", 1, &PeopleTracking::objectListCallback, this);
        m_cmd_vel_pub = m_nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);


        // IOT init

        //Init IoT with the SL_APPLICATION_TOKEN environment variable
        const char * application_token = ::getenv("SL_APPLICATION_TOKEN");
        STATUS_CODE status_iot = IoTCloud::init(application_token);
        if (status_iot != STATUS_CODE::SUCCESS) {
            std::cout << "Initiliazation error " << status_iot << std::endl;
            exit(EXIT_FAILURE);
        }    

        CallbackParameters arrow_callback_params;
        arrow_callback_params.setRemoteCallback("arrow_direction_function", CALLBACK_TYPE::ON_REMOTE_CALL, this);
        IoTCloud::registerFunction(arrow_cmd_callback, arrow_callback_params);

        CallbackParameters allow_remote_control_callback_params;
        allow_remote_control_callback_params.setRemoteCallback("remote_control_signal_function", CALLBACK_TYPE::ON_REMOTE_CALL, this);
        IoTCloud::registerFunction(allow_remote_control_callback, allow_remote_control_callback_params);
        IoTCloud::logInfo("Remote functions initialized");

    }
/////////////////////////////////////////////////////////////////        
/////////////////     Remote control callback  //////////////////
/////////////////////////////////////////////////////////////////

    static void arrow_cmd_callback(FunctionEvent& event) {
        //Get the parameters of the remote function call
        sl_iot::json params = event.getInputParameters();
        //Check if parameters are present and valid
        PeopleTracking* people_track = (PeopleTracking*) event.payload;

        // get current time
        people_track->m_last_remote_control_request = std::clock();


        if (params.find("arrow_direction") != params.end() && params["arrow_direction"].is_string()) {

            string arrow_direction = params["arrow_direction"].get<string>();
            //std::cerr << arrow_direction << endl;
            IoTCloud::logInfo("Arrow direction : " + arrow_direction );

            if (people_track->m_remote_control_enabled){
                if (arrow_direction == "up"){
                    people_track->m_command.linear.x =  0.05;
                    people_track->m_command.angular.z= 0;
                    people_track->m_command.linear.y = 0;
                    people_track->m_cmd_vel_pub.publish(people_track->m_command);
                }
                else if (arrow_direction == "down"){
                    people_track->m_command.linear.x =  -0.05;
                    people_track->m_command.angular.z= 0;
                    people_track->m_command.linear.y = 0;
                    people_track->m_cmd_vel_pub.publish(people_track->m_command);
                }
                else if (arrow_direction == "left"){
                    people_track->m_command.linear.x =  0;
                    people_track->m_command.angular.z= -0.2;
                    people_track->m_command.linear.y = 0;
                    people_track->m_cmd_vel_pub.publish(people_track->m_command);
                }
                else if (arrow_direction == "right"){
                    people_track->m_command.linear.x =  0;
                    people_track->m_command.angular.z= 0.2;
                    people_track->m_command.linear.y = 0;
                    people_track->m_cmd_vel_pub.publish(people_track->m_command);
                }
            }
            //Update the result and status of the event
            event.status = 0;
            event.result = arrow_direction;
        } 

        else {
            IoTCloud::logError("Arrow command function was used with wrong arguments.");
            event.status = 1;
            event.result = "Arrow command function  was used with wrong arguments.";
        }
    }


    static void allow_remote_control_callback(FunctionEvent& event) {
        //Get the parameters of the remote function call
        sl_iot::json params = event.getInputParameters();
        PeopleTracking* people_track = (PeopleTracking*) event.payload;
        
        //Check if parameters are present and valid
        if (params.find("remote_control_signal") != params.end() && params["remote_control_signal"].is_boolean()) {

            people_track->m_remote_control_enabled = params["remote_control_signal"].get<bool>();
            std::cerr << "inside callback  : " << people_track->m_remote_control_enabled << endl;
            
            //Stop all the current actions (when remote control is unable and when disabled)
            people_track->m_action_client.cancelAllGoals();
            people_track->m_command.linear.x = 0;
            people_track->m_command.linear.y = 0;
            people_track->m_command.angular.z = 0;
            people_track->m_cmd_vel_pub.publish(people_track->m_command);
          
            
            //Update the result and status of the event
            event.status = 0;
            event.result = people_track->m_remote_control_enabled;
        } 
        else {
            IoTCloud::logError("Remote control function was used with wrong arguments.");
            event.status = 1;
            event.result = "Remote control function was used with wrong arguments.";
        }
    }

/////////////////////////////////////////////////////////////////
/////////////////////    Autonomous control /////////////////////
/////////////////////////////////////////////////////////////////

    void objectListCallback(const zed_interfaces::Objects::ConstPtr& msg) {
        /*
         * If not in remote controle mode
         * Each time that somebody is detected, 
         * if target not already chosen, chose it and (if chosen, follow it)  ;
         * else follow chosen target
         */
        std::cerr << "main loop  : " << m_remote_control_enabled << endl;
        if (!m_remote_control_enabled){
            if (!m_target_is_chosen){
                ROS_INFO_STREAM( "\n***** chosing target *****");
                chose_target(msg);
                state_manager();
                if (m_target_is_chosen){
                    follow_target(msg);
                }
            }
            else{
                if (m_current_state == "UNKNOWN"){
                    ROS_INFO_STREAM( "State machine ERROR. UNKNOWN state while target detected");
                }
                follow_target(msg);
            }
            //redefine current state
            state_manager();
        }
    }


    void chose_target(const zed_interfaces::Objects::ConstPtr& msg){
        /*
         *choose future target ID, based on ZED object detection
         */
        double distance_min = 100;
        double object_distance_from_camera;
        int target_array_index;
        for(int i=0; i<msg->objects.size();i++)
            {
            if(msg->objects[i].label_id == m_target_is_lost_id)
                continue;

            object_distance_from_camera = std::sqrt(std::pow(msg->objects[i].position.x, 2) + std::pow(msg->objects[i].position.y, 2));
        
            //closest detected person is chosen
            if (object_distance_from_camera < distance_min && m_min_detection_distance <= object_distance_from_camera){
                distance_min = object_distance_from_camera;
                target_array_index = i;
            }
        }
        // test if closest detected person is in chosen range (and make sur someone as been detected)
        if (m_min_detection_distance <= distance_min && distance_min <= m_max_detection_distance){
            m_target_id = msg->objects[target_array_index].label_id;
            m_target_is_chosen = true;
            m_distance_from_camera = distance_min;
            ROS_INFO_STREAM("Target chosen : " << m_target_id << "  Distance : " << m_distance_from_camera);
        }
    }

    void follow_target(const zed_interfaces::Objects::ConstPtr& msg){
        /*
         *define current goal and choose between move_base move or custom commands
         */
        double alignement_angle;
        bool target_still_found = false;

        for(int i=0; i<msg->objects.size();i++){
            //person of interest
            if (msg->objects[i].label_id == m_target_id){
                //tracking_state== 1 : ok , tracking_state== 1 : searching (occlusion occured)
                if (msg->objects[i].tracking_state == 1 || msg->objects[i].tracking_state == 1){
                    
                    //compute navigation info
                    m_distance_from_camera = std::sqrt(std::pow(msg->objects[i].position.x, 2) + std::pow(msg->objects[i].position.y, 2));
                    alignement_angle = atan(msg->objects[i].position.y / msg->objects[i].position.x);

                    ROS_INFO_STREAM("\n\nTarget id : " << m_target_id << "  Distance : " << m_distance_from_camera << "  Alignement angle: " << alignement_angle);

                    if (m_current_state == "TOO_CLOSE_FROM_TARGET" || m_current_state == "DO_NOT_MOVE"){
                        //robot stop and keep target centered
                        angular_tracking(alignement_angle);
                    }

                    else if (m_current_state == "FAR_FROM_TARGET"){
                        ROS_INFO_STREAM("\n ********  Trying to reach target  ***********");
                        if (m_tracking_with_move_base){
                            track_with_move_base(msg, i);
                        }
                        else{
                            track_with_basic_cmd(alignement_angle);
                        }
                    }
                }
                //tracking_state== 0 : off (object not valid) or 3: terminate --> search a new target
                else {
		            ROS_INFO_STREAM("\n ********  Target lost. Reaching last known position  ***********");
                    // target has been lost
                    if (m_tracking_with_move_base){
                        //reach last known position anyway
                        m_action_client.waitForResult();
                        
                        if(m_action_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                        {
                            ROS_INFO("\nLast known position reached");
                        }
                        else
                        {
                            ROS_INFO("\nThe robot failed to reach last known position for some reason");
                        }
		            }

                    ROS_INFO("\n**********    Target lost    *************");
                    m_command.linear.x = 0;
                    m_command.linear.y = 0;
                    m_command.angular.z = 0;
                    m_cmd_vel_pub.publish(m_command); 

                    //find new target when next loop will occure
                    m_target_is_chosen = false;
                    m_target_id = m_target_is_lost_id;
                    m_distance_from_camera = -1;

                }
            }
        }
    }
    void track_with_move_base(const zed_interfaces::Objects::ConstPtr& msg, int object_index){
        /*
        * This function uses the person position to send a goal to move base
        * It needs to be sent in map frame, so a transform is necessary
        */
	    //wait for move_base server
        while(!m_action_client.waitForServer(ros::Duration(5.0))){
            ROS_INFO_STREAM("Waiting for the move_base action server to come up");
	    }

        //goal in robot frame
        geometry_msgs::PoseStamped goal_robot_frame;
        geometry_msgs::PoseStamped goal_map_frame;
        
        goal_robot_frame.header.frame_id = "base_footprint";
        goal_robot_frame.pose.position.x = 0.80 * msg->objects[object_index].position.x;	//nearly reach target in order to give a free goal on the costmap to move base (target cell is obstacle)		
        goal_robot_frame.pose.position.y = 0.80 * msg->objects[object_index].position.y;

        tf::Quaternion myQuaternion;
        myQuaternion.setRPY( 0, 0, atan(msg->objects[object_index].position.y / msg->objects[object_index].position.x));  // Create quaternion from roll/pitch/yaw (in radians)
        myQuaternion.normalize();

        goal_robot_frame.pose.orientation.x = myQuaternion.getX();
        goal_robot_frame.pose.orientation.y = myQuaternion.getY();
        goal_robot_frame.pose.orientation.z = myQuaternion.getZ();
        goal_robot_frame.pose.orientation.w = myQuaternion.getW();

        //transform goal to map frame
        m_tf_listener.transformPose("map", ros::Time(0), goal_robot_frame, "base_footprint", goal_map_frame);

        // m_action_client.cancelAllGoals (); 
        m_goal.target_pose.header.frame_id = "map";
        m_goal.target_pose.pose = goal_map_frame.pose;
        m_goal.target_pose.header.stamp = ros::Time::now();

        m_action_client.sendGoal(m_goal);
    }



    void track_with_basic_cmd(double const &alignement_angle){
        //compute command
        ROS_INFO("trying to reach target");
        m_action_client.cancelAllGoals();
        m_command.linear.x = m_max_robot_speed * tanh(m_Kp_speed * m_distance_from_camera); 
        m_command.linear.y = 0;
        m_command.angular.z = m_Kp_angle * alignement_angle;

        m_cmd_vel_pub.publish(m_command);
    }

    void angular_tracking(double const &alignement_angle){
        /*
         * To close from target. 
         * Angular command to keep alignment on target
         * Negative speed to reach m_target_robot_min_dist distance
         */
        ROS_INFO("****************    To close from target: rotation and move back    ***********");
        m_action_client.cancelAllGoals();
        double tolerance = 0.10; // buffer used to avoid useless switch case between "too close" or not, due to noise
        if (m_target_robot_min_dist - m_distance_from_camera - tolerance > 0){ //robot in range [0, 0.90]
            m_command.linear.x =  - m_max_robot_speed * tanh((m_target_robot_min_dist - m_distance_from_camera) * 3);
            m_command.angular.z= m_Kp_angle * alignement_angle;
        }
        else{ //robot in range [0.90, 1.10]
            m_command.linear.x = 0; 
            if (alignement_angle >= m_anglular_tolerance){
                m_command.angular.z = m_Kp_angle * alignement_angle;
            }
            else{
                m_command.angular.z = 0;
            }
        }
        m_command.linear.y = 0;
        m_cmd_vel_pub.publish(m_command);

    }


    void info_display(const zed_interfaces::Objects::ConstPtr& msg){
        for(int i=0; i<msg->objects.size();i++){
            if(msg->objects[i].label_id == m_target_is_lost_id)
                continue;

            ROS_INFO_STREAM( msg->objects[i].label
                    << " ["
                    << msg->objects[i].label_id
                    << "] - Pos. ["
                    << msg->objects[i].position.x << ","
                    << msg->objects[i].position.y << ","
                    << msg->objects[i].position.z << "] [m]"
                    << "- Conf. "
                    << msg->objects[i].confidence
                    << " - Tracking state: "
                    << static_cast<int>(msg->objects[i].tracking_state) );
        }
    }
    bool value_is_in_range(float value, float range_min, float range_max){
        if (value <= range_max && value > range_min){
            return true;
        }
        return false;
    }


    void state_manager(){
        double tolerance = 0.10;
        if (m_distance_from_camera == -1){
            m_current_state = "UNKNOWN";
        }
        else if ((m_distance_from_camera >= m_target_robot_min_dist + tolerance) ||
                 (value_is_in_range(m_distance_from_camera, m_target_robot_min_dist, m_target_robot_min_dist + tolerance) && m_current_state == "FAR_FROM_TARGET")){ //default >1.10m
            m_current_state = "FAR_FROM_TARGET";
        }
        else if ((value_is_in_range(m_distance_from_camera, m_target_robot_min_dist - tolerance, m_target_robot_min_dist + tolerance) && m_current_state == "DO_NOT_MOVE") ||
                 (value_is_in_range(m_distance_from_camera, m_target_robot_min_dist - tolerance, m_target_robot_min_dist) && m_current_state == "FAR_FROM_TARGET") ||
                 (value_is_in_range(m_distance_from_camera, m_target_robot_min_dist, m_target_robot_min_dist + tolerance) && m_current_state == "TOO_CLOSE_FROM_TARGET")||
                 (value_is_in_range(m_distance_from_camera, m_target_robot_min_dist - tolerance, m_target_robot_min_dist + tolerance) && m_current_state == "UNKNOWN") )
            m_current_state = "DO_NOT_MOVE";

        else if((m_distance_from_camera <= m_target_robot_min_dist - tolerance) ||
                (value_is_in_range(m_distance_from_camera, m_target_robot_min_dist - tolerance, m_target_robot_min_dist) && m_current_state == "TOO_CLOSE_FROM_TARGET"))
            m_current_state = "TOO_CLOSE_FROM_TARGET";
        else{
            ROS_INFO_STREAM( "ERROR: UNKNOWN state. Your state machine is broken");
        }

        ROS_INFO_STREAM("CURRENT STATE : "<< m_current_state);
    }

/////////////////////////////////////////////////////////////////
/////////////////////       Accessors       /////////////////////
/////////////////////////////////////////////////////////////////


    bool get_remote_control_enabled() const{
        return m_remote_control_enabled;
    }	


    std::clock_t get_last_remote_control_request() const{
        return m_last_remote_control_request;
    }	


    ros::Publisher get_cmd_vel_pub() const{
        return m_cmd_vel_pub;
    }


private:

/////////////////    ROS node  ///////////////
    ros::NodeHandle m_nh;  
    ros::Subscriber m_subObjList;           //subscribe to the object detection topic
    ros::Publisher m_cmd_vel_pub;           //publish robot's commands on /cmd_vel topic (when move base is not used)
    MoveBaseClient m_action_client;         //send goal to move base
    tf::TransformListener m_tf_listener;    //listen to tf (used to transform goal from robot frame to map frame)

/////////////////    Remote control  ////////////////
    // Remote control
    bool m_remote_control_enabled;
    std::clock_t m_last_remote_control_request;


////////////////    Automatic people tracking  ///////////////

    // robot states based on distance from target
    std::string m_current_state;

    //chose target 
    bool m_target_is_chosen;
    int m_target_id;
    int m_target_is_lost_id;
    double m_min_detection_distance; //in m
    double m_max_detection_distance; //in m
    double m_distance_from_camera; //in m

    //reach target
    bool m_tracking_with_move_base;
    move_base_msgs::MoveBaseGoal m_goal;  
    geometry_msgs::Twist m_command;
    double m_anglular_tolerance;
    double m_Kp_angle;              //proportianal coefficient for robot angle relative to target
    double m_Kp_speed;              //proportianal coefficient for robot speed
    double m_target_robot_min_dist; //in m, define how close the robot is suppose to follow the target 
    double m_max_robot_speed;       
};





/*
 * Node main function
 */
int main(int argc, char** argv) {
    ros::init(argc, argv, "zed_target_detection_object_detection");
    PeopleTracking PeopleTrackingObject;
    
    ros::Rate loop_rate(10);
    float efficiency_time_of_remote_command = 1; //1s
    geometry_msgs::Twist command;
    command.linear.x = 0;
    command.linear.y = 0;
    command.angular.z = 0;
    ros::Publisher cmd_publisher =  PeopleTrackingObject.get_cmd_vel_pub();
    
    int count = 0;
    while (ros::ok())
    {
        if (PeopleTrackingObject.get_remote_control_enabled())
        {
            std::clock_t current_time = std::clock();
            double elapsed_seconds_since_last_command = double(current_time - PeopleTrackingObject.get_last_remote_control_request());
            
            if (elapsed_seconds_since_last_command >= efficiency_time_of_remote_command)
            {
                cmd_publisher.publish(command); 
            }
        }
        ros::spinOnce();
        loop_rate.sleep();
    }


    ros::spin();
    return 0;
}
