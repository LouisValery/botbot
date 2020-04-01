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

#include <zed_interfaces/ObjectStamped.h>
#include <zed_interfaces/Objects.h>

#include <tf2/LinearMath/Quaternion.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
/**
 * Subscriber callbacks. The argument of the callback is a constant pointer to the received message
 */

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class PeopleTracking
{
public:
    PeopleTracking() : m_action_client("move_base", true)
    {
	//target status param
        m_target_is_chosen = false; 
        m_target_id = -1;
        m_target_is_lost_id = -1;
	
	//target choice param
	m_nh.param("/people_tracking/min_detection_distance", m_min_detection_distance, 0.7);
	m_nh.param("/people_tracking/max_detection_distance", m_max_detection_distance, 4.0);

	//target tracking param
        m_nh.param("/people_tracking/tracking_with_move_base", m_tracking_with_move_base, true); //default true if param not found

	m_nh.param("/people_tracking/target_robot_min_dist", m_target_robot_min_dist, 1.0);
	m_nh.param("/people_tracking/max_robot_speed", m_max_robot_speed, 0.22);
        m_Kp_speed = 0.3;  
        m_Kp_angle = 2.0;

        // Subscrber and publisher
        m_subObjList = m_nh.subscribe("/zed/zed_node/obj_det/objects", 1, &PeopleTracking::objectListCallback, this);
        m_cmd_vel_pub = m_nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    }



    void objectListCallback(const zed_interfaces::Objects::ConstPtr& msg) {
        //info_display(msg);
        if (!m_target_is_chosen){
            ROS_INFO_STREAM( "\n***** chosing target *****");
            chose_target(msg);
        }
        else{
            define_robot_goal(msg);
        }
    }

    void chose_target(const zed_interfaces::Objects::ConstPtr& msg){
        double distance_min = 100;
        double distance_from_camera;
        int target_array_index;
        for(int i=0; i<msg->objects.size();i++)
            {
            if(msg->objects[i].label_id == m_target_is_lost_id)
                continue;

            distance_from_camera = std::sqrt(std::pow(msg->objects[i].position.x, 2) + std::pow(msg->objects[i].position.y, 2));
        
            //closest detected person is chosen
            if (distance_from_camera < distance_min && m_min_detection_distance <= distance_from_camera){
                distance_min = distance_from_camera;
                target_array_index = i;
            }
        }
        // test if closest detected person is in chosen range (and make sur someone as been detected)
        if (m_min_detection_distance <= distance_min && distance_min <= m_max_detection_distance){
            m_target_id = msg->objects[target_array_index].label_id;
            m_target_is_chosen = true;
            ROS_INFO_STREAM("Target chosen : " << m_target_id << "  Distance : " << distance_min);
        }
    }

    void define_robot_goal(const zed_interfaces::Objects::ConstPtr& msg){

        double distance_from_camera;
        double alignement_angle;
        bool target_still_found = false;

        for(int i=0; i<msg->objects.size();i++){
            //person of interest
            if (msg->objects[i].label_id == m_target_id){
                //tracking_state== 1 : ok , tracking_state== 1 : searching (occlusion occured)
                if (msg->objects[i].tracking_state== 1 || msg->objects[i].tracking_state== 1){
                    
                    //compute navigation info
                    distance_from_camera = std::sqrt(std::pow(msg->objects[i].position.x, 2) + std::pow(msg->objects[i].position.y, 2));
                    alignement_angle = atan(msg->objects[i].position.y / msg->objects[i].position.x);

                    ROS_INFO_STREAM("\n\nTarget id : " << m_target_id << "  Distance : " << distance_from_camera << "  Alignement angle: " << alignement_angle);

                    //if too close
                    if (distance_from_camera <= m_target_robot_min_dist){
                        //robot stop and keep target centered
                        angular_tracking(distance_from_camera, alignement_angle);
                    }

                    else { //compute goal
                        ROS_INFO_STREAM("\n ********  Trying to reach target  ***********");
                        if (m_tracking_with_move_base){
                            track_with_move_base(msg, i);
                        }
                        else{
                            track_with_basic_cmd(distance_from_camera,alignement_angle);
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
		                ROS_INFO("\nLast known position reached");
		            else
		                ROS_INFO("\nThe robot failed to reach last known position for some reason");
		    }

                    ROS_INFO("\n**********    Target lost    *************");
                    m_command.linear.x = 0;
		    m_command.linear.y = 0;
                    m_command.angular.z = 0;
                    m_cmd_vel_pub.publish(m_command); 

                    //find new target when next loop will occure
                    m_target_is_chosen = false;
                    m_target_id = m_target_is_lost_id;

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
        goal_robot_frame.pose.position.x = 0.80 * msg->objects[object_index].position.x;	//reach nearly target in order to give a free goal on the costmap to move base (target cell is obstacle)		
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



    void track_with_basic_cmd(double const &distance_from_camera, double const &alignement_angle){
        //compute command
        ROS_INFO("trying to reach target");
        m_action_client.cancelAllGoals();
        m_command.linear.x = m_max_robot_speed * tanh(m_Kp_speed * distance_from_camera); 
        m_command.linear.y = 0;
        m_command.angular.z = m_Kp_angle * alignement_angle;

        m_cmd_vel_pub.publish(m_command);
    }

    void angular_tracking(double const &distance_from_camera, double const &alignement_angle){
        //compute angular command
        ROS_INFO("****************    To close from target: rotation only      ***********");
        m_action_client.cancelAllGoals();
	double buffer = 0.05; // buffer used to avoid useless switch case between "too close" or not, due to noise
	if (m_target_robot_min_dist - distance_from_camera - buffer > 0)
        	m_command.linear.x =  - m_max_robot_speed * tanh((m_target_robot_min_dist - distance_from_camera) * 3);
	else
		m_command.linear.x = 0; 

        m_command.linear.y = 0;
        m_command.angular.z= m_Kp_angle * alignement_angle;

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
    
private:
    ros::NodeHandle m_nh;  
    ros::Subscriber m_subObjList;           //subscribe to the object detection topic
    ros::Publisher m_cmd_vel_pub;           //publish robot's commands on /cmd_vel topic (when move base is not use)
    MoveBaseClient m_action_client;         //send goal to move base
    tf::TransformListener m_tf_listener;    //listen to tf (used to transform goal from robot frame to map frame)

    //chose target 
    bool m_target_is_chosen;
    int m_target_id;
    int m_target_is_lost_id;
    double m_min_detection_distance; //in m
    double m_max_detection_distance; //in m

    //reach target
    move_base_msgs::MoveBaseGoal m_goal;  
    geometry_msgs::Twist m_command;

    bool m_tracking_with_move_base;

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

    ros::spin();

    return 0;
}
