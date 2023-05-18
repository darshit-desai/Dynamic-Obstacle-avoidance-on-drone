/**
 * @file offboard_obstacle.cpp
 * @brief Offboard control of the drone based on the subscribed obstacle pose
 * 
 * Modified from reference file https://docs.px4.io/master/en/ros/mavros_offboard.html
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/GlobalPositionTarget.h>

mavros_msgs::State current_state;
mavros_msgs::PositionTarget pose_vel;
geometry_msgs::PoseStamped g_pose;
geometry_msgs::PoseStamped drone_pose_feedback;
geometry_msgs::PoseStamped tag_pose_body;

void callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    g_pose = *msg;
}
void lpp_callback(const geometry_msgs::PoseStamped::ConstPtr& lpp_msg){
drone_pose_feedback = *lpp_msg;
}

int main(int argc, char **argv)
{
    //Define the node name
    ros::init(argc, argv, "offb_obs_node");
    ros::NodeHandle nh;
    //Define the Target pose pubisher    
    ros::Publisher local_pos_pub_mavros = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 5);
    //Define the tag obstacle frame subscriber
    ros::Subscriber sub_pose_artag = nh.subscribe("/obstacle_pose",10,callback);
    //Define the drone pose subscriber
    ros::Subscriber local_info_sub = nh.subscribe <geometry_msgs::PoseStamped> ("/mavros/local_position/pose", 10, lpp_callback);
    ros::Rate rate(5.0);

    //Define the drone pose message identifier
    mavros_msgs::PositionTarget drone_pose_cmd;
    drone_pose_cmd.position.x = 0;
    drone_pose_cmd.position.y = 0;
    drone_pose_cmd.position.z = 1;
    drone_pose_cmd.type_mask = drone_pose_cmd.IGNORE_VX | drone_pose_cmd.IGNORE_VY | drone_pose_cmd.IGNORE_VZ | drone_pose_cmd.IGNORE_AFZ | drone_pose_cmd.IGNORE_AFY | drone_pose_cmd.IGNORE_AFX;
    drone_pose_cmd.coordinate_frame = drone_pose_cmd.FRAME_LOCAL_NED;
    drone_pose_cmd.yaw = 3.141592/2;

    ros::Time last_request = ros::Time::now();
    //Obstacle pose body frame
    float x_tag{0}, y_tag{0}, z_tag{0};
    //Local pose of drone
    float x_drone_feedback{0}, y_drone_feedback{0}, z_drone_feedback{0};
    //Command pose of drone
    float x_drone_cmd{0}, y_drone_cmd{0}, z_drone_cmd{0};
    float x_diff{0}, y_diff{0}, z_diff{0};
    float last_pos_y{0};
    int reached=0;
    while(ros::ok()){   
        //Transfer values from message to variable for april tag pose
        x_tag = g_pose.pose.position.x;
        y_tag = g_pose.pose.position.y;
        z_tag = g_pose.pose.position.z;
        x_drone_feedback = drone_pose_feedback.pose.position.x;
        y_drone_feedback = drone_pose_feedback.pose.position.y;
        z_drone_feedback = drone_pose_feedback.pose.position.z;
        if (z_tag==0){
            std::cout<<"No obstacle detected! Holding pose "<<std::endl;
            drone_pose_cmd.position.x=x_drone_feedback;
            drone_pose_cmd.position.y=y_drone_feedback;
            drone_pose_cmd.position.z=0.7;
            rate.sleep();
        }
        else{
            //Transfer values from message to variable for drone pose
            x_drone_feedback = drone_pose_feedback.pose.position.x;
            y_drone_feedback = drone_pose_feedback.pose.position.y;
            z_drone_feedback = drone_pose_feedback.pose.position.z;

            //Control y movement of the drone
            if (y_tag>1.2){
                y_drone_cmd = y_drone_feedback+0.1;
            }
            else if (y_tag < 1.4){
                //Dead stop control. Uncomment below code
                // if(reached==0){
                //     last_pos_y = y_drone_feedback;
                //     reached =1;
                // }
                // y_drone_cmd = last_pos_y;

                //Reactive avoidance control. Comment this part below and uncomment above to see drone performing dead
                //dead stop when it exceeds the threshold distance of 1.2 m
                y_drone_cmd = y_drone_feedback-0.1;
            }
            if (x_tag>0.05){
                x_drone_cmd = x_drone_feedback-0.1;
            }
            else if(x_tag<0.05){
                x_drone_cmd = x_drone_feedback+0.1;
            }

            //Assign z same as drone's current height
            z_drone_cmd=0.7;

            //Build a command message for the drone's updated position
            drone_pose_cmd.position.x=x_drone_feedback;
            drone_pose_cmd.position.y=y_drone_cmd;
            drone_pose_cmd.position.z=z_drone_cmd;
            std::cout<<"\nx position command of drone: "<<x_drone_cmd;
            std::cout<<"\ny position command of drone: "<<y_drone_cmd;
            std::cout<<"\nz position command of drone: "<<z_drone_cmd;
        }
        //Publish the command message
        local_pos_pub_mavros.publish(drone_pose_cmd);

        ros::spinOnce();
        rate.sleep();

    }

    return 0;
}

