/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 0;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    
    int j = 0;
    bool landing_started = false;
    bool disarm_started = false;

    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

	    if (ros::Time::now() - last_request > ros::Duration(5.0) && j < 1) {
            pose.pose.position.z += 2;
            local_pos_pub.publish(pose);
            last_request = ros::Time::now();
            j++;
        } else if (ros::Time::now() - last_request > ros::Duration(5.0) && j < 2) {
            pose.pose.position.x += 2;
            local_pos_pub.publish(pose);
            last_request = ros::Time::now();
            j++;
        }else if (ros::Time::now() - last_request > ros::Duration(5.0) && j < 3) {
            pose.pose.position.y += 2;
            local_pos_pub.publish(pose);
            last_request = ros::Time::now();
            j++;
            ROS_INFO("jumped to while");
        }  else if (ros::Time::now() - last_request > ros::Duration(5.0) && j < 4) {
            pose.pose.position.x -= 2;
            local_pos_pub.publish(pose);
            last_request = ros::Time::now();
            j++;
        } else if (ros::Time::now() - last_request > ros::Duration(5.0) && j < 5) {
            pose.pose.position.y -= 2;
            local_pos_pub.publish(pose);
            last_request = ros::Time::now();
            j++;
        } /*else if (ros::Time::now() - last_request > ros::Duration(5.0) && j < 6) {
            pose.pose.position.z -= 1;
            local_pos_pub.publish(pose);
            last_request = ros::Time::now();
            j++;
            ROS_INFO("Reached End");
        } /*else if (ros::Time::now() - last_request > ros::Duration(5.0) && j < 7) {
            pose.pose.position.z -= 0.5;
            local_pos_pub.publish(pose);
            last_request = ros::Time::now();
            j++;

        } else if (ros::Time::now() - last_request > ros::Duration(5.0) && j < 8) {
            pose.pose.position.z -= 0;
            local_pos_pub.publish(pose);
            last_request = ros::Time::now();
            j++;
            ROS_INFO("jumped to while"); 
        } */
      /*  while(!landing_started && current_state.mode == "OFFBOARD") {
            ROS_INFO("started while");
            if (pose.pose.position.z > 0.5) {
                pose.pose.position.z -= 0.9;
                local_pos_pub.publish(pose);
            } else {
                std::cout << "Vehicle has started landing..." << std::endl;
                landing_started = true;
                last_request = ros::Time::now();
            }
        } 
 
        if (landing_started) {
            if(disarm_started && current_state.armed) {
                mavros_msgs::CommandBool disarm_cmd;
                disarm_cmd.request.value = false;
                if(arming_client.call(disarm_cmd)&& disarm_cmd.response.success) {
                    ROS_INFO("Vechile Disarmed");
                }
                disarm_started = true;
                last_request = ros::Time::now();
            } else if (disarm_started && !current_state.armed) {
                ROS_INFO("Landing complete,shutting down");
                break;
            }
        }  */
        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
