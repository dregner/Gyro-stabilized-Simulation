//
// Created by daniel on 22/10/2019.
//

#include <ros/ros.h>
#include <ros/node_handle.h>
#include <iostream>
#include <fstream>

#include <control_msgs/JointControllerState.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>
#include <ignition/math/Pose3.hh>

int main(int argc, char **argv) {


    ros::init(argc, argv, "start_gyro");

    ros::NodeHandle n;


    ros::Publisher pub_vel = n.advertise<std_msgs::Float64>("/moto/gyro_velocity/command",10);
    ros::Publisher pub_pos = n.advertise<std_msgs::Float64>("moto/gyro_angle/command", 10);
    // Declare the variables


    ros::Rate poll_rate(10);
    while(pub_vel.getNumSubscribers() == 0){
        int num;

        // Input the integer
        std::cout << "Publish speed value: ";
        std::cin >> num;


        poll_rate.sleep();
        std_msgs::Float64 msg_vel;
        std_msgs::Float64 msg_pos;
        msg_vel.data = num;
        msg_pos.data = 0.9;
        pub_vel.publish(msg_vel);
        pub_pos.publish(msg_pos);
        ROS_INFO("Published %i rad/s", num);
        ros::spinOnce();
    }


}