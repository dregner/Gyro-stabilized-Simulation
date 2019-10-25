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


    ros::Publisher pub_vel = n.advertise<std_msgs::Float64>("/moto/gyro_velocity/command",1000);
    // Declare the variables

    ros::Rate poll_rate(10);
    while(pub_vel.getNumSubscribers() == 0){
        int num;

        // Input the integer
        std::cout << "Enter the integer: ";
        std::cin >> num;

        // Display the integer
        std::cout << "Entered integer is: " << num << "\n";

        poll_rate.sleep();
        std_msgs::Float64 msg_vel;
        msg_vel.data = num;
        pub_vel.publish(msg_vel);
        ROS_INFO("Published %i rad/s", num);
        ros::spinOnce();
    }


}