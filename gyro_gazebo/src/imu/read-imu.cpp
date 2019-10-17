//
// Created by daniel on 09/10/2019.
//

#include "ros/ros.h"
#include <ros/node_handle.h>
#include <sensor_msgs/Imu.h>
#include <iostream>
#include <fstream>
#include <std_msgs/Float64.h>
#include <ignition/math2/ignition/math/Pose3.hh>



double q_x, q_y, q_z, q_w, roll, pitch, yaw;
double value;
double RadToDeg = 180 / M_PI;


void read_imu(const sensor_msgs::Imu::ConstPtr &msg) {

    ignition::math::Quaterniond rpy;

    value = msg->header.seq;

    rpy.Set(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);

    roll = rpy.Roll();
    roll *= RadToDeg;

    pitch = rpy.Pitch();
    pitch *= RadToDeg;

    yaw = rpy.Yaw();
    yaw *= RadToDeg;

    ROS_INFO("Seq: [%d]", msg->header.seq);
    ROS_INFO("Orientation -> r: [%f], p: [%f], y: [%f]", roll, pitch, yaw);
}


int main(int argc, char **argv) {


    ros::init(argc, argv, "imu_listener");

    ros::NodeHandle nh;

    ros::Subscriber sub_imu = nh.subscribe("/imu_base", 10, read_imu);
    while (ros::ok()) {
        ros::spinOnce();
    }
}