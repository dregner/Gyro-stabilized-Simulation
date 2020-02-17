#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <ignition/math/Quaternion.hh>
#include <std_msgs/Float64.h>
#include <ros/node_handle.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

double roll, pitch, yaw, roll_dot, pitch_dot, yaw_dot;
double x1, x2, x3, theta;
const float K1 = -4.2762;
const float K2 = -1.1504;
const float K3 = -1.5896;
double RadToDeg = 180 / M_PI;
int countt; int Ts = 5;
    
class Control_SS {
private:
    ros::NodeHandle control;

    ros::Subscriber sub_imu;
    ros::Publisher servo;
    std_msgs::Float64 msg_servo;

    ignition::math::Quaterniond rpy;
public:
    Control_SS() {
        sub_imu = control.subscribe("/imu_base", 1000, &Control_SS::callback, this);
        servo = control.advertise<std_msgs::Float64>("/moto/gyro_angle/command", 1000);
    }

    ~Control_SS() {}

    void callback(const sensor_msgs::Imu::ConstPtr &imu) {

        ignition::math::Quaterniond rpy;

        rpy.Set(imu->orientation.w, imu->orientation.x, imu->orientation.y, imu->orientation.z);
//        ros::Rate loop_rate(1000);
//        if (countt > Ts) {
//            countt = 0;
            roll = rpy.Roll();
//            roll *= RadToDeg;

            roll_dot = imu->angular_velocity.x;

            theta += (-K1 * roll - K2 * theta - K3 * roll_dot);

            theta = std::min(0.9, std::max(-0.9, theta));
            msg_servo.data = theta;
            servo.publish(msg_servo);
//        }

//        loop_rate.sleep();
        countt++;


//    ROS_INFO("Seq: [%d]", imu->header.seq);
//    ROS_INFO("Orientation -> r: [%f], p: [%f], y: [%f]", roll, pitch, yaw);
    }
};

int main(int argc, char **argv) {

    ros::init(argc, argv, "control_state");
    Control_SS controlSs;

    while(ros::ok()){
        ros::spinOnce();
    }
    return 0;
}
