#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <ignition/math/Quaternion.hh>
#include <std_msgs/Float64.h>
#include <ros/node_handle.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

double roll, pitch, yaw, roll_dot, pitch_dot, yaw_dot;
double x1, x2, x3, theta, atuacao;
const float K1 = -3.4727;
const float K2 = -0.9386;
const float K3 = -0.9930;
double DegToRad = M_PI / 180;
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
        sub_imu = control.subscribe("/imu_base", 100, &Control_SS::callback, this);
        servo = control.advertise<std_msgs::Float64>("/moto/gyro_angle/command", 10);
    }

    ~Control_SS() {}

    float mapfloat(float x, float in_min, float in_max, float out_min, float out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    void callback(const sensor_msgs::Imu::ConstPtr &imu) {

        ignition::math::Quaterniond rpy;

        rpy.Set(imu->orientation.w, imu->orientation.x, imu->orientation.y, imu->orientation.z);
        ros::Rate loop_rate(1000);
        if (coutt > Ts) {
            roll = rpy.Roll();
            roll *= RadToDeg;

            pitch = rpy.Pitch();
            pitch *= RadToDeg;

            yaw = rpy.Yaw();
            yaw *= RadToDeg;

            roll_dot = imu->angular_velocity.x;
            pitch_dot = imu->angular_velocity.y;
            yaw_dot = imu->angular_velocity.z;

            u = -K1 * roll - K2 * theta - K3 * roll_dot;
            theta = fmin(0.7, fmax(-0.7, theta));
            msg_servo.data = atuacao;
            servo.publish(msg_servo);
        }

        loop_rate.sleep();
        countt++;


//    ROS_INFO("Seq: [%d]", imu->header.seq);
//    ROS_INFO("Orientation -> r: [%f], p: [%f], y: [%f]", roll, pitch, yaw);
    }
};

int main(int argc, char **argv) {

    ros::init(argc, argv, "control_state");
    ros::NodeHandle control;
    return 0;
}
