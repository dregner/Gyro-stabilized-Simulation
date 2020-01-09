#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <ignition/math/Quaternion.hh>
#include <std_msgs/Float64.h>
#include <ros/node_handle.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <armadillo>
#include <iostream>

// ======== CONFIGURAÇÕES FILTROD E KALMAN =========
// Tempo de amostragem 10 ms
//----------Jacobiano F e G------------
mat A(3,3, float) = 1.0053 << 0 << 0.01 << endr << 0 << 1 << 0 << endr << 1.0652<< 0 << 1.053 << endr;
mat B(3,1, float) = -0.0039 << endr << 0.01<< endr << -0.7896 << endr;

//--------SAIDAS h(x)--------
// DUAS SAIDAS POS MOTO E GIRO
mat jac_C(2,3, float) = 1 << 0 << 0 << endr << 0 << 1 << 0 << endr;

//--------Ruidos----------
// Ruido de entrada
mat ruido_w(3,3,float) = {{10^-8,0,0},{0,10^-11,0},{0,0,10^-9}};

//Ruido duas saidas
// Duas saidas
mat ruido_v(2,2,float) = {{10^-4,0},{0,10^-4}};
mat S(2,2, fill::eye);
mat P(3,3,fill::eye);
mat I(3,3,fill::eye);


double roll, pitch, yaw, roll_dot, pitch_dot, yaw_dot;
double x1=18;
double x2 = 0;
double x3 = 0, theta;
double y_1, y_2;
double xe1, xe2, xe3;
const float K1 = -4.2762;
const float K2 = -1.1504;
const float K3 = -1.5896;
double DegToRad = M_PI / 180;
double RadToDeg = 180 / M_PI;
int countt;
int Ts = 5;

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

    float mapfloat(float x, float in_min, float in_max, float out_min, float out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    void callback(const sensor_msgs::Imu::ConstPtr &imu) {

        ignition::math::Quaterniond rpy;

        rpy.Set(imu->orientation.w, imu->orientation.x, imu->orientation.y, imu->orientation.z);

        /// ATUA
        theta += (-K1 * x1 - K2 * x2 - K3 * x3);

        theta = std::min(0.9, std::max(-0.9, theta));
        msg_servo.data = theta;
        servo.publish(msg_servo);
        /// READ y(t)

//        roll = rpy.Roll();
        y_1 = rpy.Roll();
        y_2 = theta;
//      x3 = imu->angular_velocity.x;
        /// OBSERVA
        observer();

        /// ACTUALIZE
        x1 = xe1;
        x2 = xe2;
        x3 = xe3;



//    ROS_INFO("Seq: [%d]", imu->header.seq);
//    ROS_INFO("Orientation -> r: [%f], p: [%f], y: [%f]", roll, pitch, yaw);
    }

    void observer(){
        /// DISCRETO C = [1 0 0; 0 1 0]  (Ad-Bd*Kd-Ld*C) + Ld*y Ld
        /// lqr(1*Q, 10*R) Kd = (1*Q, 0.7*R) Kd = 0.7 ts = 5 ms
        xe1 = (0.7232 * x1 - 0.0001 * x2 + 0.0048 * x3) + 0.2764 * y_1;
        xe2 = (0.0214 * x1 + 0.7356 * x2 + 0.0079 * x3) + 0.2702 * y_2;
        xe3 = (-0.5008* x1 - 0.0578 * x2 + 0.9203 * x3) + 0.3563 * y_1;


    }
};

int main(int argc, char **argv) {

    ros::init(argc, argv, "control_state");
    Control_SS controlSs;

    while (ros::ok()) {
        ros::spinOnce();
    }
    return 0;
}

void kalman_code(){

    mat xa(3,1,float) = x1 << endr << x2 << endr << x3 << endr;

    mat y(2,1,float) = y1 << endr << y2 endr;

    S = (C*P)*trans(C)+v;

    K = (P*Ct)*inv(S);
    xap = xa+K*(y-C*xa)
            ;
    mat Pp = (I - K*C)*P;

    mat xp(3,1,float) = A*xap+B*u;
    P = ((A*Pp)*trans(A))+w;

    x1 = xp(1,1);
    x2 = xp(2,1);
    x3 = xp(3,1);

}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
