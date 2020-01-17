#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>

#include <ignition/math/Quaternion.hh>
#include <std_msgs/Float64.h>

#include <ros/node_handle.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

#include <armadillo>
#include <iostream>

using namespace arma;

// ======== CONFIGURAÇÕES FILTROD E KALMAN =========
// Tempo de amostragem 10 ms
//----------Jacobiano F e G------------
//mat A(3,3);
//A = 1.0053 << 0 << 0.01 << endr << 0 << 1 << 0 << endr << 1.0652<< 0 << 1.053 << endr;
mat A = {{1.0053, 0, 0.01},
         {0,      1, 0},
         {1.0652, 0, 1.053}};
vec B = {-0.0039, 0.01, -0.7896};

//--------SAIDAS h(x)--------
// DUAS SAIDAS POS MOTO E GIRO
mat jac_C = {{1, 0, 0},
             {0, 1, 0}};

//--------Ruidos----------
// Ruido de entrada
mat noise_state = {{10 ^ -8, 0,        0},
                   {0,       10 ^ -11, 0},
                   {0,       0,        10 ^ -9}};

//Ruido duas saidas
// Duas saidas
mat noise_exit = {{10 ^ -4, 0},
                  {0,       10 ^ -4}};
mat S(2, 2, fill::eye);
mat P(3, 3, fill::eye);
mat I(3, 3, fill::eye);


double roll, pitch, yaw, roll_dot, pitch_dot, yaw_dot;
double x1 = 18;
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

class Read_Kalman {
private:
    ros::NodeHandle n_kalman;

    ros::Subscriber sub_imu;
    ros::Subscriber sub_theta;
    std_msgs::Float64 msg_servo;

    ignition::math::Quaterniond rpy;

public:
    Read_Kalman() {
        sub_imu = n_kalman.subscribe("/imu_base", 10, &Read_Kalman::callback, this);
//        theta = n_kalman.subscribe("/moto/joint_state", 10,&Read_Kalman::read_theta, this);
    }

    ~Read_Kalman() {}

    float mapfloat(float x, float in_min, float in_max, float out_min, float out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    //void read_theta(const){
//
//    };

    void callback(const sensor_msgs::Imu::ConstPtr &imu) {

        ignition::math::Quaterniond rpy;

        rpy.Set(imu->orientation.w, imu->orientation.x, imu->orientation.y, imu->orientation.z);

        /// READ y(t)

//        roll = rpy.Roll();
        y_1 = rpy.Roll();
//      x3 = imu->angular_velocity.x;
        /// OBSERVA

        /// ACTUALIZE
        x1 = xe1;
        x2 = xe2;
        x3 = xe3;



//    ROS_INFO("Seq: [%d]", imu->header.seq);
//    ROS_INFO("Orientation -> r: [%f], p: [%f], y: [%f]", roll, pitch, yaw);
    }

    void observer() {
        /// DISCRETO C = [1 0 0; 0 1 0]  (Ad-Bd*Kd-Ld*C) + Ld*y Ld
        /// lqr(1*Q, 10*R) Kd = (1*Q, 0.7*R) Kd = 0.7 ts = 5 ms
        xe1 = (0.7232 * x1 - 0.0001 * x2 + 0.0048 * x3) + 0.2764 * y_1;
        xe2 = (0.0214 * x1 + 0.7356 * x2 + 0.0079 * x3) + 0.2702 * y_2;
        xe3 = (-0.5008 * x1 - 0.0578 * x2 + 0.9203 * x3) + 0.3563 * y_1;


    }

    void kalman() {

        vec xa = {x1, x2, x3};

        vec y = {y_1, y_2};

        S = (jac_C * P) * trans(jac_C) + noise_exit;

        mat K = (P * trans(jac_C)) * inv(S);
        mat xap = xa + K * (y - jac_C * xa);
        mat Pp = (I - K * jac_C) * P;

        mat xp(3, 1);
        xp = A * xap + B * theta;
        P = ((A * Pp) * trans(A)) + noise_state;

        x1 = xp(1, 1);
        x2 = xp(2, 1);
        x3 = xp(3, 1);

    }


};

int main(int argc, char **argv) {

    ros::init(argc, argv, "control_state");
    Read_Kalman Kalman;

    while (ros::ok()) {
        ros::spinOnce();
    }
    return 0;
}
