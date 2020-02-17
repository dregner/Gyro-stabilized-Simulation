#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>

#include <ignition/math/Quaternion.hh>
#include <std_msgs/Float64.h>
#include <ros/node_handle.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

#include <string>
#include <fstream>
#include <iostream>
#include "matrix.h"

double roll, pitch, yaw, roll_dot, pitch_dot, yaw_dot;
double x1 = 0;
double x2 = 0;
double x3 = 0;
double theta;
double y_1, y_2;
double xe1, xe2, xe3;
const double K1 = -4.2762;
const double K2 = -1.1504;
const double K3 = -1.5896;
double DegToRad = M_PI / 180;
double RadToDeg = 180 / M_PI;
int tout;


int Ts = 5;

class Read_Kalman {
private:
    std::ofstream obs;
    ros::NodeHandle n_kalman;
    ros::Subscriber sub_imu1;
    ros::Subscriber sub_theta1;
    std_msgs::Float64 msg_servo;

    ignition::math::Quaterniond rpy;

    Matrix matrix;

    /// Global variables for Kalman

    double A[3][3] = {{1.0053,0, 0.01},{0,1, 0},{1.0652, 0, 1.053}};
    double B[3][1] = {{-0.0039},{0.01},{-0.7896}};


///--------SAIDAS h(x)--------
// DUAS SAIDAS POS MOTO E GIRO
//mat jac_C = {{1, 0, 0},{0, 1, 0}};
    double jac_C[2][3] = {{1, 0, 0},{0, 1, 0}};

///--------Ruidos----------
/// Ruido de entrada
    double noise_state[3][3] = {{10e-8,0,0},{0,10e-11, 0},{0,0,10e-9}};

    double noise_exit[2][2] = {{10e-4, 0},{0,10e-4}};

/// ------- Variaveis Auxiliares ---------
    double jac_Ct[3][2] = {{0,0},{0,0},{0,0}};
    double s1[2][3] = {{0,0,0},{0,0,0}}; //Aux S
    double S1[2][2] = {{0,0},{0,0}}; //Aux S
    double Si[2][2] = {{0,0},{0,0}}; //Inverse S

/// K = (P * trans(jac_C)) * inv(S);
    double k1[3][2] = {{0,0},{0,0},{0,0}};
    double K[3][2] = {{0,0},{0,0},{0,0}};

/// mat xap = xa + K * (y - jac_C * xa);
    double xap1[2][1] = {{0},{0}};
    double xap2[2][1] = {{0},{0}};
    double xap3[3][1] = {{0},{0},{0}};
    double xap[3][1] = {{0},{0},{0}};
/// mat Pp = (I - K * jac_C) * P;
    double Pp1[3][3] = {{0,0,0},{0,0,0},{0,0,0}};
    double Pp2[3][3] = {{0,0,0},{0,0,0},{0,0,0}};
    double Pp[3][3] = {{0,0,0},{0,0,0},{0,0,0}};

/// xp = A * xap + B * theta;
    double xp1[3][1] = {{0},{0},{0}};
    double xp[3][1] = {{0},{0},{0}};
    double Bt[3][1] = {{0},{0},{0}};

/// P = ((A * Pp) * trans(A)) + noise_state;
    double P1[3][3] = {{0,0,0},{0,0,0},{0,0,0}};
    double At[3][3] = {{0,0,0},{0,0,0},{0,0,0}};
    double P2[3][3] = {{0,0,0},{0,0,0},{0,0,0}};
    double S[2][2] = {{1, 0},{0, 1}};
    double P[3][3] = {{1.0, 0, 0},{0, 1.0, 0},{0, 0, 1.0}};
    double I[3][3] = {{1, 0, 0},{0, 1, 0},{0, 0, 1}};


public:
    Read_Kalman() {
        sub_theta1 = n_kalman.subscribe("/moto/joint_states", 10, &Read_Kalman::read_theta, this);
        sub_imu1 = n_kalman.subscribe("/imu_base", 10, &Read_Kalman::callback, this);
        obs.open("observer_states.txt");
    }

    ~Read_Kalman() {}

    void read_theta(const sensor_msgs::JointStateConstPtr &joint) {
        theta = joint->position[1];
        y_2 = theta;
//        ROS_INFO("x2: [%f]", y_2);
    }

    void callback(const sensor_msgs::Imu::ConstPtr &imu) {

        ignition::math::Quaterniond rpy;

        rpy.Set(imu->orientation.w, imu->orientation.x, imu->orientation.y, imu->orientation.z);

        /// READ y(t)

        y_1 = rpy.Roll();
//      x3 = imu->angular_velocity.x;
        /// OBSERVA
        kalman();
//        observer();

        if (obs.is_open()) {
            obs << tout << "\t" << x1 << "\t" << x2 << "\t" << x3 << "\n";
        }
        tout++;
        ROS_INFO("x1 [%f], x2: [%f], x3: [%f]", x1, x2, x3);
    }

    void observer() {
        /// DISCRETO C = [1 0 0; 0 1 0]  (Ad-Bd*Kd-Ld*C) + Ld*y Ld
        /// lqr(1*Q, 10*R) Kd = (1*Q, 0.7*R) Kd = 0.7 ts = 5 ms
        xe1 = (0.7232 * x1 - 0.0001 * x2 + 0.0048 * x3) + 0.2764 * y_1;
        xe2 = (0.0214 * x1 + 0.7356 * x2 + 0.0079 * x3) + 0.2702 * y_2;
        xe3 = (-0.5008 * x1 - 0.0578 * x2 + 0.9203 * x3) + 0.3563 * y_1;

        x1 = xe1;
        x2 = xe2;
        x3 = xe3;


    }

    void kalman() {

        double xa[3][1] = {{x1},
                           {x2},
                           {x3}};

        double y[2][1] = {{y_1},
                          {y_2}};
        /// S = (jac_C * P) * trans(jac_C) + noise_exit;
        matrix.transpose_23(jac_C, jac_Ct);
        matrix.multiply_23_33(jac_C, P, s1);
        matrix.multiply_23_32(s1, jac_Ct, S1);
        matrix.sum_22_22(S1, noise_exit, S);

        /// K = (P * trans(jac_C)) * inv(S);
        matrix.multiply_33_32(P, jac_Ct, k1);
        matrix.inverse(S, Si);
        matrix.multiply_32_22(k1, Si, K);

        /// mat xap = xa + K * (y - jac_C * xa);
        matrix.multiply_23_31(jac_C, xa, xap1);
        matrix.sub_21_21(y, xap1, xap2);
        matrix.multiply_32_21(K, xap2, xap3);
        matrix.sum_31_31(xa, xap3, xap);

        /// mat Pp = (I - K * jac_C) * P;
        matrix.multiply_32_23(K, jac_C, Pp1);
        matrix.sub_33_33(I, Pp1, Pp2);
        matrix.multiply_33_33(Pp2, P, Pp);

        /// xp = A * xap + B * theta;
        matrix.multiply_33_31(A, xap, xp1);
        matrix.multiply_31_11(B, theta, Bt);
        matrix.sum_31_31(xp1, Bt, xp);

        /// P = ((A * Pp) * trans(A)) + noise_state;
        matrix.multiply_33_33(A, Pp, P1);
        matrix.transpose_33(A, At);
        matrix.multiply_33_33(P1, At, P2);
        matrix.sum_33_33(P2, noise_state, P);

        x1 = xp[0][0];
        x2 = xp[1][0];
        x3 = xp[2][0];

    }

};

int main(int argc, char **argv) {

    ros::init(argc, argv, "kalman_observer");
    Read_Kalman Kalman;

    while (ros::ok()) {
        ros::spinOnce();
    }
    return 0;
}
