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

/// ======== CONFIGURAÇÕES FILTROD E KALMAN =========
/// Tempo de amostragem 5 ms
///----------Jacobiano F e G------------
//mat A(3,3);
//A = 1.0053 << 0 << 0.01 << endr << 0 << 1 << 0 << endr << 1.0652<< 0 << 1.053 << endr;
//mat A = {{1.0053, 0, 0.01}, {0,      1, 0},  {1.0652, 0, 1.053}};
//mat B = {-0.0039, 0.01, -0.7896};


double A[3][3] = {{1.0053, 0, 0.01},
                  {0,      1, 0},
                  {1.0652, 0, 1.053}};
double B[3][1] = {{-0.0039},
                  {0.01},
                  {-0.7896}};


///--------SAIDAS h(x)--------
// DUAS SAIDAS POS MOTO E GIRO
//mat jac_C = {{1, 0, 0},{0, 1, 0}};
double jac_C[2][3] = {{1, 0, 0},
                      {0, 1, 0}};

///--------Ruidos----------
/// Ruido de entrada
/* mat noise_state = {{10 ^ -8, 0,        0},
                   {0,       10 ^ -11, 0},
                   {0,       0,        10 ^ -9}};*/
double noise_state[3][3] = {{10 ^ -8, 0,        0},
                            {0,       10 ^ -11, 0},
                            {0,       0,        10 ^ -9}};

///Ruido duas saidas
/// Duas saidas
/*mat noise_exit = {{10 ^ -4, 0}, {0,       10 ^ -4}};
mat S(2, 2, fill::eye);
mat P(3, 3, fill::eye);
mat I(3, 3, fill::eye);*/
double noise_exit[2][2] = {{10 ^ -4, 0},
                           {0,       10 ^ -4}};
double S[2][2] = {{1, 0},
                  {0, 1}};
double P[3][3] = {{1, 0, 0},
                  {0, 1, 0},
                  {0, 0, 1}};
double I[3][3] = {{1, 0, 0},
                  {0, 1, 0},
                  {0, 0, 1}};


double roll, pitch, yaw, roll_dot, pitch_dot, yaw_dot;
double x1 = 18;
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
int countt;
int Ts = 5;

class Read_Kalman {
private:
    ros::NodeHandle n_kalman;

    ros::Subscriber sub_imu;
    ros::Subscriber sub_theta;
    std_msgs::Float64 msg_servo;

    ignition::math::Quaterniond rpy;

    void multiply_23_33(double a[2][3], double b[3][3], double result[2][3]) {
        for (int i = 0; i < 2; i++) {
            for (int j = 0; j < 3; j++) {
                result[i][j] = 0;
                for (int k = 0; k < 3; k++)
                    result[i][j] += a[i][k] *
                                    b[k][j];
            }
        }
    }

    void multiply_23_31(double a[2][3], double b[3][1], double result[2][1]) {
        for (int i = 0; i < 2; i++) {
            for (int j = 0; j < 1; j++) {
                result[i][j] = 0;
                for (int k = 0; k < 3; k++)
                    result[i][j] += a[i][k] *
                                    b[k][j];
            }
        }
    }

    void multiply_33_32(double a[3][3], double b[3][2], double result[3][2]) {
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 2; j++) {
                result[i][j] = 0;
                for (int k = 0; k < 3; k++)
                    result[i][j] += a[i][k] *
                                    b[k][j];
            }
        }
    }

    void multiply_33_33(double a[3][3], double b[3][3], double result[3][3]) {
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                result[i][j] = 0;
                for (int k = 0; k < 3; k++)
                    result[i][j] += a[i][k] *
                                    b[k][j];
            }
        }
    }

    void multiply_33_31(double a[3][3], double b[3][1], double result[3][1]) {
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 1; j++) {
                result[i][j] = 0;
                for (int k = 0; k < 3; k++)
                    result[i][j] += a[i][k] *
                                    b[k][j];
            }
        }
    }

    void multiply_23_32(double a[2][3], double b[3][2], double result[2][2]) {
        for (int i = 0; i < 2; i++) {
            for (int j = 0; j < 2; j++) {
                result[i][j] = 0;
                for (int k = 0; k < 3; k++)
                    result[i][j] += a[i][k] *
                                    b[k][j];
            }
        }
    }

    void multiply_32_21(double a[3][2], double b[2][1], double result[3][1]) {
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 1; j++) {
                result[i][j] = 0;
                for (int k = 0; k < 2; k++)
                    result[i][j] += a[i][k] *
                                    b[k][j];
            }
        }
    }

    void multiply_31_11(double a[3][1], double b, double result[3][1]) {
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 1; j++) {
                result[i][j] = a[i][j] * b;
            }
        }
    }

    void multiply_32_23(double a[3][2], double b[2][3], double result[3][3]) {
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                result[i][j] = 0;
                for (int k = 0; k < 2; k++)
                    result[i][j] += a[i][k] *
                                    b[k][j];
            }
        }
    }

    void transpose_33(double a[3][3], double transpose[3][3]) {
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                transpose[j][i] = a[i][j];
            }
        }
    }

    void transpose_23(double a[2][3], double transpose[3][2]) {
        for (int i = 0; i < 2; ++i) {
            for (int j = 0; j < 3; ++j) {
                transpose[j][i] = a[i][j];
            }
        }
    }

    void sub_33_33(double a[3][3], double b[3][3], double result[3][3]) {
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                result[i][j] = a[i][j] - b[i][j];
            }
        }
    }

    void sub_21_21(double a[2][1], double b[2][1], double result[2][1]) {
        for (int i = 0; i < 2; ++i) {
            for (int j = 0; j < 1; ++j) {
                result[i][j] = a[i][j] - b[i][j];
            }
        }
    }

    void sum_33_33(double a[3][3], double b[3][3], double result[3][3]) {
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                result[i][j] = a[i][j] - b[i][j];
            }
        }
    }

    void sum_31_31(double a[3][1], double b[3][1], double result[3][1]) {
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 1; ++j) {
                result[i][j] = a[i][j] + b[i][j];
            }
        }
    }

    void sum_21_21(double a[2][1], double b[2][1], double result[2][1]) {
        for (int i = 0; i < 2; ++i) {
            for (int j = 0; j < 1; ++j) {
                result[i][j] = a[i][j] - b[i][j];
            }
        }
    }

    void sum_22_22(double a[2][2], double b[2][2], double result[2][2]) {
        for (int i = 0; i < 2; ++i) {
            for (int j = 0; j < 2; ++j) {
                result[i][j] = a[i][j] + b[i][j];
            }
        }
    }

    void divMatrix(double a[2][2], double b[2][2], double result[2][2]) {
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {

                result[i][j] = (double) (a[i][j]) / (b[i][j]);
            }
        }
    }


public:
    Read_Kalman() {
        sub_theta = n_kalman.subscribe("/moto/joint_state", 10, &Read_Kalman::read_theta, this);
        sub_imu = n_kalman.subscribe("/imu_base", 10, &Read_Kalman::callback, this);
    }

    ~Read_Kalman() {}

    void read_theta(const sensor_msgs::JointState::ConstPtr &joint) {
        theta = joint->position[1];
    }

    void callback(const sensor_msgs::Imu::ConstPtr &imu) {

        ignition::math::Quaterniond rpy;

        rpy.Set(imu->orientation.w, imu->orientation.x, imu->orientation.y, imu->orientation.z);

        /// READ y(t)

        y_1 = rpy.Roll();
//      x3 = imu->angular_velocity.x;
        /// OBSERVA
        kalman();


        ROS_INFO("x1 [%f], x2: [%f], x3: [%f]", x1, x2, x3);
    }

    void observer() {
        /// DISCRETO C = [1 0 0; 0 1 0]  (Ad-Bd*Kd-Ld*C) + Ld*y Ld
        /// lqr(1*Q, 10*R) Kd = (1*Q, 0.7*R) Kd = 0.7 ts = 5 ms
        xe1 = (0.7232 * x1 - 0.0001 * x2 + 0.0048 * x3) + 0.2764 * y_1;
        xe2 = (0.0214 * x1 + 0.7356 * x2 + 0.0079 * x3) + 0.2702 * y_2;
        xe3 = (-0.5008 * x1 - 0.0578 * x2 + 0.9203 * x3) + 0.3563 * y_1;


    }

    void kalman() {

        double xa[3][1] = {{x1},
                           {x2},
                           {x3}};

        double y[2][1] = {{y_1},
                          {y_2}};
        /// S = (jac_C * P) * trans(jac_C) + noise_exit;
        double s1[2][3];
        multiply_23_33(jac_C, P, s1);
        double jac_Ct[3][2];
        transpose_23(jac_C, jac_Ct);
        double S[2][2];
        multiply_23_32(s1, jac_Ct, S);
        double S_result[2][2];
        sum_22_22(S, noise_exit, S_result);

        /// K = (P * trans(jac_C)) * inv(S);
        double k1[3][2];
        multiply_33_32(P, jac_Ct, k1);
        double K[3][2];
        divMatrix(k1, S_result, K);

        /// mat xap = xa + K * (y - jac_C * xa);
        double xap1[2][1];
        multiply_23_31(jac_C, xa, xap1);
        double xap2[2][1];
        sub_21_21(y, xap1, xap2);
        double xap3[3][1];
        multiply_32_21(K, xap2, xap3);
        double xap[3][1];
        sum_31_31(xa, xap3, xap);

        /// mat Pp = (I - K * jac_C) * P;
        double Pp1[3][3];
        multiply_32_23(K, jac_C, Pp1);
        double Pp2[3][3];
        sub_33_33(I, Pp1, Pp2);
        double Pp[3][3];
        multiply_33_33(Pp2, P, Pp);

        /// xp = A * xap + B * theta;
        double xp1[3][1];
        multiply_33_31(A, xap, xp1);
        double xp[3][1];
        double Bt[3][1];
        multiply_31_11(B, theta, Bt);
        sum_31_31(xp1, Bt, xp);

        /// P = ((A * Pp) * trans(A)) + noise_state;
        double P1[3][3];
        multiply_33_33(A, Pp, P1);
        double At[3][3];
        transpose_33(A, At);
        double P2[3][3];
        multiply_33_33(P1, At, P2);
        double P[3][3];
        sum_33_33(P2, noise_state, P);

        x1 = xp[0][0];
        x2 = xp[1][0];
        x3 = xp[2][0];

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
