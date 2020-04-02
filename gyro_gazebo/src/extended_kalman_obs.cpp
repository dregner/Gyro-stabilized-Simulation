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
double y_1, y_2, y_3;
double xe1, xe2, xe3;
const double K1 = -4.2762;
const double K2 = -1.1504;
const double K3 = -1.5896;
double DegToRad = M_PI / 180;
double RadToDeg = 180 / M_PI;
int tout;


int Ts = 0.005;

class Read_Kalman {
private:
    std::ofstream obs;
    ros::NodeHandle n_kalman;
    ros::Subscriber sub_imu1;
    ros::Subscriber sub_theta1;
    std_msgs::Float64 msg_servo;

    ignition::math::Quaterniond rpy;

    Matrix matrix;

    /// Globa28l variables for Kalman
/// A =
///    1.0001       0    0.0050
///    0    1.0000         0
///    0.05  0    1.0001
    double A[3][3] = {{1.0001, 0, 0.005},
                      {0,      1, 0},
                      {0.0528, 0, 1.0001}};


    double jacobian[3][3] = {{0, 0, 0},
                             {0, 0, 0},
                             {0, 0, 0}};
    double f_jac[3][3] = {{0, 0, 1},
                          {0, 0, 0},
                          {0, 0, 0}};
    double g_jac[3][3] = {{0, 0, 0},
                          {0, 0, 0},
                          {0, 0, 0}};
    double g_jac_u[3][3] = {{0, 0, 0},
                            {0, 0, 0},
                            {0, 0, 0}};
    double res[3][3] = {{0, 0, 0},
                        {0, 0, 0},
                        {0, 0, 0}};

    double g_nl[3][1] = {{0},
                         {0},
                         {0}};
    double g_nl_u[3][1] = {{0},
                           {0},
                           {0}};
    double f_nl[3][1] = {{0},
                         {0},
                         {0}};

///--------SAIDAS h(x)--------
// DUAS SAIDAS POS MOTO E GIRO
//mat jac_C = {{1, 0, 0},{0, 1, 0}};
    double jac_C[2][3] = {{1, 0, 0},
                          {0, 1, 0}};

///--------Ruidos----------
/// Ruido de entrada
    double noise_state[3][3] = {{10e-8, 0,      0},
                                {0,     10e-11, 0},
                                {0,     0,      10e-9}};

    double noise_exit[2][2] = {{10e-8, 0},
                               {0,     10e-8}};

/// ------- Variaveis Auxiliares ---------
    double jac_Ct[3][2] = {{0, 0},
                           {0, 0},
                           {0, 0}};
    double S1[2][3] = {{0, 0, 0},
                       {0, 0, 0}}; //Aux S1
    double S2[2][2] = {{0, 0},
                       {0, 0}}; //Aux S2
    double Si[2][2] = {{0, 0},
                       {0, 0}}; //Inverse S

/// K = (P * trans(jac_C)) * inv(S);
    double k1[3][2] = {{0, 0},
                       {0, 0},
                       {0, 0}};
    double K[3][2] = {{0, 0},
                      {0, 0},
                      {0, 0}};

/// mat xap = xa + K * (y - jac_C * xa);
    double xap1[2][1] = {{0},
                         {0}};
    double xap2[2][1] = {{0},
                         {0}};
    double xap3[3][1] = {{0},
                         {0},
                         {0}};
    double xap[3][1] = {{0},
                        {0},
                        {0}};
/// mat Pp = (I - K * jac_C) * P;
    double Pp1[3][3] = {{0, 0, 0},
                        {0, 0, 0},
                        {0, 0, 0}};
    double Pp2[3][3] = {{0, 0, 0},
                        {0, 0, 0},
                        {0, 0, 0}};
    double Pp[3][3] = {{0, 0, 0},
                       {0, 0, 0},
                       {0, 0, 0}};

/// xp = A * xap + B * theta;
    double xp1[3][1] = {{0},
                        {0},
                        {0}};
    double xp2[3][1] = {{0},
                        {0},
                        {0}};
    double xp[3][1] = {{0},
                       {0},
                       {0}};

/// P = ((A * Pp) * trans(A)) + noise_state;
    double P1[3][3] = {{0, 0, 0},
                       {0, 0, 0},
                       {0, 0, 0}};
    double At[3][3] = {{0, 0, 0},
                       {0, 0, 0},
                       {0, 0, 0}};
    double P2[3][3] = {{0, 0, 0},
                       {0, 0, 0},
                       {0, 0, 0}};
    double S[2][2] = {{1, 0},
                      {0, 1}};
    double P[3][3] = {{1, 0, 0},
                      {0, 1, 0},
                      {0, 0, 1}};
    double I_3[3][3] = {{1, 0, 0},
                        {0, 1, 0},
                        {0, 0, 1}};


public:
    Read_Kalman() {
        sub_theta1 = n_kalman.subscribe("/moto/joint_states", 10, &Read_Kalman::read_theta, this);
        sub_imu1 = n_kalman.subscribe("/moto/moto/imu_base", 10, &Read_Kalman::callback, this);
        obs.open("extend_kalman_state.txt");
    }

    ~Read_Kalman() {}

    void read_theta(const sensor_msgs::JointStateConstPtr &joint) {
        theta = joint->position[1];
        y_2 = theta;
//        ROS_INFO("x2: [%f]", y_2);
    }

    void f_jacobian(double x1a, double x2a, double x3a){
        double long value = 13965983501871220625;
       double f31 = (-214861284644172625*cos(x1a))/(4398046511104*(130*(cos(x2a)*cos(x2a)) - 4753));
       double f32 = (-value*cos(x2a)*sin(x1a)*sin(x2a))/(1099511627776*((130*cos(x2)*cos(x2a) - 4753)*(130*cos(x2)*cos(x2a) - 4753)));
        f_jac[2][0] = f31;
        f_jac[2][1] = f32;
    }

    void g_jacobian(double x1a, double x2a, double x3a){
        double g32 = -(1201980*x3a - 197761500*sin(x2a) - 2437760*x3*sin(x2a)*sin(x2a) + 5265000*sin(x2a)*sin(x2a)*sin(x2a))/((130*sin(x2a)*sin(x2a) + 4623)*(130*sin(x2a)*sin(x2a) + 4623));
        double g33 = (130*sin(2*x2a))/(65*cos(2*x2a) - 4688);

        g_jac[2][1] = g32;
        g_jac[2][2] = g33;
    }

    void f_nonLin(double x1a, double x2a, double x3a) {

        double Mv = 10 + 1 + .2 + 0.2; // Massa do ve�culo sem giro [kg]
        double Mg = 3; //Massa do giro [kg]
        double Rg = 0.3; // Raio do giro [m]
        double Ag = 0.1; // Espessura giro [m]
        double Av = 0.4; // Altura ve�culo [m]
        double Lv = 1.2; // Largura ve�culo [m]
        double Dg = 0.5 - 0.05; // Dist�ncia entre centro de massa do giro e eixo de rota��o [m]
        double Dv = 0.1; // Dist�ncia entre centro de massa do ve�culo e eixo de rota��o
        double Omega = 150; // Velocidade de rota��o do giro, rpm*convers�o = rad/sec
        double g = 9.81; // Gravidade [m/s^2]

        double IG11 = (Mg * (Rg * Rg) / 4) + (Mg * (Ag * Ag) / 12); // Algum momento de inercia
        double IG33 = Mg * (Rg * Rg) / 2;
        double IB11 = Mv * (Av * Av + Lv * Lv) / 12;

        double f3 = ((Mv * Dv + Mg * Dg) * g * sin(x1a)) /
                    (IB11 + Mv * (Dv * Dv) + IG11 * (cos(x2a) * cos(x2a)) + Mg * (Dg * Dg) +
                     IG33 * (sin(x2a) * sin(x2a)));

        f_nl[2][0] = f3;
    }

    void g_nonLin(double x1a, double x2a, double x3a) {
        double Mv = 10 + 1 + .2 + 0.2; // Massa do ve�culo sem giro [kg]
        double Mg = 3; //Massa do giro [kg]
        double Rg = 0.3; // Raio do giro [m]
        double Ag = 0.1; // Espessura giro [m]
        double Av = 0.4; // Altura ve�culo [m]
        double Lv = 1.2; // Largura ve�culo [m]
        double Dg = 0.5 - 0.05; // Dist�ncia entre centro de massa do giro e eixo de rota��o [m]
        double Dv = 0.1; // Dist�ncia entre centro de massa do ve�culo e eixo de rota��o
        double Omega = 150; // Velocidade de rota��o do giro, rpm*convers�o = rad/sec
        double g = 9.81; // Gravidade [m/s^2]

        double IG11 = (Mg * (Rg * Rg) / 4) + (Mg * (Ag * Ag) / 12); // Algum momento de inercia
        double IG33 = Mg * (Rg * Rg) / 2;
        double IB11 = Mv * (Av * Av + Lv * Lv) / 12;
        double f3 = ((Mv * Dv + Mg * Dg) * g * sin(x1a)) /
                    (IB11 + Mv * (Dv * Dv) + IG11 * (cos(x2a) * cos(x2a)) + Mg * (Dg * Dg) +
                     IG33 * (sin(x2a) * sin(x2a)));

        double g3 = (-2 * cos(x2a) * sin(x2a) * x3a * (IG33 - IG11) - Omega * cos(x2a) * IG33) /
                    (IB11 + IG11 * (cos(x2a) * cos(x2a)) + Mv * (Dv*Dv) + Mg * (Dg*Dg) +
                     IG33 * (sin(x2a) * sin(x2a)));
        g_nl[2][0] = g3;
    }


    void callback(const sensor_msgs::Imu::ConstPtr &imu) {

        ignition::math::Quaterniond rpy;

        rpy.Set(imu->orientation.w, imu->orientation.x, imu->orientation.y, imu->orientation.z);

        /// READ y(t)

        y_1 = rpy.Roll();
        y_3 = imu->angular_velocity.x;//      x3 = imu->angular_velocity.x;
        /// OBSERVA
        kalman();
//        observer();

        if (obs.is_open()) {
            obs << tout << "\t" << x1 << "\t" << x2 << "\t" << x3 << "\t" << y_1 << "\t" << y_2 << "\t" << y_3 << "\n";
        }
        tout++;
        ROS_INFO("y1: [%f], y2: [%f]", y_1, y_2);
        ROS_INFO("x1: [%f], x2: [%f], x3: [%f]", x1, x2, x3);
    }

    void kalman() {

        double xa[3][1] = {{x1},
                           {x2},
                           {x3}};

        double y[2][1] = {{y_1},
                          {y_2}};
        /// S = (jac_C * P) * trans(jac_C) + noise_exit;
        matrix.transpose_23(jac_C, jac_Ct);
        matrix.multiply_23_33(jac_C, P, S1);
        matrix.multiply_23_32(S1, jac_Ct, S2);
        matrix.sum_22_22(S2, noise_exit, S);

        /// K = (P * trans(jac_C)) * inv(S);
        matrix.multiply_33_32(P, jac_Ct, k1);
        matrix.inverse(S, Si);
        matrix.multiply_32_22(k1, Si, K);

        /// mat xap = xa + K * (y - jac_C * xa);
        matrix.multiply_23_31(jac_C, xa, xap1);
        matrix.sub_21_21(y, xap1, xap2);
        matrix.multiply_32_21(K, xap2, xap3);
        matrix.sum_31_31(xa, xap3, xap);

        ///xap =

        /// mat Pp = (eye(3) - K * jac_C) * P;
        matrix.multiply_32_23(K, jac_C, Pp1);
        matrix.sub_33_33(I_3, Pp1, Pp2);
        matrix.multiply_33_33(Pp2, P, Pp);

        /// A = (f_jac + g_jac*u)*Ts + eye(3);
        f_jacobian(x1, x2, x3);
        g_jacobian(x1, x2, x3);
        matrix.multiply_33_11(g_jac, theta, g_jac_u);
        matrix.sum_33_33(f_jac, g_jac_u, res);
        matrix.multiply_33_11(res, Ts, jacobian);
        matrix.sum_33_33(jacobian, I_3, A);

        /// xp = x = Ts*(f+g*u);
        f_nonLin(x1, x2, x3);
        g_nonLin(x1, x2, x3);
        matrix.multiply_31_11(g_nl, theta, g_nl_u);
        matrix.sum_31_31(f_nl, g_nl_u, xp1);
        matrix.multiply_31_11(xp1, Ts, xp2);
        matrix.sum_31_31(xa, xp2, xp);


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

    ros::init(argc, argv, "extended_kalman_obs");
    Read_Kalman Kalman;


    while (ros::ok()) {
        ros::spinOnce();
    }
    return 0;
}
