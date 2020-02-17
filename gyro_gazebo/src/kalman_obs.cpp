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
    void multiply_32_22(double a[3][2], double b[2][2], double result[3][2]) {
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 2; j++) {
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
                result[i][j] = a[i][j] + b[i][j];
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

    void sum_22_22(double a[2][2], double b[2][2], double result[2][2]) {
        for (int i = 0; i < 2; ++i) {
            for (int j = 0; j < 2; ++j) {
                result[i][j] = a[i][j] + b[i][j];
            }
        }
    }

    void getCofactor(double A[2][2], double temp[2][2], int p, int q, int n)
    {
        int i = 0, j = 0;

        // Looping for each element of the matrix
        for (int row = 0; row < n; row++)
        {
            for (int col = 0; col < n; col++)
            {
                //  Copying into temporary matrix only those element
                //  which are not in given row and column
                if (row != p && col != q)
                {
                    temp[i][j++] = A[row][col];

                    // Row is filled, so increase row index and
                    // reset col index
                    if (j == n - 1)
                    {
                        j = 0;
                        i++;
                    }
                }
            }
        }
    }

    //finding determinant
    double determinant(double A[2][2], int n)
    {
        double D = 0; // Initialize result

        //  Base case : if matrix contains single element
        if (n == 1)
            return A[0][0];

        double temp[2][2]; // To store cofactors

        int sign = 1;  // To store sign multiplier

        // Iterate for each element of first row
        for (int f = 0; f < n; f++)
        {
            // Getting Cofactor of A[0][f]
            getCofactor(A, temp, 0, f, n);
            D += sign * A[0][f] * determinant(temp, n - 1);

            // terms are to be added with alternate sign
            sign = -sign;
        }

        return D;
    }

// Function to get adjoint of A[N][N] in adj[N][N].
    void adjoint(double A[2][2],double adj[2][2])
    {
        // temp is used to store cofactors of A[][]
        int sign = 1;
        double temp[2][2];

        for (int i=0; i<2; i++)
        {
            for (int j=0; j<2; j++)
            {
                // Get cofactor of A[i][j]
                getCofactor(A, temp, i, j, 3);

                // sign of adj[j][i] positive if sum of row
                // and column indexes is even.
                sign = ((i+j)%2==0)? 1: -1;

                // Interchanging rows and columns to get the
                // transpose of the cofactor matrix
                adj[j][i] = (sign)*(determinant(temp, 2-1));
            }
        }
    }

// Function to calculate and store inverse, returns false if
// matrix is singular
    bool inverse(double A[2][2], double inverse[2][2])
    {
        // Find determinant of A[][]
        double det = determinant(A, 2);
        if (det == 0)
        {
            std::cout << "Singular matrix, can't find its inverse";
            return false;
        }

        // Find adjoint
        double adj[2][2];
        adjoint(A, adj);

        // Find Inverse using formula "inverse(A) = adj(A)/det(A)"
        for (int i=0; i<2; i++)
            for (int j=0; j<2; j++)
                inverse[i][j] = adj[i][j]/double(det);

        return true;
    }


public:
    Read_Kalman() {
        sub_theta1 = n_kalman.subscribe("/moto/joint_state", 10, &Read_Kalman::read_theta, this);
        sub_imu1 = n_kalman.subscribe("/imu_base", 10, &Read_Kalman::callback, this);
        obs.open("observer_states.txt");
    }

    ~Read_Kalman() {}

    void read_theta(const sensor_msgs::JointState::ConstPtr &joint) {
        theta = joint->position[1];
        y_2 = theta;
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
        transpose_23(jac_C, jac_Ct);
        multiply_23_33(jac_C, P, s1);
        multiply_23_32(s1, jac_Ct, S1);
        sum_22_22(S1, noise_exit, S);

        /// K = (P * trans(jac_C)) * inv(S);
        multiply_33_32(P, jac_Ct, k1);
        inverse(S, Si);
        multiply_32_22(k1, Si, K);

        /// mat xap = xa + K * (y - jac_C * xa);
        multiply_23_31(jac_C, xa, xap1);
        sub_21_21(y, xap1, xap2);
        multiply_32_21(K, xap2, xap3);
        sum_31_31(xa, xap3, xap);

        /// mat Pp = (I - K * jac_C) * P;
        multiply_32_23(K, jac_C, Pp1);
        sub_33_33(I, Pp1, Pp2);
        multiply_33_33(Pp2, P, Pp);

        /// xp = A * xap + B * theta;
        multiply_33_31(A, xap, xp1);
        multiply_31_11(B, theta, Bt);
        sum_31_31(xp1, Bt, xp);

        /// P = ((A * Pp) * trans(A)) + noise_state;
        multiply_33_33(A, Pp, P1);
        transpose_33(A, At);
        multiply_33_33(P1, At, P2);
        sum_33_33(P2, noise_state, P);

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
