//
// Created by vant3d on 17/02/2020.
//

#include "matrix.h"

void Matrix::multiply_23_33(double a[2][3], double b[3][3], double result[2][3]) {
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 3; j++) {
            result[i][j] = 0;
            for (int k = 0; k < 3; k++)
                result[i][j] += a[i][k] * b[k][j];
        }
    }
}

void Matrix::multiply_23_31(double a[2][3], double b[3][1], double result[2][1]) {
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 1; j++) {
            result[i][j] = 0;
            for (int k = 0; k < 3; k++)
                result[i][j] += a[i][k] * b[k][j];
        }
    }
}

void Matrix::multiply_33_32(double a[3][3], double b[3][2], double result[3][2]) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 2; j++) {
            result[i][j] = 0;
            for (int k = 0; k < 3; k++)
                result[i][j] += a[i][k] * b[k][j];
        }
    }
}

void Matrix::multiply_33_33(double a[3][3], double b[3][3], double result[3][3]) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            result[i][j] = 0;
            for (int k = 0; k < 3; k++)
                result[i][j] += a[i][k] * b[k][j];
        }
    }
}

void Matrix::multiply_33_31(double a[3][3], double b[3][1], double result[3][1]) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 1; j++) {
            result[i][j] = 0;
            for (int k = 0; k < 3; k++)
                result[i][j] += a[i][k] * b[k][j];
        }
    }
}

void Matrix::multiply_23_32(double a[2][3], double b[3][2], double result[2][2]) {
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) {
            result[i][j] = 0;
            for (int k = 0; k < 3; k++)
                result[i][j] += a[i][k] * b[k][j];
        }
    }
}

void Matrix::multiply_32_21(double a[3][2], double b[2][1], double result[3][1]) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 1; j++) {
            result[i][j] = 0;
            for (int k = 0; k < 2; k++)
                result[i][j] += a[i][k] * b[k][j];
        }
    }
}
void Matrix::multiply_32_22(double a[3][2], double b[2][2], double result[3][2]) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 2; j++) {
            result[i][j] = 0;
            for (int k = 0; k < 2; k++)
                result[i][j] += a[i][k] * b[k][j];
        }
    }
}

void Matrix::multiply_31_11(double a[3][1], double b, double result[3][1]) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 1; j++) {
            result[i][j] = a[i][j] * b;
        }
    }
}

void Matrix::multiply_32_23(double a[3][2], double b[2][3], double result[3][3]) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            result[i][j] = 0;
            for (int k = 0; k < 2; k++)
                result[i][j] += a[i][k] * b[k][j];
        }
    }
}

void Matrix::transpose_33(double a[3][3], double transpose[3][3]) {
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            transpose[j][i] = a[i][j];
        }
    }
}

void Matrix::transpose_23(double a[2][3], double transpose[3][2]) {
    for (int i = 0; i < 2; ++i) {
        for (int j = 0; j < 3; ++j) {
            transpose[j][i] = a[i][j];
        }
    }
}

void Matrix::sub_33_33(double a[3][3], double b[3][3], double result[3][3]) {
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            result[i][j] = a[i][j] - b[i][j];
        }
    }
}

void Matrix::sub_21_21(double a[2][1], double b[2][1], double result[2][1]) {
    for (int i = 0; i < 2; ++i) {
        for (int j = 0; j < 1; ++j) {
            result[i][j] = a[i][j] - b[i][j];
        }
    }
}

void Matrix::sum_33_33(double a[3][3], double b[3][3], double result[3][3]) {
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            result[i][j] = a[i][j] + b[i][j];
        }
    }
}

void Matrix::sum_31_31(double a[3][1], double b[3][1], double result[3][1]) {
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 1; ++j) {
            result[i][j] = a[i][j] + b[i][j];
        }
    }
}

void Matrix::sum_22_22(double a[2][2], double b[2][2], double result[2][2]) {
    for (int i = 0; i < 2; ++i) {
        for (int j = 0; j < 2; ++j) {
            result[i][j] = a[i][j] + b[i][j];
        }
    }
}

void Matrix::divMatrix(double a[2][2], double b[2][2], double result[2][2]) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {

            result[i][j] = (a[i][j] / (b[i][j]);
        }
    }
}

//finding determinant
double Matrix::determinant(double A[2][2], int n)
{
    double D = 0; // Initialize result

    //  Base case : if matrix contains single element
    if (n == 1)
        return A[0][0];

    float temp[2][2]; // To store cofactors

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
void Matrix::adjoint(double A[2][2],double adj[2][2])
{
    if (N == 1)
    {
        adj[0][0] = 1;
        return;
    }

    // temp is used to store cofactors of A[][]
    int sign = 1, temp[2][2];

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
bool Matrix::inverse(int A[2][2], float inverse[2][2])
{
    // Find determinant of A[][]
    int det = determinant(A, 2);
    if (det == 0)
    {
        cout << "Singular matrix, can't find its inverse";
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