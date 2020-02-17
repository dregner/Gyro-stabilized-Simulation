//
// Created by vant3d on 17/02/2020.
//

#include "matrix.h"

void multiply_23_33(double a[2][3], double b[3][3], double result[2][3]) {
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 3; j++) {
            result[i][j] = 0;
            for (int k = 0; k < 3; k++)
                result[i][j] += a[i][k]*b[k][j];
        }
    }
}

void multiply_23_31(double a[2][3], double b[3][1], double result[2][1]) {
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 1; j++) {
            result[i][j] = 0;
            for (int k = 0; k < 3; k++)
                result[i][j] += a[i][k]*b[k][j];
        }
    }
}

void multiply_33_32(double a[3][3], double b[3][2], double result[3][2]) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 2; j++) {
            result[i][j] = 0;
            for (int k = 0; k < 3; k++)
                result[i][j] += a[i][k]*b[k][j];
        }
    }
}

void multiply_33_33(double a[3][3], double b[3][3], double result[3][3]) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            result[i][j] = 0;
            for (int k = 0; k < 3; k++)
                result[i][j] += a[i][k]*b[k][j];
        }
    }
}

void multiply_33_31(double a[3][3], double b[3][1], double result[3][1]) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 1; j++) {
            result[i][j] = 0;
            for (int k = 0; k < 3; k++)
                result[i][j] += a[i][k]*b[k][j];
        }
    }
}

void multiply_23_32(double a[2][3], double b[3][2], double result[2][2]) {
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) {
            result[i][j] = 0;
            for (int k = 0; k < 3; k++)
                result[i][j] += a[i][k]*b[k][j];
        }
    }
}

void multiply_32_21(double a[3][2], double b[2][1], double result[3][1]) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 1; j++) {
            result[i][j] = 0;
            for (int k = 0; k < 2; k++)
                result[i][j] += a[i][k]*b[k][j];
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
                result[i][j] += a[i][k]*b[k][j];
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

void divMatrix(double a[2][2], double b[2][2], double result[2][2]) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {

            result[i][j] = (a[i][j]/ (b[i][j]);
        }
    }
}
