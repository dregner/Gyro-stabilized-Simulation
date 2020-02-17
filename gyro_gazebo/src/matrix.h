//
// Created by vant3d on 17/02/2020.
//

#ifndef GYRO_GAZEBO_MATRIX_H
#define GYRO_GAZEBO_MATRIX_H

#pragma once


void multiply_23_33(double a[2][3], double b[3][3], double result[2][3]); 

void multiply_23_31(double a[2][3], double b[3][1], double result[2][1]); 

void multiply_33_32(double a[3][3], double b[3][2], double result[3][2]); 

void multiply_33_33(double a[3][3], double b[3][3], double result[3][3]); 

void multiply_33_31(double a[3][3], double b[3][1], double result[3][1]); 

void multiply_23_32(double a[2][3], double b[3][2], double result[2][2]); 

void multiply_32_21(double a[3][2], double b[2][1], double result[3][1]); 

void multiply_31_11(double a[3][1], double b, double result[3][1]); 

void multiply_32_23(double a[3][2], double b[2][3], double result[3][3]); 

void transpose_33(double a[3][3], double transpose[3][3]); 
    
void transpose_23(double a[2][3], double transpose[3][2]); 

void sub_33_33(double a[3][3], double b[3][3], double result[3][3]); 

void sub_21_21(double a[2][1], double b[2][1], double result[2][1]); 

void sum_33_33(double a[3][3], double b[3][3], double result[3][3]); 

void sum_31_31(double a[3][1], double b[3][1], double result[3][1]); 

void sum_22_22(double a[2][2], double b[2][2], double result[2][2]);

void divMatrix(double a[2][2], double b[2][2], double result[2][2]); 


#endif //GYRO_GAZEBO_MATRIX_H
