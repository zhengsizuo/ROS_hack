/*
 * FileName : kalman_filter.h
 * Author   : xiahouzuoxin @163.com
 * Version  : v1.0
 * Date     : 2014/9/24 20:37:01
 * Brief    : 
 * 
 * Copyright (C) MICL,USTB
 */
#ifndef  _KALMAN
#define  _KALMAN

#include <Eigen/Eigen>


/* 1 Dimension */
struct kalman1_state{
    float x;  /* state */
    float A;  /* x(n)=A*x(n-1)+u(n),u(n)~N(0,q) */
    float H;  /* z(n)=H*x(n)+w(n),w(n)~N(0,r)   */
    float q;  /* process(predict) noise convariance */
    float r;  /* measure noise convariance */
    float p;  /* estimated error convariance */
    float gain;
} ;

/* 2 Dimension */
struct kalman2_state{
    Eigen::Matrix<float, 2, 1> x;  
    Eigen::Matrix<float, 2, 2> A;  /* X(n)=A*X(n-1)+U(n),U(n)~N(0,q), 2x2 */
    Eigen::Matrix<float, 2, 2> H;     /* Z(n)=H*X(n)+W(n),W(n)~N(0,r), 1x2   */
    Eigen::Matrix<float, 2, 2> q;     /* process(predict) noise convariance,2x1 [q0,0; 0,q1] */
    Eigen::Matrix<float, 2, 2> r;        /* measure noise convariance */
    Eigen::Matrix<float, 2, 2> p;  /* estimated error convariance,2x2 [p0 p1; p2 p3] */
    Eigen::Matrix<float, 2, 1> gain;  /* 2x1 */
} ;

/* 
 * NOTES: n Dimension means the state is n dimension, 
 * measurement always 1 dimension 
 */
//extern struct  kalman1_state;
//extern struct  kalman2_state;          

extern void kalman1_init(kalman1_state *state, float init_x, float init_p);
extern float kalman1_filter(kalman1_state *state, float z_measure);
extern void kalman2_init(kalman2_state *state, float *init_x, float (*init_p)[2]);
extern float kalman2_filter(kalman2_state *state, float z_measure);

#endif  /*_KALMAN_FILTER_H*/
