/***//****************************************
      https://github.com/RagingWaves
----------------------------------------------
 * @file		Kalman_Filter.c/h
 * @dependent	Matrix.h矩阵运算库
 * @brief		通用卡尔曼滤波器
 * @mark		1.	一阶卡尔曼测试通过--22.2.26
				2.	增加二阶卡尔曼并测试通过--22.4.4
 * @version		V1.1.0
 * @auther		ZYuan
 * @url			
----------------------------------------------
****************************************//***/

#ifndef __KALMAN_FILTER_H
#define __KALMAN_FILTER_H

#include "Matrix.h"

typedef struct {
	float Q;		//过程噪声协方差
	float R;		//观测噪声协方差
	float Kg;		//卡尔曼增益
	float P_last;	//P(k-1|k-1)上一时刻最优值协方差、P(k|k)当前时刻最优值协方差
	float P_mid;	//P(k|k-1)	当前时刻预测值协方差
	float X_last;	//X(k-1|k-1)上一时刻最优值、当前时刻最优值
	float X_mid;	//X(k|k-1)	当前时刻预测值
} KalmanFilter_t;

typedef struct {
	mat Q;		//过程噪声协方差
	mat R;		//观测噪声协方差
	mat Kg;		//卡尔曼增益
	mat P_last;	//P(k-1|k-1)上一时刻最优值协方差、P(k|k)当前时刻最优值协方差
	mat P_mid;	//P(k|k-1)	当前时刻预测值协方差
	mat X_last;	//X(k-1|k-1)上一时刻最优值
	mat X_mid;	//X(k|k-1)	当前时刻预测值、当前时刻最优值
	mat A;
	mat AT;
	mat H;
	mat HT;
	mat Z;
	mat I;
} KalmanFilterSO_t;

//@attention	数据存储于初始化的结构体中，若滤波器为全局则初始化数据也应为全局。
typedef struct {
	float Q[4];
	float R[4];
	float Kg[4];
	float P_last[4];
	float P_mid[4];
	float X_last[2];
	float X_mid[2];
	float Z[2];
	float A[4];
	float AT[4];
	float H[4];
	float HT[4];
	float I[4];
} KalmanFilterSO_Init_t;

void KalmanFilter_Init(KalmanFilter_t *kf, float Q, float R);
float KalmanFilter_Calc(KalmanFilter_t *kf, float Z_mea);
void KalmanFilterSO_Init(KalmanFilterSO_t *kfso, KalmanFilterSO_Init_t *kfso_init);
void KalmanFilterSO_Calc(KalmanFilterSO_t *kfso, float Z_m1, float Z_m2, float *X_n1, float *X_n2);

#endif

