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

#include "Kalman_Filter.h"

/**
  * @brief	创建卡尔曼滤波
  * @param	创建的对象
  * @param	过程噪声协方差
  * @param	观测噪声协方差
  * @retval	void
  */
void KalmanFilter_Init(KalmanFilter_t *kf, float Q, float R)
{
	kf->Q		= Q;
	kf->R		= R;
	kf->P_last	= 0;
	kf->X_last	= 0;
	kf->X_mid	= kf->X_last;
}

/**
  * @brief	卡尔曼滤波函数，为简化运算，A取1，U(k)取0
  * @param	需要滤波的对象
  * @param	当前时刻的测量值Z(k)
  * @retval	当前时刻的最优值hat(X(k))
  */
float KalmanFilter_Calc(KalmanFilter_t *kf, float Z_mea)
{
	kf->X_mid	= kf->X_last;									//X(k|k-1)=A*X(k-1|k-1)+B*U(k)
	kf->P_mid	= kf->P_last + kf->Q;							//P(k|k-1)=A*P(k-1|k-1)*A'+Q
	kf->Kg		= kf->P_mid / (kf->P_mid + kf->R);				//Kg=P(k|k-1)*H'/(H*P(k|k-1)*H'+R)
	kf->X_last	= kf->X_mid + kf->Kg * (Z_mea - kf->X_mid);	//X(k|k)=X(k|k-1)+Kg*(Z(k)-H*X(k|k-1))
	kf->P_last	= (1 - kf->Kg) * kf->P_mid;					//P(k|k)=(1-Kg*H)*P(k|k-1)
	
	return kf->X_last;
}

/**
  * @brief	二阶卡尔曼初始化
  * @mark	为简化初始化流程，除Q、R矩阵外其他固定初值，需要再改
  * @param	需初始化的滤波器
  * @param	初始化的数据
  * @retval	void
  * @attention	数据存储于初始化的结构体中，若滤波器为全局则初始化数据也应为全局。
  */
void KalmanFilterSO_Init(KalmanFilterSO_t *kfso, KalmanFilterSO_Init_t *kfso_init)
{
	const float A_Init[4] = {1, 1, 0, 1};
	const float H_Init[4] = {1, 0, 0, 1};
	const float I_Init[4] = {1, 0, 0, 1};
	int i;
	for(i = 0; i < 4; i++)
	{
		kfso_init->A[i] = A_Init[i];
		kfso_init->H[i] = H_Init[i];
		kfso_init->I[i] = I_Init[i];
	}
	mat_init(&kfso->Q,		2, 2, kfso_init->Q);
	mat_init(&kfso->R,		2, 2, kfso_init->R);
	mat_init(&kfso->P_last,	2, 2, kfso_init->P_last);
	mat_init(&kfso->P_mid,	2, 2, kfso_init->P_mid);
	mat_init(&kfso->X_last,	2, 1, kfso_init->X_last);
	mat_init(&kfso->X_mid,	2, 1, kfso_init->X_mid);
	mat_init(&kfso->Kg,		2, 2, kfso_init->Kg);
	mat_init(&kfso->Z,		2, 1, kfso_init->Z);
	mat_init(&kfso->A,		2, 2, kfso_init->A);
	mat_init(&kfso->AT,		2, 2, kfso_init->AT);
	mat_trans(&kfso->A,		&kfso->AT);
	mat_init(&kfso->H,		2, 2, kfso_init->H);
	mat_init(&kfso->HT,		2, 2, kfso_init->HT);
	mat_trans(&kfso->H,		&kfso->HT);
	mat_init(&kfso->I,		2, 2, kfso_init->I);
}

/**
  * @brief	二阶卡尔曼计算
  * @param	使用的滤波器
  * @param	测量数据1
  * @param	测量数据2
  * @param	滤波结果1
  * @param	滤波结果2
  * @retval	void
  */
void KalmanFilterSO_Calc(KalmanFilterSO_t *kfso, float Z_m1, float Z_m2, float *X_n1, float *X_n2)
{
	float arr_temp22_1[4] = {0, 0, 0, 0};
	float arr_temp22_2[4] = {0, 0, 0, 0};
	float arr_temp21_1[2] = {0, 0};
	float arr_temp21_2[2] = {0, 0};
	mat mat_temp22_1, mat_temp22_2, mat_temp21_1, mat_temp21_2;
	
	mat_init(&mat_temp22_1, 2, 2, arr_temp22_1);
	mat_init(&mat_temp22_2, 2, 2, arr_temp22_2);
	mat_init(&mat_temp21_1, 2,1, arr_temp21_1);
	mat_init(&mat_temp21_2, 2,1, arr_temp21_2);
	
	kfso->Z.pData[0] = Z_m1;
	kfso->Z.pData[1] = Z_m2;
	
	//预测
	//1. X(k|k-1)=A*X(k-1|k-1)+B*U(k)
	mat_mult(&kfso->A, &kfso->X_last, &kfso->X_mid);
	//2. P(k|k-1)=A*P(k-1|k-1)*A'+Q
	mat_mult(&kfso->A, &kfso->P_last, &mat_temp22_1);
	mat_mult(&mat_temp22_1, &kfso->AT, &mat_temp22_2);
	mat_add(&mat_temp22_2, &kfso->Q, &kfso->P_mid);
	
	//校正
	//3. Kg=P(k|k-1)*H'/(H*P(k|k-1)*H'+R)
	mat_mult(&kfso->H, &kfso->P_mid, &mat_temp22_1);
	mat_mult(&mat_temp22_1, &kfso->HT, &mat_temp22_2);
	mat_add(&mat_temp22_2, &kfso->R, &mat_temp22_1);
	mat_inv(&mat_temp22_1, &mat_temp22_2);
	mat_mult(&kfso->P_mid, &kfso->HT, &mat_temp22_1);
	mat_mult(&mat_temp22_1, &mat_temp22_2, &kfso->Kg);
	//4. X(k|k)=X(k|k-1)+Kg*(Z(k)-H*X(k|k-1))
	mat_mult(&kfso->H, &kfso->X_mid, &mat_temp21_1);
	mat_sub(&kfso->Z, &mat_temp21_1, &mat_temp21_2);
	mat_mult(&kfso->Kg, &mat_temp21_2, &mat_temp21_1);
	mat_add(&kfso->X_mid, &mat_temp21_1, &kfso->X_last);
	//5. P(k|k)=(1-Kg*H)*P(k|k-1)
	mat_mult(&kfso->Kg, &kfso->H, &mat_temp22_1);
	mat_sub(&kfso->I, &mat_temp22_1, &mat_temp22_2);
	mat_mult(&mat_temp22_2, &kfso->P_mid, &kfso->P_last);
	
	*X_n1 = kfso->X_last.pData[0];
	*X_n2 = kfso->X_last.pData[1];
}

