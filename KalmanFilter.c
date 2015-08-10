
/************************************************
* File Name	: KalmanFilter.c
* Version	: 2015.7.15 By DHP
* Device(s)	: R5F100LE
* Tool-Chain	: CA78K0R
* Description	: MPU6050
* API		: KalmanFilterParameter_Load()
		  KalmanFilter(const double ResrcData, KalmanFilter_Typedef *  Kalman)

*******************************************************/

#include "include.h"
//#include "KalmanFilter.h"


	//Q: process noise, Q increases, the dynamic response is fast, and the convergence stability is bad.
	//R: measurement noise, R increases, the dynamic response is slow, and the convergence is good.
#define KALMAN_Q 0.02
#define KALMAN_R 2.0000	//6.0000

//KalmanFilter_Typedef Kalman_Ax, Kalman_Ay, Kalman_Az;
KalmanFilter_Typedef Kalman_Gx, Kalman_Gy, Kalman_Gz;
KalmanFilter_Typedef Kalman_Hx, Kalman_Hy, Kalman_Hz;
void KalmanFilterParameter_Init()
{
	//Kalman_Ax.Q = Kalman_Ay.Q = Kalman_Az.Q = KALMAN_Q;
	//Kalman_Ax.R = Kalman_Ay.R = Kalman_Az.R = KALMAN_R;
	Kalman_Gx.Q = Kalman_Gy.Q = Kalman_Gz.Q = KALMAN_Q;
	Kalman_Gx.R = Kalman_Gy.R = Kalman_Gz.R = KALMAN_R;
	Kalman_Hx.Q = Kalman_Hy.Q = Kalman_Hz.Q = KALMAN_Q;
	Kalman_Hx.R = Kalman_Hy.R = Kalman_Hz.R = KALMAN_R;
}


/*********** KalmanFilter ***********/

float  KalmanFilter(const float  ResrcData, KalmanFilter_Typedef *  Kalman)
{
   float x_mid;
   float  x_now;
   float  p_mid;
   float  p_now;
   float  kg;        

   x_mid = Kalman->x_last;		//x_last = x(k-1|k-1), x_mid = x(k|k-1)
   p_mid = Kalman->p_last + Kalman->Q;	//p_mid = p(k|k-1), p_last = p(k-1|k-1
   kg = p_mid / (p_mid + Kalman->R);			//kg is kalman filter gain
   x_now = x_mid + kg * (ResrcData - x_mid);//Estimated optimal value
                
   p_now = (1 - kg) * p_mid;	// covariance corresponding to the optimal value      
   Kalman->p_last = p_now; //update covariance
   Kalman->x_last = x_now; //Update system state values
   return x_now;          
 }