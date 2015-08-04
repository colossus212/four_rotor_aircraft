#ifndef __KALMANFILTER_
#define	__KALMANFILTER_

#include "r_cg_macrodriver.h"

// KalmanFilter Struct
typedef struct
{
    float  x_last;
    float  p_last;
    float  R;
	float Q;
}KalmanFilter_Typedef;

extern KalmanFilter_Typedef Kalman_Ax, Kalman_Ay, Kalman_Az, Kalman_Gx, Kalman_Gy, Kalman_Gz;

void KalmanFilterParameter_Init();
float  KalmanFilter(const float  ResrcData, KalmanFilter_Typedef *  Kalman);

#endif