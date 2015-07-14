#include "r_cg_macrodriver.h"

#ifndef _IMU_
#define _IMU_

#define RtA 	57.324841        //  180/3.1415  	
#define AtR    	0.0174533		//  1/RtA             RtA倒数

//#define Acc_G 	0.0011963		//  1/32768/4/9.8     加速度量程为4G		
//#define Gyro_G 	0.03051756	//  1/32768/1000      陀螺仪量程为 +—1000			
//#define Gyro_Gr	0.0005327   //  1/32768/1000/57.3 

// KalmanFilter结构体
typedef struct
{
    double x_last;
    double p_last;
    double R;
	double Q;
}KalmanFilter_Typedef;


struct _angle
{
        float pitch;
        float roll;
        float yaw;
};

extern struct _angle angle;

void Prepare_Data(void);
void Get_Attitude();
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az);


//void get_MPUdata();



#endif