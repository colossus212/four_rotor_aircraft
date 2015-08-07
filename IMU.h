#include "r_cg_macrodriver.h"

#ifndef _IMU_
#define _IMU_

#define RtA 	57.324841        //  180/3.1415  	
#define AtR    	0.0174533		//  1/RtA        

#define isnan(x)    ((x) != (x))

struct _angle
{
        float pitch;
        float roll;
        float yaw;
};

float COS(float x);
float SIN(float x);

void Prepare_Data(void);
void Get_Attitude();
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az);
void Get_Attitude_DMP();

float Get_Pitch();
float Get_Roll();
float Get_Yaw();

//void get_MPUdata();



#endif