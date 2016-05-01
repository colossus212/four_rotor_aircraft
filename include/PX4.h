#include "r_cg_macrodriver.h"
#ifndef _PX4_
#define _PX4_


//typedef struct
//{
//	float kp;
//	float ki;
//	float kd;
//	float error;
//	float preerror;
//	float prepreerror;
//	float integ;
//	float integ_max;
//	float deriv;
//	float output; 
//}PID_Typedef;

void PX4_Init();
void PX4_Test();
void PX4_Unlock();
void PX4_Fly();
void PX4_Stop();
void PX4_Land();

void PWM_Flash(int16_t Roll, int16_t Pitch, int16_t Yaw, uint16_t Thr, uint16_t Model);

//void PID_Parameter_Reset_Height();
//void PID_Parameter_Load_Height(float PID_P, float PID_I, float PID_D);
//void PID_Parameter_Init();
//void PID_Position(PID_Typedef * PID,float target,float measure);

void PX4_Control_Height();





#endif