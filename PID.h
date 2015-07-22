#include "r_cg_macrodriver.h"

#ifndef _PID_
#define _PID_

// PID Struct
typedef struct
{
	float kp;
	float ki;
	float kd;
	float error;
	float preerror;
	float prepreerror;
	float integ;
	float integ_max;
	float deriv;
	float output; 
}PID_Typedef;

//----PID�ṹ��ʵ����----
extern PID_Typedef pitch_angle_PID;	//pitch�ǶȻ���PID
extern PID_Typedef pitch_rate_PID;		//pitch�����ʻ���PID

extern PID_Typedef roll_angle_PID;   //roll�ǶȻ���PID
extern PID_Typedef roll_rate_PID;    //roll�����ʻ���PID

extern PID_Typedef yaw_angle_PID;    //yaw�ǶȻ���PID 
extern PID_Typedef yaw_rate_PID;     //yaw�����ʻ���PID

extern PID_Typedef	alt_PID;
extern PID_Typedef alt_vel_PID;


void PID_Parameter_Init();
void PID_Position(PID_Typedef * PID,float target,float measure);
void PID_Incremental(PID_Typedef * PID,float target,float measure);
void PID_Position_Fuzzy(PID_Typedef * PID,float target,float measure);


#endif


