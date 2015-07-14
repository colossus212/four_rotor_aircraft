#include "r_cg_macrodriver.h"
#ifndef _PID_
#define _PID_

#include "include.h"


// PID�ṹ��
typedef struct
{
    float kp;
    float ki;
    float kd;
    float error;
    float preerror;
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
void PID_Postion_Cal(PID_Typedef * PID,float target,float measure);







#endif


