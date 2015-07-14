/******************** (C) COPYRIGHT 2014 Air Nano Team ***************************
 * 文件名  ：PID.c
 * 描述    ：PID算法
 * 功能    ：
 * 函数：	void PID_Postion_Cal(PID_Typedef * PID,float target,float measure)
			void PID_Parameter_Init()
/* 输入：	
/*      	
/*		
/* 输出：                                           */
/* 备注：串级PID 控制   外环（角度环）采用PID调节   */
/*                     内环（角速度环）采用PD调节  */
/**********************************************************************************/
#include "include.h"



//----PID结构体实例化----
PID_Typedef pitch_angle_PID;	//pitch角度环的PID
PID_Typedef pitch_rate_PID;		//pitch角速率环的PID

PID_Typedef roll_angle_PID;   //roll角度环的PID
PID_Typedef roll_rate_PID;    //roll角速率环的PID

PID_Typedef yaw_angle_PID;    //yaw角度环的PID 
PID_Typedef yaw_rate_PID;     //yaw角速率环的PID

PID_Typedef	alt_PID;
PID_Typedef alt_vel_PID;



void PID_Parameter_Init()
{
	// The data of pitch
	pitch_angle_PID.kp = 5;    //5
	pitch_angle_PID.ki = 0.01;
	pitch_angle_PID.kd = 1.7;    //2
	
	pitch_rate_PID.kp = 1.9;   //1.5
	pitch_rate_PID.ki = 0;
	pitch_rate_PID.kd = 0.22;  //0.16
	

	//The data of roll
	roll_angle_PID.kp = 5;
	roll_angle_PID.ki = 0.01;
	roll_angle_PID.kd = 1.7;

	roll_rate_PID.kp = 1.9;
	roll_rate_PID.ki = 0;
	roll_rate_PID.kd = 0.22;
	
	//The data of yaw
	yaw_angle_PID.kp = 5;
	yaw_angle_PID.ki = 0;
	yaw_angle_PID.kd = 0.13;
	
	yaw_rate_PID.kp = 1.8;
	yaw_rate_PID.ki = 0;
	yaw_rate_PID.kd = 0.1;

	//limit for the max increment
	pitch_angle_PID.integ_max = 200;
	roll_angle_PID.integ_max = 200;

	alt_PID.kp = 5;
	alt_PID.ki = 0.01;
	alt_PID.kd = 0.1;

	alt_vel_PID.kp = 5;    //5
	alt_vel_PID.ki = 0.01;
	alt_vel_PID.kd = 1.7;    //2
	
}





//-----------位置式PID-----------
void PID_Postion_Cal(PID_Typedef * PID,float target,float measure)
{
		
	//误差 = 期望值 - 测量值
	PID->error = target - measure;
	
	PID->deriv = PID->error - PID->preerror;
	
	PID->integ = PID->integ + PID->error;     //积分环节
	
	if(PID->integ > PID->integ_max)
		PID->integ = PID->integ_max;   //限制积分幅度，不可过大
		else if(PID->integ < - PID->integ_max)
			PID->integ = - PID->integ_max;
			
	//PID:比例环节+积分环节+微分环节
	PID->output = (PID->kp * PID->error) + (PID->ki * PID->integ) + (PID->kd * PID->deriv);
	
	PID->preerror = PID->error;
		
}