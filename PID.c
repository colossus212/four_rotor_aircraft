/******************** (C) COPYRIGHT 2014 Air Nano Team ***************************
 * 文件名  ：PID.c
 * 描述    ：PID算法
 * 功能    ：
 * 函数：	void PID_Position(PID_Typedef * PID,float target,float measure)
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
	pitch_angle_PID.kp = 5.0 / 9;    //5
	pitch_angle_PID.ki = 0.01 / 9;
	pitch_angle_PID.kd = 1.7 / 9;    //2
	
	pitch_rate_PID.kp = 1.7 / 9;   //1.5
	pitch_rate_PID.ki = 0;
	pitch_rate_PID.kd = 0.22 / 9;  //0.16
	

	//The data of roll
	roll_angle_PID.kp = 5.0 / 9;
	roll_angle_PID.ki = 0.01 / 9;
	roll_angle_PID.kd = 1.7 / 9;

	roll_rate_PID.kp = 1.7 / 9;
	roll_rate_PID.ki = 0;
	roll_rate_PID.kd = 0.22 / 9;
	
	//The data of yaw
	yaw_angle_PID.kp = 5.0 / 9;
	yaw_angle_PID.ki = 0;
	yaw_angle_PID.kd = 1.7 / 9;
	
	yaw_rate_PID.kp = 1.7 / 9;
	yaw_rate_PID.ki = 0;
	yaw_rate_PID.kd = 0.1 / 9;

	alt_PID.kp = 5.0 / 7;
	alt_PID.ki = 0.01 / 7;
	alt_PID.kd = 1.7 / 7;

	alt_vel_PID.kp = 1.7 / 7;
	alt_vel_PID.ki = 0.01 / 7;
	alt_vel_PID.kd = 0.22 / 7;

	//limit for the max increment
	pitch_angle_PID.integ_max = 100;
	roll_angle_PID.integ_max = 100;
	yaw_angle_PID.integ_max = 100;
	
	pitch_rate_PID.integ_max = 200;
	roll_rate_PID.integ_max = 200;
	yaw_angle_PID.integ_max = 200;

	alt_PID.integ_max = 200;
	alt_vel_PID.integ_max = 200;

}





//-----------位置式PID-----------
void PID_Position(PID_Typedef * PID,float target,float measure)
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


void PID_Incremental(PID_Typedef * PID,float target,float measure)
{
	PID->error = target - measure;

	PID->output = PID->kp * (PID->error - PID->preerror) + PID->ki * PID->error + PID->kd * (PID->error - 2 * PID->preerror + PID->prepreerror);

	PID->preerror = PID->error;
	PID->prepreerror = PID->preerror;
}


void PID_Position_Fuzzy(PID_Typedef * PID,float target,float measure)
{
	float error_NB, error_NM, error_NS, error_ZO, error_PS, error_PM, error_PB;
	float preerror_NB, preerror_NM, preerror_NS, preerror_ZO, preerror_PS, preerror_PM, preerror_PB;
	float kp_NB, kp_NM, kp_NS, kp_ZO, kp_PS, kp_PM, kp_PB;
	float ki_NB, ki_NM, ki_NS, ki_ZO, ki_PS, ki_PM, ki_PB;
	float kd_NB, kd_NM, kd_NS, kd_ZO, kd_PS, kd_PM, kd_PB;
	float kp_offset, ki_offset, kd_offset;
	//haven't assign

	PID->error = target - measure;

	//rule list
	if((PID->error <= error_NB) && (PID->preerror <= preerror_NB))
	{
		kp_offset = kp_PB;
		ki_offset = ki_NB;
		kd_offset = kd_PS;
	}

	if((PID->error <= error_NB) && (PID->preerror > preerror_NB) && (PID->preerror <= preerror_NM))
	{
		kp_offset = kp_PB;
		ki_offset = ki_NB;
		kd_offset = kd_NS;
	}

	if((PID->error <= error_NB) && (PID->preerror > preerror_NM) && (PID->preerror <= preerror_NS))
	{
		kp_offset = kp_PM;
		ki_offset = ki_NM;
		kd_offset = kd_PB;
	}

	//a long rule list //to do

	PID->kp += kp_offset;
	PID->ki += ki_offset;
	PID->kd += kd_offset;

	PID->preerror = PID->error;

}