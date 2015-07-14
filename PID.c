/******************** (C) COPYRIGHT 2014 Air Nano Team ***************************
 * �ļ���  ��PID.c
 * ����    ��PID�㷨
 * ����    ��
 * ������	void PID_Postion_Cal(PID_Typedef * PID,float target,float measure)
			void PID_Parameter_Init()
/* ���룺	
/*      	
/*		
/* �����                                           */
/* ��ע������PID ����   �⻷���ǶȻ�������PID����   */
/*                     �ڻ������ٶȻ�������PD����  */
/**********************************************************************************/
#include "include.h"



//----PID�ṹ��ʵ����----
PID_Typedef pitch_angle_PID;	//pitch�ǶȻ���PID
PID_Typedef pitch_rate_PID;		//pitch�����ʻ���PID

PID_Typedef roll_angle_PID;   //roll�ǶȻ���PID
PID_Typedef roll_rate_PID;    //roll�����ʻ���PID

PID_Typedef yaw_angle_PID;    //yaw�ǶȻ���PID 
PID_Typedef yaw_rate_PID;     //yaw�����ʻ���PID

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





//-----------λ��ʽPID-----------
void PID_Postion_Cal(PID_Typedef * PID,float target,float measure)
{
		
	//��� = ����ֵ - ����ֵ
	PID->error = target - measure;
	
	PID->deriv = PID->error - PID->preerror;
	
	PID->integ = PID->integ + PID->error;     //���ֻ���
	
	if(PID->integ > PID->integ_max)
		PID->integ = PID->integ_max;   //���ƻ��ַ��ȣ����ɹ���
		else if(PID->integ < - PID->integ_max)
			PID->integ = - PID->integ_max;
			
	//PID:��������+���ֻ���+΢�ֻ���
	PID->output = (PID->kp * PID->error) + (PID->ki * PID->integ) + (PID->kd * PID->deriv);
	
	PID->preerror = PID->error;
		
}