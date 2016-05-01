/******************** (C) COPYRIGHT 2014 Air Nano Team ***************************
* File Name	: control.c
 * Version	: 2015.7.10 By DHP
 * Device(s)	: R5F100LE
 * Tool-Chain	: CA78K0R
 * description	: control Posture and Heigh
 * function		: void Control_Posture(float roll, float pitch, float yaw);
		  		  void Control_Heigh(float H);
*********************************************************************************/

#include "include.h"
#include  <math.h>

#define MOTO_MAX 40000
#define MOTO_MIN 0


uint8_t Fly_flag;
uint8_t Land_Flag = 0;
volatile float MOTO1 = 0, MOTO2 = 0, MOTO3 = 0, MOTO4 = 0;
float MOTO_THRESHOLD1 = 0, MOTO_THRESHOLD2 = 0, MOTO_THRESHOLD3 = 0, MOTO_THRESHOLD4 = 0;
float Roll_Target = 0, Pitch_Target = 0, Yaw_Target = 0, Height_Target = 100;


/******************************************************************************
* function :		void MOTO_Limiter()
* Description : 
*******************************************************************************/
 void MOTO_Limiter()
 {
	 	if(MOTO1 > MOTO_MAX) MOTO1 = MOTO_MAX;
		if(MOTO2 > MOTO_MAX) MOTO2 = MOTO_MAX;
		if(MOTO3 > MOTO_MAX) MOTO3 = MOTO_MAX;
		if(MOTO4 > MOTO_MAX) MOTO4 = MOTO_MAX;

		if(MOTO1 <= MOTO_MIN) MOTO1 = MOTO_MIN;
		if(MOTO2 <= MOTO_MIN) MOTO2 = MOTO_MIN;
		if(MOTO3 <= MOTO_MIN) MOTO3 = MOTO_MIN;
		if(MOTO4 <= MOTO_MIN) MOTO4 = MOTO_MIN;
 }

/************************************************************************************/

void Control_Posture(float roll, float pitch, float yaw)
{
		//static float roll_old,pitch_old;
	static int i = 0;
	if(i >= 2)
	{
		PID_Position( & pitch_angle_PID, pitch, Get_Pitch());	////
		PID_Position( & roll_angle_PID, roll, Get_Roll());	////
		PID_Position( & yaw_angle_PID, yaw, Get_Yaw());		////
		i = 0;
	}
	i ++;
		//************()PD***************//	
	PID_Position( & pitch_rate_PID, pitch_angle_PID.output, Get_DMP_Gyro_x());	////
	PID_Position( & roll_rate_PID, roll_angle_PID.output, Get_DMP_Gyro_y());	////
	PID_Position( & yaw_rate_PID, yaw_angle_PID.output, - Get_DMP_Gyro_z());	////
		//if don't use DMP
	//PID_Position( & pitch_rate_PID, pitch_angle_PID.output, Get_MPU6050_Gx());	////
	//PID_Position( & roll_rate_PID, roll_angle_PID.output, Get_MPU6050_Gy());	////
	//PID_Position( & yaw_rate_PID, yaw_angle_PID.output, - Get_MPU6050_Gz());	////


	/**************************************************************************/
		/*		X MODE		*/
		/*						*/	
		/*          1 head 4 		 */
		/*            \   / 		 */ 
		/*             \ /  		 */
		/*             / \                */
		/*            /   \               */
		/*           2     3              */
		/* 1: MOTO1			 2: MOTO1	  */
		/* 3: MOTO3			 4: MOTO4	  */	
		
		MOTO1 = MOTO_THRESHOLD1 - pitch_rate_PID.output - roll_rate_PID.output + yaw_rate_PID.output + alt_PID.output;
		MOTO2 = MOTO_THRESHOLD2 + pitch_rate_PID.output - roll_rate_PID.output - yaw_rate_PID.output + alt_PID.output;
		MOTO3 = MOTO_THRESHOLD3 + pitch_rate_PID.output + roll_rate_PID.output + yaw_rate_PID.output + alt_PID.output;
		MOTO4 = MOTO_THRESHOLD4 - pitch_rate_PID.output + roll_rate_PID.output - yaw_rate_PID.output + alt_PID.output;
		
	MOTO_Limiter();

	if(Fly_flag == 1) Motor_RateFlash(MOTO1, MOTO2, MOTO3, MOTO4);
		else    Motor_RateFlash(0, 0, 0, 0);
}


/***************************************************************************/

void Control_Heigh(float H)		
{
	PID_Incremental( & alt_PID, H, Get_Height());	
	

//	MOTO1 +=  alt_PID.output;
//	MOTO2 +=  alt_PID.output;
//	MOTO3 +=  alt_PID.output;
//	MOTO4 +=  alt_PID.output;
//	
//	MOTO_Limiter();
//
//	if(Fly_flag == 1) Motor_RateFlash(MOTO1, MOTO2, MOTO3, MOTO4);
//	else    Motor_RateFlash(0, 0, 0, 0);
}

uint8_t Get_Fly_Flag()
{
	return Fly_flag;
}
void Control_Fly_Flag_Off()
{
	Fly_flag = 0;
}

void Control_Fly_Flag_On()
{
	Fly_flag = 1;
}

void Control_Track_Frame()
{
	Yaw_Target = Track_Frame();
}

void Control_Track_Motion()
{
	Track_Motion();
}

void Control_Fly()
{
	R_TAU0_Channel5_Start();
	delay_ms(100);
	MOTO_THRESHOLD1 = 17000, MOTO_THRESHOLD2 = 19000, MOTO_THRESHOLD3 = 17000, MOTO_THRESHOLD4 = 17000; //27500;
	Motor_RateFlash(MOTO_THRESHOLD1, MOTO_THRESHOLD2, MOTO_THRESHOLD3, MOTO_THRESHOLD4);
	//delay_ms(100);
	delay_ms(10);	
	Control_Fly_Flag_On();			
}

void Control_Test()
{
	MOTO_THRESHOLD1 = 7500, MOTO_THRESHOLD2 = 8500, MOTO_THRESHOLD3 = 7500, MOTO_THRESHOLD4 = 7500; //27500;
	Motor_RateFlash(MOTO_THRESHOLD1, MOTO_THRESHOLD2, MOTO_THRESHOLD3, MOTO_THRESHOLD4);
}

void Control_Land()
{
	if(Get_Height() > 28)
	{
		Height_Target = Get_Height() - 5;
		Control_Heigh(Height_Target);
	}
	if(Get_Height() <= 28)
	{
		Control_Heigh(23.0);
		if(Get_Height() <= 16.0)
		{
			Motor_RateFlash(18000, 18000, 18000, 18000);
			delay_s(1);
			Motor_RateFlash(0, 0, 0, 0);
		}
	}
}

void Control_Free_Land()
{
	R_TAU0_Channel5_Stop();
	MOTO_THRESHOLD1 = 0, MOTO_THRESHOLD2 = 0, MOTO_THRESHOLD3 = 0, MOTO_THRESHOLD4 = 0;
	Motor_RateFlash(0, 0, 0, 0);
	MOTO1 = 0; MOTO2 = 0; MOTO3 = 0; MOTO4 = 0;
	//TDR01 = 4000; TDR02 = 4000; TDR03 = 4000; TDR04 = 4000;
}