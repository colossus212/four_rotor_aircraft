/******************** (C) COPYRIGHT 2014 Air Nano Team ***************************
 * 文件名  ：control.c
 * Version	: 2015.7.10 By DHP
 * Device(s)	: R5F100LE
 * Tool-Chain	: CA78K0R
 * description  ：control Posture and Heigh
 * function：  void Control_Posture(float roll, float pitch, float yaw);
		  void Control_Heigh(float H);
*********************************************************************************/

#include "include.h"
#include  <math.h>

#define MOTO_MAX 35000
#define MOTO_MIN 20000
		//6000

#if 0
//r_cg_timer_user.c中 __interrupt static void r_tau0_channel5_interrupt(void)的备份
	//中断控制：
//if(Count_for_Timer_8 == 8)
//	{
//		Count_for_Timer_8 = 0;
////		Flash_Height();
////		Control_Heigh(Height_Target); // Initialization value is 100 (cm)
//
//		//Control_Track();
//	}
//	
////	if(Count_for_Timer_8 == 6)
////	{
////		;
////	}
////	if(Count_for_Timer_8 == 5)
////	{
////		;
////	}
////	if(Count_for_Timer_8 == 4)
////	{
////		GY953_Get_Mag();
////	}	
////	if(Count_for_Timer_8 == 3)
////	{
////		GY953_Get_Gyro();
////	}
////	
////	if(Count_for_Timer_8 == 2)
////	{
////		GY953_Get_Accl();
////	}
////	
////	if(Count_for_Timer_8 == 1)
////	{
////		GY953_Get_RPY();
////	}	
//
//	if(Count_for_Timer_8 == 0)
//	{
////		Flash_Height_Prepare();
//	}
//	
//	//Get_Attitude();  // haven't use DMP
//	
//	Get_Attitude_DMP();
//
////	height = Get_Height();
//
//	Control_Posture(Roll_Target, Pitch_Target, Yaw_Target);
//
//	Count_for_Timer_8++;
//	
//     //P13.0 = ~P13.0;
#endif





uint8_t Fly_flag;
uint8_t Land_Flag = 0;
volatile float MOTO1 = 0, MOTO2 = 0, MOTO3 = 0, MOTO4 = 0;
float MOTO_THRESHOLD = 0;
float Roll_Target = 0, Pitch_Target = 0, Yaw_Target = 0, Height_Target = 100;


/******************************************************************************
* function :		void MOTO_Limiter()
* Description : 
*******************************************************************************/
 void MOTO_Limiter()
 {
	 	if(MOTO1 > MOTO_MAX) MOTO1 = MOTO_MAX - 1;
		if(MOTO2 > MOTO_MAX) MOTO2 = MOTO_MAX - 1;
		if(MOTO3 > MOTO_MAX) MOTO3 = MOTO_MAX - 1;
		if(MOTO4 > MOTO_MAX) MOTO4 = MOTO_MAX - 1;

		if(MOTO1 <= MOTO_MIN) MOTO1 = MOTO_MIN;
		if(MOTO2 <= MOTO_MIN) MOTO2 = MOTO_MIN;
		if(MOTO3 <= MOTO_MIN) MOTO3 = MOTO_MIN;
		if(MOTO4 <= MOTO_MIN) MOTO4 = MOTO_MIN;
 }

/****************************************姿态控制********************************************/

/*备注：串级PID 控制   外环（角度环）采用PID调节   */
/*                     内环（角速度环）采用PD调节  */

void Control_Posture(float roll, float pitch, float yaw)
{
		//static float roll_old,pitch_old;
	static int i = 0;
	if(i >= 2)
	{
		PID_Position( & pitch_angle_PID, pitch, Get_Pitch());	//俯仰计算//
		PID_Position( & roll_angle_PID, roll, Get_Roll());	//横滚计算//
		PID_Position( & yaw_angle_PID, yaw, Get_Yaw());		//航向计算//
		i = 0;
	}
	i ++;
		//************内环(角速度环)PD***************//	
	PID_Position( & pitch_rate_PID, pitch_angle_PID.output, Get_DMP_Gyro_x());	//俯仰计算//
	PID_Position( & roll_rate_PID, roll_angle_PID.output, Get_DMP_Gyro_y());	//横滚计算//
	PID_Position( & yaw_rate_PID, yaw_angle_PID.output, - Get_DMP_Gyro_z());	//航向计算//
		//if don't use DMP
	//PID_Position( & pitch_rate_PID, pitch_angle_PID.output, Get_MPU6050_Gx());	//俯仰计算//
	//PID_Position( & roll_rate_PID, roll_angle_PID.output, Get_MPU6050_Gy());	//横滚计算//
	//PID_Position( & yaw_rate_PID, yaw_angle_PID.output, - Get_MPU6050_Gz());	//航向计算//


	/********************************油门控制******************************************/
		/*		控制采用X模式		*/
		/*						*/	
		/*          1 机头 4 		 */
		/*            \   / 		 */ 
		/*             \ /  		 */
		/*             / \                */
		/*            /   \               */
		/*           2     3              */
		/* 1: MOTO1			 2: MOTO1	  */
		/* 3: MOTO3			 4: MOTO4	  */	
		
		MOTO1 = MOTO_THRESHOLD - pitch_rate_PID.output - roll_rate_PID.output + yaw_rate_PID.output + alt_PID.output;
		MOTO2 = MOTO_THRESHOLD + pitch_rate_PID.output - roll_rate_PID.output - yaw_rate_PID.output + alt_PID.output;
		MOTO3 = MOTO_THRESHOLD + pitch_rate_PID.output + roll_rate_PID.output + yaw_rate_PID.output + alt_PID.output;
		MOTO4 = MOTO_THRESHOLD - pitch_rate_PID.output + roll_rate_PID.output - yaw_rate_PID.output + alt_PID.output;
		
	MOTO_Limiter();

	if(Fly_flag == 1) Motor_RateFlash(MOTO1, MOTO2, MOTO3, MOTO4);
		else    Motor_RateFlash(0, 0, 0, 0);
}


/*************************************高度控制********************************************/

void Control_Heigh(float H)		//期望值,单位cm
{
	PID_Incremental( & alt_PID, H, Get_Height());	
	
		//油门控制
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

void Control_Track()
{
	Control_Posture(0, 0, Track());
	//Height_Target = 120;
}

void Control_Fly()
{
	R_TAU0_Channel5_Start();
	delay_ms(100);
	MOTO_THRESHOLD = 27500; //27500;
	Motor_RateFlash(MOTO_THRESHOLD, MOTO_THRESHOLD, MOTO_THRESHOLD, MOTO_THRESHOLD);
	//delay_ms(100);
	delay_ms(10);	
	Control_Fly_Flag_On();			
}

void Control_Test()
{
	MOTO_THRESHOLD = 7500;
	Motor_RateFlash(MOTO_THRESHOLD, MOTO_THRESHOLD, MOTO_THRESHOLD, MOTO_THRESHOLD);
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
	MOTO_THRESHOLD = 0;
	Motor_RateFlash(0, 0, 0, 0);
	MOTO1 = 0; MOTO2 = 0; MOTO3 = 0; MOTO4 = 0;
	TDR01 = 2000; TDR02 = 2000; TDR03 = 2000; TDR04 = 2000;
}