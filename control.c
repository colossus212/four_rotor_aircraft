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

#define MOTO_MAX 18000
#define MOTO_MIN 5500

#if 0
//r_cg_timer_user.c中 __interrupt static void r_tau0_channel5_interrupt(void)的备份
	//中断控制：
//	uint16_t height, tempr = 0, press = 0, IMUpersec = 100;
//	if(Count_for_Height == 8)
//	{
//		Flash_Height();
//		Control_Heigh(170);
//		Count_for_Height = 0;
//		AHRS_Captain_Flight_IMU(Get_Pitch(), Get_Roll(), Get_Yaw(), height, tempr, press, IMUpersec);
//		R_UART0_Send(Uart_Buf_IMU, 18);
//		Track();
//	}
//	
//	if(Count_for_Height == 4)
//	{
//		AHRS_Captain_Flight_Motion(Get_DMP_Acc_x(), Get_DMP_Acc_y(), Get_DMP_Acc_z(),
//			Get_DMP_Gyro_x(), Get_DMP_Gyro_y(), Get_DMP_Gyro_z(), Get_HMC5883L_Hx(), Get_HMC5883L_Hy(), Get_HMC5883L_Hz());
//
//		R_UART0_Send(Uart_Buf_Motion, 24);
//	}
//	
//	if(Count_for_Height == 0)
//	{
//		Flash_Height_Prepare();
//	}
//	
//	//Get_Attitude();  // haven't use DMP
//	
//	Get_Attitude_DMP();
//	
//	height = Get_Height();	
//
//	Control_Posture(0, 0, 0);
//	Count_for_Height++;
#endif


uint8_t Fly_flag;
volatile float MOTO1 = 0, MOTO2 = 0, MOTO3 = 0, MOTO4 = 0;


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
		
		
		MOTO1 += - pitch_rate_PID.output - roll_rate_PID.output + yaw_rate_PID.output;
		MOTO2 += + pitch_rate_PID.output - roll_rate_PID.output - yaw_rate_PID.output;
		MOTO3 += + pitch_rate_PID.output + roll_rate_PID.output + yaw_rate_PID.output;
		MOTO4 += - pitch_rate_PID.output + roll_rate_PID.output - yaw_rate_PID.output;
		
		if(MOTO1 > MOTO_MAX) MOTO1 = MOTO_MAX - 1;
		if(MOTO2 > MOTO_MAX) MOTO2 = MOTO_MAX - 1;
		if(MOTO3 > MOTO_MAX) MOTO3 = MOTO_MAX - 1;
		if(MOTO4 > MOTO_MAX) MOTO4 = MOTO_MAX - 1;

		if(MOTO1 <= MOTO_MIN) MOTO1 = MOTO_MIN;
		if(MOTO2 <= MOTO_MIN) MOTO2 = MOTO_MIN;
		if(MOTO3 <= MOTO_MIN) MOTO3 = MOTO_MIN;
		if(MOTO4 <= MOTO_MIN) MOTO4 = MOTO_MIN;

	if(Fly_flag == 1) Motor_RateFlash(MOTO1, MOTO2, MOTO3, MOTO4);
		else    Motor_RateFlash(0, 0, 0, 0);
}


/*************************************高度控制********************************************/

void Control_Heigh(float H)		//期望值,单位cm
{
	//static int j = 0;
	float AccH;
	float height = Get_Height();   //height是从超声波得到的数据
	height = height / sqrt(tan(Get_Pitch() * AtR) * tan(Get_Pitch() * AtR) + tan(Get_Roll() * AtR) * tan(Get_Roll() * AtR) + 1);
	
	//if(j >= 2)  //core 2   shell1  
	//{
			//*********外环高度PID***************//		
		PID_Position( & alt_PID, H, height);
		 //j = 0;
	//}
	//j ++;	
		//*******内环(加速度环)PID***********//
	AccH = sqrt(Get_DMP_Acc_x() * Get_DMP_Acc_x() + Get_DMP_Acc_y() * Get_DMP_Acc_y() + Get_DMP_Acc_z() * Get_DMP_Acc_z()); 
	PID_Position( & alt_vel_PID, alt_PID.output, (AccH - 8.82)* 100 );

	/********************************油门控制******************************************/

		MOTO1 += alt_vel_PID.output;
		MOTO2 += alt_vel_PID.output;
		MOTO3 += alt_vel_PID.output;
		MOTO4 += alt_vel_PID.output;
		
//		if(MOTO1 > MOTO_MAX) MOTO1 = MOTO_MAX - 1;
//		if(MOTO2 > MOTO_MAX) MOTO2 = MOTO_MAX - 1;
//		if(MOTO3 > MOTO_MAX) MOTO3 = MOTO_MAX - 1;
//		if(MOTO4 > MOTO_MAX) MOTO4 = MOTO_MAX - 1;

		if(MOTO1 <= MOTO_MIN) MOTO1 = MOTO_MIN;
		if(MOTO2 <= MOTO_MIN) MOTO2 = MOTO_MIN;
		if(MOTO3 <= MOTO_MIN) MOTO3 = MOTO_MIN;
		if(MOTO4 <= MOTO_MIN) MOTO4 = MOTO_MIN;

	if(Fly_flag == 1) Motor_RateFlash(MOTO1, MOTO2, MOTO3, MOTO4);
	else    Motor_RateFlash(0, 0, 0, 0);
}


void Control_Standby()
{
	Fly_flag = 0;
}

void Control_Fly()
{
	Fly_flag = 1;
}

void Control_Track()
{
	Control_Posture(0, 0, Track());
	//Control_Heigh(120);
}
