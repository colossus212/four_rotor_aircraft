/******************** (C) COPYRIGHT 2014 Air Nano Team ***************************
 * 文件名  ：control.c
 * Version	: 2015.7.10 By DHP
 * Device(s)	: R5F100LE
 * Tool-Chain	: CA78K0R
 * 描述    ：控制
 *函数：  void Control_posture(float roll, float pitch, float yaw);
		  void Control_Heigh(float H);

		  // to do
		  // void Control(float roll, float pitch, float yaw, float H);
		  // {
		  // 	
		  // 	void Control_posture(roll, pitch, yaw);
		  // 	void Control_Heigh(H);  //因为超声波读取高度需要 25+ ms，所以高度控制和姿态控制的频率不应该一样
		  // 
		  // }


*********************************************************************************/

#include "include.h"
#include  <math.h>


#if 0
//r_cg_timer_user.c中 __interrupt static void r_tau0_channel5_interrupt(void)的备份
	//中断控制：
	//uint8_t Buf[10];

	//if(Count_for_Height == 8)
	//{
	//	Flash_Height();
	//	Buf[6] = ((int16_t) Get_Height()) >> 8;
	//	Buf[7] = ((int16_t) Get_Height()) & 0x00ff;
	//	R_UART0_Send(Buf + 6, 2);
	//	Control_Heigh(40);
	//	Count_for_Height = 0;
	//}
	//
	//if(Count_for_Height == 0)
	//{
	//	Flash_Height_Prepare();
	//}
	//
	//Get_Attitude();

	//	Buf[0] = ((int16_t) angle.yaw) >> 8;
	//	Buf[1] = ((int16_t) angle.yaw) & 0x00ff;
	//	Buf[2] = ((int16_t) angle.roll) >> 8;
	//	Buf[3] = ((int16_t) angle.roll) & 0x00ff;
	//	Buf[4] = ((int16_t) angle.pitch) >> 8;
	//	Buf[5] = ((int16_t) angle.pitch) & 0x00ff;
	//	R_UART0_Send(Buf, 6);

	//Control_posture(0, 0, 0);
	//Count_for_Height++;
#endif


uint8_t Fly_flag;
volatile int16_t MOTO1 = 0, MOTO2 = 0, MOTO3 = 0, MOTO4 = 0;


/****************************************姿态控制********************************************/

/*备注：串级PID 控制   外环（角度环）采用PID调节   */
/*                     内环（角速度环）采用PD调节  */

void Control_posture(float roll, float pitch, float yaw)
{
	//static float roll_old,pitch_old;
	static int i = 0;

	if(i >= 2)
	{
		PID_Position( & pitch_angle_PID, pitch, angle.pitch);	//俯仰计算//
		PID_Position( & roll_angle_PID, roll, angle.roll);	//横滚计算//
		PID_Position( & yaw_angle_PID, yaw, angle.yaw);		//航向计算//
		i = 0;
	}
	i ++;	
	//************内环(角速度环)PD***************//
	PID_Position( & pitch_rate_PID, pitch_angle_PID.output, MPU6050_data.Gx * RtA);	//俯仰计算//
	PID_Position( & roll_rate_PID, roll_angle_PID.output, MPU6050_data.Gy * RtA);	//横滚计算//
	PID_Position( & yaw_rate_PID, yaw_angle_PID.output, MPU6050_data.Gz * RtA);		//航向计算//


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

	if(Fly_flag == 1) Motor_RateFlash(MOTO1, MOTO2, MOTO3, MOTO4);
	else    Motor_RateFlash(0, 0, 0, 0);
}


/*************************************高度控制********************************************/

void Control_Heigh(float H)		//期望值,单位cm
{
	//static int j = 0;
	float height = Get_Height();   //height是从超声波得到的数据

	//if(j >= 2)  //core 2   shell1  
	//{
		//*********外环高度PID***************//		
		PID_Position( & alt_PID, H, height);
		 //j = 0;
	//}
	//j ++;	
		//*******内环(加速度环)PID***********//
	PID_Position( & alt_vel_PID, alt_PID.output, MPU6050_data.Az * 100 );

	/********************************油门控制******************************************/

		MOTO1 += alt_vel_PID.output;
		MOTO2 += alt_vel_PID.output;
		MOTO3 += alt_vel_PID.output;
		MOTO4 += alt_vel_PID.output;

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