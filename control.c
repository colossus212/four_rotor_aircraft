/******************** (C) COPYRIGHT 2014 Air Nano Team ***************************
 * 文件名  ：control.c
 * 描述    ：控制         
 * 功能    ：控制 
 *函数：  void Control_posture(float roll, float pitch, float yaw);
		  void Control_altitude(float H);
		  void Control_Motor(void);

		  // to do
		  // void Control(float roll, float pitch, float yaw, float H);
		  // {
		  // 	
		  // 	void Control_posture(roll, pitch, yaw);
		  // 	void Control_altitude(H);  //因为超声波读取高度需要 25+ ms，所以高度控制和姿态控制的频率不应该一样
		  // 	void Control_Motor();
		  // }


/**********************************************************************************/

#include "include.h"
#include  <math.h>


#if 0
//r_cg_timer_user.c中 __interrupt static void r_tau0_channel5_interrupt(void)的备份
	//中断控制：
	//static uint8_t i;
	//if(i = 30)
	//{
	//	Get_Height();
	//	i = 0;
	//}
	//Get_Attitude();
	//i++；	
#endif



/****************************************姿态控制********************************************/

/*备注：串级PID 控制   外环（角度环）采用PID调节   */
/*                     内环（角速度环）采用PD调节  */

void Control_posture(float roll, float pitch, float yaw)		//在中断3中反复出现,暂时没看懂，要着重理解
{
	//static float roll_old,pitch_old;
	static int i = 0;

	if(i >= 2)  //内环进行2次控制   外环进行1次控制   内环控制频率为外环的2倍 
	{
		PID_Postion_Cal( & pitch_angle_PID, pitch, angle.pitch);	//俯仰计算//
		PID_Postion_Cal( & roll_angle_PID, roll, angle.roll);	//横滚计算//
		PID_Postion_Cal( & yaw_angle_PID, yaw, angle.yaw);		//航向计算//
		i = 0;
	}
	i ++;	
	//************内环(角速度环)PD***************//
	PID_Postion_Cal( & pitch_rate_PID, pitch_angle_PID.output, MPU6050_data.Gx);	//俯仰计算//
	PID_Postion_Cal( & roll_rate_PID, roll_angle_PID.output, MPU6050_data.Gy);	//横滚计算//
	PID_Postion_Cal( & yaw_rate_PID, yaw_angle_PID.output, MPU6050_data.Gz);		//航向计算//


}


/*************************************高度控制********************************************/

void Control_altitude(float H)		//期望值,单位cm
{
	static int j = 0;
	float height = Get_Height();   //height是从超声波得到的数据

	if(j >= 2)  //内环进行2次控制   外环进行1次控制   内环控制频率为外环的2倍 
	{
		//*********外环高度PID***************//		
		PID_Postion_Cal( & alt_PID, H, height);
		 j = 0;
	}
	j ++;	
		//*******内环(加速度环)PID***********//
	PID_Postion_Cal( & alt_vel_PID, alt_PID.output, MPU6050_data.Az);

}


int16_t pitchrate_90deg = 700;  //防止因倾斜高度下降太快  //还没有弄明白这个地方//
int16_t rollrate_90deg = 700;

uint8_t Fly_flag;
/********************************油门控制******************************************/

void Control_Motor(void)
{
	volatile int16_t MOTO1, MOTO2, MOTO3, MOTO4;

	 //油门倾角补偿，防止因倾斜高度下降太快  //还没有弄明白这个地方//
	if(pitch_rate_PID.output > pitchrate_90deg || pitch_rate_PID.output < - pitchrate_90deg || roll_rate_PID.output > rollrate_90deg || roll_rate_PID.output < - rollrate_90deg) 
	{	
		/*         控制采用X模式          */
		/*								  */	
		/*          1 机头 4              */
		/*            \   /               */ 
		/*             \ /                */
		/*             / \                */
		/*            /   \               */
		/*           2     3              */
		/* 1: MOTO1			 2: MOTO1	  */
		/* 3: MOTO3			 4: MOTO4	  */

		MOTO1 += alt_vel_PID.output + pitch_rate_PID.output - roll_rate_PID.output + yaw_rate_PID.output;
		MOTO2 += alt_vel_PID.output - pitch_rate_PID.output - roll_rate_PID.output - yaw_rate_PID.output;
		MOTO3 += alt_vel_PID.output - pitch_rate_PID.output + roll_rate_PID.output + yaw_rate_PID.output;
		MOTO4 += alt_vel_PID.output + pitch_rate_PID.output + roll_rate_PID.output - yaw_rate_PID.output;
		
		if(MOTO1 <= 0) MOTO1 = 0;
		if(MOTO2 <= 0) MOTO2 = 0;
		if(MOTO3 <= 0) MOTO3 = 0;
		if(MOTO4 <= 0) MOTO4 = 0;
	}
	else
	{																																																																																																																																																																																																																																																																																																																																																																																																																																
		 MOTO1 =0; MOTO2 = 0; MOTO3 = 0; MOTO4 = 0;
		 pitch_angle_PID.integ = 0;						//还没有弄明白这个地方//
		 roll_angle_PID.integ = 0;
	}
	
	if(Fly_flag == 0) Motor_RateFlash(MOTO1, MOTO2, MOTO3, MOTO4);
	else    Motor_RateFlash(0, 0, 0, 0);
}
