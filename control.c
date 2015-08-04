/******************** (C) COPYRIGHT 2014 Air Nano Team ***************************
 * �ļ���  ��control.c
 * Version	: 2015.7.10 By DHP
 * Device(s)	: R5F100LE
 * Tool-Chain	: CA78K0R
 * description  ��control Posture and Heigh
 * function��  void Control_Posture(float roll, float pitch, float yaw);
		  void Control_Heigh(float H);
*********************************************************************************/

#include "include.h"
#include  <math.h>

#define MOTO_MAX 30000
#define MOTO_MIN 6000

#if 0
//r_cg_timer_user.c�� __interrupt static void r_tau0_channel5_interrupt(void)�ı���
	//�жϿ��ƣ�
//uint16_t height, tempr = 0, press = 0, IMUpersec = 100;
////	uint8_t Mag_Buf[8];
//	if(Count_for_Timer_8 == 8)
//	{
////		Flash_Height();
////		Control_Heigh(170);
//		Count_for_Timer_8 = 0;
////		Mag_Buf[0] = 0xAA;
////		Mag_Buf[1] = ((int16_t)(Get_HMC5883L_Hx())) >> 8;
////		Mag_Buf[2] = ((int16_t)(Get_HMC5883L_Hx()));
////		Mag_Buf[3] = ((int16_t)(Get_HMC5883L_Hy())) >> 8;
////		Mag_Buf[4] = ((int16_t)(Get_HMC5883L_Hy()));
////		Mag_Buf[5] = ((int16_t)(Get_HMC5883L_Hz())) >> 8;
////		Mag_Buf[6] = ((int16_t)(Get_HMC5883L_Hz()));
////		Mag_Buf[7] = 0xBB;
////		R_UART0_Send(Mag_Buf, 8);
//
//		AHRS_Captain_Flight_Motion((int16_t)(Get_DMP_Acc_x()), (int16_t)(Get_DMP_Acc_y()), (int16_t)(Get_DMP_Acc_z()),
//			(int16_t)(Get_DMP_Gyro_x()), (int16_t)(Get_DMP_Gyro_y()), (int16_t)(Get_DMP_Gyro_z()),
//			(int16_t)(Get_HMC5883L_Hx()), (int16_t)(Get_HMC5883L_Hy()), (int16_t)(Get_HMC5883L_Hz()));
//
//		R_UART0_Send(Uart_Buf_Motion, 24);		
////		Control_Track();
//	}
//	
//	if(Count_for_Timer_8 == 4)
//	{
//		AHRS_Captain_Flight_IMU((int16_t)(10 * Get_Yaw()), (int16_t)(10 * Get_Pitch()), (int16_t)(10 * Get_Roll()),
//				height, tempr, press, IMUpersec);
//		R_UART0_Send(Uart_Buf_IMU, 18);
//	}
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
//	Control_Posture(0, 0, 0);
//	Count_for_Timer_8++;
//	
//	
//     //P13.0 = ~P13.0;
#endif


uint8_t Fly_flag;
volatile float MOTO1 = 0, MOTO2 = 0, MOTO3 = 0, MOTO4 = 0;
float Roll_Target = 0, Pitch_Target = 0, Yaw_Target = 0, Height_Target = 100;

/****************************************��̬����********************************************/

/*��ע������PID ����   �⻷���ǶȻ�������PID����   */
/*                     �ڻ������ٶȻ�������PD����  */

void Control_Posture(float roll, float pitch, float yaw)
{
		//static float roll_old,pitch_old;
	static int i = 0;
	if(i >= 2)
	{
		PID_Position( & pitch_angle_PID, pitch, Get_Pitch());	//��������//
		PID_Position( & roll_angle_PID, roll, Get_Roll());	//�������//
		PID_Position( & yaw_angle_PID, yaw, Get_Yaw());		//�������//
		i = 0;
	}
	i ++;
		//************�ڻ�(���ٶȻ�)PD***************//	
	PID_Position( & pitch_rate_PID, pitch_angle_PID.output, Get_DMP_Gyro_x());	//��������//
	PID_Position( & roll_rate_PID, roll_angle_PID.output, Get_DMP_Gyro_y());	//�������//
	PID_Position( & yaw_rate_PID, yaw_angle_PID.output, - Get_DMP_Gyro_z());	//�������//
		//if don't use DMP
	//PID_Position( & pitch_rate_PID, pitch_angle_PID.output, Get_MPU6050_Gx());	//��������//
	//PID_Position( & roll_rate_PID, roll_angle_PID.output, Get_MPU6050_Gy());	//�������//
	//PID_Position( & yaw_rate_PID, yaw_angle_PID.output, - Get_MPU6050_Gz());	//�������//


	/********************************���ſ���******************************************/
		/*		���Ʋ���Xģʽ		*/
		/*						*/	
		/*          1 ��ͷ 4 		 */
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


/*************************************�߶ȿ���********************************************/

void Control_Heigh(float H)		//����ֵ,��λcm
{
	//static int j = 0;
	float AccH;
	float height = Get_Height();   //height�Ǵӳ������õ�������
	height = height / sqrt(tan(Get_Pitch() * AtR) * tan(Get_Pitch() * AtR) + tan(Get_Roll() * AtR) * tan(Get_Roll() * AtR) + 1);
	
	//if(j >= 2)  //core 2   shell1  
	//{
			//*********�⻷�߶�PID***************//		
		PID_Position( & alt_PID, H, height);
		 //j = 0;
	//}
	//j ++;	
		//*******�ڻ�(���ٶȻ�)PID***********//
	AccH = sqrt(Get_DMP_Acc_x() * Get_DMP_Acc_x() + Get_DMP_Acc_y() * Get_DMP_Acc_y() + Get_DMP_Acc_z() * Get_DMP_Acc_z()); 
		//if don't use DMP
	//AccH = sqrt(Get_MPU6050_Ax() * Get_MPU6050_Ax() + Get_MPU6050_Ay() * Get_MPU6050_Ay() + Get_MPU6050_Az() * Get_MPU6050_Az()); 
	
	PID_Position( & alt_vel_PID, alt_PID.output, (AccH - 9.82)* 100 );

	/********************************���ſ���******************************************/

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

void Control_Flying_Off()
{
	//Height_Target = 40;
	//delay_s(2);
	//Height_Target = 80;
	//delay_s(2);
	//Height_Target = 100;
}

void Control_Land()
{
	
}
