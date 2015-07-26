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

#define MOTO_MAX 10000
#define MOTO_MIN 0

#if 0
//r_cg_timer_user.c�� __interrupt static void r_tau0_channel5_interrupt(void)�ı���
	//�жϿ��ƣ�
	//uint16_t height, tempr = 0, press = 0, IMUpersec = 100;
	//if(Count_for_Height == 8)
	//{
	//	Flash_Height();
	//	Control_Heigh(170);
	//	Count_for_Height = 0;
	//	AHRS_Captain_Flight_IMU(angle.yaw, angle.pitch, angle.roll, height, tempr, press, IMUpersec);
	//	R_UART0_Send(Uart_Buf_IMU, 18);
	//}
	//
	//if(Count_for_Height == 4)
	//{
	//	AHRS_Captain_Flight_Motion(DMP_DATA.dmp_accx, DMP_DATA.dmp_accy, DMP_DATA.dmp_accz,
	//		DMP_DATA.dmp_gyrox , DMP_DATA.dmp_gyroy, DMP_DATA.dmp_gyroz, X_HMC, Y_HMC, Z_HMC);

	//	R_UART0_Send(Uart_Buf_Motion, 24);
	//}
	//
	//if(Count_for_Height == 0)
	//{
	//	Flash_Height_Prepare();
	//}
	//
	////Get_Attitude();  // haven't use DMP

	//// Get DMP data and fix yaw
	//DMP_Routing();
	//DMP_Get_YawPitchRoll();
	//Get_Attitude_DMP();
	//
	//height = Get_Height();	

	//Control_Posture(0, 0, 0);
	//Count_for_Height++;
#endif


uint8_t Fly_flag;
volatile float MOTO1 = 0, MOTO2 = 0, MOTO3 = 0, MOTO4 = 0;


/****************************************��̬����********************************************/

/*��ע������PID ����   �⻷���ǶȻ�������PID����   */
/*                     �ڻ������ٶȻ�������PD����  */

void Control_Posture(float roll, float pitch, float yaw)
{
	//static float roll_old,pitch_old;
	static int i = 0;

	if(i >= 2)
	{
		PID_Position( & pitch_angle_PID, pitch, angle.pitch);	//��������//
		PID_Position( & roll_angle_PID, roll, angle.roll);	//�������//
		PID_Position( & yaw_angle_PID, yaw, angle.yaw);		//�������//
		i = 0;
	}
	i ++;	
	//************�ڻ�(���ٶȻ�)PD***************//
	PID_Position( & pitch_rate_PID, pitch_angle_PID.output, DMP_DATA.dmp_gyrox);	//��������//
	PID_Position( & roll_rate_PID, roll_angle_PID.output, DMP_DATA.dmp_gyroy);	//�������//
	PID_Position( & yaw_rate_PID, yaw_angle_PID.output, DMP_DATA.dmp_gyroz);		//�������//


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
	height = height / sqrt(tan(angle.pitch * AtR) * tan(angle.pitch * AtR) + tan(angle.roll * AtR) * tan(angle.roll * AtR) + 1);
	
	//if(j >= 2)  //core 2   shell1  
	//{
		//*********�⻷�߶�PID***************//		
		PID_Position( & alt_PID, H, height);
		 //j = 0;
	//}
	//j ++;	
		//*******�ڻ�(���ٶȻ�)PID***********//
	AccH = sqrt(DMP_DATA.dmp_accx * DMP_DATA.dmp_accx + DMP_DATA.dmp_accy * DMP_DATA.dmp_accy + DMP_DATA.dmp_accz * DMP_DATA.dmp_accz); 
	PID_Position( & alt_vel_PID, alt_PID.output, (AccH - 8.82)* 100 );

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