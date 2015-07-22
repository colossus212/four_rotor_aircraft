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


#if 0
//r_cg_timer_user.c�� __interrupt static void r_tau0_channel5_interrupt(void)�ı���
	//�жϿ��ƣ�
	//uint8_t Buf[20];

	//if(Count_for_Height == 8)
	//{
	//	Flash_Height();
	//	Buf[6] = ((int16_t) Get_Height()) >> 8;
	//	Buf[7] = ((int16_t) Get_Height()) & 0x00ff;
	//	Control_Heigh(170);
	//	Count_for_Height = 0;
	//}
	//
	//if(Count_for_Height == 0)
	//{
	//	Flash_Height_Prepare();
	//}
	//
	// //Get_Attitude();  // haven't use DMP

	// // Get DMP data and fix yaw
	//DMP_Routing();
	//DMP_Get_YawPitchRoll();
	//Get_Attitude_DMP();
	//
	//	Buf[0] = ((int16_t) angle.yaw) >> 8;
	//	Buf[1] = ((int16_t) angle.yaw) & 0x00ff;
	//	Buf[2] = ((int16_t) angle.roll) >> 8;
	//	Buf[3] = ((int16_t) angle.roll) & 0x00ff;
	//	Buf[4] = ((int16_t) angle.pitch) >> 8;
	//	Buf[5] = ((int16_t) angle.pitch) & 0x00ff;

	//	//Buf[8] = ((int16_t) MOTO1) >> 8;
	//	//Buf[9] = ((int16_t) MOTO1) & 0x00ff;
	//	//Buf[10] = ((int16_t) MOTO2) >> 8;
	//	//Buf[11] = ((int16_t) MOTO2) & 0x00ff;
	//	//Buf[12] = ((int16_t) MOTO3) >> 8;
	//	//Buf[13] = ((int16_t) MOTO3) & 0x00ff;
	//	//Buf[14] = ((int16_t) MOTO4) >> 8;
	//	//Buf[15] = ((int16_t) MOTO4) & 0x00ff;

	//	R_UART0_Send(Buf, 8);

	//Control_Posture(0, 0, 0);
	//Count_for_Height++;
#endif


uint8_t Fly_flag;
volatile int16_t MOTO1 = 0, MOTO2 = 0, MOTO3 = 0, MOTO4 = 0;


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

	if(Fly_flag == 1) Motor_RateFlash(MOTO1, MOTO2, MOTO3, MOTO4);
	else    Motor_RateFlash(0, 0, 0, 0);
}


/*************************************�߶ȿ���********************************************/

void Control_Heigh(float H)		//����ֵ,��λcm
{
	//static int j = 0;
	float height = Get_Height();   //height�Ǵӳ������õ�������

	//if(j >= 2)  //core 2   shell1  
	//{
		//*********�⻷�߶�PID***************//		
		PID_Position( & alt_PID, H, height);
		 //j = 0;
	//}
	//j ++;	
		//*******�ڻ�(���ٶȻ�)PID***********//
	PID_Position( & alt_vel_PID, alt_PID.output, DMP_DATA.dmp_accz * 100 );

	/********************************���ſ���******************************************/

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