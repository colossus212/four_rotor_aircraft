/******************** (C) COPYRIGHT 2014 Air Nano Team ***************************
 * �ļ���  ��control.c
 * ����    ������         
 * ����    ������ 
 *������  void Control_posture(float roll, float pitch, float yaw);
		  void Control_altitude(float H);
		  void Control_Motor(void);

		  // to do
		  // void Control(float roll, float pitch, float yaw, float H);
		  // {
		  // 	
		  // 	void Control_posture(roll, pitch, yaw);
		  // 	void Control_altitude(H);  //��Ϊ��������ȡ�߶���Ҫ 25+ ms�����Ը߶ȿ��ƺ���̬���Ƶ�Ƶ�ʲ�Ӧ��һ��
		  // 	void Control_Motor();
		  // }


/**********************************************************************************/

#include "include.h"
#include  <math.h>


#if 0
//r_cg_timer_user.c�� __interrupt static void r_tau0_channel5_interrupt(void)�ı���
	//�жϿ��ƣ�
	//static uint8_t i;
	//if(i = 30)
	//{
	//	Get_Height();
	//	i = 0;
	//}
	//Get_Attitude();
	//i++��	
#endif



/****************************************��̬����********************************************/

/*��ע������PID ����   �⻷���ǶȻ�������PID����   */
/*                     �ڻ������ٶȻ�������PD����  */

void Control_posture(float roll, float pitch, float yaw)		//���ж�3�з�������,��ʱû������Ҫ�������
{
	//static float roll_old,pitch_old;
	static int i = 0;

	if(i >= 2)  //�ڻ�����2�ο���   �⻷����1�ο���   �ڻ�����Ƶ��Ϊ�⻷��2�� 
	{
		PID_Postion_Cal( & pitch_angle_PID, pitch, angle.pitch);	//��������//
		PID_Postion_Cal( & roll_angle_PID, roll, angle.roll);	//�������//
		PID_Postion_Cal( & yaw_angle_PID, yaw, angle.yaw);		//�������//
		i = 0;
	}
	i ++;	
	//************�ڻ�(���ٶȻ�)PD***************//
	PID_Postion_Cal( & pitch_rate_PID, pitch_angle_PID.output, MPU6050_data.Gx);	//��������//
	PID_Postion_Cal( & roll_rate_PID, roll_angle_PID.output, MPU6050_data.Gy);	//�������//
	PID_Postion_Cal( & yaw_rate_PID, yaw_angle_PID.output, MPU6050_data.Gz);		//�������//


}


/*************************************�߶ȿ���********************************************/

void Control_altitude(float H)		//����ֵ,��λcm
{
	static int j = 0;
	float height = Get_Height();   //height�Ǵӳ������õ�������

	if(j >= 2)  //�ڻ�����2�ο���   �⻷����1�ο���   �ڻ�����Ƶ��Ϊ�⻷��2�� 
	{
		//*********�⻷�߶�PID***************//		
		PID_Postion_Cal( & alt_PID, H, height);
		 j = 0;
	}
	j ++;	
		//*******�ڻ�(���ٶȻ�)PID***********//
	PID_Postion_Cal( & alt_vel_PID, alt_PID.output, MPU6050_data.Az);

}


int16_t pitchrate_90deg = 700;  //��ֹ����б�߶��½�̫��  //��û��Ū��������ط�//
int16_t rollrate_90deg = 700;

uint8_t Fly_flag;
/********************************���ſ���******************************************/

void Control_Motor(void)
{
	volatile int16_t MOTO1, MOTO2, MOTO3, MOTO4;

	 //������ǲ�������ֹ����б�߶��½�̫��  //��û��Ū��������ط�//
	if(pitch_rate_PID.output > pitchrate_90deg || pitch_rate_PID.output < - pitchrate_90deg || roll_rate_PID.output > rollrate_90deg || roll_rate_PID.output < - rollrate_90deg) 
	{	
		/*         ���Ʋ���Xģʽ          */
		/*								  */	
		/*          1 ��ͷ 4              */
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
		 pitch_angle_PID.integ = 0;						//��û��Ū��������ط�//
		 roll_angle_PID.integ = 0;
	}
	
	if(Fly_flag == 0) Motor_RateFlash(MOTO1, MOTO2, MOTO3, MOTO4);
	else    Motor_RateFlash(0, 0, 0, 0);
}
