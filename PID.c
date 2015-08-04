/******************** (C) COPYRIGHT 2014 Air Nano Team ***************************
 * �ļ���  ��PID.c
 * ����    ��PID�㷨
 * ����    ��
 * ������	void PID_Position(PID_Typedef * PID,float target,float measure)
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
	pitch_angle_PID.kp = 0;    //5.0
	pitch_angle_PID.ki = 0;  // 0.01
	pitch_angle_PID.kd = 0;    //1.7
	
	pitch_rate_PID.kp = 0;   // 0.21 modualated
	pitch_rate_PID.ki = 0;	//
	pitch_rate_PID.kd = 0;  //0.05 modualated ??
	

	//The data of roll
	roll_angle_PID.kp = 0;  // 0.35 modualated
	roll_angle_PID.ki = 0; // 0.012 modualated
	roll_angle_PID.kd = 0; // 0.15 modualated

	roll_rate_PID.kp = 0; // 0.22 modualated
	roll_rate_PID.ki = 0;
	roll_rate_PID.kd = 0; // 0.01 modualated
	
	//The data of yaw
	yaw_angle_PID.kp = 0;   // 5.0
	yaw_angle_PID.ki = 0;
	yaw_angle_PID.kd = 0; //0.13
	
	yaw_rate_PID.kp = 0; // 1.8 modualated
	yaw_rate_PID.ki = 0;
	yaw_rate_PID.kd = 0; //0.05 modualated

	alt_PID.kp = 0 ; //5.0
	alt_PID.ki = 0; //0.01
	alt_PID.kd = 0; //1.7

	alt_vel_PID.kp = 0; //1.7
	alt_vel_PID.ki = 0; //0.01
	alt_vel_PID.kd = 0; //0.22

	//limit for the max increment
	pitch_angle_PID.integ_max = 100;
	roll_angle_PID.integ_max = 100;
	yaw_angle_PID.integ_max = 100;
	
	pitch_rate_PID.integ_max = 500;
	roll_rate_PID.integ_max = 500;
	yaw_rate_PID.integ_max = 500;

	alt_PID.integ_max = 200;
	alt_vel_PID.integ_max = 200;

}





//-----------λ��ʽPID-----------
void PID_Position(PID_Typedef * PID,float target,float measure)
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


void PID_Incremental(PID_Typedef * PID,float target,float measure)
{
	PID->error = target - measure;

	PID->output = PID->kp * (PID->error - PID->preerror) + PID->ki * PID->error + PID->kd * (PID->error - 2 * PID->preerror + PID->prepreerror);

	PID->preerror = PID->error;
	PID->prepreerror = PID->preerror;
}


void PID_Position_Fuzzy(PID_Typedef * PID,float target,float measure)
{
	float error_NB, error_NM, error_NS, error_ZO, error_PS, error_PM, error_PB;
	float preerror_NB, preerror_NM, preerror_NS, preerror_ZO, preerror_PS, preerror_PM, preerror_PB;
	float kp_NB, kp_NM, kp_NS, kp_ZO, kp_PS, kp_PM, kp_PB;
	float ki_NB, ki_NM, ki_NS, ki_ZO, ki_PS, ki_PM, ki_PB;
	float kd_NB, kd_NM, kd_NS, kd_ZO, kd_PS, kd_PM, kd_PB;
	float kp_offset, ki_offset, kd_offset;
	//haven't assign

	PID->error = target - measure;

	//rule list
	if((PID->error <= error_NB) && (PID->preerror <= preerror_NB))
	{
		kp_offset = kp_PB;
		ki_offset = ki_NB;
		kd_offset = kd_PS;
	}

	if((PID->error <= error_NB) && (PID->preerror > preerror_NB) && (PID->preerror <= preerror_NM))
	{
		kp_offset = kp_PB;
		ki_offset = ki_NB;
		kd_offset = kd_NS;
	}

	if((PID->error <= error_NB) && (PID->preerror > preerror_NM) && (PID->preerror <= preerror_NS))
	{
		kp_offset = kp_PM;
		ki_offset = ki_NM;
		kd_offset = kd_PB;
	}

	//a long rule list //to do

	PID->kp += kp_offset;
	PID->ki += ki_offset;
	PID->kd += kd_offset;

	PID->preerror = PID->error;

}



#if 0
// ������ r_cg_serial_user.c ,���ڿ��Ʒɻ��͵���
//    
//	switch(rx_data)
//	{
//		case 0x11: 
//		{
//			Control_Standby();
//			R_TAU0_Channel5_Stop();
//			Motor_RateFlash(0, 0, 0, 0);
//			MOTO1 = 0; MOTO2 = 0; MOTO3 = 0; MOTO4 = 0;
//			TDR01 = 2000; TDR02 = 2000; TDR03 = 2000; TDR04 = 2000;
//		} break;
//		
//		case 0x23: Control_Fly(); R_TAU0_Channel5_Start(); break;
//		case 0x22:
//		{
//			Control_Fly(); R_TAU0_Channel5_Start();
//			MOTO1 = 21000; MOTO2 = 21000; MOTO3 = 21000; MOTO4 = 21000;
//			Motor_RateFlash(MOTO1, MOTO2, MOTO3, MOTO4);
//		} break;
//		case 0x24:
//		{
//			MOTO1 += 1000; MOTO2 += 1000; MOTO3 += 1000; MOTO4 += 1000;
//			Motor_RateFlash(MOTO1, MOTO2, MOTO3, MOTO4);
//		} break;
//		case 0x25:
//		{
//			MOTO1 -= 1000; MOTO2 -= 1000; MOTO3 -= 1000; MOTO4 -= 1000;
//			Motor_RateFlash(MOTO1, MOTO2, MOTO3, MOTO4);
//		} break;
//
//		case 0x26: 
//		{
//			MOTO1 += 200;
//			Motor_RateFlash(MOTO1, MOTO2, MOTO3, MOTO4);
//		} break;
//		case 0x27: 
//		{
//			MOTO1 -= 200;
//			Motor_RateFlash(MOTO1, MOTO2, MOTO3, MOTO4);
//		} break;
//
//		case 0x28: 
//		{
//			MOTO2 += 200;
//			Motor_RateFlash(MOTO1, MOTO2, MOTO3, MOTO4);
//		} break;
//		case 0x29: 
//		{
//			MOTO2 -= 200;
//			Motor_RateFlash(MOTO1, MOTO2, MOTO3, MOTO4);
//		} break;
//		
//		case 0x2a: 
//		{
//			MOTO3 += 200;
//			Motor_RateFlash(MOTO1, MOTO2, MOTO3, MOTO4);
//		} break;
//		case 0x2b: 
//		{
//			MOTO3 -= 200;
//			Motor_RateFlash(MOTO1, MOTO2, MOTO3, MOTO4);
//		} break;
//		
//		case 0x2c: 
//		{
//			MOTO4 += 200;
//			Motor_RateFlash(MOTO1, MOTO2, MOTO3, MOTO4);
//		} break;
//		case 0x2d: 
//		{
//			MOTO4 -= 200;
//			Motor_RateFlash(MOTO1, MOTO2, MOTO3, MOTO4);
//		} break;
//
//		case 0x31: yaw_rate_PID.kp += 1; break;
//		case 0x32: yaw_rate_PID.kp += 0.1; break;
//		case 0x33: yaw_rate_PID.kp += 0.01; break;
//		
//		case 0x34: pitch_rate_PID.kp += 1; break;
//		case 0x35: pitch_rate_PID.kp += 0.1; break;
//		case 0x36: pitch_rate_PID.kp += 0.01; break;
//		
//		case 0x37: roll_rate_PID.kp += 1; break;
//		case 0x38: roll_rate_PID.kp += 0.1; break;
//		case 0x39: roll_rate_PID.kp += 0.01; break;		
//
//		case 0x3a: yaw_rate_PID.kp -= 0.1; break;
//		case 0x3b: yaw_rate_PID.kp -= 0.01; break;
//		
//		case 0x3c: pitch_rate_PID.kp -= 0.1; break;
//		case 0x3d: pitch_rate_PID.kp -= 0.01; break;
//		
//		case 0x3e: roll_rate_PID.kp -= 0.1; break;
//		case 0x3f: roll_rate_PID.kp -= 0.01; break;
//		
//		
//		case 0x41: yaw_rate_PID.ki += 0.1; break;
//		case 0x42: yaw_rate_PID.ki += 0.01; break;
//		case 0x43: yaw_rate_PID.ki += 0.001; break;
//		
//		case 0x44: pitch_rate_PID.ki += 0.1; break;
//		case 0x45: pitch_rate_PID.ki += 0.01; break;
//		case 0x46: pitch_rate_PID.ki += 0.001; break;
//		
//		case 0x47: roll_rate_PID.ki += 0.1; break;
//		case 0x48: roll_rate_PID.ki += 0.01; break;
//		case 0x49: roll_rate_PID.ki += 0.001; break;		
//
//		case 0x4a: yaw_rate_PID.ki -= 0.01; break;
//		case 0x4b: yaw_rate_PID.ki -= 0.001; break;
//		
//		case 0x4c: pitch_rate_PID.ki -= 0.01; break;
//		case 0x4d: pitch_rate_PID.ki -= 0.001; break;
//		
//		case 0x4e: roll_rate_PID.ki -= 0.01; break;
//		case 0x4f: roll_rate_PID.ki -= 0.001; break;
//		
//		
//		case 0x51: yaw_rate_PID.kd += 0.1; break;
//		case 0x52: yaw_rate_PID.kd += 0.01; break;
//		case 0x53: yaw_rate_PID.kd += 0.001; break;
//
//		case 0x54: pitch_rate_PID.kd += 0.1; break;
//		case 0x55: pitch_rate_PID.kd += 0.01; break;
//		case 0x56: pitch_rate_PID.kd += 0.001; break;
//		
//		case 0x57: roll_rate_PID.kd += 0.1; break;
//		case 0x58: roll_rate_PID.kd += 0.01; break;
//		case 0x59: roll_rate_PID.kd += 0.001; break;
//
//		case 0x5a: yaw_rate_PID.kd -= 0.01; break;
//		case 0x5b: yaw_rate_PID.kd -= 0.001; break;
//		
//		case 0x5c: pitch_rate_PID.kd -= 0.01; break;
//		case 0x5d: pitch_rate_PID.kd -= 0.001; break;
//		
//		case 0x5e: roll_rate_PID.kd -= 0.01; break;
//		case 0x5f: roll_rate_PID.kd -= 0.001; break;
//		
//		
//		case 0x61: yaw_angle_PID.kp += 1; break;
//		case 0x62: yaw_angle_PID.kp += 0.1; break;
//		case 0x63: yaw_angle_PID.kp += 0.01; break;
//
//		case 0x64: pitch_angle_PID.kp += 1; break;
//		case 0x65: pitch_angle_PID.kp += 0.1; break;
//		case 0x66: pitch_angle_PID.kp += 0.01; break;
//		
//		case 0x67: roll_angle_PID.kp += 1; break;
//		case 0x68: roll_angle_PID.kp += 0.1; break;
//		case 0x69: roll_angle_PID.kp += 0.01; break;
//
//		case 0x6a: yaw_angle_PID.kp -= 0.1; break;
//		case 0x6b: yaw_angle_PID.kp -= 0.01; break;
//		
//		case 0x6c: pitch_angle_PID.kp -= 0.1; break;
//		case 0x6d: pitch_angle_PID.kp -= 0.01; break;
//		
//		case 0x6e: roll_angle_PID.kp -= 0.1; break;
//		case 0x6f: roll_angle_PID.kp -= 0.01; break;
//		
//		
//		case 0x71: yaw_angle_PID.ki += 0.1; break;
//		case 0x72: yaw_angle_PID.ki += 0.01; break;
//		case 0x73: yaw_angle_PID.ki += 0.001; break;
//
//		case 0x74: pitch_angle_PID.ki += 0.1; break;
//		case 0x75: pitch_angle_PID.ki += 0.01; break;
//		case 0x76: pitch_angle_PID.ki += 0.001; break;
//		
//		case 0x77: roll_angle_PID.ki += 0.1; break;
//		case 0x78: roll_angle_PID.ki += 0.01; break;
//		case 0x79: roll_angle_PID.ki += 0.001; break;
//
//		case 0x7a: yaw_angle_PID.ki -= 0.01; break;
//		case 0x7b: yaw_angle_PID.ki -= 0.001; break;
//		
//		case 0x7c: pitch_angle_PID.ki -= 0.01; break;
//		case 0x7d: pitch_angle_PID.ki -= 0.001; break;
//		
//		case 0x7e: roll_angle_PID.ki -= 0.01; break;
//		case 0x7f: roll_angle_PID.ki -= 0.001; break;
//		
//		
//		case 0x81: yaw_angle_PID.kd += 0.1; break;
//		case 0x82: yaw_angle_PID.kd += 0.01; break;
//		case 0x83: yaw_angle_PID.kd += 0.001; break;
//
//		case 0x84: pitch_angle_PID.kd += 0.1; break;
//		case 0x85: pitch_angle_PID.kd += 0.01; break;
//		case 0x86: pitch_angle_PID.kd += 0.001; break;
//		
//		case 0x87: roll_angle_PID.kd += 0.1; break;
//		case 0x88: roll_angle_PID.kd += 0.01; break;
//		case 0x89: roll_angle_PID.kd += 0.001; break;
//
//		case 0x8a: yaw_angle_PID.kd -= 0.01; break;
//		case 0x8b: yaw_angle_PID.kd -= 0.001; break;
//		
//		case 0x8c: pitch_angle_PID.kd -= 0.01; break;
//		case 0x8d: pitch_angle_PID.kd -= 0.001; break;
//		
//		case 0x8e: roll_angle_PID.kd -= 0.01; break;
//		case 0x8f: roll_angle_PID.kd -= 0.001; break;
//		
//		
//		case 0x91: Roll_Target = 0; break;
//		case 0x92: Roll_Target = 30; break;
//		case 0x93: Roll_Target = - 30; break;
//		
//		case 0x94: Pitch_Target = 0; break;
//		case 0x95: Pitch_Target = 30; break;
//		case 0x96: Pitch_Target = - 30; break;
//		
//		case 0x97: Yaw_Target = 0; break;
//		case 0x98: Yaw_Target = 40; break;
//		case 0x99: Yaw_Target = - 40; break;
//		case 0x9a: Yaw_Target = 80; break;
//		case 0x9b: Yaw_Target = -80; break;
//		case 0x9c: Yaw_Target = 120; break;
//		case 0x9d: Yaw_Target = -120; break;
//		//case 0x9e: Yaw_Target = 160; break;
//		//case 0x9f: Yaw_Target = -160; break;
//		
//		//case 0xa0: Height_Target = 100; break;
//		//case 0xa1: Height_Target = 150; break;
//		//case 0xa2: Herght_Target = 50; break;
//	}
#endif