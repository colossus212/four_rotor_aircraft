/*************************************************************************
* File Name	: PID.c
* Version	: 2015.7.8 By DHP
* Device(s)	: R5F100LE
* Tool-Chain	: CA78K0R
* Description	: ADNS_3080
* API		: 
		  void PID_Position(PID_Typedef * PID,float target,float measure)
		  void PID_Parameter_Init()
		 		
*************************************************************************/

#include "include.h"


PID_Typedef pitch_angle_PID;
PID_Typedef pitch_rate_PID;

PID_Typedef roll_angle_PID;
PID_Typedef roll_rate_PID;

PID_Typedef yaw_angle_PID;
PID_Typedef yaw_rate_PID;

PID_Typedef	alt_PID;
//PID_Typedef alt_vel_PID;


float Get_PID_Pitch_Angle_Kp()
{
	return pitch_angle_PID.kp;
}

float Get_PID_Pitch_Rate_Kp()
{
	return pitch_rate_PID.kp;
}

float Get_PID_Roll_Angle_Kp()
{
	return roll_angle_PID.kp;
}

float Get_PID_Roll_Rate_Kp()
{
	return roll_rate_PID.kp;
}

float Get_PID_Yaw_Angle_Kp()
{
	return yaw_angle_PID.kp;
}

float Get_PID_Yaw_Rate_Kp()
{
	return yaw_rate_PID.kp;
}

float Get_PID_alt_Kp()
{
	return alt_PID.kp;
}


float Get_PID_Pitch_Angle_Ki()
{
	return pitch_angle_PID.ki;
}

float Get_PID_Pitch_Rate_Ki()
{
	return pitch_rate_PID.ki;
}

float Get_PID_Roll_Angle_Ki()
{
	return roll_angle_PID.ki;
}

float Get_PID_Roll_Rate_Ki()
{
	return roll_rate_PID.ki;
}

float Get_PID_Yaw_Angle_Ki()
{
	return yaw_angle_PID.ki;
}

float Get_PID_Yaw_Rate_Ki()
{
	return yaw_rate_PID.ki;
}

float Get_PID_alt_Ki()
{
	return alt_PID.ki;
}

float Get_PID_Pitch_Angle_Kd()
{
	return pitch_angle_PID.kd;
}

float Get_PID_Pitch_Rate_Kd()
{
	return pitch_rate_PID.kd;
}

float Get_PID_Roll_Angle_Kd()
{
	return roll_angle_PID.kd;
}

float Get_PID_Roll_Rate_Kd()
{
	return roll_rate_PID.kd;
}

float Get_PID_Yaw_Angle_Kd()
{
	return yaw_angle_PID.kd;
}

float Get_PID_Yaw_Rate_Kd()
{
	return yaw_rate_PID.kd;
}

float Get_PID_alt_Kd()
{
	return alt_PID.kd;
}

void PID_Parameter_Load_Rate(float roll_kp, float roll_ki, float roll_kd,
				float pitch_kp, float pitch_ki, float pitch_kd,
				float yaw_kp, float yaw_ki, float yaw_kd)
{
	pitch_rate_PID.kp = pitch_kp;
	pitch_rate_PID.ki = pitch_ki;
	pitch_rate_PID.kd = pitch_kd;
	
	roll_rate_PID.kp = roll_kp;
	roll_rate_PID.ki = roll_ki;
	roll_rate_PID.kd = roll_kd;
	
	yaw_rate_PID.kp = yaw_kp;
	yaw_rate_PID.ki = yaw_ki;
	yaw_rate_PID.kd = yaw_kd;
}

void PID_Parameter_Load_Angle(float roll_kp, float roll_ki, float roll_kd,
				float pitch_kp, float pitch_ki, float pitch_kd,
				float yaw_kp, float yaw_ki, float yaw_kd)
{
	pitch_angle_PID.kp = pitch_kp;   
	pitch_angle_PID.ki = pitch_ki;	
	pitch_angle_PID.kd = pitch_kd;
	
	roll_angle_PID.kp = roll_kp;
	roll_angle_PID.ki = roll_ki;
	roll_angle_PID.kd = roll_kd;
	
	yaw_angle_PID.kp = yaw_kp;
	yaw_angle_PID.ki = yaw_ki;
	yaw_angle_PID.kd = yaw_kd;
}

void PID_Parameter_Load_Height(float roll_kp, float roll_ki, float roll_kd,
				float pitch_kp, float pitch_ki, float pitch_kd,
				float yaw_kp, float yaw_ki, float yaw_kd)
{	
	alt_PID.kp = roll_kp; 
	alt_PID.ki = roll_ki; 
	alt_PID.kd = roll_kd; 
	
	yaw_angle_PID.kp = pitch_kp;   
	yaw_angle_PID.ki = pitch_ki;	
	yaw_angle_PID.kd = pitch_kd; 
	
	yaw_rate_PID.kp = yaw_kp;
	yaw_rate_PID.ki = yaw_ki;
	yaw_rate_PID.kd = yaw_kd; 
}

void PID_Parameter_Reset_Rate()
{	
	pitch_rate_PID.integ = 0;
	roll_rate_PID.integ = 0;
	yaw_rate_PID.integ = 0;
}
void PID_Parameter_Reset_Angle()
{	
	pitch_angle_PID.integ = 0;
	roll_angle_PID.integ = 0;
	yaw_angle_PID.integ = 0;
	alt_PID.integ = 0;
}


void PID_Parameter_Init()
{
	// The data of pitch
	pitch_angle_PID.kp = 1.3;    // 1.3 modualated
	pitch_angle_PID.ki = 0.12;  // 0.12 modualated
	pitch_angle_PID.kd = 0;    // 0 modualated
	
	pitch_rate_PID.kp = 78;   // 78 modualated
	pitch_rate_PID.ki = 0.35;	// 0.35 modualated
	pitch_rate_PID.kd = 7;  // 7 modualated 
	

	//The data of roll
	roll_angle_PID.kp = 1.5;  // 1.5 modualated
	roll_angle_PID.ki = 0.13; // 0.13 modualated
	roll_angle_PID.kd = 0; // 0 modualated

	roll_rate_PID.kp = 130; // 130 modualated
	roll_rate_PID.ki = 0.5; // 0.5 modualated
	roll_rate_PID.kd = 5.5; // 5.5 modualated

	
	//The data of yaw
	yaw_angle_PID.kp = 0;   // 
	yaw_angle_PID.ki = 0;
	yaw_angle_PID.kd = 0; //
	
	yaw_rate_PID.kp = 0; // 18
	yaw_rate_PID.ki = 0;
	yaw_rate_PID.kd = 0; //0.05

	alt_PID.kp = 0 ; //5.0
	alt_PID.ki = 0; //0.01
	alt_PID.kd = 0; //1.7

	//alt_vel_PID.kp = 0; //1.7
	//alt_vel_PID.ki = 0; //0.01
	//alt_vel_PID.kd = 0; //0.22

	//limit for the max increment
	pitch_angle_PID.integ_max = 500;
	roll_angle_PID.integ_max = 500;
	yaw_angle_PID.integ_max = 500;
	
	pitch_rate_PID.integ_max = 2500;
	roll_rate_PID.integ_max = 2500;
	yaw_rate_PID.integ_max = 2500;

	alt_PID.integ_max = 1500;
	//alt_vel_PID.integ_max = 1000;

}


void PID_Position(PID_Typedef * PID,float target,float measure)
{
	PID->error = target - measure;
	
	PID->deriv = PID->error - PID->preerror;
	
	PID->integ = PID->integ + PID->error;
	
	if(PID->integ > PID->integ_max)
		PID->integ = PID->integ_max;
		else if(PID->integ < - PID->integ_max)
			PID->integ = - PID->integ_max;
			
	PID->output = (PID->kp * PID->error)
			+ (PID->ki * PID->integ)
			+ (PID->kd * PID->deriv * 100);
	
	PID->preerror = PID->error;
	if(isnan(PID->output)) PID->output = 0.0f;		
}


void PID_Incremental(PID_Typedef * PID,float target,float measure)
{
	PID->error = target - measure;

	PID->output = PID->kp * (PID->error - PID->preerror)
			+ PID->ki * PID->error
			+ PID->kd * (PID->error - 2 * PID->preerror + PID->prepreerror);

	PID->preerror = PID->error;
	PID->prepreerror = PID->preerror;
}

#if 0
//void PID_Position_Fuzzy(PID_Typedef * PID,float target,float measure)
//{
//	float error_NB, error_NM, error_NS, error_ZO, error_PS, error_PM, error_PB;
//	float preerror_NB, preerror_NM, preerror_NS, preerror_ZO, preerror_PS, preerror_PM, preerror_PB;
//	float kp_NB, kp_NM, kp_NS, kp_ZO, kp_PS, kp_PM, kp_PB;
//	float ki_NB, ki_NM, ki_NS, ki_ZO, ki_PS, ki_PM, ki_PB;
//	float kd_NB, kd_NM, kd_NS, kd_ZO, kd_PS, kd_PM, kd_PB;
//	float kp_offset, ki_offset, kd_offset;
//	//haven't assign
//
//	PID->error = target - measure;
//
//	//rule list
//	if((PID->error <= error_NB) && (PID->preerror <= preerror_NB))
//	{
//		kp_offset = kp_PB;
//		ki_offset = ki_NB;
//		kd_offset = kd_PS;
//	}
//
//	if((PID->error <= error_NB) && (PID->preerror > preerror_NB) && (PID->preerror <= preerror_NM))
//	{
//		kp_offset = kp_PB;
//		ki_offset = ki_NB;
//		kd_offset = kd_NS;
//	}
//
//	if((PID->error <= error_NB) && (PID->preerror > preerror_NM) && (PID->preerror <= preerror_NS))
//	{
//		kp_offset = kp_PM;
//		ki_offset = ki_NM;
//		kd_offset = kd_PB;
//	}
//
//	//a long rule list //to do
//
//	PID->kp += kp_offset;
//	PID->ki += ki_offset;
//	PID->kd += kd_offset;
//
//	PID->preerror = PID->error;
//
//}
#endif