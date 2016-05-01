/************************************************
* File Name	: PX4.c
* Version	: 2015.8.13 By DHP
* Device(s)	: R5F100LE
* Tool-Chain	: CA78K0R
* Description	: PX4
* API		: 
		  void PX4_Init();
		  void PX4_Test();
		  void PX4_Unlock();
		  void PX4_Fly();
		  void PX4_Stop();
		  void PX4_Land();
		  void PX4_Control_Height();
		  
************************************************/
#include "include.h"

#define Roll_PWM TDR01
#define Pitch_PWM TDR02
#define Thr_PWM TDR03
#define Yaw_PWM TDR04
#define Model_PWM TDR07


#define PWM_Max 1000  // test percent: 40000 / 50000 = 80 %
//uint16_t PWM_ZeroOffset;
uint16_t Roll_ZeroOffset, Pitch_ZeroOffset, Yaw_ZeroOffset;	
	//float PWM_RatioOffset;
uint8_t Fly_Flag = 0;
uint8_t Land_Flag = 1;
int16_t Roll_Target = 0, Pitch_Target = 0, Yaw_Target = 0;
uint16_t Height_Target = 50, Thr_Target = 0, Model = 2000;
uint16_t Thr_Rise = 1200, Thr_Fall = 750;
extern uint32_t Count_test;
//	Roll_Target	-1000 - 1000
//	Pitch_Target	-1000 - 1000
//	Yaw_Target	-1000 - 1000
//	Thr_Target	0 - 2000
//	Model		0 - 2000		
//PID_Typedef	alt_PID;


/******************************************************************************
* function :		void PX4_StandBy()
* Description : 
*******************************************************************************/
void PX4_StandBy()
{
	Roll_PWM = Roll_ZeroOffset;
	Pitch_PWM = Pitch_ZeroOffset;
	Thr_PWM = 2000;
	Yaw_PWM = Yaw_ZeroOffset;
	Model_PWM = 1922;
}
/******************************************************************************
* function :		void PX4_Unlock()
* Description : 
*******************************************************************************/
void PX4_Unlock()
{
	Roll_PWM = 2016;
	Pitch_PWM = 4046;
	Thr_PWM = 2096;
	Yaw_PWM = 4048;
	Model_PWM = 1922;
	delay_s(3);
	PX4_StandBy();
}
/******************************************************************************
* function :		void PX4_Lock()
* Description : 
*******************************************************************************/
void PX4_Lock()
{
	Roll_PWM = 4000;
	Pitch_PWM = 4032;
	Thr_PWM = 2096;
	Yaw_PWM = 2016;
	Model_PWM = 1922;
	delay_s(3);
	PX4_StandBy();
}

/******************************************************************************
* function :		void PX4_Init()
* Description : 
*******************************************************************************/
void PX4_Init()
{	
	// 因为设置了 PWM 输出频率为 50 Hz，周期为 20 ms， TDR00 = 0x9C3FU
	// 而电调要求控制油门为1-2ms ，在50Hz下，占空比是 5% - 10%
	// 故TDR的范围为 TDR00 * 0.05 ~ TDR00 * 0.10
		//(TDR00 + 1) * 0.05;
	Roll_ZeroOffset = 3124;
	Pitch_ZeroOffset = 2895;
	Yaw_ZeroOffset = 3042;
	
	//PWM_RatioOffset = 1;	//(TDR00 + 1) * (0.10 - 0.05) / 50000(PWM_Max);
	//PX4_Unlock();
	PX4_StandBy();
	R_TAU0_Channel0_Start();  //方波输出，电机驱动
	PX4_StandBy();
	delay_ms(100);
}



/***********************************************
* function :		void PWM_Flash(int16_t Roll, int16_t Pitch, int16_t Yaw, uint16_t Thr, uint16_t Model)
* Description : 	Roll	-1000 - 1000
			Pitch	-1000 - 1000
			Thr	0 - 2000
			Yaw	-1000 - 1000
			Model	0 - 2000			
***********************************************/
void PWM_Flash(int16_t Roll, int16_t Pitch, int16_t Yaw, uint16_t Thr, uint16_t Model)
{	
	if(Roll > PWM_Max) Roll = PWM_Max;
	if(Pitch > PWM_Max) Pitch = PWM_Max;
	if(Thr > 2000) Thr = 2000;
	if(Yaw > PWM_Max) Yaw = PWM_Max;
	if(Model > 2000) Model = 2000;
	
	if(Roll < -1000) Roll = -1000;
	if(Pitch < -1000) Pitch = -1000;
	if(Thr < 0) Thr = 0;
	if(Yaw < -1000) Yaw = -1000;
	if(Model < 0) Model = 0;
	
	if(Fly_Flag == 1)
	{
		Roll_PWM = Roll + Roll_ZeroOffset;
		Pitch_PWM = Pitch + Pitch_ZeroOffset;
		Thr_PWM = Thr + 2000;
		Yaw_PWM = Yaw + Yaw_ZeroOffset;
		Model_PWM = Model + 2000;
	}else PX4_StandBy();
}


/******************************************************************************
* function :		void PX4_Fly()
* Description : 
*******************************************************************************/
void PX4_Fly()
{
	Fly_Flag = 1;
	Land_Flag = 0;
	Model = 2000;
	Count_test = 0;
	Thr_Target = 1200;
	PWM_Flash(Roll_Target, Pitch_Target, Yaw_Target, Thr_Target, Model);
	delay_s(2);
//	delay_ms(300);
//	Thr_Target = 1000;
//	PWM_Flash(Roll_Target, Pitch_Target, Yaw_Target, Thr_Target, Model);	
}
/******************************************************************************
* function :		void PX4_Test()
* Description : 
*******************************************************************************/
void PX4_Test()
{
	Fly_Flag = 1;
	Thr_Target = 200;
	PWM_Flash(0, 0, 0, Thr_Target, Model);
}
/******************************************************************************
* function :		void PX4_Stop()
* Description : 
*******************************************************************************/
void PX4_Stop()
{
	Thr_Target = 0;
//	Roll_Target = 0;
//	Pitch_Target = 0;
//	Yaw_Target = 0;
	Thr_PWM = 2000;
	PWM_Flash(0, 0, 0, Thr_Target, Model);
	delay_ms(3000);
	PX4_Lock();
	Fly_Flag = 0;
}
/******************************************************************************
* function :		void PX4_Land()
* Description : 
*******************************************************************************/
void PX4_Land()
{
	if(Get_Height() > 10) Thr_PWM = 2000 + 650;
	else PX4_Stop();
}

//void PID_Position(PID_Typedef * PID,float target,float measure)
//{
//	PID->error = target - measure;
//	
//	PID->deriv = PID->error - PID->preerror;
//	
//	PID->integ = PID->integ + PID->error;
//	
//	if(PID->integ > PID->integ_max)
//		PID->integ = PID->integ_max;
//		else if(PID->integ < - PID->integ_max)
//			PID->integ = - PID->integ_max;
//			
//	PID->output = (PID->kp * PID->error)
//			+ (PID->ki * PID->integ)
//			+ (PID->kd * PID->deriv * 100);
//	
//	PID->preerror = PID->error;
//	if(isnan(PID->output)) PID->output = 0.0f;		
//}
//
//void PID_Parameter_Init()
//{
//	alt_PID.kp = 0 ; //5.0
//	alt_PID.ki = 0; //0.01
//	alt_PID.kd = 0; //1.7
//		//limit for the max increment
//	alt_PID.integ_max = 1500;
//}
//
//void PID_Parameter_Load_Height(float PID_P, float PID_I, float PID_D)
//{
//	alt_PID.kp = PID_P;
//	alt_PID.ki = PID_I;
//	alt_PID.kd = PID_D;
//}
//void PID_Parameter_Reset_Height()
//{
//	alt_PID.integ = 0;
//}


/******************************************************************************
* function :		void PX4_Control_Height()
* Description : 
*******************************************************************************/
uint16_t Height_Thro_Low = 15, Height_Thro_High = 10;

void PX4_Control_Height()
{
//	PID_Position(&alt_PID, Height_Target, Get_Height());
//	if(Thr_Target < 500) Thr_Target = 500;
//	if(Thr_Target > 1500) Thr_Target = 1500;
//	if(alt_PID.output > 300) alt_PID.output = 300;
//	if(alt_PID.output < - 300) alt_PID.output = - 300;
//	Thr_Target = 1000 + alt_PID.output;
//	PWM_Flash(Roll_Target, Pitch_Target, Yaw_Target, Thr_Target, Model);	
	if(Get_Height() < Height_Target - Height_Thro_Low)
	Thr_Target = Thr_Rise;
	else if(Get_Height() > Height_Target + Height_Thro_High)
		Thr_Target = Thr_Fall;
		else Thr_Target = 1000;
	PWM_Flash(Roll_Target, Pitch_Target, Yaw_Target, Thr_Target, Model);
}