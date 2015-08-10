/************************************************
* File Name	: HCSR_04.c
* Version	: 2015.7.10 By DHP
* Device(s)	: R5F100LE
* Tool-Chain	: CA78K0R
* Description	: ³¬Éù²¨Ä£¿é
* API		: void HCSR04_Init();
		  float Get_Height();
		  void Flash_Height_Prepare(); 
		  void Flash_Height(); // this function should wait to run after Flash_Height_Prepare() 25ms
		  
		  // Trig connect to P5.3 (Pin 36)
		  // Echo connect to TI06 (P0.6 as Pin 30)
************************************************/


#include "include.h"
#include <math.h>

#define Trig P5.3

float TIME=0;
float HCSR_Height = 0;
float Height_Calibration_Factor;
uint32_t width = 0;

void HCSR04_Init()
{
	PM5 &= ~(_08_PMn3_MODE_INPUT);
	Trig = 0;
}

float Get_Height()
{
	float Height;  
	Height = HCSR_Height / Height_Calibration_Factor;
	return Height;
}


void Flash_Height_Prepare()
{
	R_TAU0_Channel6_Start();
	Trig = 1;
	delay_us(20);
	Trig = 0;
	//Height_Calibration_Factor = sqrt(tan(Get_Pitch() * AtR) * tan(Get_Pitch() * AtR) + tan(Get_Roll() * AtR) * tan(Get_Roll() * AtR) + 1);
	Height_Calibration_Factor = 1 / cos(Get_Pitch() * AtR) * cos(Get_Roll() * AtR);
}
void Flash_Height()
{	
	R_TAU0_Channel6_Get_PulseWidth(& width);	
	R_TAU0_Channel6_Stop();
	
	TIME = width / 2;//1.5;	
	if(TIME >100 && TIME <20000)	
	HCSR_Height = TIME / 58.8;	
}