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

#define Trig P5.3

float TIME=0;
float HCSR_Height = 0;
uint32_t width = 0;

void HCSR04_Init()
{
	PM5 &= ~(_08_PMn3_MODE_INPUT);
	Trig = 0;
}

float Get_Height()
{
	return HCSR_Height;
}


void Flash_Height_Prepare()
{
	R_TAU0_Channel6_Start();
	Trig = 1;
	delay_us(20);
	Trig = 0;
}
void Flash_Height()
{	
	R_TAU0_Channel6_Get_PulseWidth(& width);	
	R_TAU0_Channel6_Stop();
	
	TIME = width / 1.5;	
	if(TIME >100 && TIME <20000)	
	HCSR_Height = TIME / 58.8;	
}