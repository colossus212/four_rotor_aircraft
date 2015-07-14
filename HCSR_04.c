/************************************************
* File Name	: HCSR_04.c
* Version	: 2015.7.10 By DHP
* Device(s)	: R5F100LE
* Tool-Chain	: CA78K0R
* Description	: 超声波模块
* API		: void HCSR04_Init()
		  float Get_Height() //返回 高度值
		  // Trig 端接 P5.3 即 Pin 36
		  // Echo 端接 TI06 即 P0.6 即 Pin 30
		 
************************************************/


#include "include.h"

float TIME=0;

#define Trig P5.3


void HCSR04_Init()
{
	PM5 &= ~(_08_PMn3_MODE_INPUT);
	Trig = 0;
}



float Get_Height()
{
	float height = 0;
	uint32_t width = 0;
	
	R_TAU0_Channel6_Start();  
	
	Trig = 1;
	delay_us(20);	// 触发信号 10 us 以上
	
	Trig = 0;
	delay_ms(25);	// 等待 Echo 信号持续 1 - 23 ms 
	
	R_TAU0_Channel6_Get_PulseWidth(& width);
	
	R_TAU0_Channel6_Stop();
	
	TIME = width / 7.1;
	
	if(TIME >100 && TIME <20000)
	height = TIME / 58.8;
	return height;
}