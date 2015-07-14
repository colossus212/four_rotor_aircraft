/************************************************
* File Name	: HCSR_04.c
* Version	: 2015.7.10 By DHP
* Device(s)	: R5F100LE
* Tool-Chain	: CA78K0R
* Description	: ������ģ��
* API		: void HCSR04_Init()
		  float Get_Height() //���� �߶�ֵ
		  // Trig �˽� P5.3 �� Pin 36
		  // Echo �˽� TI06 �� P0.6 �� Pin 30
		 
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
	delay_us(20);	// �����ź� 10 us ����
	
	Trig = 0;
	delay_ms(25);	// �ȴ� Echo �źų��� 1 - 23 ms 
	
	R_TAU0_Channel6_Get_PulseWidth(& width);
	
	R_TAU0_Channel6_Stop();
	
	TIME = width / 2.01;
	
	if(TIME >220 && TIME <20000)
	height = TIME / 58.8;
	return height;
}