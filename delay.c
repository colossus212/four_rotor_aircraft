/*****************************************
* File Name: delay.c
* Version: 2015-06-20
* E-mail£º131180064@smail.nju.edu.cn
* IDE£ºCubeSuite+
* API:
	void delay_us(uint16_t t)
	void delay_ms(uint16_t t)
	void delay_s(uint16_t t)
******************************************/

#include "include.h"

// #define delay_time_i 2

void delay_250_ns()
{
	//NOP();// NOP(); //NOP(); NOP();
	//NOP(); NOP(); //NOP(); NOP();
}
void delay_us(uint16_t t)
{
	uint8_t i;
	while (t--)
	{
		NOP(); NOP(); NOP(); NOP(); NOP();
		NOP(); NOP(); NOP(); NOP(); NOP(); 
		NOP(); NOP(); NOP(); NOP(); NOP(); 
		NOP(); NOP(); NOP(); NOP(); NOP();
		NOP(); NOP();
	}
}

void delay_ms(uint16_t t)
{
	while (t--)
	{
		delay_us(1000);
	}
}

void delay_s(uint16_t t)
{
	while (t--)
	{
		delay_ms(1000);
	}
}

void Delay(unsigned long delay_time)
{
   long i;
   
   for(i=0; i < delay_time; i++);
 
}