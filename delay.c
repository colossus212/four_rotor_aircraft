/*****************************************
��д�ߣ�DHP
����E-mail��131180064@smail.nju.edu.cn
���뻷����CubeSuite+
����ʱ��: 2015-06-20
���ܣ�
�ṩ��ʱAPI  �� ΢�뼶 �� ���뼶 ��ʱ
******************************************/

#include "include.h"

void delay_us(uint8_t t)
{
	uint8_t i;
	while (t--)
	{
		for(i = 0; i < delay_time_i; i++);
	}
}

void delay_ms(uint8_t t)
{
	while (t--)
	{
		delay_us(1000);
	}
}

void Delay(unsigned long delay_time)
{
   long i;
   
   for(i=0; i<delay_time; i++);
 
}

/*void delay_ms(u16 nms)
{	 		  	  
	u32 temp;		   
	SysTick->LOAD=(u32)nms*fac_ms;//ʱ�����(SysTick->LOADΪ24bit)
	SysTick->VAL =0x00;           //��ռ�����
	SysTick->CTRL=0x01 ;          //��ʼ����  
	do
	{
		temp=SysTick->CTRL;
	}
	while(temp&0x01&&!(temp&(1<<16)));//�ȴ�ʱ�䵽��   
	SysTick->CTRL=0x00;       //�رռ�����
	SysTick->VAL =0X00;       //��ռ�����	  
}   */


//��ʱnus
//nusΪҪ��ʱ��us��.
/**************************ʵ�ֺ���********************************************
*����ԭ��:		void delay_us(u32 nus)
*��������:		΢�뼶��ʱ  ��ʱnus  nms<=1864 
*******************************************************************************/		    								   
/*
void delay_us(u32 nus)
{		
	u32 temp;	    	 
	SysTick->LOAD=nus*fac_us; //ʱ�����	  		 
	SysTick->VAL=0x00;        //��ռ�����
	SysTick->CTRL=0x01 ;      //��ʼ���� 	 
	do
	{
		temp=SysTick->CTRL;
	}
	while(temp&0x01&&!(temp&(1<<16)));//�ȴ�ʱ�䵽��   
	SysTick->CTRL=0x00;       //�رռ�����
	SysTick->VAL =0X00;       //��ռ�����	 
}*/
