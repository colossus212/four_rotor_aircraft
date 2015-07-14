/*****************************************
编写者：DHP
作者E-mail：131180064@smail.nju.edu.cn
编译环境：CubeSuite+
初版时间: 2015-06-20
功能：
提供延时API  有 微秒级 和 毫秒级 延时
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
	SysTick->LOAD=(u32)nms*fac_ms;//时间加载(SysTick->LOAD为24bit)
	SysTick->VAL =0x00;           //清空计数器
	SysTick->CTRL=0x01 ;          //开始倒数  
	do
	{
		temp=SysTick->CTRL;
	}
	while(temp&0x01&&!(temp&(1<<16)));//等待时间到达   
	SysTick->CTRL=0x00;       //关闭计数器
	SysTick->VAL =0X00;       //清空计数器	  
}   */


//延时nus
//nus为要延时的us数.
/**************************实现函数********************************************
*函数原型:		void delay_us(u32 nus)
*功　　能:		微秒级延时  延时nus  nms<=1864 
*******************************************************************************/		    								   
/*
void delay_us(u32 nus)
{		
	u32 temp;	    	 
	SysTick->LOAD=nus*fac_us; //时间加载	  		 
	SysTick->VAL=0x00;        //清空计数器
	SysTick->CTRL=0x01 ;      //开始倒数 	 
	do
	{
		temp=SysTick->CTRL;
	}
	while(temp&0x01&&!(temp&(1<<16)));//等待时间到达   
	SysTick->CTRL=0x00;       //关闭计数器
	SysTick->VAL =0X00;       //清空计数器	 
}*/
