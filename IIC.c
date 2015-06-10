/************************************************
* File Name    : IIC.c
* Version      : 2015.6.10 By DHP
* Device(s)    : R5F100LE
* Tool-Chain   : CA78K0R
* Description  : IIC
************************************************/


#include "IIC.h"
#include "r_cg_macrodriver.h"
#include "r_cg_cgc.h"
#include "r_cg_port.h"
#include "r_cg_timer.h"
#include "r_cg_userdefine.h"



void IIC_Init(void) // initilize P5.0 and P5.1
{
	SCL_OUTMODE();	//set P5.0 output mode
	SDA_OUTMODE();	//set P5.1 output mode
	IIC_SDA=1U;
	IIC_SCL=1U;
}


//IIC 6050 communication start

void IIC_Start(void)
{
	SDA_OUTMODE();     //set P5.1 output mode
	IIC_SDA=1U;	  	  
	IIC_SCL=1U;
	delay_us(4);
 	IIC_SDA=0U;	//START : when CLK is high,DATA change form high to low 
	delay_us(4);
	IIC_SCL=0U; 
}

//IIC与6050通信结束的标志

void IIC_Stop(void)
{	SDA_OUTMODE();	//set P5.1 output mode
	IIC_SCL=0U;
	IIC_SDA=0U;	//STOP : when CLK is high DATA change form low to high
 	delay_us(4);
	IIC_SCL=1U;
	delay_us(4);
	IIC_SDA=1U;							   	
}



uint8_t IIC_Wait_Ack(void)	//wait for ACK
{
	uint8_t ucErrTime=0;
	SDA_INMODE();      //set P5.0 input mode 
	IIC_SDA=1;delay_us(1);	   
	IIC_SCL=1;delay_us(1);	 
	while(READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop();
			return 1;
		}
	}
	IIC_SCL=0U;	   
	return 0;  
} 



void IIC_Ack(void)
{
	IIC_SCL=0U;
	SDA_OUTMODE();
	IIC_SDA=0U; 	//SDA is low，SCL change form low to high and then low
	delay_us(2);
	IIC_SCL=1U;
	delay_us(2);
	IIC_SCL=0U;
}
		    
void IIC_NAck(void)
{
	IIC_SCL=0U;
	SDA_OUTMODE();
	IIC_SDA=1U;  	//SDA is high，refuse ACK 
	delay_us(2);
	IIC_SCL=1U;
	delay_us(2);
	IIC_SCL=0U;
}


uint8_t IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_INMODE();
  	for(i=0;i<8;i++ )
	{
		IIC_SCL=0U;
		delay_us(2);
		IIC_SCL=1U;
		receive<<=1; 
		if(READ_SDA)receive++;	//READ_SDA is P5.1
		delay_us(1); 
	}
	
    	if (!ack)
        	IIC_NAck();	//refuse ACK 
    	else
        	IIC_Ack();	//ACK 
    	return receive;
}



void IIC_Send_Byte(unsigned char txd)	//send Byte
{                        
    uint8_t t;
    SDA_OUTMODE();
    IIC_SCL=0U;
    for(t=0;t<8;t++)
    {
	IIC_SDA=(txd&0x80)>>7;
	txd<<=1;	  
	delay_us(2);   
	IIC_SCL=1U;
	delay_us(2); 
	IIC_SCL=0U;	
	delay_us(2);
    }	 
}

void SCL_OUTMODE()	//set P5.0 output mode
{
	PM5 &= ~(_01_PMn0_MODE_INPUT);
}

void SDA_INMODE()	//set P5.0 input mode
{
	PM5|=_02_PMn1_MODE_INPUT;	//set P5.0 input mode
	PU5|=_02_PUn1_PULLUP_ON;	//set P5.1 with pullup resistor
	IIC_SDA=1U;	//set P5.1 as high
}

void SDA_OUTMODE()  //set P5.0 output mode
{
	PM5 &= ~(_02_PMn1_MODE_INPUT);
	PU5 &= ~(_02_PUn1_PULLUP_ON);
}

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