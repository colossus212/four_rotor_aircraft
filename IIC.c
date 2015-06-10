
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


//IIC与6050通信开始的标志
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



uint8_t IIC_Wait_Ack(void)//等待应答 
{
	uint8_t ucErrTime=0;
	SDA_INMODE();      //设置SDA为输入模式 
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
	IIC_SDA=0U; //SDA为低，而SCL经历了从低到高再到低的变化过程 
	delay_us(2);
	IIC_SCL=1U;
	delay_us(2);
	IIC_SCL=0U;
}
		    
void IIC_NAck(void)
{
	IIC_SCL=0U;
	SDA_OUTMODE();
	IIC_SDA=1U;  //SDA为高，不应答 
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
		if(READ_SDA)receive++;	//READ_SDA也即是P5.1
		delay_us(1); 
	}
	
    	if (!ack)
        	IIC_NAck();//拒绝应答 
    	else
        	IIC_Ack(); //应答  
    	return receive;
}



void IIC_Send_Byte(unsigned char txd)//发送数据 
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

void SDA_INMODE()//SDA为输入模式
{
	PM5|=_02_PMn1_MODE_INPUT;//设置P5.1为输入模式
	PU5|=_02_PUn1_PULLUP_ON;//设置P5.1上拉电阻
	IIC_SDA=1U;//设5.1位高电平
}

void SDA_OUTMODE()  //配置P5.1为输出模式
{
	PM5 &= ~(_02_PMn1_MODE_INPUT);
	PU5 &= ~(_02_PUn1_PULLUP_ON);
}

void delay_us(uint8_t t)
{
	uint8_t i;
	while (t--)
	{
		for(i=0;i<13;i++);
	}
}

void delay_ms(uint8_t t)
{
	while (t--)
	{
		delay_us(1000);
	}
}