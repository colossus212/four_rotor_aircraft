/************************************************
* File Name	: SPI.c
* Version	: 2015.7.19 By DHP
* Device(s)	: R5F100LE
* Tool-Chain	: CA78K0R
* Description	: SPI for ADNS_3080
* API		: 
		  uint8_t SPI_Write_Bytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t * data)
		  uint8_t SPI_Write_Byte(uint8_t dev, uint8_t reg, uint8_t length, uint8_t * data)
		  uint8_t SPI_Read_Byte(uint8_t dev, uint8_t reg, uint8_t * data)
		 		
************************************************/


#include "SPI.h"

#define SPI_SCLK P5.3
#define SPI_MOSI P5.4
#define SPI_MISO P1.5
#define SPI_NCS P1.0

void SCLK_OUTMODE()	//set P5.3 output mode
{
	PM5 &= ~(_08_PMn3_MODE_INPUT);
}

void MISO_INMODE()	//set P1.5 input mode
{
	PM1 |= _20_PMn5_MODE_INPUT;	//set P1.5 input mode
	PU1 |= _20_PUn5_PULLUP_ON;	//set P1.5 with pullup resistor
	SPI_MISO = 1U;	//set P5.1 as high
}

void MOSI_OUTMODE()  //set P5.4 output mode
{
	PM5 &= ~(_10_PMn4_MODE_INPUT);
	PU5 &= ~(_10_PUn4_PULLUP_ON);
}

void NCS_OUTMODE()  //set P1.0 output mode
{
	PM1 &= ~(_02_PMn1_MODE_INPUT);
	PU1 &= ~(_02_PUn1_PULLUP_ON);
}


/**************************实现函数********************************************
*函数原型:		void SPI_Init(void)
*功　　能:		初始化 SPI 对应的接口引脚。
*******************************************************************************/
void SPI_Init(void)
{
	SCLK_OUTMODE();
	MISO_INMODE();
	MOSI_OUTMODE();
	NCS_OUTMODE();
	SPI_SCLK = 1U;
	SPI_MOSI = 1U;
	SPI_MISO = 1U;
	SPI_NCS = 1U;
}

/**************************实现函数********************************************
*函数原型:		void SPI_Start(void)
*功　　能:		SPI A3080 communication start
*******************************************************************************/

void SPI_Start(void)
{
	SPI_SCLK = 1U;
	SPI_MOSI = 1U;
	SPI_MISO = 1U;
	SPI_NCS = 1U;
	delay_us(1);
 	SPI_NCS = 0U;
	delay_us(1);   //From NCS falling edge to first SCLK rising edge need 120 ns
	SPI_SCLK = 0U;
}


/**************************实现函数********************************************
*函数原型:		void SPI_Stop(void)
*功　　能:	    SPI A3080 communication stop
*******************************************************************************/

void SPI_Stop(void)
{	
	SPI_SCLK = 0U;
	delay_us(1); //From last SCLK falling edge to NCS rising edge,  for valid MISO data transfer need 120 ns
	SPI_NCS = 1U;
	delay_us(1);
}


/**************************实现函数********************************************
*函数原型:		uint8_t SPI_Write_Bytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t* data)
*功　　能:	    将多个字节写入指定设备 指定寄存器
*输入		dev  目标设备地址
*		reg	  寄存器地址
*		length 要写的字节数
*		*data  将要写的数据的首地址
*返回   返回是否成功
*******************************************************************************/ 

uint8_t SPI_Write_Bytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t * data)
{
	uint16_t txd;
	uint8_t i, j;
	if(dev == ADNS3080_Address) SPI_Start();
	for(j = 0; j < length; j++)
	{
		txd = ((uint16_t) (reg | 0x80) << 8) | data[i];
		for(i = 0; i < 16; i++)
		{
			SPI_MOSI = (txd & 0x80) >> 15;
			txd <<= 1;	  
			delay_us(1);   
			SPI_SCLK = 1U;
			delay_us(1); 
			SPI_SCLK = 0U;	
			delay_us(1);
		}
		delay_us(5); // SPI time between write commands (1 + 1 + 1) * 16 + 5 = 53 > 50 us
	}
	SPI_Stop();
	return 1;
}

/**************************实现函数********************************************
*函数原型:		uint8_t SPI_Write_Byte(uint8_t dev, uint8_t reg, uint8_t data)
*功　　能:	    写入指定设备 指定寄存器一个字节
*输入		dev  目标设备地址
*		reg	   寄存器地址
*		data  将要写入的字节
*返回   1
*******************************************************************************/ 

uint8_t SPI_Write_Byte(uint8_t dev, uint8_t reg, uint8_t length, uint8_t * data)
{
	return SPI_Write_Bytes(dev, reg, 1, &data);
	return 1;	
}


/**************************实现函数********************************************
*函数原型:		uint8_t SPI_Read_Byte(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data)
*功　　能:	    读取指定设备 指定寄存器的 length个值
*输入		dev  目标设备地址
*		reg	  寄存器地址
*		length 要读的字节数
*		*data  读出的数据将要存放的指针
*返回   读出来的字节数量
*******************************************************************************/ 

uint8_t SPI_Read_Byte(uint8_t dev, uint8_t reg, uint8_t * data)
{
	uint8_t i, j;
	uint8_t txd;
	uint8_t rxd;
	if(dev == ADNS3080_Address) SPI_Start();

		txd = reg & (~ 0x80);
		rxd = 0x00;
		for(i = 0; i < 8; i++)
		{
			SPI_MOSI = (txd & 0x80) >> 7;
			txd <<= 1;	  
			delay_us(1);   
			SPI_SCLK = 1U;
			delay_us(1); 
			SPI_SCLK = 0U;	
			delay_us(1);
		}
		if(reg == 0x02) delay_us(55); // tSRAD >= 50 us for non-motion read; tSRAD-MOT >= 75 us for register 0x02
		else delay_us(30);

		for(j = 0; j < 8; j++ )
		{
			SPI_SCLK = 0U;
			delay_us(1);
			SPI_SCLK = 1U;
			rxd <<= 1; 
			if(SPI_MISO) rxd++;
			delay_us(1);
		}
	delay_us(1);
	SPI_Stop();
}
