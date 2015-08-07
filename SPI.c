/************************************************
* File Name	: SPI.c
* Version	: 2015.7.19 By DHP
* Device(s)	: R5F100LE
* Tool-Chain	: CA78K0R
* Description	: SPI for ADNS_3080
* API		: 
		  SPI_Init();
		  uint8_t SPI_Write_Bytes(uint8_t reg, uint8_t length, uint8_t * data);
		  uint8_t SPI_Write_Byte(uint8_t reg, uint8_t data);

		  uint8_t SPI_Read_Byte(uint8_t reg);
		  void SPI_Read_Bytes(uint8_t reg, uint8_t length, uint8_t *data);

		  void SPI_Burst_Mode_Read(uint8_t reg, uint16_t length, uint8_t * data);
		  void SPI_Burst_Mode_Write(uint8_t reg, uint16_t length, uint8_t * data);
		 		
************************************************/

#include "SPI.h"
#include "r_cg_port.h"
#include "delay.h"

#define SPI_SCLK P5.2  // pin 35
#define SPI_MOSI P5.4  // pin 37
#define SPI_MISO P1.5  // pin 41
#define SPI_NCS P1.0   // pin 46

void SCLK_OUTMODE()	//set P5.2 output mode
{
	PM5 &= ~(_04_PMn2_MODE_INPUT);
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
	PM1 &= ~(_01_PMn0_MODE_INPUT);
	PU1 &= ~(_01_PMn0_MODE_INPUT);
}


/**************************ʵ�ֺ���********************************************
*����ԭ��:		void SPI_Init(void)
*��������:		��ʼ�� SPI ��Ӧ�Ľӿ����š�
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

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void SPI_Start(void)
*��������:		SPI A3080 communication start
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
	SPI_MOSI = 0U;
	SPI_MISO = 0U;
}


/**************************ʵ�ֺ���********************************************
*����ԭ��:		void SPI_Stop(void)
*��������:	    SPI A3080 communication stop
*******************************************************************************/

void SPI_Stop(void)
{
	delay_us(1); //From last SCLK falling edge to NCS rising edge,  for valid MISO data transfer need 120 ns
	SPI_NCS = 1U;
	delay_us(1);
	SPI_SCLK = 1U;
}


/**************************ʵ�ֺ���********************************************
*����ԭ��:		void SPI_Send_Byte(unsigned char txd)
*��������:	    SPI����һ���ֽ�
*******************************************************************************/
void SPI_Send_Byte(unsigned char txd)	//send Byte 2 * 8 = 16 us
{
	uint8_t i;
	for(i = 0; i < 8; i++)
	{
		SPI_SCLK = 0U;
		//delay_us(1);		
		delay_250_ns();
		SPI_MOSI = (txd & 0x80) >> 7;
		txd <<= 1;	  
			////delay_us(1);   
		SPI_SCLK = 1U;
		//delay_us(1);		
		delay_250_ns();
	}
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void SPI_Receive_Byte()
*��������:	    SPI����һ���ֽ�
*******************************************************************************/
uint8_t SPI_Receive_Byte()  // need 2 * 8 = 16 us
{
	uint8_t i, receive;
	for(i = 0; i < 8; i++ )
	{
		SPI_SCLK = 0U;
		//delay_us(1);		
		delay_250_ns();
		SPI_SCLK = 1U;
		//delay_us(1);		
		delay_250_ns();
		receive <<= 1; 
		if(SPI_MISO) receive++;
			////delay_us(1);
	}
	return receive;
}


/**************************ʵ�ֺ���********************************************
*����ԭ��:		uint8_t SPI_Write_Bytes(uint8_t reg, uint8_t length, uint8_t* data)
*��������:	    ������ֽ�д��ָ���豸 ָ���Ĵ���
*����		dev  Ŀ���豸��ַ
*		reg	  �Ĵ�����ַ
*		length Ҫд���ֽ���
*		*data  ��Ҫд�����ݵ��׵�ַ
*����   �����Ƿ�ɹ�
*******************************************************************************/ 

uint8_t SPI_Write_Bytes(uint8_t reg, uint8_t length, uint8_t * data)
{
	uint16_t txd;
	uint8_t i, j;
	SPI_Start();
	for(j = 0; j < length; j++)
	{
		txd = ((uint16_t) (reg | 0x80) << 8) | data[j];
		for(i = 0; i < 16; i++)
		{
			SPI_SCLK = 0U;
			SPI_MOSI = (txd & 0x8000) >> 15;
			txd <<= 1;
			delay_us(1);
			SPI_SCLK = 1U;
			delay_us(1);
		}
		delay_us(20); // SPI time between write commands (1 + 1) * 16 + 20 = 52 > 50 us
	}
	SPI_Stop();
	return 1;
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		uint8_t SPI_Write_Byte(uint8_t reg, uint8_t data)
*��������:	    д��ָ���豸 ָ���Ĵ���һ���ֽ�
*����		dev  Ŀ���豸��ַ
*		reg	   �Ĵ�����ַ
*		data  ��Ҫд����ֽ�
*����   1
*******************************************************************************/ 

uint8_t SPI_Write_Byte(uint8_t reg, uint8_t data)
{
	return SPI_Write_Bytes(reg, 1, &data);
	return 1;
}





/**************************ʵ�ֺ���********************************************
*����ԭ��:		uint8_t SPI_Read_Byte(uint8_t reg)
*��������:	    ��ȡָ���豸 ָ���Ĵ���
*����		dev  Ŀ���豸��ַ
*		reg	  �Ĵ�����ַ
*����   ����������
*******************************************************************************/ 

uint8_t SPI_Read_Byte(uint8_t reg)
{
	uint8_t txd;
	uint8_t rxd;
	SPI_Start();
	txd = reg & (~ 0x80);
	rxd = 0x00;
	
	SPI_Send_Byte(txd);

	if((reg == 0x02) || (reg == 0x40) || (reg == 0x50)) delay_us(63); // tSRAD >= 50 us for non-motion read; tSRAD-MOT >= 75 us for register 0x02
		else delay_us(38);

	rxd = SPI_Receive_Byte();
	delay_us(1);
	SPI_Stop();
	return rxd;
}


/**************************ʵ�ֺ���********************************************
*����ԭ��:		void SPI_Read_Bytes(uint8_t reg, uint8_t length, uint8_t *data)
*��������:	    ��ȡָ���豸 ָ���Ĵ����� length��ֵ
*����		dev  Ŀ���豸��ַ
*		reg	  �Ĵ�����ַ
*		length Ҫ�����ֽ���
*		*data  ���������ݽ�Ҫ��ŵ�ָ��
*******************************************************************************/ 

void SPI_Read_Bytes(uint8_t reg, uint8_t length, uint8_t *data)
{
	uint8_t k;
	uint8_t txd;
	SPI_Start();

		txd = reg & (~ 0x80);
		for(k = 0; k < length; k++)
		{			
			SPI_Send_Byte(txd + k);

			// tSRAD >= 50 us for non-motion read; tSRAD-MOT >= 75 us for register 0x02
			if((reg + k) == 0x02) delay_us(63); 
			else delay_us(38);

			data[k] = SPI_Receive_Byte();
		}
	delay_us(1);
	SPI_Stop();
}

/******************************************************************************
* function :		void SPI_Burst_Mode_Read(uint8_t reg, uint16_t length, uint8_t * data)
* Description : Burst mode is a special serial port operation mode 
* which may be used to reduce the serial transaction time for three predefined operations:
* motion read and SROM download and frame capture.
* The speed improvement is achieved by continuous data clocking to 
* or from multiple registers without the need to specify the register address,
* and by not requiring the normal delay period between data bytes.
*******************************************************************************/ 

void SPI_Burst_Mode_Read(uint8_t reg, uint16_t length, uint8_t * data)
{
	uint16_t k;
	uint8_t txd;

	SPI_Start();
	txd = reg & (~ 0x80);
	SPI_Send_Byte(txd);
	delay_us(63);

	for(k = 0; k < length; k++)
	{
		data[k] = SPI_Receive_Byte();	
	}
	delay_us(1);
	SPI_Stop();
	delay_us(4); //Time NCS must be held high to exit burst mode
}


/******************************************************************************
* function :		void SPI_Burst_Mode_Write(uint8_t reg, uint16_t length, uint8_t * data)
* Description : Burst mode is a special serial port operation mode 
* which may be used to reduce the serial transaction time for three predefined operations:
* motion read and SROM download and frame capture.
* The speed improvement is achieved by continuous data clocking to 
* or from multiple registers without the need to specify the register address,
* and by not requiring the normal delay period between data bytes.
*******************************************************************************/ 

void SPI_Burst_Mode_Write(uint8_t reg, uint16_t length, uint8_t * data)
{
	uint16_t k;
	uint8_t txd;
	SPI_Start();
	txd = reg | 0x80;
	SPI_Send_Byte(txd);
	for(k = 0; k < length; k++)
	{
		txd = data[k];
		SPI_Send_Byte(txd);
	}
	delay_us(1);
	SPI_Stop();
	delay_us(4); //Time NCS must be held high to exit burst mode
}