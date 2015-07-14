/************************************************
* File Name	: IIC.c
* Version	: 2015.6.10 By DHP
* Device(s)	: R5F100LE
* Tool-Chain	: CA78K0R
* Description	: IIC
* API		: void IIC_Init(void)
		  uint8_t IIC_Read_Bytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data)
		  uint8_t IIC_Write_Bytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t* data)
		  uint8_t IIC_Write_Byte(uint8_t dev, uint8_t reg, uint8_t data)
		  uint8_t IIC_Write_Bits(uint8_t dev, uint8_t reg, uint8_t bitStart, uint8_t length, uint8_t data)
		 		
************************************************/


#include "include.h"

#define	IIC_SDA	P5.1
#define IIC_SCL P5.0
#define READ_SDA P5.1

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



/**************************ʵ�ֺ���********************************************
*����ԭ��:		void IIC_Init(void)
*��������:		��ʼ��I2C��Ӧ�Ľӿ����š�
*******************************************************************************/

void IIC_Init(void) // initilize P5.0 and P5.1
{
	SCL_OUTMODE();	//set P5.0 output mode
	SDA_OUTMODE();	//set P5.1 output mode
	IIC_SDA=1U;
	IIC_SCL=1U;
}


/**************************ʵ�ֺ���********************************************
*����ԭ��:		void IIC_Start(void)
*��������:		IIC 6050 communication start
*******************************************************************************/

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


/**************************ʵ�ֺ���********************************************
*����ԭ��:		void IIC_Stop(void)
*��������:	    //����IICֹͣ�ź�
*******************************************************************************/

void IIC_Stop(void)
{
	SDA_OUTMODE();	//set P5.1 output mode
	IIC_SCL=0U;
	IIC_SDA=0U;	//STOP : when CLK is high DATA change form low to high
 	delay_us(4);
	IIC_SCL=1U;
	IIC_SDA=1U;
	delay_us(4);
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		uint8_t IIC_Wait_Ack(void)
*��������:	    �ȴ�Ӧ���źŵ��� 
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
*******************************************************************************/

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

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void IIC_Ack(void)
*��������:	    ����ACKӦ��
*******************************************************************************/

void IIC_Ack(void)
{
	IIC_SCL=0U;
	SDA_OUTMODE();
	IIC_SDA=0U; 	//SDA is low��SCL change form low to high and then low
	delay_us(2);
	IIC_SCL=1U;
	delay_us(2);
	IIC_SCL=0U;
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void IIC_NAck(void)
*��������:	    ����NACKӦ��
*******************************************************************************/
void IIC_NAck(void)
{
	IIC_SCL = 0U;
	SDA_OUTMODE();
	IIC_SDA = 1U;  	//SDA is high��refuse ACK 
	delay_us(2);
	IIC_SCL = 1U;
	delay_us(2);
	IIC_SCL = 0U;
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		uint8_t IIC_Receive_Byte(unsigned char ack)
*��������:	    //��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK 
*******************************************************************************/ 
uint8_t IIC_Receive_Byte(unsigned char ack)
{
	unsigned char i,receive = 0;
	SDA_INMODE();
  	for(i = 0; i < 8; i++ )
	{
		IIC_SCL = 0U;
		delay_us(2);
		IIC_SCL = 1U;
		receive <<= 1; 
		if(READ_SDA)receive++;	//READ_SDA is P5.1
		delay_us(1); 
	}
	
    	if (!ack)
        	IIC_NAck();	//refuse ACK 
    	else
        	IIC_Ack();	//ACK 
    	return receive;
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void IIC_Send_Byte(unsigned char txd)
*��������:	    IIC����һ���ֽ�
*******************************************************************************/

void IIC_Send_Byte(unsigned char txd)	//send Byte
{                        
    uint8_t t;
    SDA_OUTMODE();
    IIC_SCL=0U;
    for(t = 0; t < 8; t++)
    {
	IIC_SDA = (txd & 0x80) >> 7;
	txd <<= 1;	  
	delay_us(2);   
	IIC_SCL = 1U;
	delay_us(2); 
	IIC_SCL = 0U;	
	delay_us(2);
    }	 
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		uint8_t IIC_Read_Bytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data)
*��������:	    ��ȡָ���豸 ָ���Ĵ����� length��ֵ
����		dev  Ŀ���豸��ַ
		reg	  �Ĵ�����ַ
		length Ҫ�����ֽ���
		*data  ���������ݽ�Ҫ��ŵ�ָ��
����   ���������ֽ�����
*******************************************************************************/ 

uint8_t IIC_Read_Bytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data)
{
    	uint8_t count = 0;
	uint8_t temp;
	IIC_Start();
	IIC_Send_Byte(dev);	   //����д����
	IIC_Wait_Ack();
	IIC_Send_Byte(reg);   //���͵�ַ
	IIC_Wait_Ack();	  
	IIC_Start();
	IIC_Send_Byte(dev + 1);  //�������ģʽ	
	IIC_Wait_Ack();
	
    	for(count = 0; count < length; count++)
	{
		 
		 if(count != (length - 1) )
		 	temp = IIC_Receive_Byte(1);  //��ACK�Ķ�����
		 	else  
			temp = IIC_Receive_Byte(0);	 //���һ���ֽ�NACK

		data[count] = temp;
	}
	IIC_Stop();//����һ��ֹͣ����
	return count;
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		uint8_t IIC_Write_Bytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t* data)
*��������:	    ������ֽ�д��ָ���豸 ָ���Ĵ���
����		dev  Ŀ���豸��ַ
		reg	  �Ĵ�����ַ
		length Ҫд���ֽ���
		*data  ��Ҫд�����ݵ��׵�ַ
����   �����Ƿ�ɹ�
*******************************************************************************/ 
uint8_t IIC_Write_Bytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t* data)
{
  
 	uint8_t count = 0;
	IIC_Start();
	IIC_Send_Byte(dev);	   //����д����
	IIC_Wait_Ack();
	IIC_Send_Byte(reg);   //���͵�ַ
	IIC_Wait_Ack();	  
	for(count = 0; count < length; count++)
	{
		IIC_Send_Byte(data[count]); 
		IIC_Wait_Ack(); 
	 }
	IIC_Stop();//����һ��ֹͣ����

	return 1; //status == 0;
}



/**************************ʵ�ֺ���********************************************
*����ԭ��:		uint8_t IIC_Write_Byte(uint8_t dev, uint8_t reg, uint8_t data)
*��������:	    д��ָ���豸 ָ���Ĵ���һ���ֽ�
����		dev  Ŀ���豸��ַ
		reg	   �Ĵ�����ַ
		data  ��Ҫд����ֽ�
����   1
*******************************************************************************/ 
uint8_t IIC_Write_Byte(uint8_t dev, uint8_t reg, uint8_t data)
{
	return IIC_Write_Bytes(dev, reg, 1, &data);
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		uint8_t IIC_Write_Bits(uint8_t dev, uint8_t reg, uint8_t bitStart, uint8_t length, uint8_t data)
*��������:	    �� �޸� д ָ���豸 ָ���Ĵ���һ���ֽ� �еĶ��λ
����	dev  Ŀ���豸��ַ
		reg	   �Ĵ�����ַ
		bitStart  Ŀ���ֽڵ���ʼλ
		length   λ����
		data    ��Ÿı�Ŀ���ֽ�λ��ֵ
����   	�ɹ� Ϊ1 
 		ʧ��Ϊ0
*******************************************************************************/ 
uint8_t IIC_Write_Bits(uint8_t dev, uint8_t reg, uint8_t bitStart, uint8_t length, uint8_t data)
{

    uint8_t b;
    if (IIC_Read_Bytes(dev, reg, 1, &b) != 0) 
    {
        uint8_t mask = (0xFF << (bitStart + 1)) | 0xFF >> ((8 - bitStart) + length - 1);
        data <<= (8 - length);
        data >>= (7 - bitStart);
        b &= mask;
        b |= data;
        return IIC_Write_Byte(dev, reg, b);
    }
	else 
	{
        	return 0;
    	}
}