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
		  
		  #define IIC_SDA P5.1  //  Pin 34
		  #define IIC_SCL P5.0  //  Pin 33
		 		
************************************************/


#include "include.h"

#define	IIC_SDA	P5.1  //  Pin 34
#define IIC_SCL P5.0  //  Pin 33
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



/**************************实现函数********************************************
*函数原型:		void IIC_Init(void)
*功　　能:		初始化I2C对应的接口引脚。
*******************************************************************************/

void IIC_Init(void) // initilize P5.0 and P5.1
{
	SCL_OUTMODE();	//set P5.0 output mode
	SDA_OUTMODE();	//set P5.1 output mode
	IIC_SDA=1U;
	IIC_SCL=1U;
}


/**************************实现函数********************************************
*函数原型:		void IIC_Start(void)
*功　　能:		IIC 6050 communication start
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


/**************************实现函数********************************************
*函数原型:		void IIC_Stop(void)
*功　　能:	    //产生IIC停止信号
*******************************************************************************/

void IIC_Stop(void)
{
	SDA_OUTMODE();	//set P5.1 output mode
	IIC_SCL = 0U;
	IIC_SDA = 0U;	//STOP : when CLK is high DATA change form low to high
 	delay_us(4);
	IIC_SCL = 1U;
	delay_us(4);
	IIC_SDA = 1U;
	delay_us(4);
}

/**************************实现函数********************************************
*函数原型:		uint8_t IIC_Wait_Ack(void)
*功　　能:	    等待应答信号到来 
*返回值：1，接收应答失败
*        0，接收应答成功
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

/**************************实现函数********************************************
*函数原型:		void IIC_Ack(void)
*功　　能:	    产生ACK应答
*******************************************************************************/

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

/**************************实现函数********************************************
*函数原型:		void IIC_NAck(void)
*功　　能:	    产生NACK应答
*******************************************************************************/
void IIC_NAck(void)
{
	IIC_SCL = 0U;
	SDA_OUTMODE();
	IIC_SDA = 1U;  	//SDA is high，refuse ACK 
	delay_us(2);
	IIC_SCL = 1U;
	delay_us(2);
	IIC_SCL = 0U;
}

/**************************实现函数********************************************
*函数原型:		uint8_t IIC_Receive_Byte(unsigned char ack)
*功　　能:	    //读1个字节，ack=1时，发送ACK，ack=0，发送nACK 
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

/**************************实现函数********************************************
*函数原型:		void IIC_Send_Byte(unsigned char txd)
*功　　能:	    IIC发送一个字节
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

/**************************实现函数********************************************
*函数原型:		uint8_t IIC_Read_Bytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data)
*功　　能:	    读取指定设备 指定寄存器的 length个值
*输入		dev  目标设备地址
*		reg	  寄存器地址
*		length 要读的字节数
*		*data  读出的数据将要存放的指针
*返回   读出来的字节数量
*******************************************************************************/ 

uint8_t IIC_Read_Bytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data)
{
    	uint8_t count = 0;
	uint8_t temp;
	IIC_Start();
	IIC_Send_Byte(dev);	   
	IIC_Wait_Ack();
	IIC_Send_Byte(reg);   
	IIC_Wait_Ack();	  
	IIC_Start();
	IIC_Send_Byte(dev + 1);  	
	IIC_Wait_Ack();
	
    	for(count = 0; count < length; count++)
	{
		 
		 if(count != (length - 1) )
		 	temp = IIC_Receive_Byte(1);  
		 	else  
			temp = IIC_Receive_Byte(0);	

		data[count] = temp;
	}
	IIC_Stop();
	return count;
}

/**************************实现函数********************************************
*函数原型:		uint8_t IIC_Write_Bytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t* data)
*功　　能:	    将多个字节写入指定设备 指定寄存器
*输入		dev  目标设备地址
*		reg	  寄存器地址
*		length 要写的字节数
*		*data  将要写的数据的首地址
*返回   返回是否成功
*******************************************************************************/ 
uint8_t IIC_Write_Bytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t* data)
{
  
 	uint8_t count = 0;
	IIC_Start();
	IIC_Send_Byte(dev);	  
	IIC_Wait_Ack();
	IIC_Send_Byte(reg);   
	IIC_Wait_Ack();	  
	for(count = 0; count < length; count++)
	{
		IIC_Send_Byte(data[count]); 
		IIC_Wait_Ack(); 
	 }
	IIC_Stop();

	return 1; //status == 0;
}



/**************************实现函数********************************************
*函数原型:		uint8_t IIC_Write_Byte(uint8_t dev, uint8_t reg, uint8_t data)
*功　　能:	    写入指定设备 指定寄存器一个字节
*输入		dev  目标设备地址
*		reg	   寄存器地址
*		data  将要写入的字节
*返回   1
*******************************************************************************/ 
uint8_t IIC_Write_Byte(uint8_t dev, uint8_t reg, uint8_t data)
{
	return IIC_Write_Bytes(dev, reg, 1, &data);
	return 1; //status == 0;
}

/**************************实现函数********************************************
*函数原型:		uint8_t IIC_Write_Bits(uint8_t dev, uint8_t reg, uint8_t bitStart, uint8_t length, uint8_t data)
*功　　能:	    读 修改 写 指定设备 指定寄存器一个字节 中的多个位
*输入	dev  目标设备地址
*		reg	   寄存器地址
*		bitStart  目标字节的起始位
*		length   位长度
*		data    存放改变目标字节位的值
*返回   	成功 为1 
*		失败为0
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