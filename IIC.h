#include "r_cg_macrodriver.h"
#ifndef _IIC_
#define _IIC_


#define	IIC_SDA	P5.1
#define IIC_SCL P5.0
#define READ_SDA P5.1
#define delay_time_i 13

void delay_us(uint8_t t);
void delay_ms(uint8_t t);
void SCL_OUTMODE();
void SDA_INMODE();	//set SDA input mode
void SDA_OUTMODE();
uint8_t IIC_Read_Byte(unsigned char ack);
void IIC_Send_Byte(unsigned char txd);	//send data
void IIC_Init(void);
void IIC_Start(void);
void IIC_Stop(void);
uint8_t IIC_Wait_Ack(void);
void IIC_Ack(void);
void IIC_NAck(void);

#endif