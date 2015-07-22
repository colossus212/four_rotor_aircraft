#include "r_cg_macrodriver.h"
#ifndef _IIC_
#define _IIC_






void SCL_OUTMODE();
void SDA_INMODE();	//set SDA input mode
void SDA_OUTMODE();

void IIC_Init(void);

void IIC_Start(void);
void IIC_Stop(void);

uint8_t IIC_Wait_Ack(void);
void IIC_Ack(void);
void IIC_NAck(void);

uint8_t IIC_Receive_Byte(unsigned char ack);
void IIC_Send_Byte(unsigned char txd);	//send data
uint8_t IIC_Read_Bytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data);
uint8_t IIC_Write_Bytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t* data);
uint8_t IIC_Write_Byte(uint8_t dev, uint8_t reg, uint8_t data);
uint8_t IIC_Write_Bits(uint8_t dev, uint8_t reg, uint8_t bitStart, uint8_t length, uint8_t data);
uint8_t IIC_Write_Bit(uint8_t dev, uint8_t reg, uint8_t bitNum, uint8_t data);

#endif