#ifndef _SPI_
#define _SPI_

#include "r_cg_macrodriver.h"


#define ADNS3080_Address 0x17

uint8_t SPI_Write_Bytes(uint8_t reg, uint8_t length, uint8_t * data);
uint8_t SPI_Write_Byte(uint8_t reg, uint8_t data);

uint8_t SPI_Read_Byte(uint8_t reg);
void SPI_Read_Bytes(uint8_t reg, uint8_t length, uint8_t *data);

void SPI_Burst_Mode_Read(uint8_t reg, uint16_t length, uint8_t * data);
void SPI_Burst_Mode_Write(uint8_t reg, uint16_t length, uint8_t * data);

#endif