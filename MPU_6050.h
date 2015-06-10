#include "r_cg_macrodriver.h"
#ifndef _MPU_6050_
#define _MPU_6050_


#define	SMPLRT_DIV		0x19	//SAMPLE RATE DIVIDER, value: 0x07 (125Hz)
#define	CONFIG			0x1A	//CONFIGURATION, value: 0x06 (5Hz)
#define	GYRO_CONFIG		0x1B	//GYROSCOPE CONFIGURATION, value: 0x18 (2000deg/s)
#define	ACCEL_CONFIG		0x1C	//ACCELEROMETER CONFIGURATION, value: 0x01 (2G,5Hz)

#define	ACCEL_XOUT_H	0x3B	//0
#define	ACCEL_XOUT_L	0x3C	//1
#define	ACCEL_YOUT_H	0x3D	//2
#define	ACCEL_YOUT_L	0x3E	//3
#define	ACCEL_ZOUT_H	0x3F	//4
#define	ACCEL_ZOUT_L	0x40	//5

#define	TEMP_OUT_H	0x41	//6
#define	TEMP_OUT_L	0x42	//7

#define	GYRO_XOUT_H	0x43	//8
#define	GYRO_XOUT_L	0x44	//9
#define	GYRO_YOUT_H	0x45	//10
#define	GYRO_YOUT_L	0x46	//11
#define	GYRO_ZOUT_H	0x47	//12
#define	GYRO_ZOUT_L	0x48	//13

#define	PWR_MGMT_1		0x6B	//POWER MANAGEMENT 1,value:0x00
#define	WHO_AM_I			0x75	//WHO AM I(value 0x68)
#define	SlaveAddress	0xD0	//



//typedefine
#ifndef __TYPEDEF_
#define __TYPEDEF

typedef struct 
{
	int16_t Ax,Ay,Az,Tt,Gx,Gy,Gz;
}	MPU_TypeDef;

#endif

void MPU6050_RD_XYZ(MPU_TypeDef* Mympu);
void MPU6050_RD_Avval(MPU_TypeDef* Mympu);
uint8_t MPU6050_ReadI2C(uint8_t REG_Address);
void MPU6050_WriteI2C(uint8_t REG_Address,uint8_t REG_data);
void MPU6050_Init(void);
void get_MPUdata();
void MPU6050_start();
void F_up_try();





#endif