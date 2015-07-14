
/************************************************
* File Name	: MPU_6050.c
* Version	: 2015.6.10 By DHP
* Device(s)	: R5F100LE
* Tool-Chain	: CA78K0R
* Description	: MPU6050
* API		: void MPU6050_Init(void)	初始化 MPU6050 的寄存器,计算零偏，并开启采样时钟
		  void MPU6050_Read_RawData();  获得 MPU6050 的六个测量值，已经转换单位并减去零偏 输出角度值
		  
		  float Get_MPU6050_Ax();
		  float Get_MPU6050_Ay();
		  float Get_MPU6050_Az();
		  
		  float Get_MPU6050_Gx();
		  float Get_MPU6050_Gy();
		  float Get_MPU6050_Gz();
		  
		  float Get_MPU6050_Ax_old();
		  float Get_MPU6050_Ay_old();
		  float Get_MPU6050_Az_old();
		  
		  float Get_MPU6050_Gx_old();
		  float Get_MPU6050_Gy_old();
		  float Get_MPU6050_Gz_old();
		  
		  
		  
		  extern MPU6050_Struct MPU6050_data, MPU6050_data_old;
		  
		  //void MPU6050_RawData_Average() 获得 MPU6050 三个加速度 8 次，并平均
		  
************************************************/

#include "include.h"

MPU6050_Struct MPU6050_data;

const float pi = 3.1415926;

float AcceRatio;   
float GyroRatio;
float TemperatureRatio = 340.0;
float TemperatureOffset = - 36.53;

uint8_t sample_times = 2000;	// for Get_Offset
float Ax_Offset, Ay_Offset, Az_Offset, Gx_Offset, Gy_Offset, Gz_Offset;	// zero offset
uint8_t MPU6050_Offset_Done = 0;

float Get_MPU6050_Ax()
{
	return MPU6050_data.Ax;
}

float Get_MPU6050_Ay()
{
	return MPU6050_data.Ay;
}

float Get_MPU6050_Az()
{
	return MPU6050_data.Az;
}

float Get_MPU6050_Gx()
{
	return MPU6050_data.Gx;
}

float Get_MPU6050_Gy()
{
	return MPU6050_data.Gy;
}

float Get_MPU6050_Gz()
{
	return MPU6050_data.Gz;
}

/*
float Get_MPU6050_Ax_old()
{
	return MPU6050_data_old.Ax;
}

float Get_MPU6050_Ay_old()
{
	return MPU6050_data_old.Ay;
}

float Get_MPU6050_Az_old()
{
	return MPU6050_data_old.Az;
}

float Get_MPU6050_Gx_old()
{
	return MPU6050_data_old.Gx;
}

float Get_MPU6050_Gy_old()
{
	return MPU6050_data_old.Gy;
}

float Get_MPU6050_Gz_old()
{
	return MPU6050_data_old.Gz;
}
*/


float Get_MPU6050_Gx_A()
{
	return MPU6050_data.Gx * pi / 180;
}
float Get_MPU6050_Gy_A()
{
	return MPU6050_data.Gy * pi / 180;
}
float Get_MPU6050_Gz_A()
{
	return MPU6050_data.Gz * pi / 180;
}



void MPU6050_Get_Offset()
{	
	
	uint8_t i;
	float Ax, Ay, Az, Gx, Gy, Gz;		//The raw data of accelerometers and gyroscope
	Ax=0;Ay=0;Az=0;
	Gx=0;Gy=0;Gz=0;		//initialize variables as the raw data of accelerometers and gyroscope
	
	
	for (i=0;i<sample_times;i++)
	{
		MPU6050_Read_RawData();
		Ax += MPU6050_data.Ax;
		Ay += MPU6050_data.Ay; 
		Az += MPU6050_data.Az;
		Gx += MPU6050_data.Gx;
		Gy += MPU6050_data.Gy;		
		Gz += MPU6050_data.Gz;
	}
	
	Ax_Offset = Ax / sample_times;
	Ay_Offset = Ay / sample_times;
	Az_Offset = Az / sample_times;
	Gx_Offset = Gx / sample_times;
	Gy_Offset = Gy / sample_times;
	Gz_Offset = Gz / sample_times;
	
	MPU6050_Offset_Done = 1;
	
	/**************************************
	AngleAx_Offset = ( atan(Ax / Az)*180 ) / pi;
  	AngleAy_Offset = ( atan(Ay / Az)*180 ) / pi;
	AngleGx_Offset = Gx / sample_times;
	AngleGy_Offset = Gy / sample_times;
	AngleGz_Offset = Gz / sample_times;
	**************************************/
	
}


/**************************实现函数********************************************
*函数原型:		void MPU6050_Set_Accel_Range(uint8_t range)
*功　　能:	    设置  MPU6050 加速度计的最大量程 
* AFS_SEL=0    ±2	g	16,384    LSB/g       
* AFS_SEL=1    ±4	g	8,192     LSB/g 
* AFS_SEL=2    ±8	g	4,096     LSB/g
* AFS_SEL=3    ±16	g	2,048     LSB/g 
*******************************************************************************/
void MPU6050_Set_Accel_Range(uint8_t range)
{
	IIC_Write_Bits(MPU6050_Address, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, range);
	switch(range)
	{
		case 0: AcceRatio = 16384.0; break;
		case 1: AcceRatio = 8192.0; break;
		case 2: AcceRatio = 4096.0; break;
		case 3: AcceRatio = 2048.0; break;
	}
}


/**************************实现函数********************************************
*函数原型:		void MPU6050_Set_Gyro_Range(uint8_t range)
*功　　能:	    设置  MPU6050 陀螺仪的最大量程
* FS_SEL=0    ±250    o/s	131     LSB/(o/s) 
* FS_SEL=1    ±500    o/s	65.5    LSB/(o/s) 
* FS_SEL=2    ±1000   o/s	32.8    LSB/(o/s)
* FS_SEL=3    ±2000   o/s	16.4    LSB/(o/s) 
*******************************************************************************/
void MPU6050_Set_Gyro_Range(uint8_t range)
{
	IIC_Write_Bits(MPU6050_Address, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, range);
	switch(range)
	{
		case 0: GyroRatio = 131.0; break;
		case 1: GyroRatio = 65.5; break;
		case 2: GyroRatio = 32.8; break;
		case 3: GyroRatio = 16.4; break;
	}
}



/**************************实现函数********************************************
*函数原型:		void MPU6050_Init(void)
*功　　能:	    初始化 MPU6050 以进入可用状态。
*******************************************************************************/
void MPU6050_Init(void)
{
	uint8_t Add[1];
	MPU6050_getDeviceID(Add);
	
	IIC_Write_Byte(MPU6050_Address, MPU6050_RA_PWR_MGMT_1, 0x00);	//awake
	IIC_Write_Byte(MPU6050_Address, MPU6050_RA_SMPLRT_DIV, 0x07);
	IIC_Write_Byte(MPU6050_Address, MPU6050_RA_CONFIG, 0x06);
	IIC_Write_Byte(MPU6050_Address, MPU6050_RA_GYRO_CONFIG, 0x18); //FS_SEL=0    ±250    o/s	131     LSB/(o/s) 
	GyroRatio = 131.0;
	
	IIC_Write_Byte(MPU6050_Address, MPU6050_RA_ACCEL_CONFIG, 0x00); //AFS_SEL=0    ±2	g	16,384    LSB/g 
	AcceRatio = 16384.0;
	
	delay_ms(100);
	
	
	MPU6050_Get_Offset();
	
	
}

/**************************实现函数********************************************
*函数原型:		void MPU6050_getDeviceID(uint8_t * Add)
*功　　能:	    读取  MPU6050 WHO_AM_I 标识	 将得到 0x68
*******************************************************************************/
void MPU6050_getDeviceID(uint8_t * Add) 
{
    IIC_Read_Bytes(MPU6050_Address, MPU6050_RA_WHO_AM_I, 1, Add);
}


//读取3个轴的数据 
//x,y,z读取到的数据 

void MPU6050_Read_RawData()
{
	uint8_t buf[14];
	//MPU6050_data_old.Ax = MPU6050_data.Ax; etc
	
	IIC_Read_Bytes(MPU6050_Address, MPU6050_RA_ACCEL_XOUT_H, 14, buf);
	
	if(MPU6050_Offset_Done == 0)
	{
		MPU6050_data.Ax = ( (float) (( ((int16_t)buf[0]) <<8) + buf[1]) ) / AcceRatio; 
		MPU6050_data.Ay = ( (float) (( ((int16_t)buf[2]) <<8) + buf[3]) ) / AcceRatio; 
		MPU6050_data.Az = ( (float) (( ((int16_t)buf[4]) <<8) + buf[5]) ) / AcceRatio; 
		MPU6050_data.Gx = ( (float) ( (( ((int16_t)buf[8]) <<8) + buf[9])) ) / GyroRatio ; 
		MPU6050_data.Gy = ( (float) ( (( ((int16_t)buf[10]) <<8) + buf[11])) ) / GyroRatio ; 
		MPU6050_data.Gz = ( (float) ( (( ((int16_t)buf[12]) <<8) + buf[13])) ) / GyroRatio ;
	}
		else
		{
			MPU6050_data.Ax = ( (float) (( ((int16_t)buf[0]) <<8) + buf[1]) ) / AcceRatio - Ax_Offset; 
			MPU6050_data.Ay = ( (float) (( ((int16_t)buf[2]) <<8) + buf[3]) ) / AcceRatio - Ay_Offset; 
			MPU6050_data.Az = ( (float) (( ((int16_t)buf[4]) <<8) + buf[5]) ) / AcceRatio - Az_Offset; 
			MPU6050_data.Tt = ( (float) (( ((int16_t)buf[6]) <<8) + buf[7]) ) / TemperatureRatio - TemperatureOffset;
			MPU6050_data.Gx = ( (float) ( (( ((int16_t)buf[8]) <<8) + buf[9])) ) / GyroRatio - Gx_Offset; 
			MPU6050_data.Gy = ( (float) ( (( ((int16_t)buf[10]) <<8) + buf[11])) ) / GyroRatio  - Gy_Offset; 
			MPU6050_data.Gz = ( (float) ( (( ((int16_t)buf[12]) <<8) + buf[13])) ) / GyroRatio  - Gz_Offset;			
		}
	
}


//读取ADXL的平均值 
//x,y,z读取8次后取平均值 

void MPU6050_RawData_Average()
{
	uint8_t i;  
	   
	for(i=0;i<8;i++)
	{
		MPU6050_Read_RawData();
		// delay_ms(10);
		MPU6050_data.Ax += MPU6050_data.Ax;
		MPU6050_data.Ay += MPU6050_data.Ay;
		MPU6050_data.Az += MPU6050_data.Az;	   
	}
	MPU6050_data.Ax = MPU6050_data.Ax / 8;
	MPU6050_data.Ay = MPU6050_data.Ay / 8;
	MPU6050_data.Az = MPU6050_data.Az / 8;
}