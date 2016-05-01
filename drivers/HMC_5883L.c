
/************************************************
* File Name	: HMC_5883L.c
* Version	: 2015.7.12 By DHP
* Device(s)	: R5F100LE
* Tool-Chain	: CA78K0R
* Description	: Magnet
* API		: HMC5883L_Init(void)
		  void Multiple_Read_HMC5883L(void)

		  float Get_HMC5883L_Hx();
		  float Get_HMC5883L_Hy();
		  float Get_HMC5883L_Hz();
		  
		  extern int16_t X_HMC, Y_HMC, Z_HMC;
		 
************************************************/

#include "include.h"
#include  <math.h>
#define  FILL_NUM  10


float X_HMC,Y_HMC,Z_HMC;

//float x_offset = 0, y_offset = 0, z_offset = 0, x_gain = 1, y_gain = 1, z_gain = 1;
//float x_offset = 31.8836, y_offset = - 147.1236, z_offset = 329.2807, x_gain = 386.0970, y_gain = 432.0223, z_gain = 362.6828;

const float x_offset = 189.8817, y_offset = - 96.9144, z_offset = 77.0083, x_gain = 368.2151, y_gain = 395.9817, z_gain = 363.5229;

//int16_t X_BUFF[FILL_NUM],Y_BUFF[FILL_NUM],Z_BUFF[FILL_NUM];

float Get_HMC5883L_Hx()
{
	float Hx;
	Hx = X_HMC;
	Hx = KalmanFilter(Hx, & Kalman_Hx);
	return Hx;
}

float Get_HMC5883L_Hy()
{
	float Hy;
	Hy = Y_HMC;
	Hy = KalmanFilter(Hy, & Kalman_Hy);
	return Hy;
}

float Get_HMC5883L_Hz()
{
	float Hz;
	Hz = Z_HMC;
	Hz = KalmanFilter(Hz, & Kalman_Hz);
	return Hz;
}

uint8_t HMC5883L_Init(void)
{
	uint8_t date;
   	date = IIC_Write_Byte(HMC5883L_ADDRESS, 0x02, 0x00);
  	return date; 		 
}



void Multiple_Read_HMC5883L(void)
{      	
	float x,y,z;
 	//uint8_t i;
	uint8_t BUF[8];
	//static uint8_t filter_cnt = 0;
	//int32_t temp1 = 0, temp2 = 0, temp3 = 0;
	
	 
	IIC_Read_Bytes(HMC5883L_ADDRESS, 0x03, 6, BUF);
    
	x = BUF[0] << 8 | BUF[1]; //Combine MSB and LSB of X Data output register;
	y = BUF[4] << 8 | BUF[5]; //Combine MSB and LSB of Y Data output register;
	z = BUF[2] << 8 | BUF[3]; //Combine MSB and LSB of Z Data output register;
	  
	//for(i=0;i<FILL_NUM;i++)  //10深度的滑动滤波
	//{
	  	//temp1 += X_BUFF[i];
		//   temp2 += Y_BUFF[i];
		//   temp3 += Z_BUFF[i];
	// }
	//x = temp1 / FILL_NUM;
	//y = temp2 / FILL_NUM;
	//z = temp3 / FILL_NUM;
		
	//filter_cnt++;
	//if(filter_cnt==FILL_NUM)	filter_cnt=0;

//	HMC5883L_Calibration();	    
//	X_HMC = x_gain * x + x_offset;
//	Y_HMC = y_gain * y + y_offset;
//	Z_HMC = 1.073 * (z +30);

	X_HMC = (x - x_offset) / x_gain * 300;
	Y_HMC = (y - y_offset) / y_gain * 300;
	Z_HMC = (z - y_offset) / z_gain * 300;
	
}


//float x_max = 258, x_min = - 236, y_max = 189, y_min = -334;
//void HMC5883L_Calibration()
//{
//	float x_max = 0, x_min = 0, y_max = 0, y_min = 0;
//	if(x > x_max) x_max = x;
//	if(x < x_min) x_min = x;
//	if(y > y_max) y_max = y;
//	if(y < y_min) y_min = y;
//	y_gain = (x_max - x_min) / (y_max - y_min);
//	x_offset = x_gain * (0.5 * (x_max - x_min) - x_max);
//	y_offset = y_gain * (0.5 * (y_max - y_min) - y_max);
//}
