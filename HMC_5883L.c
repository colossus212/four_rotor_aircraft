
/************************************************
* File Name	: HMC_5883L.c
* Version	: 2015.7.12 By DHP
* Device(s)	: R5F100LE
* Tool-Chain	: CA78K0R
* Description	: 磁力计模块
* API		: HMC5883L_Init(void)
		  void Multiple_Read_HMC5883L(void)

		  int16_t Get_HMC5883L_Hx();
		  int16_t Get_HMC5883L_Hy();
		  int16_t Get_HMC5883L_Hz();
		  
		  //extern int16_t Get_HMC5883L_Hx(), Y_HMC, Z_HMC;
		  
		  待解决问题： 还不清楚磁力计读出值的单位和使用方法
		 
************************************************/

#include "include.h"
#include  <math.h>
#define  FILL_NUM  10


int16_t X_HMC,Y_HMC,Z_HMC,x,y,z;

int16_t x_offest=0,y_offest=0;
double y_gain=1;

//int16_t X_BUFF[FILL_NUM],Y_BUFF[FILL_NUM],Z_BUFF[FILL_NUM];

int16_t Get_HMC5883L_Hx()
{
	return X_HMC;
}

int16_t Get_HMC5883L_Hy()
{
	return Y_HMC;
}

int16_t Get_HMC5883L_Hz()
{
	return Z_HMC;
}

uint8_t HMC5883L_Init(void)
{
	uint8_t date;
   	date = IIC_Write_Byte(HMC5883L_ADDRESS, 0x02, 0x00);
  	return date; 		 
}



void Multiple_Read_HMC5883L(void)
{      	
 	uint8_t i;
	uint8_t BUF[8];
	static uint8_t filter_cnt = 0;
	int32_t temp1 = 0, temp2 = 0, temp3 = 0;
	
	 
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
	    
	X_HMC = 1 *(x + x_offest);
	Y_HMC = (double)(y_gain * (y + y_offest));
	Z_HMC = (double)(1.073 * (z +30));
	
}