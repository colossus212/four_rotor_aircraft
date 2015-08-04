
/************************************************
* File Name	: GY_953.c
* Version	: 2015.8.3 By DHP
* Device(s)	: R5F100LE
* Tool-Chain	: CA78K0R
* Description	: AHRS, nine axil sensor 
* API		: 
		 // Tx Pin 59
		 // Rx Pin 60
		 void GY953_Init();
		 void GY953_Get_Range();
		 void GY953_Get_RPY();
		 void GY953_Get_Accl();
		 void GY953_Get_Gyro();
		 void GY953_Get_Mag();
		 void GY953_Receive_Process();
************************************************/

#include "GY_953.h"
#include "delay.h"
#include "r_cg_serial.h"
#include "KalmanFilter.h"

int8_t GY953_Receive[9];
int8_t Check;

float GY953_Roll, GY953_Pitch, GY953_Yaw;
float GY953_Acc_x, GY953_Acc_y, GY953_Acc_z;
float GY953_Gyro_x, GY953_Gyro_y, GY953_Gyro_z;
float GY953_Mag_x, GY953_Mag_y, GY953_Mag_z;
float GY953_Gyro_Range = 0, GY953_Accl_Range = 0, GY953_Mag_Range = 0;
	// Accel_Range +- 2 g , Gyro_Range +- 2000dps ;
static const float AcceRatio = 16384.0, GyroRatio = 16.4;

static const uint8_t Baud_Rate_115200[3] = {0xa5, 0xaf, 0x54};
static const uint8_t Acc_Gyro_Calibration[3] = {0xa5, 0x57, 0xfc};
static const uint8_t Mag_Calibration[3] = {0xa5, 0x58, 0xfd};

//static const uint8_t Acc_Init[3] = {0xa5, 0x15, 0xba};
//static const uint8_t Gyro_Init[3] = {0xa5, 0x25, 0xca};
//static const uint8_t Mag_Init[3] = {0xa5, 0x35, 0xda};
//static const uint8_t RPY_Init[3] = {0xa5, 0x45, 0xea};

static const uint8_t Get_Accu_Freq[3] = {0xa5, 0x75, 0x1a};
static const uint8_t Get_Range[3] = {0xa5, 0x85, 0x2a};	// Accl Gyro Mag
static const uint8_t Get_RPY[3] = {0xa5, 0x95, 0x3a};
static const uint8_t Get_Quat[3] = {0xa5, 0xb5, 0x5a};
static const uint8_t Get_Accl[3] = {0xa5, 0xc5, 0x6a};
static const uint8_t Get_Gyro[3] = {0xa5, 0xd5, 0x7a};
static const uint8_t Get_Mag[3] = {0xa5, 0xe5, 0x8a};


void GY953_Init()
{
	R_UART1_Send(Baud_Rate_115200, 3);
	delay_ms(2);
	R_UART1_Send(Acc_Gyro_Calibration, 3);
	delay_ms(10);
	R_UART1_Send(Mag_Calibration, 3);
	delay_ms(10);
	GY953_Get_Range();
//	R_UART1_Send(Acc_Init, 3);
//	delay_ms(2);
//	R_UART1_Send(Gyro_Init, 3);
//	delay_ms(2);
//	R_UART1_Send(Mag_Init, 3);
//	delay_ms(2);
//	R_UART1_Send(RPY_Init, 3);
//	delay_ms(2);
}

void GY953_Get_Range()
{
	R_UART1_Send(Get_Range, 3);
	delay_us(300);
}

void GY953_Get_RPY()
{
	R_UART1_Send(Get_RPY, 3);
	//delay_us(300);
}

void GY953_Get_Accl()
{
	R_UART1_Send(Get_Accl, 3);
	//delay_us(300);
}

void GY953_Get_Gyro()
{
	R_UART1_Send(Get_Gyro, 3);
	//delay_us(300);
}

void GY953_Get_Mag()
{
	R_UART1_Send(Get_Mag, 3);
	//delay_us(300);
}


void GY953_Receive_Process()
{
	Check = 0x5A + 0x5A + GY953_Receive[0] + GY953_Receive[1] + GY953_Receive[2] + GY953_Receive[3]
		+ GY953_Receive[4] + GY953_Receive[5] + GY953_Receive[6] + GY953_Receive[7];
	if(Check == GY953_Receive[8])
	{
		switch(GY953_Receive[0])
		{
			case 0x45:
			{
				GY953_Roll = (float)(GY953_Receive[2] * 256 + GY953_Receive[3]);
				GY953_Pitch = (float)(GY953_Receive[4] * 256 + GY953_Receive[5]);
				GY953_Yaw = (float)(GY953_Receive[6] * 256 + GY953_Receive[7]);
				GY953_Roll = GY953_Roll / 100;
				GY953_Pitch = GY953_Pitch / 100;
				GY953_Yaw = GY953_Yaw / 100;
			}break;
		
			case 0x15:
			{
				GY953_Acc_x = (float)(GY953_Receive[2] * 256 + GY953_Receive[3]);
				GY953_Acc_y = (float)(GY953_Receive[4] * 256 + GY953_Receive[5]);
				GY953_Acc_z = (float)(GY953_Receive[6] * 256 + GY953_Receive[7]);
				GY953_Acc_x /= AcceRatio;
				GY953_Acc_y /= AcceRatio;
				GY953_Acc_z /= AcceRatio;
			}break;
	
			case 0x25:
			{
				GY953_Gyro_x = (float)(GY953_Receive[2] * 256 + GY953_Receive[3]);
				GY953_Gyro_y = (float)(GY953_Receive[4] * 256 + GY953_Receive[5]);
				GY953_Gyro_z = (float)(GY953_Receive[6] * 256 + GY953_Receive[7]);
				GY953_Gyro_x /=  GyroRatio;
				GY953_Gyro_y /=  GyroRatio;
				GY953_Gyro_z /=  GyroRatio;
				GY953_Gyro_x = KalmanFilter(GY953_Gyro_x, & Kalman_Gx);
				GY953_Gyro_y = KalmanFilter(GY953_Gyro_y, & Kalman_Gy);
				GY953_Gyro_z = KalmanFilter(GY953_Gyro_z, & Kalman_Gz);
			}break;

			case 0x35:
			{
				GY953_Mag_x = (float)(GY953_Receive[2] * 256 + GY953_Receive[3]);
				GY953_Mag_y = (float)(GY953_Receive[4] * 256 + GY953_Receive[5]);
				GY953_Mag_z = (float)(GY953_Receive[6] * 256 + GY953_Receive[7]);
				//GY953_Mag_x =  ;
				//GY953_Mag_y =  ;
				//GY953_Mag_z =  ;
				//to do
			}break;
			
			case 0x85:
			{
				GY953_Accl_Range = (float)(GY953_Receive[2] * 256 + GY953_Receive[3]);
				GY953_Gyro_Range = (float)(GY953_Receive[4] * 256 + GY953_Receive[5]);
				GY953_Mag_Range = (float)(GY953_Receive[6] * 256 + GY953_Receive[7]);
			}break;
		}
	}
}

#if 0
// backup from r_cg_serial_user.c // r_uart1_interrupt_receive
//
//	if((rx_data == 0x5a) && (Byte_Count == 0))Byte_Count = 1;
//	else if((rx_data==0x5a) && (Byte_Count == 1))Byte_Count = 2;
//	else if(Byte_Count >= 2)
//	{
//		GY953_Receive[(Byte_Count - 2)] = rx_data;
//		Byte_Count++;
//		if(Byte_Count > 10)
//		{
//			GY953_Receive_Process();
//			Byte_Count = 0;
//		}
//	}
//
#endif