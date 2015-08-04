#include "AHRS_Captain_Flight.h"
#include "r_cg_serial.h"
#include "delay.h"

uint8_t Uart_Buf_IMU[18];
uint8_t Uart_Buf_Motion[24];

/**************************实现函数********************************************
*函数原型:		void AHRS_Captain_Flight_IMU(int16_t yaw,int16_t pitch,int16_t roll
				,int16_t alt,int16_t tempr,int16_t press)
*功　　能:		向上位机发送经过解算后的姿态数据
输入参数：
		int16_t yaw 经过解算后的航向角度。单位为 0.1 度 0 -> 3600  对应 0 -> 360.0度
		int16_t pitch 解算得到的俯仰角度，单位 0.1 度。-900 - 900 对应 -90.0 -> 90.0 度
		int16_t roll  解算后得到的横滚角度，单位 0.1 度。 -1800 -> 1800 对应 -180.0  ->  180.0度
		int16_t alt   气压高度。 单位 1 cm。  范围一个整型变量
		// int16_t tempr 温度 。 单位0.1摄氏度   范围：直到你的电路板不能正常工作
		// int16_t press 气压压力。单位10Pa  一个大气压强在101300pa 这个已经超过一个整型的范围。需要除以10再发给上位机
		// int16_t IMUpersec  姿态解算速率。运算IMUpersec每秒。
输出参数：没有	
*******************************************************************************/
void AHRS_Captain_Flight_IMU(int16_t yaw, int16_t pitch, int16_t roll,
	int16_t alt, int16_t tempr, int16_t press, int16_t IMUpersec)
{
	//uint8_t i;
//	yaw *= 10;
//	pitch *= 10;
//	roll *= 10;
//	alt *= 10;
	Uart_Buf_IMU[0] = 0xA5;
	Uart_Buf_IMU[1] = 0x5A;
	Uart_Buf_IMU[2] = 14+2;
	Uart_Buf_IMU[3] = 0xA1;
	//R_UART0_Send(Uart_Buf_IMU, 4);
	
	Uart_Buf_IMU[16] = 0xaF+2;

	if(yaw<0)yaw = 32768 - yaw;
	Uart_Buf_IMU[4] = yaw>>8;
	//R_UART0_Send(Uart_Buf_IMU + 4, 1);
	Uart_Buf_IMU[16] += Uart_Buf_IMU[4];
	Uart_Buf_IMU[5] = yaw;
	//R_UART0_Send(Uart_Buf_IMU + 5, 1);
	Uart_Buf_IMU[16] += Uart_Buf_IMU[5];
	
	if(pitch<0) pitch = 32768 - pitch;
	Uart_Buf_IMU[6] = pitch >> 8;
	//R_UART0_Send(Uart_Buf_IMU + 6, 1);
	Uart_Buf_IMU[16] += Uart_Buf_IMU[6];
	Uart_Buf_IMU[7] = pitch;
	//R_UART0_Send(Uart_Buf_IMU + 7, 1);
	Uart_Buf_IMU[16] += Uart_Buf_IMU[7];

	if(roll<0)roll=32768-roll;
	Uart_Buf_IMU[8]=roll>>8;
	//R_UART0_Send(Uart_Buf_IMU + 8, 1);
	Uart_Buf_IMU[16]+=Uart_Buf_IMU[8];
	Uart_Buf_IMU[9]=roll;
	//R_UART0_Send(Uart_Buf_IMU + 9, 1);
	Uart_Buf_IMU[16]+=Uart_Buf_IMU[9];

	if(alt<0)alt=32768-alt;
	Uart_Buf_IMU[10]=alt>>8;
	//R_UART0_Send(Uart_Buf_IMU + 10, 1);
	Uart_Buf_IMU[16]+=Uart_Buf_IMU[10];
	Uart_Buf_IMU[11]=alt;
	//R_UART0_Send(Uart_Buf_IMU + 11, 1);
	Uart_Buf_IMU[16]+=Uart_Buf_IMU[11];

	if(tempr<0)tempr=32768-tempr;
	Uart_Buf_IMU[12]=tempr>>8;
	//R_UART0_Send(Uart_Buf_IMU + 12, 1);
	Uart_Buf_IMU[16]+=Uart_Buf_IMU[12];
	Uart_Buf_IMU[13]=tempr;
	//R_UART0_Send(Uart_Buf_IMU + 13, 1);
	Uart_Buf_IMU[16]+=Uart_Buf_IMU[13];

	if(press<0)press=32768-press;
	Uart_Buf_IMU[14]=press>>8;
	//R_UART0_Send(Uart_Buf_IMU + 14, 1);
	Uart_Buf_IMU[16]+=Uart_Buf_IMU[14];
	Uart_Buf_IMU[15]=press;
	//R_UART0_Send(Uart_Buf_IMU + 15, 1);
	Uart_Buf_IMU[16]+=Uart_Buf_IMU[15];

	Uart_Buf_IMU[16] = Uart_Buf_IMU[16]%256;
	Uart_Buf_IMU[17] = 0xaa;
	//R_UART0_Send(Uart_Buf_IMU + 16, 1);
	//R_UART0_Send(Uart_Buf_IMU + 17, 1);
	//R_UART0_Send(Uart_Buf_IMU, 18);
	//for(i = 0; i < 18; i++)
	//{
	//	TXD0 = * (Uart_Buf_IMU + i);
	//	delay_us(20);
	//}

}


/**************************实现函数********************************************
*函数原型:		void AHRS_Captain_Flight_Motion(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz,
					int16_t hx,int16_t hy,int16_t hz)
*功　　能:		向上位机发送当前传感器的输出值
输入参数：
	int16_t ax  加速度 X轴ADC输出 范围 ：一个有符号整型
	int16_t ay  加速度 Y轴ADC输出 范围 ：一个有符号整型
	int16_t az  加速度 Z轴ADC输出 范围 ：一个有符号整型
	int16_t gx  陀螺仪 X轴ADC输出 范围 ：一个有符号整型
	int16_t gy  陀螺仪 Y轴ADC输出 范围 ：一个有符号整型
	int16_t gz  陀螺仪 Z轴ADC输出 范围 ：一个有符号整型
	int16_t hx  磁罗盘 X轴ADC输出 范围 ：一个有符号整型
	int16_t hy  磁罗盘 Y轴ADC输出 范围 ：一个有符号整型
	int16_t hz  磁罗盘 Z轴ADC输出 范围 ：一个有符号整型
	
输出参数：没有	
*******************************************************************************/
void AHRS_Captain_Flight_Motion(int16_t ax, int16_t ay, int16_t az,
	int16_t gx ,int16_t gy, int16_t gz, int16_t hx, int16_t hy, int16_t hz)
{
	
	Uart_Buf_Motion[0] = 0xA5;
	Uart_Buf_Motion[1] = 0x5A;
	Uart_Buf_Motion[2] = 14+8;
	Uart_Buf_Motion[3] = 0xA2;
	//R_UART0_Send(Uart_Buf_Motion, 4);

	Uart_Buf_Motion[22] = 0xaF+9;
	
	if(ax <0) ax=32768-ax;
	Uart_Buf_Motion[4]=ax>>8;
	//R_UART0_Send(Uart_Buf_Motion, 1);
	Uart_Buf_Motion[22]+=Uart_Buf_Motion[4];
	Uart_Buf_Motion[5]=ax;
	//R_UART0_Send(Uart_Buf_Motion, 1);
	Uart_Buf_Motion[22]+=Uart_Buf_Motion[5];

	if(ay<0)ay=32768-ay;
	Uart_Buf_Motion[6]=ay>>8;
	//R_UART0_Send(Uart_Buf_Motion, 1);
	Uart_Buf_Motion[22]+=Uart_Buf_Motion[6];
	Uart_Buf_Motion[7]=ay;
	//R_UART0_Send(Uart_Buf_Motion, 1);
	Uart_Buf_Motion[22]+=Uart_Buf_Motion[7];

	if(az<0)az=32768-az;
	Uart_Buf_Motion[8]=az>>8;
	//R_UART0_Send(Uart_Buf_Motion, 1);
	Uart_Buf_Motion[22]+=Uart_Buf_Motion[8];
	Uart_Buf_Motion[9]=az;
	//R_UART0_Send(Uart_Buf_Motion, 1);
	Uart_Buf_Motion[22]+=Uart_Buf_Motion[9];

	if(gx<0)gx=32768-gx;
	Uart_Buf_Motion[10]=gx>>8;
	//R_UART0_Send(Uart_Buf_Motion, 1);
	Uart_Buf_Motion[22]+=Uart_Buf_Motion[10];
	Uart_Buf_Motion[11]=gx;
	//R_UART0_Send(Uart_Buf_Motion, 1);
	Uart_Buf_Motion[22]+=Uart_Buf_Motion[11];

	if(gy<0)gy=32768-gy;
	Uart_Buf_Motion[12]=gy>>8;
	//R_UART0_Send(Uart_Buf_Motion, 1);
	Uart_Buf_Motion[22]+=Uart_Buf_Motion[12];
	Uart_Buf_Motion[13]=gy;
	//R_UART0_Send(Uart_Buf_Motion, 1);
	Uart_Buf_Motion[22]+=Uart_Buf_Motion[13];

	if(gz<0)gz=32768-gz;
	Uart_Buf_Motion[14]=gz>>8;
	//R_UART0_Send(Uart_Buf_Motion, 1);
	Uart_Buf_Motion[22]+=Uart_Buf_Motion[14];
	Uart_Buf_Motion[15]=gz;
	//R_UART0_Send(Uart_Buf_Motion, 1);
	Uart_Buf_Motion[22]+=Uart_Buf_Motion[15];

	if(hx<0)hx=32768-hx;
	Uart_Buf_Motion[16]=hx>>8;
	//R_UART0_Send(Uart_Buf_Motion, 1);
	Uart_Buf_Motion[22]+=Uart_Buf_Motion[16];
	Uart_Buf_Motion[17]=hx;
	//R_UART0_Send(Uart_Buf_Motion, 1);
	Uart_Buf_Motion[22]+=Uart_Buf_Motion[17];

	if(hy<0)hy=32768-hy;
	Uart_Buf_Motion[18]=hy>>8;
	//R_UART0_Send(Uart_Buf_Motion, 1);
	Uart_Buf_Motion[22]+=Uart_Buf_Motion[18];
	Uart_Buf_Motion[19]=hy;
	//R_UART0_Send(Uart_Buf_Motion, 1);
	Uart_Buf_Motion[22]+=Uart_Buf_Motion[19];

	if(hz<0)hz=32768-hz;
	Uart_Buf_Motion[20]=hz>>8;
	//R_UART0_Send(Uart_Buf_Motion, 1);
	Uart_Buf_Motion[22]+=Uart_Buf_Motion[20];
	Uart_Buf_Motion[21]=hz;
	//R_UART0_Send(Uart_Buf_Motion, 1);
	Uart_Buf_Motion[22]+=Uart_Buf_Motion[21];

	
	Uart_Buf_Motion[22] = Uart_Buf_Motion[22]%256;
	Uart_Buf_Motion[23] = 0xaa;
	//R_UART0_Send(Uart_Buf_Motion + 2, 1);
	//R_UART0_Send(Uart_Buf_Motion + 3, 1);
}