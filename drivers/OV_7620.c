#include "include.h"

uint8_t dcim[V_RESOLUTION][H_RESOLUTION];

extern volatile bit iica0_sendend_flag;
extern volatile bit iica0_receiveend__flag;

void OV7620_Init(void)
{
	uint8_t tx_data[2] = {0x11,0x03};	
	iica0_receiveend__flag = 0;
	R_IIC10_Master_Send(0x42,tx_data,2);
	while(iica0_receiveend__flag == 0);
}

void picture_two()
{
}
char picture_process()
{
}
void data_cal()
{
}
uint8_t dis(uint8_t a,uint8_t b)
{
}
int dis_int(int a,int b)
{
}


uint8_t getio(void)
{
	uint8_t rec_data=((P7<<2)&0x54)+((P6<<1)&0x0A);
	if(P7.6) rec_data+=0x80;
	if(P3.0) rec_data+=0x20;
	if(P0.5) rec_data+=0x01;	
	return rec_data;
}