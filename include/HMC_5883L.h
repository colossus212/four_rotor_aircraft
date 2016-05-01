#include "r_cg_macrodriver.h"
#ifndef _HMC_5883L_
#define _HMC_5883L_



#define	HMC5883L_ADDRESS   0x3C	 

uint8_t HMC5883L_Init(void);

void Multiple_Read_HMC5883L();
void HMC5883L_Calibration();

float Get_HMC5883L_Hx();
float Get_HMC5883L_Hy();
float Get_HMC5883L_Hz();

#endif

