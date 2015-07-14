#include "r_cg_macrodriver.h"
#ifndef _HMC_5883L_
#define _HMC_5883L_



#define	HMC5883L_ADDRESS   0x3C	 

uint8_t HMC5883L_Init(void);

void Multiple_Read_HMC5883L();

int16_t Get_HMC5883L_Hx();
int16_t Get_HMC5883L_Hy();
int16_t Get_HMC5883L_Hz();

extern int16_t X_HMC, Y_HMC, Z_HMC;

#endif

