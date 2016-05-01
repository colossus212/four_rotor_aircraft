#include "r_cg_macrodriver.h"
#ifndef _HCSR_04_
#define _HCSR_04_

#define RtA 	57.324841        //  180/3.1415  	
#define AtR    	0.0174533		//  1/RtA   
#define isnan(x)    ((x) != (x))

void HCSR04_Init();
float Get_Height();
void Flash_Height_Prepare();
void Flash_Height();



#endif