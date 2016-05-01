#include "r_cg_macrodriver.h"
#ifndef _GY_953_
#define _GY_953_

extern int8_t GY953_Receive[9];

void GY953_Init();

void GY953_Get_Range();
void GY953_Get_RPY();
void GY953_Get_Accl();
void GY953_Get_Gyro();
void GY953_Get_Mag();

void GY953_Receive_Process();

#endif