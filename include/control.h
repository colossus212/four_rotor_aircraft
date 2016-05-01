#ifndef __CONTROL_
#define __CONTROL_

#include "r_cg_macrodriver.h"

extern volatile float MOTO1, MOTO2, MOTO3, MOTO4;

void Control_Fly_Flag_Off();
void Control_Fly_Flag_On();
uint8_t Get_Fly_Flag();

void Control_Posture(float roll, float pitch, float yaw);
void Control_Heigh(float H);
void Control_Track_Frame();
void Control_Track_Motion();

void Control_Test();
void Control_Fly();
void Control_Land();
void Control_Free_Land();


#endif