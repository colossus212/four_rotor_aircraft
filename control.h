#ifndef __CONTROL_
#define __CONTROL_

#include "r_cg_macrodriver.h"

extern volatile float MOTO1, MOTO2, MOTO3, MOTO4;

void Control_Fly_Flag_Off();
void Control_Fly_Flag_On();

void Control_Posture(float roll, float pitch, float yaw);
void Control_Heigh(float H);
void Control_Track();


#endif