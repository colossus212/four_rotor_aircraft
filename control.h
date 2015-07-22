#ifndef __CONTROL_
#define __CONTROL_

#include "r_cg_macrodriver.h"

extern volatile int16_t MOTO1, MOTO2, MOTO3, MOTO4;

void Control_Standby();
void Control_Fly();

void Control_posture(float roll, float pitch, float yaw);
void Control_Heigh(float H);



#endif