#include "r_cg_macrodriver.h"

#ifndef _TRACK_
#define _TRACK_

//void Data_Collection();

//void Edge_Detection();
void Track_Init();
float Track_Frame();  // return the Target of Yaw
float Track_Motion();

#endif