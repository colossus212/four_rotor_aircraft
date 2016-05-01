#include "r_cg_macrodriver.h"

#ifndef _OV_7620_
#define _OV_7620_



#define H_RESOLUTION 7  //maximum:320,multiple of 4
#define V_RESOLUTION 120   //maximum:240,multiple of 2
#define H_STEP 1    //for change Horizontal resolution
#define V_STEP 1    //for change Vertical resolution
#define THRESHOLD 0x60
#define ARRAY_SIZE 3000


extern uint8_t dcim[V_RESOLUTION][H_RESOLUTION];


void OV7620_Init(void);
void picture_two();
char picture_process();
void data_cal();
uint8_t dis(uint8_t a,uint8_t b);
int dis_int(int a,int b);
uint8_t getio(void);

#endif