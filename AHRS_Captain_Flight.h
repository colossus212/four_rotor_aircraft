#include "r_cg_macrodriver.h"

#ifndef _AHRS_CAPTAIN_FLIGHT_
#define _AHRS_CAPTAIN_FLIGHT_

void AHRS_Captain_Flight_IMU(int16_t yaw, int16_t pitch, int16_t roll,
	int16_t alt, int16_t tempr, int16_t press, int16_t IMUpersec);


void AHRS_Captain_Flight_Motion(int16_t ax, int16_t ay, int16_t az,
	int16_t gx ,int16_t gy, int16_t gz, int16_t hx, int16_t hy, int16_t hz);

extern uint8_t Uart_Buf_IMU[18];
extern uint8_t Uart_Buf_Motion[24];

#endif