#include "r_cg_macrodriver.h"

#ifndef _ADNS_3080_
#define _ADNS_3080_



void ADNS3080_Init();
void ADNS3080_Motion_Read();
void ADNS3080_SROM_Download();
void ADNS3080_Frame_Capture();
void ADNS3080_Frame_Data_Correct();
uint8_t Get_Frame_Data(uint8_t count);
uint8_t Get_Frame_Data_Matrix(uint8_t i, uint8_t j);


#define	ADNS3080_RA_Product_ID	0x00
#define	ADNS3080_RA_Revision_ID	0x01
#define	ADNS3080_RA_Motion	0x02
#define	ADNS3080_RA_Delta_X	0x03
#define	ADNS3080_RA_Delta_Y	0x04
#define	ADNS3080_RA_SQUAL	0x05
#define	ADNS3080_RA_Pixel_Sum	0x06
#define	ADNS3080_RA_Maximum_Pixel	0x07

#define	ADNS3080_RA_Configuration_bits	0x0a
#define	ADNS3080_RA_Extended_Config	0x0b
#define	ADNS3080_RA_Data_Out_Lower	0x0c
#define	ADNS3080_RA_Data_Out_Upper	0x0d
#define	ADNS3080_RA_Shutter_Lower	0x0e
#define	ADNS3080_RA_Shutter_Upper	0x0f
#define	ADNS3080_RA_Frame_Period_Lower	0x10
#define	ADNS3080_RA_Frame_Period_Upper	0x11
#define	ADNS3080_RA_Motion_Clear	0x12
#define	ADNS3080_RA_Frame_Capture	0x13
#define	ADNS3080_RA_SROM_Enable	0x14

#define	ADNS3080_RA_Frame_Period_Max_Bound_Lower	0x19
#define	ADNS3080_RA_Frame_Period_Max_Bound_Upper	0x1a
#define	ADNS3080_RA_Frame_Period_Min_Bound_Lower	0x1b
#define	ADNS3080_RA_Frame_Period_Min_Bound_Upper	0x1c
#define	ADNS3080_RA_Shutter_Max_Bound_Lower	0x1d
#define	ADNS3080_RA_Shutter_Max_Bound_Upper	0x1e
#define	ADNS3080_RA_SROM_ID	0x1f

#define	ADNS3080_RA_Observation	0x3d

#define	ADNS3080_RA_Inverse_Product_ID	0x3f
#define	ADNS3080_RA_Pixel_Burst	0x40
#define	ADNS3080_RA_Motion_Burst	0x50
#define	ADNS3080_RA_SROM_Load	0x60

#endif