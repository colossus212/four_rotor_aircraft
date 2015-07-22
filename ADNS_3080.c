#include "ADNS_3080.h"
#include "SPI.h"

/******************************************************************************
* function :		void ADNS3080_Motion_Read(uint8_t * data)
* Description : This mode is activated by reading the Motion_Burst register.  
* The ADNS-3080 will respond with the contents of the 
* Motion, Delta_X, Delta_Y, SQUAL, Shutter_Upper, Shutter_Lower and Maximum_Pixel
* registers in that order. 
*******************************************************************************/ 

void ADNS3080_Motion_Read(uint8_t * data)
{
	SPI_Burst_Mode_Read(ADNS3080_Motion_Burst, 7, data);
}

/******************************************************************************
* function :		void ADNS3080_SROM_Download()
* Description : This function is used to load the 
* Agilent-supplied firmware file contents into the ADNS-3080.  
* The firmware file is an ASCII text file with each 2-character byte 
* (hexadecimal representation) on a single line. 
*******************************************************************************/ 

void ADNS3080_SROM_Download()
{
	//Perform hardware reset by toggling the RESET pin
	//to do

	// Write 0x44 to register 0x20
	SPI_Write_Byte(0x20, 0x44);
	//Write 0x07 to register 0x23
	SPI_Write_Byte(0x23, 0x07);
	//Write 0x88 to register 0x24
	SPI_Write_Byte(0x24, 0x88);
	//Wait at least 1 frame period
	delay_ms(frame_period);          // frame_period is to be defined !!!

	//Write 0x18 to register 0x14 (SROM_Enable register)
	SPI_Write_Byte(ADNS3080_SROM_Enable, 0x18);
	//Begin burst mode write of data file to register 0x60 (SROM_Load register)
	SPI_Burst_Mode_Write(ADNS3080_SROM_Load, 1986, SROM);

	// Agilent recommends reading the SROM_ID register to verify that the download was successful. 
		//to do

	// In addition, a self-test may be executed, 
	// which performs a CRC on the SROM contents and reports the results in a register.
	// The test is initiated by writing a particular value to the SROM_Enable register; 
		//to do

	// the result is placed in the Data_Out register.  
		//to do

	// See those register descriptions for more details.
}


/******************************************************************************
* function :		void ADNS3080_Frame_Capture(uint8_t * data)
* Description : This is a fast way to download a full array of pixel values from a single frame.  
* This mode disables navigation and overwrites any downloaded firmware. 
* A hardware reset is required to restore navigation, 
* and the firmware must be reloaded afterwards if required.
*******************************************************************************/ 

void ADNS3080_Frame_Capture(uint8_t * data)
{
	NOP();
}
