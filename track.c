/*************************************************************************
* File Name	: track.c
* Version	: 2015.7.28 By DHP
* Device(s)	: R5F100LE
* Tool-Chain	: CA78K0R
* Description	: tracking
* API		: 
		  void Track_Init();
		  float Track();  // return the Target of Yaw
*************************************************************************/


#include "track.h"
#include "ADNS_3080.h"
#include <math.h>
#include "IMU.h"

const static float pi = 3.1415926;
uint8_t Frame_Gradient_Matrix[30][30];

//uint8_t Frame_Data_Matrix[30][30];
//void Data_Collection()
//{
//	uint8_t i, j;
//	ADNS3080_Frame_Capture();
//		// Turn Frame_Data[900] to Frame_Data_Matrix[30][30]
//	for(i = 0; i < 30; i++)
//		for(j = 0; j < 30; j++)
//			Frame_Data_Matrix[i][j] = Get_Frame_Data(i * 30 + j);
//}

// Krisch_Operator to do

/******************************************************************************
* function :		void Dirivative_Operator()
* Description : 
*******************************************************************************/ 
void Dirivative_Operator()
{
	const int8_t Dirivative_Operator_x[3][3] = {0, 0, 0,
						-1, 0, 1,
						0, 0, 0};
	const int8_t Dirivative_Operator_y[3][3] = {0, 1, 0,
						0, 0, 0,
						0, -1, 0};
	uint8_t i, j, k, l;
	int16_t Gx = 0, Gy = 0; 
	for(i = 1; i < 29; i++)
		for(j = 1; j < 29; j++)
			{
				Gx = 0; Gy = 0;
				for(k = 0; k < 3; k++)
					for(l = 0; l < 3; l++)
						{
							Gx += Get_Frame_Data_Matrix(i + k - 1, j + l -1) * Dirivative_Operator_x[k][l];
							Gy += Get_Frame_Data_Matrix(i + k - 1, j + l -1) * Dirivative_Operator_y[k][l];
						}
				Frame_Gradient_Matrix[i][j] = sqrt(Gx * Gx + Gy * Gy) / 2;
				// to simplify operation, it also can be
				// Frame_Gradient_Matrix[i][j] = abs(Gx) + abs(Gy) / 2;
			}
			// ignore the direction	
}

/******************************************************************************
* function :		void Robert_Operator()
* Description : 
*******************************************************************************/ 
void Robert_Operator()
{
	const int8_t Robert_Operator_x[2][2] = {1, 0,
						0, -1};
	const int8_t Robert_Operator_y[2][2] = {-1, 0,
						0, 1};
	uint8_t i, j, k, l;
	int16_t Gx = 0, Gy = 0; 
	for(i = 0; i < 29; i++)
		for(j = 0; j < 29; j++)
			{
				Gx = 0; Gy = 0;
				for(k = 0; k < 2; k++)
					for(l = 0; l < 2; l++)
						{
							Gx += Get_Frame_Data_Matrix(i + k, j + l) * Robert_Operator_x[k][l];
							Gy += Get_Frame_Data_Matrix(i + k, j + l) * Robert_Operator_y[k][l];
						}
				Frame_Gradient_Matrix[i][j] = sqrt(Gx * Gx + Gy * Gy);
				// to simplify operation, it also can be
				// Frame_Gradient_Matrix[i][j] = abs(Gx) + abs(Gy);
			}
			// ignore the direction
}

/******************************************************************************
* function :		void Sobel_Operator()
* Description : 
*******************************************************************************/ 
void Sobel_Operator()
{
	const int8_t Sobel_Operator_x[3][3] = {-1, 0, 1,
						-2, 0, 2,
						-1, 0, 1};
	const int8_t Sobel_Operator_y[3][3] = {-1, -2, -1,
						0, 0, 0,
						1, 2, 1};
	uint8_t i, j, k, l;
	int16_t Gx = 0, Gy = 0; 
	for(i = 1; i < 29; i++)
		for(j = 1; j < 29; j++)
			{
				Gx = 0; Gy = 0;
				for(k = 0; k < 3; k++)
					for(l = 0; l < 3; l++)
						{
							Gx += Get_Frame_Data_Matrix(i + k - 1, j + l -1) * Sobel_Operator_x[k][l];
							Gy += Get_Frame_Data_Matrix(i + k - 1, j + l -1) * Sobel_Operator_y[k][l];
						}
						// consider the Frame_Gradient_Matrix is also 8 bit
						// the max of abs(Gx) is 4 * 255, Gy as well
				Frame_Gradient_Matrix[i][j] = sqrt(Gx * Gx + Gy * Gy) / 8;
				// to simplify operation, it also can be
				// Frame_Gradient_Matrix[i][j] = abs(Gx) + abs(Gy) / 8;
			}
			// ignore the direction
}

/******************************************************************************
* function :		void Prewitt_Operator()
* Description : 
*******************************************************************************/ 
void Prewitt_Operator()
{
	const int8_t Prewitt_Operator_x[3][3] = {-1, 0, 1,
						-1, 0, 1,
						-1, 0, 1};
	const int8_t Prewitt_Operator_y[3][3] = {-1, -1, -1,
						0, 0, 0,
						1, 1, 1};
	uint8_t i, j, k, l;
	int16_t Gx = 0, Gy = 0; 
	for(i = 1; i < 29; i++)
		for(j = 1; j < 29; j++)
			{
				Gx = 0; Gy = 0;
				for(k = 0; k < 3; k++)
					for(l = 0; l < 3; l++)
						{
							Gx += Get_Frame_Data_Matrix(i + k - 1, j + l -1) * Prewitt_Operator_x[k][l];
							Gy += Get_Frame_Data_Matrix(i + k - 1, j + l -1) * Prewitt_Operator_y[k][l];
						}
						// consider the Frame_Gradient_Matrix is also 8 bit
						// the max of abs(Gx) is 3 * 255, Gy as well
				Frame_Gradient_Matrix[i][j] = sqrt(Gx * Gx + Gy * Gy) / 6;
				// to simplify operation, it also can be
				// Frame_Gradient_Matrix[i][j] = abs(Gx) + abs(Gy) / 6;
			}
			// ignore the direction
}

/******************************************************************************
* function :		void Laplacian_Operator()
* Description : 
*******************************************************************************/ 
void Laplacian_Operator()
{
	const int8_t Laplacian_Operator_ONE[3][3] = {0, -1, 0,
							-1, 4, -1,
							0, -1, 0};
	const int8_t Laplacian_Operator_TWO[3][3] = {-1, -1, -1,
							-1, 8, -1,
							-1, -1, -1};
	uint8_t i, j, k, l;
	int16_t ONE = 0, TWO = 0; 
	for(i = 1; i < 29; i++)
		for(j = 1; j < 29; j++)
			{
				ONE = 0; //TWO = 0;
				for(k = 0; k < 3; k++)
					for(l = 0; l < 3; l++)
						{
							ONE += Get_Frame_Data_Matrix(i + k - 1, j + l -1) * Laplacian_Operator_ONE[k][l];
							//TWO += Get_Frame_Data_Matrix(i + k - 1, j + l -1) * Laplacian_Operator_TWO[k][l];
						}
						// consider the Frame_Gradient_Matrix is also 8 bit
						// the max of abs(ONE) is 4 * 255, TWO is the twice
				Frame_Gradient_Matrix[i][j] = abs(ONE) / 4;
				//Frame_Gradient_Matrix[i][j] = abs(TWO) / 8;
			}
}

/******************************************************************************
* function :		void Canny_Operator()
* Description : 
*******************************************************************************/
void Canny_Operator()
{
	const uint8_t Gauss_filter[5][5] = {2, 4, 5, 4, 2,
					4, 9, 12, 9, 4,
					5, 12, 15, 12, 5,
					4, 9, 12, 9, 4,
					2, 4, 5, 4, 2};
	const int8_t Dirivative_Operator_x[3][3] = {0, 0, 0,
						-1, 0, 1,
						0, 0, 0};
	const int8_t Dirivative_Operator_y[3][3] = {0, 1, 0,
						0, 0, 0,
						0, -1, 0};
	uint8_t i, j, k, l; 
	int16_t Gx = 0, Gy = 0; 
	uint16_t SUM;
	for(i = 2; i < 28; i++)
		for(j = 2; j < 28; j++)
			{
				SUM = 0;
				for(k = 0; k < 5; k++)
					for(l = 0; l < 5; l++)
						{
							SUM += Get_Frame_Data_Matrix(i + k - 2, j + l -2) * Gauss_filter[k][l];
						}
						// consider the Frame_Gradient_Matrix is also 8 bit
						// the max of abs(SUM) is 159 * 255
				Frame_Gradient_Matrix[i][j] = SUM / 159;
			}


	for(i = 1; i < 29; i++)
		for(j = 1; j < 29; j++)
			{
				Gx = 0; Gy = 0;
				for(k = 0; k < 3; k++)
					for(l = 0; l < 3; l++)
						{
							Gx += Frame_Gradient_Matrix[i + k - 1][j + l -1] * Dirivative_Operator_x[k][l];
							Gy += Frame_Gradient_Matrix[i + k - 1][j + l -1] * Dirivative_Operator_y[k][l];
						}
						// consider the Frame_Gradient_Matrix is also 8 bit
				Frame_Gradient_Matrix[i][j] = sqrt(Gx * Gx + Gy * Gy) / 2;
				// to simplify operation, it also can be
				// Frame_Gradient_Matrix[i][j] = abs(Gx) + abs(Gy) / 2;
			}
}


/******************************************************************************
* function :		void Edge_Detection()
* Description : 
*******************************************************************************/
void Edge_Detection()
{
	ADNS3080_Frame_Capture();
	Sobel_Operator();
}

///******************************************************************************
//* function :		void Line_Detection_Hough()
//* Description : 
//*******************************************************************************/
//const static float pi_Value = 3.1415926 / 30;
//void Line_Detection_Hough()
//{
//	uint8_t i, j;
//	float Theta, Radius;
//	uint8_t Hough[30][30];
//	uint8_t Radius_Value = 0, Theta_Value = 0;
//	ADNS3080_Frame_Capture();
//	for(i = 0; i < 30; i++)
//		for(j = 0; j < 30; j++)
//		{
//			Theta = 0;
//			for(Theta_Value = 0; Theta_Value < 30; Theta_Value ++)
//			{
//				Radius = ((i * COS(Theta) + j * SIN(Theta)) / 1.41421356 + 29) / 2;
//				if(Radius > 29) Radius = 29;
//				if(Radius < 0) Radius = 0;
//				Radius_Value = (uint8_t) Radius;				
//				Hough[Radius_Value][Theta_Value] = Get_Frame_Data_Matrix(i, j);				
//				Theta += pi_Value;
//			}
//		}
//	// From Hough space to Cartesian coordinates
//	//to do	
//}

/******************************************************************************
* function :		void Frame_Binaryzation()
* Description : 
*******************************************************************************/
//uint8_t Frame_Binaryzation_Matrix[30][30];
void Frame_Binaryzation()
{
	uint8_t i, j;
 	uint8_t Max = 0;
 	uint8_t Min = 255;
	uint16_t Sum_H = 0;
	uint16_t Sum_L = 0;
	uint8_t Count_H = 0;
	uint8_t Count_L = 0;
	float Threshold, Threshold_Line;
	
 	for(i = 0; i < 30; i++)
		for(j = 0; j < 30; j++)
         		{
				if(Get_Frame_Data_Matrix(i, j) > Max) Max = Get_Frame_Data_Matrix(i, j);
         			if(Get_Frame_Data_Matrix(i, j) < Min) Min = Get_Frame_Data_Matrix(i, j);
			}
	Threshold = (Max + Min) / 3;
 	Threshold_Line = Threshold;
 
 	for(i = 0; i < 30; i++)
	{
		for(j = 0; j < 30; j++)
		{
			if(Get_Frame_Data_Matrix(i, j) > Threshold_Line)
			{
				Sum_H += Get_Frame_Data_Matrix(i, j);
				Count_H++;
				//Frame_Binaryzation_Matrix(i, j) = 255;
			}
			if(Get_Frame_Data_Matrix(i, j) <= Threshold_Line)
			{
				Sum_L += Get_Frame_Data_Matrix(i, j);
				Count_L++;
				//Frame_Binaryzation_Matrix(i, j) = 0;
			}
		}
		Threshold_Line = ((Sum_H / (Count_H + 0.0001)) + (Sum_L / (Count_L + 0.0001))) / 2;
	}
}

/******************************************************************************
* function :		void Line_Detection_()
* Description : 	
*******************************************************************************/
void Line_Detection_()
{

}

/******************************************************************************
* function :		void Circle_Detection_()
* Description : 	
*******************************************************************************/
void Circle_Detection_()
{
	
}

/******************************************************************************
* function :		void Track_Init()
* Description : 
*******************************************************************************/
void Track_Init()
{
	uint8_t i, j;
	for(i = 0; i < 30; i++)
		for(j = 0; j < 30; j++)
			Frame_Gradient_Matrix[i][j] = 0;
}


/******************************************************************************
* function :		float Track()
* Description : 
*******************************************************************************/
float Track()
{
	int8_t count_x, count_y;
	uint8_t Current_x, Current_y;
	uint8_t Destination_x, Destination_y;
	float Target_yaw;
	Edge_Detection();
		// Get the current Position
		// the position on the Frame is related to Pitch and Roll
	count_x = tan(Get_Roll()) * 8 * 1600 / 25.4;
	count_y = tan(Get_Pitch()) * 8 * 1600 / 25.4;
		// Get Destination
		// the Destination is related to Frame_Gradient_Matrix
	// Destination_x = 
	// to do
		// Get the target of yaw
	Target_yaw = atan2(Destination_y - Current_y, Destination_x - Destination_x);
	
	// to do
	return Target_yaw;
		// http://blog.csdn.net/jia20003/article/details/7724530
}
