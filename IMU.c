
/**********************************************************************************
 * �ļ���  ��IMU.c
 * ����    ����̬����         
**********************************************************************************/

#include "include.h"
#include "math.h"


struct _angle angle;

/***********************	
	Q:����������Q���󣬶�̬��Ӧ��죬�����ȶ��Ա仵
	R:����������R���󣬶�̬��Ӧ�����������ȶ��Ա��	
***********************/

#define KALMAN_Q        0.02
#define KALMAN_R        6.0000

KalmanFilter_Typedef Kalman_Ax, Kalman_Ay, Kalman_Az;

void KalmanFilterParameter_Load()
{
	Kalman_Ax.Q = Kalman_Ay.Q = Kalman_Az.Q = KALMAN_Q;
	Kalman_Ax.R = Kalman_Ay.R = Kalman_Az.R = KALMAN_R;
}

/*********** �������˲�***********/

static double KalmanFilter(const double ResrcData, KalmanFilter_Typedef *  Kalman)
{
   double x_mid;
   double x_now;
   double p_mid;
   double p_now;
   double kg;        

   x_mid = Kalman->x_last;		//x_last = x(k-1|k-1), x_mid = x(k|k-1)
   p_mid = Kalman->p_last + Kalman->Q;	//p_mid = p(k|k-1), p_last = p(k-1|k-1), Q=����
   kg = p_mid / (p_mid + Kalman->R);			//kgΪkalman filter ���������棬RΪ����
   x_now = x_mid + kg * (ResrcData - x_mid);//���Ƴ�������ֵ
                
   p_now = (1 - kg) * p_mid;//����ֵ��Ӧ��covariance       
   Kalman->p_last = p_now; //����covarianceֵ
   Kalman->x_last = x_now; //����ϵͳ״ֵ̬
   return x_now;          
 }

//   ������ƽ��������
float Q_rsqrt(float number)	   //������һ������ƽ������������ţ�ٵ�����������Ϊ�㷨����
{
	long i;
	float x2, y;
	const float threehalfs = 1.5F;
 
	x2 = number * 0.5F;
	y  = number;
	i  = * ( long * ) &y;                      
	i  = 0x5f3759df - ( i >> 1 );    //???��������           
	y  = * ( float * ) &i;
	y  = y * ( threehalfs - ( x2 * y * y ) );   // 1st iteration ����һ��ţ�ٵ�����
	return y;
} 

//  ��float�����ݾ���ֵ
float FL_ABS(float x)
{
   if(x < 0)  return -x;
	 else return x; 
}

/*   �������Ǻ�����̩��չ��ʽ �����ֵ*/
float COS(float x)		//cosx��̩��չ������ֵ
{
	float result;
  result = 1 - x * x/2;
	return result; 
}

float SIN(float y)	  //siny��̩��չ������ֵ
{
	float result;
  result = y - y * y * y /6;
	return result; 
}

/***********************************************
  * @brief  �ɱ���������Ӧ����
  * @param  None                                                  
  * @retval None
************************************************/
float VariableParameter(float error)   //�������ֵ������������
{
	float  result = 0;
	
	if(error < 0)
	{
	   error = -error;
	}
	
  	if(error >0.8f)
	{
	   error = 0.8f;
	}
	
	result = 1 - 1.28 * error;
	
	if(result < 0)
	{
	   result = 0;
	}
	
	return result;
}
/*************************************/

void Prepare_Data(void)
{       
	MPU6050_Read_RawData();         //��ȡ6050
	Multiple_Read_HMC5883L();   //��ȡ�ش�����
	
	KalmanFilterParameter_Load();
	MPU6050_data.Ax = KalmanFilter(MPU6050_data.Ax, & Kalman_Ax);
	MPU6050_data.Ay = KalmanFilter(MPU6050_data.Ay, & Kalman_Ay);
	MPU6050_data.Az = KalmanFilter(MPU6050_data.Az, & Kalman_Az);
	
}


//��Ԫ��
float qa0, qa1, qa2, qa3;     //��Ԫ����  �õ���ǰ��̬
#define Kp 0.8f               // �������ֿ�������P  proportional gain governs rate of convergence to accelerometer����������������ٶȵļ��ٶȼ�/magnetometer ��ǿ��
#define Ki 0.0015f            // �������ֿ�������I     integral gain governs rate of convergence of gyroscope biases

#define halfT 0.00125f        //��̬�������ڣ�һ������Ԫ��΢�����ʱ�õġ� �������ڵ�һ��  ������ 2.5MS �ɼ�һ��  ���� halfT��1.25MS

/**************************************
 * ��������Get_Attitude()
 * ����  ���õ���ǰ��̬
 * ����  ����
 * ���  ����
 * ����  ���ⲿ����
 *************************************/
void Get_Attitude()
{
	Prepare_Data();
	
	IMUupdate(	MPU6050_data.Gx * AtR, MPU6050_data.Gy * AtR, MPU6050_data.Gz * AtR, MPU6050_data.Ax, MPU6050_data.Ay, MPU6050_data.Az);	
}

float q0 = 1, q1 = 0, q2 = 0, q3 = 0;    // quaternion elements representing the estimated orientation
float exInt = 0, eyInt = 0, ezInt = 0;    // scaled integral error	 ����м����
float vx, vy, vz;// wx, wy, wz;	 ��ǰ��̬����ֱ�����ϵķ�����N�����е���ֱ������B����ϵ�еı�ʾ��

void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az)
{
  float norm;	//����ռ������ķ������м����
	int16_t Xr,Yr;
  
  float ex, ey, ez;	//���ٶȼƱ�ʾ����ת�͵�ǰ��̬����ֱ�����ϵķ�����B�����е����������

  // �Ȱ���Щ�õõ���ֵ���
  float q0q0 = q0*q0;
  float q0q1 = q0*q1;
  float q0q2 = q0*q2;
  //  float q0q3 = q0*q3;//
  float q1q1 = q1*q1;
  //  float q1q2 = q1*q2;//
  float q1q3 = q1*q3;
  float q2q2 = q2*q2;
  float q2q3 = q2*q3;
  float q3q3 = q3*q3;
	
	if(ax*ay*az==0)
 		return;
		
  norm = Q_rsqrt(ax*ax + ay*ay + az*az);       //acc���ݹ�һ������
  ax = ax *norm;
  ay = ay * norm;
  az = az * norm;

  // estimated direction of gravity and flux (v and w)              �����������������/��Ǩ
  
  vx = 2*(q1q3 - q0q2);						//��Ԫ����xyz�ı�ʾ
  vy = 2*(q0q1 + q2q3);
  vz = q0q0 - q1q1 - q2q2 + q3q3 ;

  // error is sum of cross product between reference direction of fields and direction measured by sensors
  ex = (ay*vz - az*vy) ;                           		 //�������������õ���־������
  ey = (az*vx - ax*vz) ;
  ez = (ax*vy - ay*vx) ;

  exInt = exInt + VariableParameter(ex) * ex * Ki;		//�������л���
  eyInt = eyInt + VariableParameter(ey) * ey * Ki;
  ezInt = ezInt + VariableParameter(ez) * ez * Ki;
// adjusted gyroscope measurements

	gx = gx + Kp *  VariableParameter(ex) * ex + exInt;			  //��������������ǵ�xyz
	gy = gy + Kp *  VariableParameter(ey) * ey + eyInt;	
	gz = gz + Kp *  VariableParameter(ez) * ez + ezInt;	
  								
  // integrate quaternion rate and normalise			  //��Ԫ�ص�΢�ַ���	����Ԫ���ĸ���
  q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
  q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
  q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
  q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;

  // normalise quaternion	   ����������Ԫ��
  norm = Q_rsqrt(q0q0 + q1q1 + q2q2 + q3q3);
  q0 = q0 * norm;
  q1 = q1 * norm;
  q2 = q2 * norm;
  q3 = q3 * norm;

  qa0 = q0;
  qa1 = q1;
  qa2 = q2;
  qa3 = q3;


//	
  	angle.roll = atan2(2*q2q3 + 2*q0q1, -2*q1q1 - 2*q2q2 + 1); // roll
	angle.pitch = asin(-2*q1q3 + 2*q0q2); // pitch
	
	/*          ���ڵش���ν�����ǲ���                       */    
	/*�ο�  http://baike.baidu.com/view/1239157.htm?fr=aladdin */
	
	//Xr = X_HMC * COS(angle.pitch) + Y_HMC * SIN(-angle.pitch) * SIN(-angle.roll) - Z_HMC * COS(angle.roll) * SIN(-angle.pitch);
	//Yr = Y_HMC * COS(angle.roll) + Z_HMC * SIN(-angle.roll);
	
	angle.yaw = atan2( (double)Y_HMC, (double)X_HMC ) * RtA; // yaw 
	angle.roll *= RtA;
	angle.pitch *= RtA;

}

/********************** ֱ����ŷ���ǽ�����̬ ���������˲�************************

float t=0.025;	//�˲�������ʱ��
float r_xz=0.25,r_yx=0.25,r_yz=0.25;
float q_xz=0.0025,q_yx=0.0025,q_yz=0.0025;
float k_xz=0,k_yx=0,k_yz=0;
float Aax,Aay;

float p_xz=1,p_yx=1,p_yz=1;
float aax,aay,accx,accy,accz;

void get_MPUdata()
{
	MPU6050_Read_RawData(mpu_ptr);
	accx = (float)mpu_ptr->Ax;
	accy = (float)mpu_ptr->Ay;
	accz = (float)mpu_ptr->Az;
	
	aax = atan(accx / accz)*180 / pi-AngleAx_Offset;
  	aay = atan(accy / accz)*180 / pi-AngleAy_Offset;
	
	Gx = (float)mpu_ptr->Gx / GyroRatio - AngleGx_Offset;
	Gy = (float)mpu_ptr->Gy / GyroRatio - AngleGy_Offset;
	Gz = (float)mpu_ptr->Gz / GyroRatio - AngleGz_Offset;
	
	Aay = Aay - t * Gx;
	p_xz = p_xz + q_xz;
	k_xz = p_xz / (p_xz + r_xz);
	Aay = Aay + k_xz * (aay - Aay);
	p_xz = (1 - k_xz) * p_xz;
	
  	Aax = Aax + t * Gy;
  	p_yz = p_yz + q_yz;
  	k_yz = p_yz / (p_yz + r_yz);
  	Aax = Aax + k_yz * (aax - Aax);
  	p_yz = (1 - k_yz) * p_yz;		//�������˲�
}


******************************************************/