
/**********************************************************************************
 * File Name  ��IMU.c  
 * Version	: 2015.7.22 By DHP
 * Device(s)	: R5F100LE
 * Tool-Chain	: CA78K0R
 * Description	: IMU
 * API		: 
		  void Get_Attitude_DMP()
		  void Get_Attitude()
**********************************************************************************/

#include "include.h"
#include "math.h"
//#include "KalmanFilter.h"

#define isnan(x)    ((x) != (x))

struct _angle angle;
const static float pi = 3.1415926;

//   ������ƽ��������

float Q_rsqrt(float number)	   
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
  * @param  None  //�������ֵ������������
  * @retval None
************************************************/
float VariableParameter(float error)   
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

//	Get_MPU6050_Ax() = KalmanFilter(Get_MPU6050_Ax(), & Kalman_Ax);
//	Get_MPU6050_Ay() = KalmanFilter(Get_MPU6050_Ay(), & Kalman_Ay);
//	Get_MPU6050_Az() = KalmanFilter(Get_MPU6050_Az(), & Kalman_Az);
	
}


//��Ԫ��
float qa0, qa1, qa2, qa3;     //��Ԫ����  �õ���ǰ��̬
#define Kp 0.8f               // �������ֿ�������P  proportional gain governs rate of convergence to accelerometer����������������ٶȵļ��ٶȼ�/magnetometer ��ǿ��
#define Ki 0.0015f            // �������ֿ�������I     integral gain governs rate of convergence of gyroscope biases

#define halfT 0.005f        //��̬�������ڣ�һ������Ԫ��΢�����ʱ�õġ� �������ڵ�һ��  ������ 10 MS �ɼ�һ��  ���� halfT�� 5 MS

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
	
	IMUupdate(Get_MPU6050_Gx() * AtR, Get_MPU6050_Gy() * AtR, Get_MPU6050_Gz() * AtR, Get_MPU6050_Ax(), Get_MPU6050_Ay(), Get_MPU6050_Az());	
}

float q0 = 1, q1 = 0, q2 = 0, q3 = 0;    // quaternion elements representing the estimated orientation
float exInt = 0, eyInt = 0, ezInt = 0;    // scaled integral error	 ����м����
float vx, vy, vz;// wx, wy, wz;	 ��ǰ��̬����ֱ�����ϵķ�����N�����е���ֱ������B����ϵ�еı�ʾ��
float Xr,Yr;


void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az)
{
  float norm;	//����ռ������ķ������м����
	
  
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

  exInt = exInt + VariableParameter(ex) * Ki;		//�������л���
  eyInt = eyInt + VariableParameter(ey) * Ki;
  ezInt = ezInt + VariableParameter(ez) * Ki;
// adjusted gyroscope measurements

	gx = gx + Kp *  VariableParameter(ex) + exInt;			  //��������������ǵ�xyz
	gy = gy + Kp *  VariableParameter(ey) + eyInt;	
	gz = gz + Kp *  VariableParameter(ez) + ezInt;	
  								
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
  	angle.roll = asin(-2*q1q3 + 2*q0q2); // pitch
	angle.pitch = atan2(2*q2q3 + 2*q0q1, -2*q1q1 - 2*q2q2 + 1); // roll
	
	/*          ���ڵش���ν�����ǲ���                       */    
	/*�ο�  http://baike.baidu.com/view/1239157.htm?fr=aladdin */
	
	Xr = Get_HMC5883L_Hx() * COS(-angle.roll) + Get_HMC5883L_Hy() * SIN(angle.roll) * SIN(-angle.pitch) - Get_HMC5883L_Hz() * COS(angle.pitch) * SIN(angle.roll);
	Yr = Get_HMC5883L_Hy() * COS(angle.pitch) + Get_HMC5883L_Hz() * SIN(-angle.pitch);
	
	angle.yaw = atan2( (double)Yr, (double)Xr ) * RtA; // yaw 
	angle.roll *= RtA;
	angle.pitch *= RtA;

}

// a varient of asin() that checks the input ranges and ensures a
// valid angle as output. If nan is given as input then zero is
// returned.
float dmpsafe_asin(float v)
{
	if (isnan(v)) 
	{
		return 0.0;
	}
	if (v >= 1.0)
	{
		return pi/2;
	}
	if (v <= -1.0) 
	{
		return -pi/2;
	}
	return asin(v);
}



float q[4];
	//���������� DMP_Routing ����
float  dmp_pitch;  //DMP������ĸ�����	��λ����
float  dmp_roll;    //DMP��ת��		   ��λ����
float  dmp_yaw;		//DMP ����ǣ�����û�д����Ʋ��룬����ǻ�Ʈ  ��λ����
float  dmp_gyrox;	// ������ X�� ���ٶ�   ��λ����ÿ��
float  dmp_gyroy;   // ������ Y�� ���ٶ�   ��λ����ÿ��
float  dmp_gyroz;   // ������ Z�� ���ٶ�   ��λ����ÿ��
float  dmp_accx;	// ���ٶȼ� X��   ��λ��m/S^2
float  dmp_accy;	// ���ٶȼ� Y��   ��λ��m/S^2
float  dmp_accz;	// ���ٶȼ� Z��   ��λ��m/S^2

void Get_Attitude_DMP()
{
		// ��Ԫ��
	float  qtemp[4],norm ;	
		// Get DMP data and fix yaw
	DMP_Routing();
		//��ȡ�ش�����
	Multiple_Read_HMC5883L();	
		 
	dmp_gyrox = Get_DMP_Gyro_x();
	dmp_gyroy = Get_DMP_Gyro_y();
	dmp_gyroz = Get_DMP_Gyro_z();
		//acc sensitivity to +/-    4 g
		//���ٶ� ת�ɵ�λ�� m/S^2
	dmp_accx = Get_DMP_Acc_x();
	dmp_accy = Get_DMP_Acc_y();
	dmp_accz = Get_DMP_Acc_z();
		//��ȡDMP����Ԫ��
	qtemp[0] = Get_DMP_qw(); 	
	qtemp[1] = Get_DMP_qx();
	qtemp[2] = Get_DMP_qy();
	qtemp[3] = Get_DMP_qz();
		// ��Ԫ����һ��
	norm = Q_rsqrt(qtemp[0]*qtemp[0] + qtemp[1]*qtemp[1] + qtemp[2]*qtemp[2] + qtemp[3]*qtemp[3]);
	q[0] = qtemp[0] * norm;
	q[1] = qtemp[1] * norm;
	q[2] = qtemp[2] * norm;
	q[3] = qtemp[3] * norm;
	
	dmp_roll = (atan2(2.0*(q[0]*q[1] + q[2]*q[3]), 1 - 2.0*(q[1]*q[1] + q[2]*q[2])))* 180/pi;
		// we let safe_asin() handle the singularities near 90/-90 in pitch
	dmp_pitch = dmpsafe_asin(2.0*(q[0]*q[2] - q[3]*q[1]))* 180/pi;
		//ע�⣺�˴����㷴�ˣ�������ϵ��
		
	//dmp_yaw = -atan2(2.0*(q[0]*q[3] + q[1]*q[2]),1 - 2.0*(q[2]*q[2] + q[3]*q[3]))* 180/pi;
  	//#ifdef YAW_CORRECT
	//dmp_yaw = -dmp_yaw;
	//#endif
	
		//ע�⣺ǰ����㷴�ˣ���������pitch �� rollҪ��������
	//angle.yaw =   DMP_DATA.dmp_yaw;
	angle.pitch =  dmp_roll;
	angle.roll =  dmp_pitch;
	
	Xr = Get_HMC5883L_Hx() * COS(-angle.roll) + Get_HMC5883L_Hy() * SIN(angle.roll) * SIN(-angle.pitch) - Get_HMC5883L_Hz() * COS(angle.pitch) * SIN(angle.roll);
	Yr = Get_HMC5883L_Hy() * COS(angle.pitch) + Get_HMC5883L_Hz() * SIN(-angle.pitch);
	
	angle.yaw = atan2( (double)Yr, (double)Xr ) * RtA;
}


float Get_Pitch()
{
	return angle.pitch;
}
float Get_Roll()
{
	return angle.roll;
}
float Get_Yaw()
{
	return angle.yaw;
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
