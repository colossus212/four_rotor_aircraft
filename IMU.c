
/**********************************************************************************
 * 文件名  ：IMU.c  
 * Version	: 2015.7.10 By DHP
 * Device(s)	: R5F100LE
 * Tool-Chain	: CA78K0R
 * Description	: 姿态解算
 * API		: void Get_Attitude()
**********************************************************************************/

#include "include.h"
#include "math.h"
//#include "KalmanFilter.h"

struct _angle angle;


//   快速求平方根倒数

float Q_rsqrt(float number)	   
{
	long i;
	float x2, y;
	const float threehalfs = 1.5F;
 
	x2 = number * 0.5F;
	y  = number;
	i  = * ( long * ) &y;                      
	i  = 0x5f3759df - ( i >> 1 );    //???这是神马？           
	y  = * ( float * ) &i;
	y  = y * ( threehalfs - ( x2 * y * y ) );   // 1st iteration （第一次牛顿迭代）
	return y;
} 

//  求float型数据绝对值
float FL_ABS(float x)
{
   if(x < 0)  return -x;
	 else return x; 
}

/*   采用三角函数的泰勒展开式 求近似值*/
float COS(float x)		//cosx的泰勒展开近似值
{
	float result;
  result = 1 - x * x/2;
	return result; 
}

float SIN(float y)	  //siny的泰勒展开近似值
{
	float result;
  result = y - y * y * y /6;
	return result; 
}

/***********************************************
  * @brief  可变增益自适应参数
  * @param  None  //计算误差值，抑制误差过大
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
	MPU6050_Read_RawData();         //读取6050
	Multiple_Read_HMC5883L();   //读取地磁数据

	MPU6050_data.Ax = KalmanFilter(MPU6050_data.Ax, & Kalman_Ax);
	MPU6050_data.Ay = KalmanFilter(MPU6050_data.Ay, & Kalman_Ay);
	MPU6050_data.Az = KalmanFilter(MPU6050_data.Az, & Kalman_Az);
	
}


//四元数
float qa0, qa1, qa2, qa3;     //四元数法  得到当前姿态
#define Kp 0.8f               // 比例积分控制器的P  proportional gain governs rate of convergence to accelerometer比例增益控制收敛速度的加速度计/magnetometer 磁强计
#define Ki 0.0015f            // 比例积分控制器的I     integral gain governs rate of convergence of gyroscope biases

#define halfT 0.0025f        //姿态更新周期，一般是四元数微分求解时用的。 采样周期的一半  本程序 2.5MS 采集一次  所以 halfT是1.25MS

/**************************************
 * 函数名：Get_Attitude()
 * 描述  ：得到当前姿态
 * 输入  ：无
 * 输出  ：无
 * 调用  ：外部调用
 *************************************/
void Get_Attitude()
{
	Prepare_Data();
	
	IMUupdate(	MPU6050_data.Gx * AtR, MPU6050_data.Gy * AtR, MPU6050_data.Gz * AtR, MPU6050_data.Ax, MPU6050_data.Ay, MPU6050_data.Az);	
}

float q0 = 1, q1 = 0, q2 = 0, q3 = 0;    // quaternion elements representing the estimated orientation
float exInt = 0, eyInt = 0, ezInt = 0;    // scaled integral error	 误差中间变量
float vx, vy, vz;// wx, wy, wz;	 当前姿态在竖直方向上的分量（N坐标中的竖直分量在B坐标系中的表示）
float Xr,Yr;


void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az)
{
  float norm;	//计算空间向量的范数的中间变量
	
  
  float ex, ey, ez;	//加速度计表示的旋转和当前姿态在竖直方向上的分量在B坐标中的向量外积差

  // 先把这些用得到的值算好
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
		
  norm = Q_rsqrt(ax*ax + ay*ay + az*az);       //acc数据归一化处理
  ax = ax *norm;
  ay = ay * norm;
  az = az * norm;

  // estimated direction of gravity and flux (v and w)              估计重力方向和流量/变迁
  
  vx = 2*(q1q3 - q0q2);						//四元素中xyz的表示
  vy = 2*(q0q1 + q2q3);
  vz = q0q0 - q1q1 - q2q2 + q3q3 ;

  // error is sum of cross product between reference direction of fields and direction measured by sensors
  ex = (ay*vz - az*vy) ;                           		 //向量外积在相减得到差分就是误差
  ey = (az*vx - ax*vz) ;
  ez = (ax*vy - ay*vx) ;

  exInt = exInt + VariableParameter(ex) * Ki;		//对误差进行积分
  eyInt = eyInt + VariableParameter(ey) * Ki;
  ezInt = ezInt + VariableParameter(ez) * Ki;
// adjusted gyroscope measurements

	gx = gx + Kp *  VariableParameter(ex) + exInt;			  //修正过后的陀螺仪的xyz
	gy = gy + Kp *  VariableParameter(ey) + eyInt;	
	gz = gz + Kp *  VariableParameter(ez) + ezInt;	
  								
  // integrate quaternion rate and normalise			  //四元素的微分方程	，四元数的更新
  q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
  q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
  q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
  q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;

  // normalise quaternion	   正常化的四元数
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
	
	/*          关于地磁如何进行倾角补偿                       */    
	/*参考  http://baike.baidu.com/view/1239157.htm?fr=aladdin */
	
	Xr = X_HMC * COS(-angle.roll) + Y_HMC * SIN(angle.roll) * SIN(-angle.pitch) - Z_HMC * COS(angle.pitch) * SIN(angle.roll);
	Yr = Y_HMC * COS(angle.pitch) + Z_HMC * SIN(-angle.pitch);
	
	angle.yaw = atan2( (double)Yr, (double)Xr ) * RtA; // yaw 
	angle.roll *= RtA;
	angle.pitch *= RtA;

}

void Get_Attitude_DMP()
{	
	Multiple_Read_HMC5883L();   //读取地磁数据
	Xr = X_HMC * COS(-angle.roll) + Y_HMC * SIN(angle.roll) * SIN(-angle.pitch) - Z_HMC * COS(angle.pitch) * SIN(angle.roll);
	Yr = Y_HMC * COS(angle.pitch) + Z_HMC * SIN(-angle.pitch);
	angle.yaw = atan2( (double)Yr, (double)Xr ) * RtA;
}


/********************** 直接用欧拉角解算姿态 含卡夫曼滤波************************

float t=0.025;	//滤波器采样时间
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
  	p_yz = (1 - k_yz) * p_yz;		//卡尔曼滤波
}


******************************************************/