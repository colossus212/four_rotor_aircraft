
/************************************************
* File Name	: MPU_6050.c
* Version	: 2015.7.22 By DHP
* Device(s)	: R5F100LE
* Tool-Chain	: CA78K0R
* Description	: MPU6050
* API		:
		  #define MPU6050_Int P4.3

		  void MPU6050_Init(void);	初始化 MPU6050 的寄存器,计算零偏
		  uint8_t MPU6050_DMP_Initialize(void); //DMP初始化
		  void DMP_Routing(void);	 //DMP 线程，主要用于读取和处理DMP的结果   [需要定时调用]
		  
		  float Get_DMP_Gyro_x();
		  float Get_DMP_Gyro_y();
		  float Get_DMP_Gyro_z();
		  float Get_DMP_Acc_x();
		  float Get_DMP_Acc_y();
		  float Get_DMP_Acc_z();
		  float Get_DMP_qw();
		  float Get_DMP_qx();
		  float Get_DMP_qy();
		  float Get_DMP_qz();

		  void MPU6050_Read_RawData();  获得 MPU6050 的六个测量值，已经转换单位并减去零偏 输出角度值
		  extern MPU6050_Struct MPU6050_data ;
		  float Get_MPU6050_Ax();
		  float Get_MPU6050_Ay();
		  float Get_MPU6050_Az();		  
		  float Get_MPU6050_Gx();
		  float Get_MPU6050_Gy();
		  float Get_MPU6050_Gz();
		  //void MPU6050_RawData_Average() 
*************************************************************/

#include "include.h"
#include "math.h"
//#include "KalmanFilter.h"


#define MPU6050_Int P4.3


MPU6050_Struct MPU6050_data;

const static float pi = 3.1415926;

static float AcceRatio;   
static float GyroRatio;
float TemperatureRatio = 340.0;
float TemperatureOffset = - 36.53;

uint8_t sample_times = 2000;	// for Get_Offset
float Ax_Offset, Ay_Offset, Az_Offset, Gx_Offset, Gy_Offset, Gz_Offset;	// zero offset
uint8_t MPU6050_Offset_Done = 0;


uint8_t buffer[14];
int16_t  MPU6050_FIFO[6][11];


float Get_MPU6050_Ax()
{
	MPU6050_data.Ax = KalmanFilter(MPU6050_data.Ax, & Kalman_Ax);
	return MPU6050_data.Ax;
}

float Get_MPU6050_Ay()
{
	MPU6050_data.Ay = KalmanFilter(MPU6050_data.Ay, & Kalman_Ay);
	return MPU6050_data.Ay;
}

float Get_MPU6050_Az()
{
	MPU6050_data.Az = KalmanFilter(MPU6050_data.Az, & Kalman_Az);
	return MPU6050_data.Az;
}

float Get_MPU6050_Gx()
{
	MPU6050_data.Gx = KalmanFilter(MPU6050_data.Gx, & Kalman_Gx);
	return MPU6050_data.Gx;
}

float Get_MPU6050_Gy()
{
	MPU6050_data.Gy = KalmanFilter(MPU6050_data.Gy, & Kalman_Gy);
	return MPU6050_data.Gy;
}

float Get_MPU6050_Gz()
{
	MPU6050_data.Gz = KalmanFilter(MPU6050_data.Gz, & Kalman_Gz);
	return MPU6050_data.Gz;
}

/*
float Get_MPU6050_Ax_old()
{
	return MPU6050_data_old.Ax;
}

float Get_MPU6050_Ay_old()
{
	return MPU6050_data_old.Ay;
}

float Get_MPU6050_Az_old()
{
	return MPU6050_data_old.Az;
}

float Get_MPU6050_Gx_old()
{
	return MPU6050_data_old.Gx;
}

float Get_MPU6050_Gy_old()
{
	return MPU6050_data_old.Gy;
}

float Get_MPU6050_Gz_old()
{
	return MPU6050_data_old.Gz;
}
*/


float Get_MPU6050_Gx_A()
{
	return MPU6050_data.Gx * pi / 180;
}
float Get_MPU6050_Gy_A()
{
	return MPU6050_data.Gy * pi / 180;
}
float Get_MPU6050_Gz_A()
{
	return MPU6050_data.Gz * pi / 180;
}



void MPU6050_Get_Offset()
{	
	
	uint8_t i;
	float Ax, Ay, Az, Gx, Gy, Gz;		//The raw data of accelerometers and gyroscope
	Ax=0;Ay=0;Az=0;
	Gx=0;Gy=0;Gz=0;		//initialize variables as the raw data of accelerometers and gyroscope
	
	
	for (i=0;i<sample_times;i++)
	{
		MPU6050_Read_RawData();
		Ax += MPU6050_data.Ax;
		Ay += MPU6050_data.Ay; 
		//Az += MPU6050_data.Az;
		Gx += MPU6050_data.Gx;
		Gy += MPU6050_data.Gy;		
		Gz += MPU6050_data.Gz;
	}
	
	Ax_Offset = Ax / sample_times;
	Ay_Offset = Ay / sample_times;
	//Az_Offset = Az / sample_times;
	Gx_Offset = Gx / sample_times;
	Gy_Offset = Gy / sample_times;
	Gz_Offset = Gz / sample_times;
	
	MPU6050_Offset_Done = 1;
	
	/**************************************
	AngleAx_Offset = ( atan(Ax / Az)*180 ) / pi;
  	AngleAy_Offset = ( atan(Ay / Az)*180 ) / pi;
	AngleGx_Offset = Gx / sample_times;
	AngleGy_Offset = Gy / sample_times;
	AngleGz_Offset = Gz / sample_times;
	**************************************/
	
}


/**************************实现函数********************************************
*函数原型:		void MPU6050_Set_Accel_Range(uint8_t range)
*功　　能:	    设置  MPU6050 加速度计的最大量程 
* AFS_SEL=0    ±2	g	16,384    LSB/g       
* AFS_SEL=1    ±4	g	8,192     LSB/g 
* AFS_SEL=2    ±8	g	4,096     LSB/g
* AFS_SEL=3    ±16	g	2,048     LSB/g 
*******************************************************************************/
void MPU6050_Set_Accel_Range(uint8_t range)
{
	IIC_Write_Bits(MPU6050_Address, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, range);
	switch(range)
	{
		case 0: AcceRatio = 16384.0; break;
		case 1: AcceRatio = 8192.0; break;
		case 2: AcceRatio = 4096.0; break;
		case 3: AcceRatio = 2048.0; break;
	}
}


/**************************实现函数********************************************
*函数原型:		void MPU6050_Set_Gyro_Range(uint8_t range)
*功　　能:	    设置  MPU6050 陀螺仪的最大量程
* FS_SEL=0    ±250    o/s	131     LSB/(o/s) 
* FS_SEL=1    ±500    o/s	65.5    LSB/(o/s) 
* FS_SEL=2    ±1000   o/s	32.8    LSB/(o/s)
* FS_SEL=3    ±2000   o/s	16.4    LSB/(o/s) 
*******************************************************************************/
void MPU6050_Set_Gyro_Range(uint8_t range)
{
	IIC_Write_Bits(MPU6050_Address, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, range);
	switch(range)
	{
		case 0: GyroRatio = 131.0; break;
		case 1: GyroRatio = 65.5; break;
		case 2: GyroRatio = 32.8; break;
		case 3: GyroRatio = 16.4; break;
	}
}



/**************************实现函数********************************************
*函数原型:		void MPU6050_Init(void)
*功　　能:	    初始化 MPU6050 以进入可用状态。
*******************************************************************************/
void MPU6050_Init(void)
{
	uint8_t Add[1];
	MPU6050_Get_Device_ID(Add);
	
	MPU6050_Int_Init();
	
	IIC_Write_Byte(MPU6050_Address, MPU6050_RA_PWR_MGMT_1, 0x00);	//awake
	IIC_Write_Byte(MPU6050_Address, MPU6050_RA_SMPLRT_DIV, 0x07);
	IIC_Write_Byte(MPU6050_Address, MPU6050_RA_CONFIG, 0x06);
	IIC_Write_Byte(MPU6050_Address, MPU6050_RA_GYRO_CONFIG, 0x18); //FS_SEL=0    ±250    o/s	131     LSB/(o/s) 
	GyroRatio = 131.0;
	
	IIC_Write_Byte(MPU6050_Address, MPU6050_RA_ACCEL_CONFIG, 0x00); //AFS_SEL=0    ±2	g	16,384    LSB/g 
	AcceRatio = 16384.0;
	
	delay_ms(100);
	
	
	MPU6050_Get_Offset();
	
	
}

/**************************实现函数********************************************
*函数原型:		void MPU6050_Get_Device_ID(uint8_t * Add)
*功　　能:	    读取  MPU6050 WHO_AM_I 标识	 将得到 0x68
*******************************************************************************/
void MPU6050_Get_Device_ID(uint8_t * Add) 
{
    IIC_Read_Bytes(MPU6050_Address, MPU6050_RA_WHO_AM_I, 1, Add);
}


//读取3个轴的数据 
//x,y,z读取到的数据 

void MPU6050_Read_RawData()
{
	uint8_t buf[14];
	//MPU6050_data_old.Ax = MPU6050_data.Ax; etc
	
	IIC_Read_Bytes(MPU6050_Address, MPU6050_RA_ACCEL_XOUT_H, 14, buf);
	
	if(MPU6050_Offset_Done == 0)
	{
		MPU6050_data.Ax = ( (float) (( ((int16_t)buf[0]) <<8) + buf[1]) ) / AcceRatio; 
		MPU6050_data.Ay = ( (float) (( ((int16_t)buf[2]) <<8) + buf[3]) ) / AcceRatio; 
		MPU6050_data.Az = ( (float) (( ((int16_t)buf[4]) <<8) + buf[5]) ) / AcceRatio; 
		MPU6050_data.Gx = ( (float) ( (( ((int16_t)buf[8]) <<8) + buf[9])) ) / GyroRatio ; 
		MPU6050_data.Gy = ( (float) ( (( ((int16_t)buf[10]) <<8) + buf[11])) ) / GyroRatio ; 
		MPU6050_data.Gz = ( (float) ( (( ((int16_t)buf[12]) <<8) + buf[13])) ) / GyroRatio ;
	}
		else
		{
			MPU6050_data.Ax = ( (float) (( ((int16_t)buf[0]) <<8) + buf[1]) ) / AcceRatio - Ax_Offset; 
			MPU6050_data.Ay = ( (float) (( ((int16_t)buf[2]) <<8) + buf[3]) ) / AcceRatio - Ay_Offset; 
			MPU6050_data.Az = ( (float) (( ((int16_t)buf[4]) <<8) + buf[5]) ) / AcceRatio; //- Az_Offset; 
			MPU6050_data.Tt = ( (float) (( ((int16_t)buf[6]) <<8) + buf[7]) ) / TemperatureRatio - TemperatureOffset;
			MPU6050_data.Gx = ( (float) ( (( ((int16_t)buf[8]) <<8) + buf[9])) ) / GyroRatio - Gx_Offset; 
			MPU6050_data.Gy = ( (float) ( (( ((int16_t)buf[10]) <<8) + buf[11])) ) / GyroRatio  - Gy_Offset; 
			MPU6050_data.Gz = ( (float) ( (( ((int16_t)buf[12]) <<8) + buf[13])) ) / GyroRatio  - Gz_Offset;			
		}
	
}


//读取ADXL的平均值 
//x,y,z读取8次后取平均值 

void MPU6050_RawData_Average()
{
	uint8_t i;  
	   
	for(i=0;i<8;i++)
	{
		MPU6050_Read_RawData();
		// delay_ms(10);
		MPU6050_data.Ax += MPU6050_data.Ax;
		MPU6050_data.Ay += MPU6050_data.Ay;
		MPU6050_data.Az += MPU6050_data.Az;	   
	}
	MPU6050_data.Ax = MPU6050_data.Ax / 8;
	MPU6050_data.Ay = MPU6050_data.Ay / 8;
	MPU6050_data.Az = MPU6050_data.Az / 8;
}


MPU6050_Int_Init()
{
	PM4 |= _08_PMn3_MODE_INPUT;	//set P4.3 input mode
	//PU4 |= _08_PUn3_PULLUP_ON;	//set P4.3 with pullup resistor
	MPU6050_Int = 0U;
}



/**************************实现函数********************************************
*函数原型:		unsigned char MPU6050_is_DRY(void)
*功　　能:	    检查 MPU6050的中断引脚，测试是否完成转换
返回 1  转换完成
0 数据寄存器还没有更新
*******************************************************************************/
unsigned char MPU6050_is_DRY(void)
{	
    if(MPU6050_Int == 1)
	{
	  return 1;
	}
	else return 0;
}

/**************************实现函数********************************************
*函数原型:		void MPU6050_Set_Clock_Source(uint8_tsource)
*功　　能:	    设置  MPU6050 的时钟源
 * CLK_SEL | Clock Source
 * --------+--------------------------------------
 * 0       | Internal oscillator
 * 1       | PLL with X Gyro reference
 * 2       | PLL with Y Gyro reference
 * 3       | PLL with Z Gyro reference
 * 4       | PLL with external 32.768kHz reference
 * 5       | PLL with external 19.2MHz reference
 * 6       | Reserved
 * 7       | Stops the clock and keeps the timing generator in reset
*******************************************************************************/
void MPU6050_Set_Clock_Source(uint8_t source)
{
    IIC_Write_Bits(MPU6050_Address, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, source);
}

/** Trigger a full device reset.
 * A small delay of ~50ms may be desirable after triggering a reset.
 * @see MPU6050_RA_PWR_MGMT_1
 * @see MPU6050_PWR1_DEVICE_RESET_BIT
 */
void MPU6050_Reset(void)
{
    IIC_Write_Bit(MPU6050_Address, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_DEVICE_RESET_BIT, 1);
}



/**************************实现函数********************************************
*函数原型:		void MPU6050_Set_Sleep_Enabled(uint8_tenabled)
*功　　能:	    设置  MPU6050 是否进入睡眠模式
				enabled =1   睡觉
			    enabled =0   工作
*******************************************************************************/
void MPU6050_Set_Sleep_Enabled(uint8_t enabled) 
{
    IIC_Write_Bit(MPU6050_Address, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, enabled);
}


/**************************实现函数********************************************
*函数原型:		uint8_t MPU6050_Test_Connection(void)
*功　　能:	    检测MPU6050 是否已经连接
*******************************************************************************/
uint8_t MPU6050_Test_Connection(void) 
{
	uint8_t Add[1];
	MPU6050_Get_Device_ID(Add);
   if(Add[0] == 0x68)  //0b01101000;
   return 1;
   else return 0;
}

/**************************实现函数********************************************
*函数原型:		void MPU6050_Set_I2C_Master_Mode_Enabled(uint8_tenabled)
*功　　能:	    设置 MPU6050 是否为AUX I2C线的主机
*******************************************************************************/
void MPU6050_Set_I2C_Master_Mode_Enabled(uint8_t enabled) 
{
    IIC_Write_Bit(MPU6050_Address, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_EN_BIT, enabled);
}

/**************************实现函数********************************************
*函数原型:		void MPU6050_Set_I2C_Bypass_Enabled(uint8_tenabled)
*功　　能:	    设置 MPU6050 是否为AUX I2C线的主机
*******************************************************************************/
void MPU6050_Set_I2C_Bypass_Enabled(uint8_t enabled) 
{
    IIC_Write_Bit(MPU6050_Address, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_I2C_BYPASS_EN_BIT, enabled);
}

/**************************实现函数********************************************
*函数原型:		void MPU6050_Check()
*功　　能:	  检测IIC总线上的MPU6050是否存在
*******************************************************************************/
void MPU6050_Check(void) 
{ 
  switch(MPU6050_Test_Connection())
  {
    case 0: //printf("未检测到MPU6050...\r\n");
      break;
    case 1: //printf("已检测到MPU6050...\r\n");
      break;
  }
} 





// BANK_SEL register
void MPU6050_Set_Memory_Bank(uint8_t bank, uint8_t prefetchEnabled, uint8_t userBank) 
{
    bank &= 0x1F;
    if (userBank) bank |= 0x20;
    if (prefetchEnabled) bank |= 0x40;
    IIC_Write_Byte(MPU6050_Address, MPU6050_RA_BANK_SEL, bank);
}

// MEM_START_ADDR register
void MPU6050_Set_Memory_Start_Address(uint8_t address) 
{
    IIC_Write_Byte(MPU6050_Address, MPU6050_RA_MEM_START_ADDR, address);
}

// MEM_R_W register
uint8_t MPU6050_Read_Memory_Byte(void)
{
    IIC_Read_Bytes(MPU6050_Address, MPU6050_RA_MEM_R_W, 1 , buffer);
    return buffer[0];
}

// XG_OFFS_USR* registers
int16_t MPU6050_Get_XGyro_Offset(void) 
{
    IIC_Read_Bytes(MPU6050_Address, MPU6050_RA_XG_OFFS_USRH, 2, buffer);
    return (((int16_t)buffer[0]) << 8) | buffer[1];
}
int16_t MPU6050_Get_YGyro_Offset(void) 
{
    IIC_Read_Bytes(MPU6050_Address, MPU6050_RA_YG_OFFS_USRH, 2, buffer);
    return (((int16_t)buffer[0]) << 8) | buffer[1];
}
int16_t MPU6050_Get_ZGyro_Offset(void) 
{
    IIC_Read_Bytes(MPU6050_Address, MPU6050_RA_ZG_OFFS_USRH, 2, buffer);
    return (((int16_t)buffer[0]) << 8) | buffer[1];
}


uint8_t verifyBuffer[MPU6050_DMP_MEMORY_CHUNK_SIZE];
uint8_t progBuffer[MPU6050_DMP_MEMORY_CHUNK_SIZE];

uint8_t MPU6050_Write_Memory_Block(const uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address, uint8_t verify, uint8_t useProgMem) 
{
    uint8_t chunkSize;
    //uint8_t *verifyBuffer;
    uint8_t *tprogBuffer;
    uint16_t i;
    uint8_t j;
	MPU6050_Set_Memory_Bank(bank,0,0);
    MPU6050_Set_Memory_Start_Address(address);
    for (i = 0; i < dataSize;) 
	{
        // determine correct chunk size according to bank position and data size
        chunkSize = MPU6050_DMP_MEMORY_CHUNK_SIZE;

        // make sure we don't go past the data size
        if (i + chunkSize > dataSize) chunkSize = dataSize - i;

        // make sure this chunk doesn't go past the bank boundary (256 bytes)
        if (chunkSize > 256 - address) chunkSize = 256 - address;
        
        // write the chunk of data as specified
        tprogBuffer = (uint8_t*)data + i;
     

        IIC_Write_Bytes(MPU6050_Address, MPU6050_RA_MEM_R_W, chunkSize, tprogBuffer);

        // verify data if needed
        if (verify) 
		{
            MPU6050_Set_Memory_Bank(bank,0,0);
            MPU6050_Set_Memory_Start_Address(address);
            IIC_Read_Bytes(MPU6050_Address, MPU6050_RA_MEM_R_W, chunkSize, verifyBuffer);

			for(j = 0; j < chunkSize; j++)
			{
				if(tprogBuffer[j] != verifyBuffer[j]) 
				return 0; // uh oh.
			}
        }

        // increase byte index by [chunkSize]
        i += chunkSize;

        // uint8_tautomatically wraps to 0 at 256
        address += chunkSize;

        // if we aren't done, update bank (if necessary) and address
        if (i < dataSize)
		{
            if (address == 0) bank ++;
            MPU6050_Set_Memory_Bank(bank, 0, 0);
            MPU6050_Set_Memory_Start_Address(address);
        }
    }
    return 1;
}

uint8_t MPU6050_Write_DMP_Configuration_Set(const uint8_t *data, uint16_t dataSize, uint8_t useProgMem) 
{
    uint8_t *progBuffer, success, special;
    uint16_t i;

    // config set data is a long string of blocks with the following structure:
    // [bank] [offset] [length] [byte[0], byte[1], ..., byte[length]]

    uint8_t bank, offset, length;

    for (i = 0; i < dataSize;) 
	{
            bank = data[i++];
            offset = data[i++];
            length = data[i++];

        // write data or perform special action
        if (length > 0) 
		{
            // regular block of data to write
            progBuffer = (uint8_t*)data + i;
           
            success = MPU6050_Write_Memory_Block(progBuffer, length, bank, offset, 1, 0);
            i += length;
        } 
		else 
		{
            // special instruction
            // NOTE: this kind of behavior (what and when to do certain things)
            // is totally undocumented. This code is in here based on observed
            // behavior only, and exactly why (or even whether) it has to be here
            // is anybody's guess for now.
         
            special = data[i++];
            if (special == 0x01) 
			{
                // enable DMP-related interrupts
                
                IIC_Write_Byte(MPU6050_Address, MPU6050_RA_INT_ENABLE, 0x32);  // single operation

                success = 1;
            } 
			else 
			{
                // unknown special command
                success = 0;
            }
        }
        
        if (!success) {
            return 0; // uh oh
        }
    }
    return 1;
}

uint8_t MPU6050_Write_Prog_Memory_Block(const uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address, uint8_t verify)
{
	return MPU6050_Write_Memory_Block(data, dataSize, bank, address, verify, 0);
}

uint8_t MPU6050_Write_Prog_DMP_Configuration_Set(const uint8_t *data, uint16_t dataSize)
{
    return MPU6050_Write_DMP_Configuration_Set(data, dataSize, 0);
}



/** Set full interrupt enabled status.
 * Full register byte for all interrupts, for quick reading. Each bit should be
 * set 0 for disabled, 1 for enabled.
 * @param enabled New interrupt enabled status
 * @see getIntFreefallEnabled()
 * @see MPU6050_RA_INT_ENABLE
 * @see MPU6050_INTERRUPT_FF_BIT
 **/
void MPU6050_Set_Int_Enabled(uint8_t enabled) 
{
    IIC_Write_Byte(MPU6050_Address, MPU6050_RA_INT_ENABLE, enabled);
}

/** Set gyroscope sample rate divider.
 * @param rate New sample rate divider
 * @see getRate()
 * @see MPU6050_RA_SMPLRT_DIV
 */
void MPU6050_Set_Rate(uint8_t rate) 
{
    IIC_Write_Byte(MPU6050_Address, MPU6050_RA_SMPLRT_DIV, rate);
}



/** Set digital low-pass filter configuration.
 * @param mode New DLFP configuration setting
 * @see getDLPFBandwidth()
 * @see MPU6050_DLPF_BW_256
 * @see MPU6050_RA_CONFIG
 * @see MPU6050_CFG_DLPF_CFG_BIT
 * @see MPU6050_CFG_DLPF_CFG_LENGTH
 */
void MPU6050_Set_DLPF_Mode(uint8_t mode) 
{
    IIC_Write_Bits(MPU6050_Address, MPU6050_RA_CONFIG, MPU6050_CFG_DLPF_CFG_BIT, MPU6050_CFG_DLPF_CFG_LENGTH, mode);
}



/** Set external FSYNC configuration.
 * @see getExternalFrameSync()
 * @see MPU6050_RA_CONFIG
 * @param sync New FSYNC configuration value
 */
void MPU6050_Set_External_Frame_Sync(uint8_t sync) 
{
    IIC_Write_Bits(MPU6050_Address, MPU6050_RA_CONFIG, MPU6050_CFG_EXT_SYNC_SET_BIT, MPU6050_CFG_EXT_SYNC_SET_LENGTH, sync);
}

void MPU6050_Set_DMP_Config1(uint8_t config) 
{
    IIC_Write_Byte(MPU6050_Address, MPU6050_RA_DMP_CFG_1, config);
}

void MPU6050_Set_DMP_Config2(uint8_t config) 
{
    IIC_Write_Byte(MPU6050_Address, MPU6050_RA_DMP_CFG_2, config);
}

void MPU6050_Set_OTP_Bank_Valid(uint8_t enabled) 
{
    IIC_Write_Bit(MPU6050_Address, MPU6050_RA_XG_OFFS_TC, MPU6050_TC_OTP_BNK_VLD_BIT, enabled);
}


void MPU6050_Set_XGyro_Offset(int16_t offset) 
{
	buffer[0] = offset>>8;
	buffer[1] = offset&0x00ff;
    IIC_Write_Bytes(MPU6050_Address, MPU6050_RA_XG_OFFS_USRH, 2, buffer);
}
void MPU6050_Set_YGyro_Offset(int16_t offset) 
{
	buffer[0] = offset>>8;
	buffer[1] = offset&0x00ff;
    IIC_Write_Bytes(MPU6050_Address, MPU6050_RA_YG_OFFS_USRH, 2, buffer);
}

void MPU6050_Set_ZGyro_Offset(int16_t offset) 
{
    buffer[0] = offset>>8;
	buffer[1] = offset&0x00ff;
    IIC_Write_Bytes(MPU6050_Address, MPU6050_RA_ZG_OFFS_USRH, 2, buffer);
}

/** Reset the FIFO.
 * This bit resets the FIFO buffer when set to 1 while FIFO_EN equals 0. This
 * bit automatically clears to 0 after the reset has been triggered.
 * @see MPU6050_RA_USER_CTRL
 * @see MPU6050_USERCTRL_FIFO_RESET_BIT
 */
void MPU6050_Reset_FIFO(void) 
{
    IIC_Write_Bit(MPU6050_Address, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_FIFO_RESET_BIT, 1);
}

/** Get current FIFO buffer size.
 * This value indicates the number of bytes stored in the FIFO buffer. This
 * number is in turn the number of bytes that can be read from the FIFO buffer
 * and it is directly proportional to the number of samples available given the
 * set of sensor data bound to be stored in the FIFO (register 35 and 36).
 * @return Current FIFO buffer size
 */
uint16_t MPU6050_Get_FIFO_Count(void) 
{
    IIC_Read_Bytes(MPU6050_Address, MPU6050_RA_FIFO_COUNTH, 2, buffer);
    return (((uint16_t)buffer[0]) << 8) | buffer[1];
}

/** Set free-fall event acceleration threshold.
 * @param threshold New motion detection acceleration threshold value (LSB = 2mg)
 * @see getMotionDetectionThreshold()
 * @see MPU6050_RA_MOT_THR
 */
void MPU6050_Set_Motion_Detection_Threshold(uint8_t threshold) 
{
    IIC_Write_Byte(MPU6050_Address, MPU6050_RA_MOT_THR, threshold);
}

/** Set zero motion detection event acceleration threshold.
 * @param threshold New zero motion detection acceleration threshold value (LSB = 2mg)
 * @see getZeroMotionDetectionThreshold()
 * @see MPU6050_RA_ZRMOT_THR
 */
void MPU6050_Set_Zero_Motion_Detection_Threshold(uint8_t threshold)
{
    IIC_Write_Byte(MPU6050_Address, MPU6050_RA_ZRMOT_THR, threshold);
}

/** Set motion detection event duration threshold.
 * @param duration New motion detection duration threshold value (LSB = 1ms)
 * @see getMotionDetectionDuration()
 * @see MPU6050_RA_MOT_DUR
 */
void MPU6050_Set_Motion_Detection_Duration(uint8_t duration) 
{
    IIC_Write_Byte(MPU6050_Address, MPU6050_RA_MOT_DUR, duration);
}

/** Set zero motion detection event duration threshold.
 * @param duration New zero motion detection duration threshold value (LSB = 1ms)
 * @see getZeroMotionDetectionDuration()
 * @see MPU6050_RA_ZRMOT_DUR
 */
void MPU6050_Set_Zero_Motion_Detection_Duration(uint8_t duration) 
{
    IIC_Write_Byte(MPU6050_Address, MPU6050_RA_ZRMOT_DUR, duration);
}

void MPU6050_Read_Memory_Block(uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address)
{
    uint8_t chunkSize;
	uint16_t i;
	MPU6050_Set_Memory_Bank(bank,0,0);
    MPU6050_Set_Memory_Start_Address(address);
    
    for (i = 0; i < dataSize;) {
        // determine correct chunk size according to bank position and data size
        chunkSize = MPU6050_DMP_MEMORY_CHUNK_SIZE;

        // make sure we don't go past the data size
        if (i + chunkSize > dataSize) chunkSize = dataSize - i;

        // make sure this chunk doesn't go past the bank boundary (256 bytes)
        if (chunkSize > 256 - address) chunkSize = 256 - address;

        // read the chunk of data as specified
        IIC_Write_Bytes(MPU6050_Address, MPU6050_RA_MEM_R_W, chunkSize, data + i);
        
        // increase byte index by [chunkSize]
        i += chunkSize;

        // uint8_tautomatically wraps to 0 at 256
        address += chunkSize;

        // if we aren't done, update bank (if necessary) and address
        if (i < dataSize) {
            if (address == 0) bank++;
            MPU6050_Set_Memory_Bank(bank,0,0);
            MPU6050_Set_Memory_Start_Address(address);
        }
    }
}

void MPU6050_Get_FIFO_Bytes(uint8_t *data, uint8_t length) 
{
    IIC_Read_Bytes(MPU6050_Address, MPU6050_RA_FIFO_R_W, length, data);
}

/** Get full set of interrupt status bits.
 * These bits clear to 0 after the register has been read. Very useful
 * for getting multiple INT statuses, since each single bit read clears
 * all of them because it has to read the whole byte.
 * @return Current interrupt status
 * @see MPU6050_RA_INT_STATUS
 */
uint8_t MPU6050_Get_Int_Status(void) 
{
	uint8_t temp;
	IIC_Read_Bytes(MPU6050_Address, MPU6050_RA_INT_STATUS, 1, &temp);
    return temp;
}

void MPU6050_Set_DMP_Enabled(uint8_t enabled) 
{
    IIC_Write_Bit(MPU6050_Address, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_DMP_EN_BIT, enabled);
}

uint8_t MPU6050_Get_OTP_Bank_Valid(void) 
{
	uint8_t  temp;
	IIC_Read_Bytes(MPU6050_Address, MPU6050_RA_XG_OFFS_TC, 1, &temp);
    return temp&(1<<MPU6050_TC_OTP_BNK_VLD_BIT);
}

int8_t MPU6050_Get_XGyro_Offset_TC(void)
{
	uint8_t  temp;
	IIC_Read_Bytes(MPU6050_Address, MPU6050_RA_XG_OFFS_TC, 1, &temp);
	temp &= 0x3F;
    return temp;
}

void MPU6050_Set_XGyro_Offset_TC(int8_t offset) 
{
    IIC_Write_Bits(MPU6050_Address, MPU6050_RA_XG_OFFS_TC, MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH, offset);
}


// YG_OFFS_TC register
int8_t MPU6050_Get_YGyro_Offset_TC(void)
{
	uint8_t  temp;
	IIC_Read_Bytes(MPU6050_Address, MPU6050_RA_YG_OFFS_TC, 1, &temp);
	temp &= 0x3F;
    return temp;
}
void MPU6050_Set_YGyro_Offset_TC(int8_t offset) 
{
    IIC_Write_Bits(MPU6050_Address, MPU6050_RA_YG_OFFS_TC, MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH, offset);
}

// ZG_OFFS_TC register
int8_t MPU6050_Get_ZGyro_Offset_TC(void) 
{
	uint8_t  temp;
	IIC_Read_Bytes(MPU6050_Address, MPU6050_RA_ZG_OFFS_TC, 1, &temp);
	temp &= 0x3F;
    return temp;
}
void MPU6050_Set_ZGyro_Offset_TC(int8_t offset) 
{
    IIC_Write_Bits(MPU6050_Address, MPU6050_RA_ZG_OFFS_TC, MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH, offset);
}

/** Set the I2C address of the specified slave (0-3).
 * @param num Slave number (0-3)
 * @param address New address for specified slave
 * @see getSlaveAddress()
 * @see MPU6050_RA_I2C_SLV0_ADDR
 */
void MPU6050_Set_Slave_Address(uint8_t num, uint8_t address) 
{
    if (num > 3) return;
    IIC_Write_Byte(MPU6050_Address, MPU6050_RA_I2C_SLV0_ADDR + num*3, address);
}

/** Reset the I2C Master.
 * This bit resets the I2C Master when set to 1 while I2C_MST_EN equals 0.
 * This bit automatically clears to 0 after the reset has been triggered.
 * @see MPU6050_RA_USER_CTRL
 * @see MPU6050_USERCTRL_I2C_MST_RESET_BIT
 */
void MPU6050_Reset_I2C_Master(void) 
{
    IIC_Write_Bit(MPU6050_Address, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_RESET_BIT, 1);
}

/** Set FIFO enabled status.
 * @param enabled New FIFO enabled status
 * @see getFIFOEnabled()
 * @see MPU6050_RA_USER_CTRL
 * @see MPU6050_USERCTRL_FIFO_EN_BIT
 */
void MPU6050_Set_FIFO_Enabled(uint8_t enabled) 
{
    IIC_Write_Bit(MPU6050_Address, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_FIFO_EN_BIT, enabled);
}

void MPU6050_Reset_DMP(void) 
{
    IIC_Write_Bit(MPU6050_Address, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_DMP_RESET_BIT, 1);
}


void DMP_Covert_Data(void);

/* ================================================================================================ *
 | Default MotionApps v2.0 42-byte FIFO packet structure:                                           |
 |                                                                                                  |
 | [QUAT W][      ][QUAT X][      ][QUAT Y][      ][QUAT Z][      ][GYRO X][      ][GYRO Y][      ] |
 |   0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15  16  17  18  19  20  21  22  23  |
 |                                                                                                  |
 | [GYRO Z][      ][ACC X ][      ][ACC Y ][      ][ACC Z ][      ][      ]                         |
 |  24  25  26  27  28  29  30  31  32  33  34  35  36  37  38  39  40  41                          |
 * ================================================================================================ */

// this block of memory gets written to the MPU on start-up, and it seems
// to be volatile memory, so it has to be done each time (it only takes ~1
// second though)

const unsigned char dmpMemory[MPU6050_DMP_CODE_SIZE] = 
{
    // bank 0, 256 bytes
    0xFB, 0x00, 0x00, 0x3E, 0x00, 0x0B, 0x00, 0x36, 0x00, 0x01, 0x00, 0x02, 0x00, 0x03, 0x00, 0x00,
    0x00, 0x65, 0x00, 0x54, 0xFF, 0xEF, 0x00, 0x00, 0xFA, 0x80, 0x00, 0x0B, 0x12, 0x82, 0x00, 0x01,
    0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x28, 0x00, 0x00, 0xFF, 0xFF, 0x45, 0x81, 0xFF, 0xFF, 0xFA, 0x72, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x03, 0xE8, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x7F, 0xFF, 0xFF, 0xFE, 0x80, 0x01,
    0x00, 0x1B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x3E, 0x03, 0x30, 0x40, 0x00, 0x00, 0x00, 0x02, 0xCA, 0xE3, 0x09, 0x3E, 0x80, 0x00, 0x00,
    0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00,
    0x41, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x0B, 0x2A, 0x00, 0x00, 0x16, 0x55, 0x00, 0x00, 0x21, 0x82,
    0xFD, 0x87, 0x26, 0x50, 0xFD, 0x80, 0x00, 0x00, 0x00, 0x1F, 0x00, 0x00, 0x00, 0x05, 0x80, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00,
    0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x6F, 0x00, 0x02, 0x65, 0x32, 0x00, 0x00, 0x5E, 0xC0,
    0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0xFB, 0x8C, 0x6F, 0x5D, 0xFD, 0x5D, 0x08, 0xD9, 0x00, 0x7C, 0x73, 0x3B, 0x00, 0x6C, 0x12, 0xCC,
    0x32, 0x00, 0x13, 0x9D, 0x32, 0x00, 0xD0, 0xD6, 0x32, 0x00, 0x08, 0x00, 0x40, 0x00, 0x01, 0xF4,
    0xFF, 0xE6, 0x80, 0x79, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0xD0, 0xD6, 0x00, 0x00, 0x27, 0x10,

    // bank 1, 256 bytes
    0xFB, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00,
    0x00, 0x00, 0xFA, 0x36, 0xFF, 0xBC, 0x30, 0x8E, 0x00, 0x05, 0xFB, 0xF0, 0xFF, 0xD9, 0x5B, 0xC8,
    0xFF, 0xD0, 0x9A, 0xBE, 0x00, 0x00, 0x10, 0xA9, 0xFF, 0xF4, 0x1E, 0xB2, 0x00, 0xCE, 0xBB, 0xF7,
    0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x04, 0x00, 0x02, 0x00, 0x02, 0x02, 0x00, 0x00, 0x0C,
    0xFF, 0xC2, 0x80, 0x00, 0x00, 0x01, 0x80, 0x00, 0x00, 0xCF, 0x80, 0x00, 0x40, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x14,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x03, 0x3F, 0x68, 0xB6, 0x79, 0x35, 0x28, 0xBC, 0xC6, 0x7E, 0xD1, 0x6C,
    0x80, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0xB2, 0x6A, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xF0, 0x00, 0x00, 0x00, 0x30,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x25, 0x4D, 0x00, 0x2F, 0x70, 0x6D, 0x00, 0x00, 0x05, 0xAE, 0x00, 0x0C, 0x02, 0xD0,

    // bank 2, 256 bytes
    0x00, 0x00, 0x00, 0x00, 0x00, 0x65, 0x00, 0x54, 0xFF, 0xEF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x01, 0x00, 0x00, 0x44, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x00, 0x00, 0x00, 0x01, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x65, 0x00, 0x00, 0x00, 0x54, 0x00, 0x00, 0xFF, 0xEF, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x1B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00,
    0x00, 0x1B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    // bank 3, 256 bytes
    0xD8, 0xDC, 0xBA, 0xA2, 0xF1, 0xDE, 0xB2, 0xB8, 0xB4, 0xA8, 0x81, 0x91, 0xF7, 0x4A, 0x90, 0x7F,
    0x91, 0x6A, 0xF3, 0xF9, 0xDB, 0xA8, 0xF9, 0xB0, 0xBA, 0xA0, 0x80, 0xF2, 0xCE, 0x81, 0xF3, 0xC2,
    0xF1, 0xC1, 0xF2, 0xC3, 0xF3, 0xCC, 0xA2, 0xB2, 0x80, 0xF1, 0xC6, 0xD8, 0x80, 0xBA, 0xA7, 0xDF,
    0xDF, 0xDF, 0xF2, 0xA7, 0xC3, 0xCB, 0xC5, 0xB6, 0xF0, 0x87, 0xA2, 0x94, 0x24, 0x48, 0x70, 0x3C,
    0x95, 0x40, 0x68, 0x34, 0x58, 0x9B, 0x78, 0xA2, 0xF1, 0x83, 0x92, 0x2D, 0x55, 0x7D, 0xD8, 0xB1,
    0xB4, 0xB8, 0xA1, 0xD0, 0x91, 0x80, 0xF2, 0x70, 0xF3, 0x70, 0xF2, 0x7C, 0x80, 0xA8, 0xF1, 0x01,
    0xB0, 0x98, 0x87, 0xD9, 0x43, 0xD8, 0x86, 0xC9, 0x88, 0xBA, 0xA1, 0xF2, 0x0E, 0xB8, 0x97, 0x80,
    0xF1, 0xA9, 0xDF, 0xDF, 0xDF, 0xAA, 0xDF, 0xDF, 0xDF, 0xF2, 0xAA, 0xC5, 0xCD, 0xC7, 0xA9, 0x0C,
    0xC9, 0x2C, 0x97, 0x97, 0x97, 0x97, 0xF1, 0xA9, 0x89, 0x26, 0x46, 0x66, 0xB0, 0xB4, 0xBA, 0x80,
    0xAC, 0xDE, 0xF2, 0xCA, 0xF1, 0xB2, 0x8C, 0x02, 0xA9, 0xB6, 0x98, 0x00, 0x89, 0x0E, 0x16, 0x1E,
    0xB8, 0xA9, 0xB4, 0x99, 0x2C, 0x54, 0x7C, 0xB0, 0x8A, 0xA8, 0x96, 0x36, 0x56, 0x76, 0xF1, 0xB9,
    0xAF, 0xB4, 0xB0, 0x83, 0xC0, 0xB8, 0xA8, 0x97, 0x11, 0xB1, 0x8F, 0x98, 0xB9, 0xAF, 0xF0, 0x24,
    0x08, 0x44, 0x10, 0x64, 0x18, 0xF1, 0xA3, 0x29, 0x55, 0x7D, 0xAF, 0x83, 0xB5, 0x93, 0xAF, 0xF0,
    0x00, 0x28, 0x50, 0xF1, 0xA3, 0x86, 0x9F, 0x61, 0xA6, 0xDA, 0xDE, 0xDF, 0xD9, 0xFA, 0xA3, 0x86,
    0x96, 0xDB, 0x31, 0xA6, 0xD9, 0xF8, 0xDF, 0xBA, 0xA6, 0x8F, 0xC2, 0xC5, 0xC7, 0xB2, 0x8C, 0xC1,
    0xB8, 0xA2, 0xDF, 0xDF, 0xDF, 0xA3, 0xDF, 0xDF, 0xDF, 0xD8, 0xD8, 0xF1, 0xB8, 0xA8, 0xB2, 0x86,

    // bank 4, 256 bytes
    0xB4, 0x98, 0x0D, 0x35, 0x5D, 0xB8, 0xAA, 0x98, 0xB0, 0x87, 0x2D, 0x35, 0x3D, 0xB2, 0xB6, 0xBA,
    0xAF, 0x8C, 0x96, 0x19, 0x8F, 0x9F, 0xA7, 0x0E, 0x16, 0x1E, 0xB4, 0x9A, 0xB8, 0xAA, 0x87, 0x2C,
    0x54, 0x7C, 0xB9, 0xA3, 0xDE, 0xDF, 0xDF, 0xA3, 0xB1, 0x80, 0xF2, 0xC4, 0xCD, 0xC9, 0xF1, 0xB8,
    0xA9, 0xB4, 0x99, 0x83, 0x0D, 0x35, 0x5D, 0x89, 0xB9, 0xA3, 0x2D, 0x55, 0x7D, 0xB5, 0x93, 0xA3,
    0x0E, 0x16, 0x1E, 0xA9, 0x2C, 0x54, 0x7C, 0xB8, 0xB4, 0xB0, 0xF1, 0x97, 0x83, 0xA8, 0x11, 0x84,
    0xA5, 0x09, 0x98, 0xA3, 0x83, 0xF0, 0xDA, 0x24, 0x08, 0x44, 0x10, 0x64, 0x18, 0xD8, 0xF1, 0xA5,
    0x29, 0x55, 0x7D, 0xA5, 0x85, 0x95, 0x02, 0x1A, 0x2E, 0x3A, 0x56, 0x5A, 0x40, 0x48, 0xF9, 0xF3,
    0xA3, 0xD9, 0xF8, 0xF0, 0x98, 0x83, 0x24, 0x08, 0x44, 0x10, 0x64, 0x18, 0x97, 0x82, 0xA8, 0xF1,
    0x11, 0xF0, 0x98, 0xA2, 0x24, 0x08, 0x44, 0x10, 0x64, 0x18, 0xDA, 0xF3, 0xDE, 0xD8, 0x83, 0xA5,
    0x94, 0x01, 0xD9, 0xA3, 0x02, 0xF1, 0xA2, 0xC3, 0xC5, 0xC7, 0xD8, 0xF1, 0x84, 0x92, 0xA2, 0x4D,
    0xDA, 0x2A, 0xD8, 0x48, 0x69, 0xD9, 0x2A, 0xD8, 0x68, 0x55, 0xDA, 0x32, 0xD8, 0x50, 0x71, 0xD9,
    0x32, 0xD8, 0x70, 0x5D, 0xDA, 0x3A, 0xD8, 0x58, 0x79, 0xD9, 0x3A, 0xD8, 0x78, 0x93, 0xA3, 0x4D,
    0xDA, 0x2A, 0xD8, 0x48, 0x69, 0xD9, 0x2A, 0xD8, 0x68, 0x55, 0xDA, 0x32, 0xD8, 0x50, 0x71, 0xD9,
    0x32, 0xD8, 0x70, 0x5D, 0xDA, 0x3A, 0xD8, 0x58, 0x79, 0xD9, 0x3A, 0xD8, 0x78, 0xA8, 0x8A, 0x9A,
    0xF0, 0x28, 0x50, 0x78, 0x9E, 0xF3, 0x88, 0x18, 0xF1, 0x9F, 0x1D, 0x98, 0xA8, 0xD9, 0x08, 0xD8,
    0xC8, 0x9F, 0x12, 0x9E, 0xF3, 0x15, 0xA8, 0xDA, 0x12, 0x10, 0xD8, 0xF1, 0xAF, 0xC8, 0x97, 0x87,

    // bank 5, 256 bytes
    0x34, 0xB5, 0xB9, 0x94, 0xA4, 0x21, 0xF3, 0xD9, 0x22, 0xD8, 0xF2, 0x2D, 0xF3, 0xD9, 0x2A, 0xD8,
    0xF2, 0x35, 0xF3, 0xD9, 0x32, 0xD8, 0x81, 0xA4, 0x60, 0x60, 0x61, 0xD9, 0x61, 0xD8, 0x6C, 0x68,
    0x69, 0xD9, 0x69, 0xD8, 0x74, 0x70, 0x71, 0xD9, 0x71, 0xD8, 0xB1, 0xA3, 0x84, 0x19, 0x3D, 0x5D,
    0xA3, 0x83, 0x1A, 0x3E, 0x5E, 0x93, 0x10, 0x30, 0x81, 0x10, 0x11, 0xB8, 0xB0, 0xAF, 0x8F, 0x94,
    0xF2, 0xDA, 0x3E, 0xD8, 0xB4, 0x9A, 0xA8, 0x87, 0x29, 0xDA, 0xF8, 0xD8, 0x87, 0x9A, 0x35, 0xDA,
    0xF8, 0xD8, 0x87, 0x9A, 0x3D, 0xDA, 0xF8, 0xD8, 0xB1, 0xB9, 0xA4, 0x98, 0x85, 0x02, 0x2E, 0x56,
    0xA5, 0x81, 0x00, 0x0C, 0x14, 0xA3, 0x97, 0xB0, 0x8A, 0xF1, 0x2D, 0xD9, 0x28, 0xD8, 0x4D, 0xD9,
    0x48, 0xD8, 0x6D, 0xD9, 0x68, 0xD8, 0xB1, 0x84, 0x0D, 0xDA, 0x0E, 0xD8, 0xA3, 0x29, 0x83, 0xDA,
    0x2C, 0x0E, 0xD8, 0xA3, 0x84, 0x49, 0x83, 0xDA, 0x2C, 0x4C, 0x0E, 0xD8, 0xB8, 0xB0, 0xA8, 0x8A,
    0x9A, 0xF5, 0x20, 0xAA, 0xDA, 0xDF, 0xD8, 0xA8, 0x40, 0xAA, 0xD0, 0xDA, 0xDE, 0xD8, 0xA8, 0x60,
    0xAA, 0xDA, 0xD0, 0xDF, 0xD8, 0xF1, 0x97, 0x86, 0xA8, 0x31, 0x9B, 0x06, 0x99, 0x07, 0xAB, 0x97,
    0x28, 0x88, 0x9B, 0xF0, 0x0C, 0x20, 0x14, 0x40, 0xB8, 0xB0, 0xB4, 0xA8, 0x8C, 0x9C, 0xF0, 0x04,
    0x28, 0x51, 0x79, 0x1D, 0x30, 0x14, 0x38, 0xB2, 0x82, 0xAB, 0xD0, 0x98, 0x2C, 0x50, 0x50, 0x78,
    0x78, 0x9B, 0xF1, 0x1A, 0xB0, 0xF0, 0x8A, 0x9C, 0xA8, 0x29, 0x51, 0x79, 0x8B, 0x29, 0x51, 0x79,
    0x8A, 0x24, 0x70, 0x59, 0x8B, 0x20, 0x58, 0x71, 0x8A, 0x44, 0x69, 0x38, 0x8B, 0x39, 0x40, 0x68,
    0x8A, 0x64, 0x48, 0x31, 0x8B, 0x30, 0x49, 0x60, 0xA5, 0x88, 0x20, 0x09, 0x71, 0x58, 0x44, 0x68,

    // bank 6, 256 bytes
    0x11, 0x39, 0x64, 0x49, 0x30, 0x19, 0xF1, 0xAC, 0x00, 0x2C, 0x54, 0x7C, 0xF0, 0x8C, 0xA8, 0x04,
    0x28, 0x50, 0x78, 0xF1, 0x88, 0x97, 0x26, 0xA8, 0x59, 0x98, 0xAC, 0x8C, 0x02, 0x26, 0x46, 0x66,
    0xF0, 0x89, 0x9C, 0xA8, 0x29, 0x51, 0x79, 0x24, 0x70, 0x59, 0x44, 0x69, 0x38, 0x64, 0x48, 0x31,
    0xA9, 0x88, 0x09, 0x20, 0x59, 0x70, 0xAB, 0x11, 0x38, 0x40, 0x69, 0xA8, 0x19, 0x31, 0x48, 0x60,
    0x8C, 0xA8, 0x3C, 0x41, 0x5C, 0x20, 0x7C, 0x00, 0xF1, 0x87, 0x98, 0x19, 0x86, 0xA8, 0x6E, 0x76,
    0x7E, 0xA9, 0x99, 0x88, 0x2D, 0x55, 0x7D, 0x9E, 0xB9, 0xA3, 0x8A, 0x22, 0x8A, 0x6E, 0x8A, 0x56,
    0x8A, 0x5E, 0x9F, 0xB1, 0x83, 0x06, 0x26, 0x46, 0x66, 0x0E, 0x2E, 0x4E, 0x6E, 0x9D, 0xB8, 0xAD,
    0x00, 0x2C, 0x54, 0x7C, 0xF2, 0xB1, 0x8C, 0xB4, 0x99, 0xB9, 0xA3, 0x2D, 0x55, 0x7D, 0x81, 0x91,
    0xAC, 0x38, 0xAD, 0x3A, 0xB5, 0x83, 0x91, 0xAC, 0x2D, 0xD9, 0x28, 0xD8, 0x4D, 0xD9, 0x48, 0xD8,
    0x6D, 0xD9, 0x68, 0xD8, 0x8C, 0x9D, 0xAE, 0x29, 0xD9, 0x04, 0xAE, 0xD8, 0x51, 0xD9, 0x04, 0xAE,
    0xD8, 0x79, 0xD9, 0x04, 0xD8, 0x81, 0xF3, 0x9D, 0xAD, 0x00, 0x8D, 0xAE, 0x19, 0x81, 0xAD, 0xD9,
    0x01, 0xD8, 0xF2, 0xAE, 0xDA, 0x26, 0xD8, 0x8E, 0x91, 0x29, 0x83, 0xA7, 0xD9, 0xAD, 0xAD, 0xAD,
    0xAD, 0xF3, 0x2A, 0xD8, 0xD8, 0xF1, 0xB0, 0xAC, 0x89, 0x91, 0x3E, 0x5E, 0x76, 0xF3, 0xAC, 0x2E,
    0x2E, 0xF1, 0xB1, 0x8C, 0x5A, 0x9C, 0xAC, 0x2C, 0x28, 0x28, 0x28, 0x9C, 0xAC, 0x30, 0x18, 0xA8,
    0x98, 0x81, 0x28, 0x34, 0x3C, 0x97, 0x24, 0xA7, 0x28, 0x34, 0x3C, 0x9C, 0x24, 0xF2, 0xB0, 0x89,
    0xAC, 0x91, 0x2C, 0x4C, 0x6C, 0x8A, 0x9B, 0x2D, 0xD9, 0xD8, 0xD8, 0x51, 0xD9, 0xD8, 0xD8, 0x79,

    // bank 7, 138 bytes (remainder)
    0xD9, 0xD8, 0xD8, 0xF1, 0x9E, 0x88, 0xA3, 0x31, 0xDA, 0xD8, 0xD8, 0x91, 0x2D, 0xD9, 0x28, 0xD8,
    0x4D, 0xD9, 0x48, 0xD8, 0x6D, 0xD9, 0x68, 0xD8, 0xB1, 0x83, 0x93, 0x35, 0x3D, 0x80, 0x25, 0xDA,
    0xD8, 0xD8, 0x85, 0x69, 0xDA, 0xD8, 0xD8, 0xB4, 0x93, 0x81, 0xA3, 0x28, 0x34, 0x3C, 0xF3, 0xAB,
    0x8B, 0xF8, 0xA3, 0x91, 0xB6, 0x09, 0xB4, 0xD9, 0xAB, 0xDE, 0xFA, 0xB0, 0x87, 0x9C, 0xB9, 0xA3,
    0xDD, 0xF1, 0xA3, 0xA3, 0xA3, 0xA3, 0x95, 0xF1, 0xA3, 0xA3, 0xA3, 0x9D, 0xF1, 0xA3, 0xA3, 0xA3,
    0xA3, 0xF2, 0xA3, 0xB4, 0x90, 0x80, 0xF2, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3,
    0xA3, 0xB2, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xB0, 0x87, 0xB5, 0x99, 0xF1, 0xA3, 0xA3, 0xA3,
    0x98, 0xF1, 0xA3, 0xA3, 0xA3, 0xA3, 0x97, 0xA3, 0xA3, 0xA3, 0xA3, 0xF3, 0x9B, 0xA3, 0xA3, 0xDC,
    0xB9, 0xA7, 0xF1, 0x26, 0x26, 0x26, 0xD8, 0xD8, 0xFF
};

// thanks to Noah Zerkin for piecing this stuff together!
const unsigned char dmpConfig[MPU6050_DMP_CONFIG_SIZE] = 
{
//  BANK    OFFSET  LENGTH  [DATA]
    0x03,   0x7B,   0x03,   0x4C, 0xCD, 0x6C,         // FCFG_1 inv_set_gyro_calibration
    0x03,   0xAB,   0x03,   0x36, 0x56, 0x76,         // FCFG_3 inv_set_gyro_calibration
    0x00,   0x68,   0x04,   0x02, 0xCB, 0x47, 0xA2,   // D_0_104 inv_set_gyro_calibration
    0x02,   0x18,   0x04,   0x00, 0x05, 0x8B, 0xC1,   // D_0_24 inv_set_gyro_calibration
    0x01,   0x0C,   0x04,   0x00, 0x00, 0x00, 0x00,   // D_1_152 inv_set_accel_calibration
    0x03,   0x7F,   0x06,   0x0C, 0xC9, 0x2C, 0x97, 0x97, 0x97, // FCFG_2 inv_set_accel_calibration
    0x03,   0x89,   0x03,   0x26, 0x46, 0x66,         // FCFG_7 inv_set_accel_calibration
    0x00,   0x6C,   0x02,   0x20, 0x00,               // D_0_108 inv_set_accel_calibration
    0x02,   0x40,   0x04,   0x00, 0x00, 0x00, 0x00,   // CPASS_MTX_00 inv_set_compass_calibration
    0x02,   0x44,   0x04,   0x00, 0x00, 0x00, 0x00,   // CPASS_MTX_01
    0x02,   0x48,   0x04,   0x00, 0x00, 0x00, 0x00,   // CPASS_MTX_02
    0x02,   0x4C,   0x04,   0x00, 0x00, 0x00, 0x00,   // CPASS_MTX_10
    0x02,   0x50,   0x04,   0x00, 0x00, 0x00, 0x00,   // CPASS_MTX_11
    0x02,   0x54,   0x04,   0x00, 0x00, 0x00, 0x00,   // CPASS_MTX_12
    0x02,   0x58,   0x04,   0x00, 0x00, 0x00, 0x00,   // CPASS_MTX_20
    0x02,   0x5C,   0x04,   0x00, 0x00, 0x00, 0x00,   // CPASS_MTX_21
    0x02,   0xBC,   0x04,   0x00, 0x00, 0x00, 0x00,   // CPASS_MTX_22
    0x01,   0xEC,   0x04,   0x00, 0x00, 0x40, 0x00,   // D_1_236 inv_apply_endian_accel
    0x03,   0x7F,   0x06,   0x0C, 0xC9, 0x2C, 0x97, 0x97, 0x97, // FCFG_2 inv_set_mpu_sensors
    0x04,   0x02,   0x03,   0x0D, 0x35, 0x5D,         // CFG_MOTION_BIAS inv_turn_on_bias_from_no_motion
    0x04,   0x09,   0x04,   0x87, 0x2D, 0x35, 0x3D,   // FCFG_5 inv_set_bias_update
    0x00,   0xA3,   0x01,   0x00,                     // D_0_163 inv_set_dead_zone
                 // SPECIAL 0x01 = enable interrupts
    0x00,   0x00,   0x00,   0x01, // SET INT_ENABLE at i=22, SPECIAL INSTRUCTION
    0x07,   0x86,   0x01,   0xFE,                     // CFG_6 inv_set_fifo_interupt
    0x07,   0x41,   0x05,   0xF1, 0x20, 0x28, 0x30, 0x38, // CFG_8 inv_send_quaternion
    0x07,   0x7E,   0x01,   0x30,                     // CFG_16 inv_set_footer
    0x07,   0x46,   0x01,   0x9A,                     // CFG_GYRO_SOURCE inv_send_gyro
    0x07,   0x47,   0x04,   0xF1, 0x28, 0x30, 0x38,   // CFG_9 inv_send_gyro -> inv_construct3_fifo
    0x07,   0x6C,   0x04,   0xF1, 0x28, 0x30, 0x38,   // CFG_12 inv_send_accel -> inv_construct3_fifo
    0x02,   0x16,   0x02,   0x00, 0x01                // D_0_22 inv_set_fifo_rate

    // This very last 0x01 WAS a 0x09, which drops the FIFO rate down to 20 Hz. 0x07 is 25 Hz,
    // 0x01 is 100Hz. Going faster than 100Hz (0x00=200Hz) tends to result in very noisy data.
    // DMP output frequency is calculated easily using this equation: (200Hz / (1 + value))

    // It is important to make sure the host processor can keep up with reading and processing
    // the FIFO output at the desired rate. Handling FIFO overflow cleanly is also a good idea.
};

const unsigned char dmpUpdates[MPU6050_DMP_UPDATES_SIZE] = 
{
    0x01,   0xB2,   0x02,   0xFF, 0xFF,
    0x01,   0x90,   0x04,   0x09, 0x23, 0xA1, 0x35,
    0x01,   0x6A,   0x02,   0x06, 0x00,
    0x01,   0x60,   0x08,   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00,   0x60,   0x04,   0x40, 0x00, 0x00, 0x00,
    0x01,   0x62,   0x02,   0x00, 0x00,
    0x00,   0x60,   0x04,   0x00, 0x40, 0x00, 0x00
};



uint16_t dmpPacketSize;	 //FIFO数据	包字节数
struct DMP_FIFO_map DMP_DATA; //FIFO的数据解析，参考头文件的结构体定义

//取两个数中最小的那个
uint8_t min(uint8_t x ,uint8_t y)
{
	if( x < y)return x;
	else return y;
}


//初始化DMP引擎
uint8_t MPU6050_DMP_Initialize(void)
{
	uint8_t dmpUpdate[16], j;
	uint16_t pos = 0;
	uint8_t fifoCount;
	uint8_t fifoBuffer[128];
	int8_t xgOffset	, ygOffset , zgOffset;
	
    	// reset device
    	//======配置DMP引擎=========
    	//复位MPU6050.
    MPU6050_Reset();
    delay_ms(50); // wait after reset 50ms
  	  //禁止休眠模式.
    MPU6050_Set_Sleep_Enabled(0);
 	   //读取MPU6050硬件版本.
 	   //正在选择用户内存块.
    MPU6050_Set_Memory_Bank(0x10, 1, 1);
 	   //printf(("正在选择用户内存字节...\r\n"));
    MPU6050_Set_Memory_Start_Address(0x06);
	    //printf(("正在检查硬件版本...\r\n"));
    MPU6050_Read_Memory_Byte();
   	 ////printf(("Revision @ user[16][6] = \r\n"));
   	 //printf(("复位内存块...\r\n"));
    MPU6050_Set_Memory_Bank(0, 0, 0);
 	   // check OTP bank valid
 	   //printf(("读取OTP块有效标志...\r\n"));
    MPU6050_Get_OTP_Bank_Valid();
	//     //printf("OTP bank is ");
	//     //printf(otpValid ? ("valid!") : ("invalid!"));
    	// get X/Y/Z gyro offsets
    	//printf(("读取加速度偏移值...\r\n"));
    xgOffset = MPU6050_Get_XGyro_Offset_TC();
    ygOffset = MPU6050_Get_YGyro_Offset_TC();
    zgOffset = MPU6050_Get_ZGyro_Offset_TC(); 
		
  	

	// setup weird slave stuff (?)
    	//printf(("设置从器件地址为0x7F...\r\n"));
    MPU6050_Set_Slave_Address(0, 0x7F);
   	 //printf(("禁止IIC主器件模式...\r\n"));
    MPU6050_Set_I2C_Master_Mode_Enabled(0);
  	  //主控制器的I2C与	MPU6050的AUXI2C	直通。控制器可以直接访问HMC5883L
    MPU6050_Set_I2C_Bypass_Enabled(1);	     
   	 //printf(("设置从器件地址为0x68...\r\n"));
    MPU6050_Set_Slave_Address(0, 0x68);
   	 //printf(("复位IIC主器件控制权...\r\n"));
    MPU6050_Reset_I2C_Master();
    delay_ms(20);

   	 // load DMP code into memory banks
   	 //printf(("正在写入DMP代码段到MPU6050 \r\n"));
    if (MPU6050_Write_Prog_Memory_Block(dmpMemory, MPU6050_DMP_CODE_SIZE, 0, 0, 1)) 
    {
    	    //printf(("DMP代码写入校验成功...\r\n"));
    	    //printf(("配置DMP和有关设置...\r\n"));
   	     // write DMP configuration
   	      //printf(("正在写入DMP配置代码到MPU6050内存...\r\n"));
        if (MPU6050_Write_Prog_DMP_Configuration_Set(dmpConfig, MPU6050_DMP_CONFIG_SIZE)) 
	{
      	      //printf(("DMP配置代码写入校验成功\r\n"));
     	       //printf(("设置Z轴角速度时钟源...\r\n"));
            MPU6050_Set_Clock_Source(MPU6050_CLOCK_PLL_ZGYRO);
    	        //printf(("使能DMP引擎和FIFO中断...\r\n"));
            MPU6050_Set_Int_Enabled(0x12);
          	//printf(("设置DMP采样率为200Hz...\r\n"));
            MPU6050_Set_Rate(4); // 1khz / (1 + 4) = 200 Hz
        	//printf(("设置外部同步帧到TEMP_OUT_L[0]...\r\n"));
            MPU6050_Set_External_Frame_Sync(MPU6050_EXT_SYNC_TEMP_OUT_L);
       	     	//printf(("设置DLPF带宽为42Hz...\r\n"));
            MPU6050_Set_DLPF_Mode(MPU6050_DLPF_BW_42);
            	//printf(("设置角速度精度为 +/- 2000 deg/sec...\r\n"));
            MPU6050_Set_Gyro_Range(MPU6050_GYRO_FS_2000);
            	//printf(("设置DMP配置字节...\r\n"));
            MPU6050_Set_DMP_Config1(0x03);
            MPU6050_Set_DMP_Config2(0x00);
           	//printf(("清楚OTP块标志...\r\n"));
            MPU6050_Set_OTP_Bank_Valid(0);
            	//printf(("设置X/Y/Z轴角速度为先前值...\r\n"));
            MPU6050_Set_XGyro_Offset_TC(xgOffset);
            MPU6050_Set_YGyro_Offset_TC(ygOffset);
            MPU6050_Set_ZGyro_Offset_TC(zgOffset);
						
				//		DMPCalibrate();		//tobe tested
						
            //printf(("写入最后内存跟新到 1/7 ...\r\n"));
            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = dmpUpdates[pos];
            MPU6050_Write_Memory_Block(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1], 1, 0);
            //printf(("写入最后内存跟新到 2/7 ...\r\n"));
            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = dmpUpdates[pos];
            MPU6050_Write_Memory_Block(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1], 0, 0);
            //printf(("正在复位FIFO...\r\n"));
            MPU6050_Reset_FIFO();
            //printf(("正在读取FIFO计数...\r\n"));
            fifoCount = MPU6050_Get_FIFO_Count();
            MPU6050_Get_FIFO_Bytes(fifoBuffer, fifoCount);
            //printf(("正在设置运动阈值为2...\r\n"));
            MPU6050_Set_Motion_Detection_Threshold(2);
            //printf(("正在设置0运动检测阈值为156...\r\n"));
            MPU6050_Set_Zero_Motion_Detection_Threshold(156);
            //printf(("正在设置运动检测持续时间为80...\r\n"));
            MPU6050_Set_Motion_Detection_Duration(80);
            //printf(("正在设置0运动检测持续时间为0...\r\n"));
            MPU6050_Set_Zero_Motion_Detection_Duration(0);
            //printf(("复位FIFO...\r\n"));
            MPU6050_Reset_FIFO();
            //printf(("正在使能FIFO...\r\n"));
            MPU6050_Set_FIFO_Enabled(1);
            //printf(("正在使能DMP...\r\n"));
            MPU6050_Set_DMP_Enabled(1);
            //printf(("复位DMPDMP...\r\n"));
            MPU6050_Reset_DMP();
			      //printf(("写入最后内存跟新到 3/7 ......\r\n"));
            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = dmpUpdates[pos];
            MPU6050_Write_Memory_Block(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1], 0, 0);
            //printf(("写入最后内存跟新到 4/7 ......\r\n"));
            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = dmpUpdates[pos];
            MPU6050_Write_Memory_Block(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1], 0, 0);
            //printf(("写入最后内存跟新到 5/7 ......\r\n"));
            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = dmpUpdates[pos];
            MPU6050_Write_Memory_Block(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1], 0, 0);
            //printf(("等待FIFO计数>=2...\r\n"));
            while ((fifoCount = MPU6050_Get_FIFO_Count()) < 3);
            //printf(("复位 FIFO...\r\n"));
            MPU6050_Get_FIFO_Bytes(fifoBuffer, min(fifoCount, 128)); // safeguard only 128 bytes
            //printf(("读取中断状态...\r\n"));
            MPU6050_Get_Int_Status();
            //printf(("写入最后内存跟新到 6/7 ......\r\n"));
            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = dmpUpdates[pos];
            MPU6050_Read_Memory_Block(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);
            //printf(("等待FIFO计数>=2...\r\n"));
            while ((fifoCount = MPU6050_Get_FIFO_Count()) < 3);
            //printf(("正在读取FIFO...\r\n"));
            MPU6050_Get_FIFO_Bytes(fifoBuffer, min(fifoCount, 128)); // safeguard only 128 bytes
            //printf(("正在读取中断状态...\r\n"));
            MPU6050_Get_Int_Status();
            //printf(("写入最后内存跟新到 7/7 ......\r\n"));
            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = dmpUpdates[pos];
            MPU6050_Write_Memory_Block(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1], 0, 0);
            //printf(("DMP设置一切正常...\r\n"));

            //printf(("关闭DMP引擎...\r\n"));
            MPU6050_Set_DMP_Enabled(0);
            //printf(("设置内部42字节缓冲包...\r\n"));
            dmpPacketSize = 42;	 
            //printf(("最后一次复位FIFO和中断状态...\r\n"));
            MPU6050_Reset_FIFO();
            MPU6050_Get_Int_Status();
            //printf(("打开DMP引擎...\r\n"));
			      MPU6050_Set_DMP_Enabled(1);
			      //printf(("DMP引擎准备就绪,等待第一次数据中断...\r\n"));
			      MPU6050_Get_Int_Status();

        } else {
            //printf(("DMP引擎配置校验出错...\r\n"));
            return 2; // configuration block loading failed
        }
    } else {
         //printf(("DMP代码校验出错.\r\n"));
        return 1; // main binary block loading failed
    }
    //printf(("======DMP引擎初始化完成========\r\n"));
    return 0; // success
}

uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
static uint16_t test_counter = 0;
static uint16_t test_counter_wrong = 0;
static uint16_t test_counter_wrong_2 = 0;
float Quaternion_Normalization;
float Quaternion_Normalization_Min = 268200000.0, Quaternion_Normalization_Max = 268600000.0;


void DMP_Routing(void)
{
	int i;	
	struct DMP_FIFO_map DMP_DATA_Last_Time;
	uint8_t* ptr = (uint8_t*) & DMP_DATA;	
	DMP_DATA_Last_Time = DMP_DATA;	
	
	//while((MPU6050_is_DRY() == 0) && (fifoCount < dmpPacketSize));
		mpuIntStatus = MPU6050_Get_Int_Status();	
		// get current FIFO count
	    fifoCount = MPU6050_Get_FIFO_Count();
		// check for overflow (this should never happen unless our code is too inefficient)
	if ((mpuIntStatus & 0x10) || fifoCount == 1024) 
	{
	      		// reset so we can continue cleanly
		MPU6050_Reset_FIFO();
	    		// otherwise, check for DMP data ready interrupt (this should happen frequently)
	}else if (mpuIntStatus & 0x02) 
		{
				// wait for correct available data length, should be a VERY short wait
			while (fifoCount < dmpPacketSize) fifoCount = MPU6050_Get_FIFO_Count();
				// read a packet from FIFO
			MPU6050_Get_FIFO_Bytes(fifoBuffer, dmpPacketSize);
				// track FIFO count here in case there is > 1 packet available
				// (this lets us immediately read more without waiting for an interrupt)
			fifoCount -= dmpPacketSize;

			for(i=0 ; i < dmpPacketSize; i+=2) 
			{
				ptr[i]   = fifoBuffer[i+1];  //数据大小端的处理。
				ptr[i+1] = fifoBuffer[i];
			}
			
			//We can detect a corrupted FIFO by monitoring the quaternion data and
			//ensuring that the magnitude is always normalized to one. This
			//shouldn't happen in normal operation, but if an I2C error occurs,
			//the FIFO reads might become misaligned.	
			//Let's start by scaling down the quaternion data to avoid long long
			//math.
		
			Quaternion_Normalization = ((float)DMP_DATA.qw) * ((float)DMP_DATA.qw) + ((float)DMP_DATA.qx) * ((float)DMP_DATA.qx) 
						+ ((float)DMP_DATA.qy) * ((float)DMP_DATA.qy) + ((float)DMP_DATA.qz) * ((float)DMP_DATA.qz);

			if(Quaternion_Normalization > Quaternion_Normalization_Max || Quaternion_Normalization < Quaternion_Normalization_Min)
			{
				
				if(test_counter == (test_counter_wrong + 1)) 
					test_counter_wrong_2++;
				if(test_counter_wrong_2 == 3) 
				{
					MPU6050_Reset_FIFO();
					test_counter_wrong_2 = 0;
				}
					// if wrong, the data remain the last time
				DMP_DATA = DMP_DATA_Last_Time;
				test_counter_wrong = test_counter;
			}
			
			test_counter ++;
			
		}
		if(test_counter >= 60000)
		{
			test_counter = 0;
		}
	

}


//static const float gyro_max = 1000;
	//DMP_DATA.GYROx 即为直接的角度deg
float Get_DMP_Gyro_x()
{
	//if(DMP_DATA.GYROx > gyro_max) DMP_DATA.GYROx = gyro_max;
	//if(DMP_DATA.GYROx < - gyro_max) DMP_DATA.GYROx = - gyro_max;
	float Gx;
	Gx = ((float)DMP_DATA.GYROx);
	Gx = KalmanFilter(Gx, & Kalman_Gx);
	return Gx;
}

float Get_DMP_Gyro_y()
{
	//if(DMP_DATA.GYROy > gyro_max) DMP_DATA.GYROy = gyro_max;
	//if(DMP_DATA.GYROy < - gyro_max) DMP_DATA.GYROy = - gyro_max;
	float Gy;
	Gy = ((float)DMP_DATA.GYROy);
	Gy = KalmanFilter(Gy, & Kalman_Gy);
	return Gy;
}

float Get_DMP_Gyro_z()
{
	//if(DMP_DATA.GYROz > gyro_max) DMP_DATA.GYROz = gyro_max;
	//if(DMP_DATA.GYROz < - gyro_max) DMP_DATA.GYROz = - gyro_max;
	float Gz;
	Gz = ((float)DMP_DATA.GYROz);
	Gz = KalmanFilter(Gz, & Kalman_Gz);
	return Gz;
}

float Get_DMP_Acc_x()
{
	return (((float)DMP_DATA.ACCx)/DMP_ACC_SCALE)*ONE_G;
}

float Get_DMP_Acc_y()
{
	return (((float)DMP_DATA.ACCy)/DMP_ACC_SCALE)*ONE_G;
}

float Get_DMP_Acc_z()
{
	return (((float)DMP_DATA.ACCz)/DMP_ACC_SCALE)*ONE_G;
}

float Get_DMP_qw()
{
	return ((float)DMP_DATA.qw);
}

float Get_DMP_qx()
{
	return ((float)DMP_DATA.qx);
}

float Get_DMP_qy()
{
	return ((float)DMP_DATA.qy);
}

float Get_DMP_qz()
{
	return ((float)DMP_DATA.qz);
}