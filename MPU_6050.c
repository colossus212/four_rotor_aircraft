
#include "MPU_6050.h"
#include "IIC.h"
#include "r_cg_macrodriver.h"
#include "r_cg_cgc.h"
#include "r_cg_port.h"
#include "r_cg_timer.h"
#include "r_cg_userdefine.h"



const float pi = 3.1415926;
float r_xz=0.25,r_yx=0.25,r_yz=0.25;
float q_xz=0.0025,q_yx=0.0025,q_yz=0.0025;
float Ax,Ay,Az,Gx,Gy,Gz;                       //The raw data of accelerometers and gyroscope
float k_xz=0,k_yx=0,k_yz=0;
float Aax,Aay;
float oAax,oAay,oGx,oGy,oGz;	//ƫ��
float p_xz=1,p_yx=1,p_yz=1;
float aax,aay,accx,accy,accz;
float AcceRatio = 16384.0;   
float GyroRatio = 131.0;
float t=0.025;	
uint8_t sample_times=200;
MPU_TypeDef data,*mpu_ptr=&data;



void MPU6050_start(){//�����￪ʼ 
	uint8_t i;
	Ax=0;Ay=0;Az=0;
	Gx=0;Gy=0;Gz=0;//���ٶȼ�������ԭʼ���� 
	
	MPU6050_Init(); 
	for (i=0;i<sample_times;i++){
		MPU6050_RD_XYZ(mpu_ptr);//��ȡAX,AY,AZ��������ٶ� 
		Ax+=mpu_ptr->Ax;
		Ay+=mpu_ptr->Ay;   //���������Ķ������һ���ṹ�������ˣ���userdefine���� 
		Az+=mpu_ptr->Az;
		Gx+=mpu_ptr->Gx;
		Gy+=mpu_ptr->Gy;		
		Gz+=mpu_ptr->Gz;
	}	
	oAax = ( atan(Ax / Az)*180 ) / pi;
  	oAay = ( atan(Ay / Az)*180 ) / pi;
	oGx  =Gx/sample_times/GyroRatio;
	oGy  =Gy/sample_times/GyroRatio;
	oGz  =Gz/sample_times/GyroRatio;
	R_TAU0_Channel5_Start();
}



void get_MPUdata()
{
	MPU6050_RD_XYZ(mpu_ptr);
	accx=(float)mpu_ptr->Ax;
	accy=(float)mpu_ptr->Ay;
	accz=(float)mpu_ptr->Az;
	aax = atan(accx / accz)*180 / pi-oAax;
  	aay = atan(accy / accz)*180 / pi-oAay;
	Gx=(float)mpu_ptr->Gx/GyroRatio-oGx;
	Gy=(float)mpu_ptr->Gy/GyroRatio-oGy;
	Gz=(float)mpu_ptr->Gz/GyroRatio-oGz;
	Aay=Aay-t*Gx;
	p_xz=p_xz+q_xz;
	k_xz=p_xz/(p_xz+r_xz);
	Aay=Aay+k_xz*(aay-Aay);
	p_xz=(1-k_xz)*p_xz;
  	Aax=Aax+t*Gy;
  	p_yz=p_yz+q_yz;
  	k_yz=p_yz/(p_yz+r_yz);
  	Aax=Aax+k_yz*(aax-Aax);
  	p_yz=(1-k_yz)*p_yz;
}

//**************************************
//��IIC�豸��ȡһ���ֽ�����
//**************************************
uint8_t MPU6050_ReadI2C(uint8_t REG_Address)
{
	uint8_t REG_data;
	IIC_Start();                  //��ʼ�ź� 
	IIC_Send_Byte(SlaveAddress);  //���������ǵ�ַ+д�ź�
	REG_data=IIC_Wait_Ack();	   
	IIC_Send_Byte(REG_Address);   //���ʹ洢��Ԫ��ַ����0��ʼ 
	REG_data=IIC_Wait_Ack();	   
	IIC_Start();                  //��ʼ�ź� 
	IIC_Send_Byte(SlaveAddress+1);//���������ǵ�ַ+���ź�
	REG_data=IIC_Wait_Ack();	   
  	REG_data=IIC_Read_Byte(0);		//��ȡһ���ֽڣ��������ٶ�������NAK�������Ĵ�������
	IIC_Stop();	                  //ֹͣ�ź� 
	return REG_data;
}

//**************************************
//��IIC�豸д��һ���ֽ�����
//**************************************
void MPU6050_WriteI2C(uint8_t REG_Address,uint8_t REG_data)
{
    IIC_Start();                   //��ʼ�ź� 
    IIC_Send_Byte(SlaveAddress);   //���������ǵ�ַ+д�ź� 
    IIC_Wait_Ack();	   
    IIC_Send_Byte(REG_Address);    //�ڲ��Ĵ�����ַ 
    IIC_Wait_Ack(); 	 										  		   
    IIC_Send_Byte(REG_data);       //�ڲ��Ĵ������� 
    IIC_Wait_Ack(); 	 										  		   
    IIC_Stop();                    //����ֹͣ�ź� 
}

//**************************************
//��ʼ��MPU6050
//**************************************
void MPU6050_Init(void)
{
	uint8_t Add;//������ַ 
	IIC_Init();//��ʼ��IIC�˿� 
	Add=MPU6050_ReadI2C(WHO_AM_I);   //��ȡ������ID��ַ 
	//printf("ID=%x\r\n",Add);
	MPU6050_WriteI2C(PWR_MGMT_1, 0x00);	//�������״̬ 
	MPU6050_WriteI2C(SMPLRT_DIV, 0x07);
	MPU6050_WriteI2C(CONFIG, 0x06);
	MPU6050_WriteI2C(GYRO_CONFIG, 0x18);
	MPU6050_WriteI2C(ACCEL_CONFIG, 0x00);
	delay_ms(100);
}


//��ȡ3��������� 
//x,y,z��ȡ�������� 
void MPU6050_RD_XYZ(MPU_TypeDef* Mympu)//��ȡAX����,GX���� 
{
	uint8_t buf[14];
	uint8_t i;
	IIC_Start();  //IICͨ�ſ�ʼ				 
	IIC_Send_Byte(SlaveAddress);	//����������д��ַ+д�źš�	 
	IIC_Wait_Ack();	     //�ȴ�Ӧ�� 
	IIC_Send_Byte(0x3B);   		//���͸üĴ�����ַ 
	IIC_Wait_Ack(); 	 										  		   
 	IIC_Start();  	 	   		//??D?????���
	IIC_Send_Byte(SlaveAddress+1);	//����������д��ַ+���ź� 
	IIC_Wait_Ack();
	for(i=0;i<14;i++)
	{
		if(i==13)buf[i]=IIC_Read_Byte(0);//���һ������Ӧ��NACK  
		else buf[i]=IIC_Read_Byte(1);	//ack=1��Ӧ�� 
 	}	        	   
	IIC_Stop();					//����һ��ֹͣ���� 
	Mympu->Ax=(short)(((uint16_t)buf[0]<<8)+buf[1]); 
	Mympu->Ay=(short)(((uint16_t)buf[2]<<8)+buf[3]); 
	Mympu->Az=(short)(((uint16_t)buf[4]<<8)+buf[5]); 
	Mympu->Tt=(short)(((uint16_t)buf[6]<<8)+buf[7]);
	Mympu->Gx=(short)(((uint16_t)buf[8]<<8)+buf[9]); 
	Mympu->Gy=(short)(((uint16_t)buf[10]<<8)+buf[11]); 
	Mympu->Gz=(short)(((uint16_t)buf[12]<<8)+buf[13]);
}

//��ȡADXL��ƽ��ֵ 
//x,y,z��ȡ10�κ�ȡƽ��ֵ 
void MPU6050_RD_Avval(MPU_TypeDef* Mympu)
{
	uint8_t i;  
	MPU_TypeDef tMympu;   
	for(i=0;i<10;i++)
	{
		MPU6050_RD_XYZ(Mympu);//ͨ�����������������ȡ6050��õ�ֵ 
		delay_ms(10);
		tMympu.Ax += (short)Mympu->Ax;
		tMympu.Ay += (short)Mympu->Ay;
		tMympu.Az += (short)Mympu->Az;	   
	}
	Mympu->Ax=tMympu.Ax/10;
	Mympu->Ay=tMympu.Ay/10;
	Mympu->Az=tMympu.Az/10;
} 