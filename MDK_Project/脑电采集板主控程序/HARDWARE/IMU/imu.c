#include "imu.h"
#include "myiic.h"
#include "mpu6050_reg.h"
#include "delay.h"
#include "usart.h"
#include <math.h>
#include <string.h>
#include "stdlib.h"


float imu_yaw_angle=0;
float imu_yaw_angular_speed=0;
float imu_pitch_angular_speed=0;
//---------------------------------------------------------------------------------------------------
// Variable definitions
static volatile float twoKp = twoKpDef;                                           // 2 * proportional gain (Kp)
static volatile float twoKi = twoKiDef;                                           // 2 * integral gain (Ki)
static volatile float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f; // integral error terms scaled by Ki
volatile float        q0 = 1.0f;
volatile float        q1 = 0.0f;
volatile float        q2 = 0.0f;
volatile float        q3 = 0.0f;

static volatile float gx, gy, gz, ax, ay, az, mx, my, mz;  
uint8_t mpu_buff[14];  /* buffer to save imu raw data */
uint8_t ist_buff[6];   /* buffer to save IST8310 raw data */

static imu_t imu = {
                    {0,0,0},                     //atti
                    {0,0,0,0,0,0,0,0,0,0},       //rip
                    {0,0,0,0,0,0,0,0,0,0},       //raw
                    {ACC_X_OFFSET,ACC_Y_OFFSET,ACC_Z_OFFSET,GYRO_X_OFFSET,GYRO_Y_OFFSET,GYRO_Z_OFFSET,MAG_X_OFFSET,MAG_Y_OFFSET,MAG_Z_OFFSET},         //offset
                    };

volatile float IST8310_FIFO[3][6] = {0};    //[0]-[4]Ϊ���5������ [5]Ϊ5�����ݵ�ƽ��ֵ 
                                     //ע���Ŵ������Ĳ���Ƶ���������Ե����г�


void init_quaternion(float hx, float hy);


 
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F407������
//MPU6050 ��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2014/5/9
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	

//IIC����д
//addr:������ַ 
//reg:�Ĵ�����ַ
//len:д�볤��
//buf:������
//����ֵ:0,����
//    ����,�������
u8 MPU_Write_Len(u8 addr,u8 reg,u8 len,u8 *buf)
{
	u8 i; 
    IIC_Start(); 
	IIC_Send_Byte((addr<<1)|0);//����������ַ+д����	
	if(IIC_Wait_Ack())	//�ȴ�Ӧ��
	{
		IIC_Stop();		 
		return 1;		
	}
    IIC_Send_Byte(reg);	//д�Ĵ�����ַ
    IIC_Wait_Ack();		//�ȴ�Ӧ��
	for(i=0;i<len;i++)
	{
		IIC_Send_Byte(buf[i]);	//��������
		if(IIC_Wait_Ack())		//�ȴ�ACK
		{
			IIC_Stop();	 
			return 1;		 
		}		
	}    
    IIC_Stop();	 
	return 0;	
} 
//IIC������
//addr:������ַ
//reg:Ҫ��ȡ�ļĴ�����ַ
//len:Ҫ��ȡ�ĳ���
//buf:��ȡ�������ݴ洢��
//����ֵ:0,����
//    ����,�������
u8 MPU_Read_Len(u8 addr,u8 reg,u8 len,u8 *buf)
{ 
 	IIC_Start(); 
	IIC_Send_Byte((addr<<1)|0);//����������ַ+д����	
	if(IIC_Wait_Ack())	//�ȴ�Ӧ��
	{
		IIC_Stop();		 
		return 1;		
	}
    IIC_Send_Byte(reg);	//д�Ĵ�����ַ
    IIC_Wait_Ack();		//�ȴ�Ӧ��
    IIC_Start();
	IIC_Send_Byte((addr<<1)|1);//����������ַ+������	
    IIC_Wait_Ack();		//�ȴ�Ӧ�� 
	while(len)
	{
		if(len==1)*buf=IIC_Read_Byte(0);//������,����nACK 
		else *buf=IIC_Read_Byte(1);		//������,����ACK  
		len--;
		buf++; 
	}    
    IIC_Stop();	//����һ��ֹͣ���� 
	return 0;	
}
//IICдһ���ֽ� 
//reg:�Ĵ�����ַ
//data:����
//����ֵ:0,����
//    ����,�������
u8 MPU_Write_Byte(u8 reg,u8 data) 				 
{ 
    IIC_Start(); 
	IIC_Send_Byte((MPU_ADDR<<1)|0);//����������ַ+д����	
	if(IIC_Wait_Ack())	//�ȴ�Ӧ��
	{
		IIC_Stop();		 
		return 1;		
	}
    IIC_Send_Byte(reg);	//д�Ĵ�����ַ
    IIC_Wait_Ack();		//�ȴ�Ӧ�� 
	IIC_Send_Byte(data);//��������
	if(IIC_Wait_Ack())	//�ȴ�ACK
	{
		IIC_Stop();	 
		return 1;		 
	}		 
    IIC_Stop();	 
	return 0;
}
//IIC��һ���ֽ� 
//reg:�Ĵ�����ַ 
//����ֵ:����������
u8 MPU_Read_Byte(u8 reg)
{
	u8 res;
    IIC_Start(); 
	IIC_Send_Byte((MPU_ADDR<<1)|0);//����������ַ+д����	
	IIC_Wait_Ack();		//�ȴ�Ӧ�� 
    IIC_Send_Byte(reg);	//д�Ĵ�����ַ
    IIC_Wait_Ack();		//�ȴ�Ӧ��
    IIC_Start();
	IIC_Send_Byte((MPU_ADDR<<1)|1);//����������ַ+������	
    IIC_Wait_Ack();		//�ȴ�Ӧ�� 
	res=IIC_Read_Byte(0);//��ȡ����,����nACK 
    IIC_Stop();			//����һ��ֹͣ���� 
	return res;		
}
//����MPU6050�����Ǵ����������̷�Χ
//fsr:0,��250dps;1,��500dps;2,��1000dps;3,��2000dps
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
u8 MPU_Set_Gyro_Fsr(u8 fsr)
{
	return MPU_Write_Byte(MPU_GYRO_CFG_REG,fsr<<3);//���������������̷�Χ  
}
//����MPU6050���ٶȴ����������̷�Χ
//fsr:0,��2g;1,��4g;2,��8g;3,��16g
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
u8 MPU_Set_Accel_Fsr(u8 fsr)
{
	return MPU_Write_Byte(MPU_ACCEL_CFG_REG,fsr<<3);//���ü��ٶȴ����������̷�Χ  
}
//����MPU6050�����ֵ�ͨ�˲���
//lpf:���ֵ�ͨ�˲�Ƶ��(Hz)
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
u8 MPU_Set_LPF(u16 lpf)
{
	u8 data=0;
	if(lpf>=188)data=1;
	else if(lpf>=98)data=2;
	else if(lpf>=42)data=3;
	else if(lpf>=20)data=4;
	else if(lpf>=10)data=5;
	else data=6; 
	return MPU_Write_Byte(MPU_CFG_REG,data);//�������ֵ�ͨ�˲���  
}
//����MPU6050�Ĳ�����(�ٶ�Fs=1KHz)
//rate:4~1000(Hz)
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
u8 MPU_Set_Rate(u16 rate)
{
	u8 data;
	if(rate>1000)rate=1000;
	if(rate<4)rate=4;
	data=1000/rate-1;
	data=MPU_Write_Byte(MPU_SAMPLE_RATE_REG,data);	//�������ֵ�ͨ�˲���
 	return MPU_Set_LPF(rate/2);	//�Զ�����LPFΪ�����ʵ�һ��
}

//�õ��¶�ֵ
//����ֵ:�¶�ֵ(������100��)
short MPU_Get_Temperature(void)
{
    u8 buf[2]; 
    short raw;
	float temp;
	MPU_Read_Len(MPU_ADDR,MPU_TEMP_OUTH_REG,2,buf); 
    raw=((u16)buf[0]<<8)|buf[1];  
    temp=36.53+((double)raw)/340;  
    return temp*100;;
}
//�õ�������ֵ(ԭʼֵ)
//gx,gy,gz:������x,y,z���ԭʼ����(������)
//����ֵ:0,�ɹ�
//    ����,�������
u8 MPU_Get_Gyroscope(short *gx,short *gy,short *gz)
{
    u8 buf[6],res;  
	res=MPU_Read_Len(MPU_ADDR,MPU_GYRO_XOUTH_REG,6,buf);
	if(res==0)
	{
		*gx=((u16)buf[0]<<8)|buf[1];  
		*gy=((u16)buf[2]<<8)|buf[3];  
		*gz=((u16)buf[4]<<8)|buf[5];
	} 	
    return res;;
}
//�õ����ٶ�ֵ(ԭʼֵ)
//gx,gy,gz:������x,y,z���ԭʼ����(������)
//����ֵ:0,�ɹ�
//    ����,�������
u8 MPU_Get_Accelerometer(short *ax,short *ay,short *az)
{
    u8 buf[6],res;  
	res=MPU_Read_Len(MPU_ADDR,MPU_ACCEL_XOUTH_REG,6,buf);
	if(res==0)
	{
		*ax=((u16)buf[0]<<8)|buf[1];  
		*ay=((u16)buf[2]<<8)|buf[3];  
		*az=((u16)buf[4]<<8)|buf[5];
	} 	
    return res;;
}




//��ʼ��MPU6050
//����ֵ:0,�ɹ�
//    ����,�������
u8 MPU_Init(void)
{ 
	u8 res;
	IIC_Init();//��ʼ��IIC����
	MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X80);	//��λMPU6050
    delay_ms(100);
	MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X00);	//����MPU6050 
	MPU_Set_Gyro_Fsr(3);					//�����Ǵ�����,��2000dps
	MPU_Set_Accel_Fsr(0);					//���ٶȴ�����,��2g
	MPU_Set_Rate(50);						//���ò�����50Hz
	MPU_Write_Byte(MPU_INT_EN_REG,0X00);	//�ر������ж�
	MPU_Write_Byte(MPU_USER_CTRL_REG,0X00);	//I2C��ģʽ�ر�
	MPU_Write_Byte(MPU_FIFO_EN_REG,0X00);	//�ر�FIFO
	MPU_Write_Byte(MPU_INTBP_CFG_REG,0X80);	//INT���ŵ͵�ƽ��Ч
	res=MPU_Read_Byte(MPU_DEVICE_ID_REG);
	if(res==MPU_ADDR)//����ID��ȷ
	{
		MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X01);	//����CLKSEL,PLL X��Ϊ�ο�
		MPU_Write_Byte(MPU_PWR_MGMT2_REG,0X00);	//���ٶ��������Ƕ�����
		MPU_Set_Rate(50);						//���ò�����Ϊ50Hz
 	}else return 1;
	return 0;
}


/**
  * @brief          mpu��ist���ݼ�ȥ��Ư����ת��Ϊ���ʵ�λ, ����ֵ����ƽ���˲�
  * @author         ���˻�
  * @param[in]      NULL
  * @retval         ���ؿ�
  */
static void imu_calibrate_unit_convert(void){
//unit:m/s2
    imu.rip.ax = (float)((imu.raw.ax - imu.offset.ax) * 9.80665f / 16384.0f);
    imu.rip.ay = (float)((imu.raw.ay - imu.offset.ay) * 9.80665f / 16384.0f);
    imu.rip.az = (float)((imu.raw.az - imu.offset.az) * 9.80665f / 16384.0f);

/* +-1000dps -> rad/s */
    imu.rip.gx = (float)((imu.raw.gx - imu.offset.gx) /32.8f /57.3f);
    imu.rip.gy = (float)((imu.raw.gy - imu.offset.gy) /32.8f /57.3f);
    imu.rip.gz = (float)((imu.raw.gz - imu.offset.gz) /32.8f /57.3f);

/* uT */
    imu.rip.mx = (float)((imu.raw.mx - imu.offset.mx) * MAG_SEN);
    imu.rip.my = (float)((imu.raw.my - imu.offset.my) * MAG_SEN);
    imu.rip.mz = (float)((imu.raw.mz - imu.offset.mz) * MAG_SEN);

    
//    gyro_moving_average_filter(&imu.rip);
/* ���϶� */
    imu.rip.temp = imu.raw.temp * MPU6500_TEMPERATURE_FACTOR + MPU6500_TEMPERATURE_OFFSET;
}
/**
  * @brief          ���ٶȼƵ�ͨ�˲�
  * @author         ���˻�
  * @param[in]      ת��Ϊ���ʵ�λ�ļ��ٶ�ֵ�Ľṹ��ָ��
  * @retval         ���ؿ�
  */
static void accel_low_pass_filter(imu_ripdata_t * mpudata){
    static uint8_t updata_count=0;

    static double accel_fliter_1[3] = {0.0f, 0.0f, 0.0f};
    static double accel_fliter_2[3] = {0.0f, 0.0f, 0.0f};
    static double accel_fliter_3[3] = {0.0f, 0.0f, 0.0f};
    static const double fliter_num[3] = {1.929454039488895f, -0.93178349823448126f, 0.002329458745586203f};

    if(updata_count==0){
        accel_fliter_1[0] = accel_fliter_2[0] = accel_fliter_3[0] = mpudata->ax;
        accel_fliter_1[1] = accel_fliter_2[1] = accel_fliter_3[1] = mpudata->ay;
        accel_fliter_1[2] = accel_fliter_2[2] = accel_fliter_3[2] = mpudata->az;
        updata_count++;
    }
    else{
        accel_fliter_1[0] = accel_fliter_2[0];
        accel_fliter_2[0] = accel_fliter_3[0];

        accel_fliter_3[0] = accel_fliter_2[0] * fliter_num[0] + accel_fliter_1[0] * fliter_num[1] + mpudata->ax * fliter_num[2];

        accel_fliter_1[1] = accel_fliter_2[1];
        accel_fliter_2[1] = accel_fliter_3[1];

        accel_fliter_3[1] = accel_fliter_2[1] * fliter_num[0] + accel_fliter_1[1] * fliter_num[1] + mpudata->ay * fliter_num[2];

        accel_fliter_1[2] = accel_fliter_2[2];
        accel_fliter_2[2] = accel_fliter_3[2];

        accel_fliter_3[2] = accel_fliter_2[2] * fliter_num[0] + accel_fliter_1[2] * fliter_num[1] + mpudata->az * fliter_num[2];
    }
    
    mpudata->ax = accel_fliter_3[0];
    mpudata->ay = accel_fliter_3[1];
    mpudata->az = accel_fliter_3[2];
}
/**
  * @brief          ���¼��ٶȡ����ٶȡ�����ֵ
  * @author         ���˻�
  * @retval         ���ؿ�
  */
static void imu_get_data(void)
{
    static uint8_t init_quaternion_flag=0;
    MPU_Read_Len(MPU_ADDR,MPU_ACCEL_XOUTH_REG,14,mpu_buff);
    
    //MPU6500_Read_Regs(MPU6500_ACCEL_XOUT_H, mpu_buff, 14);

    imu.raw.ax = (int16_t)(mpu_buff[0] << 8 | mpu_buff[1]);
    imu.raw.ay = (int16_t)(mpu_buff[2] << 8 | mpu_buff[3]);
    imu.raw.az = (int16_t)(mpu_buff[4] << 8 | mpu_buff[5]);

    imu.raw.temp = (int16_t)(mpu_buff[6] << 8 | mpu_buff[7]);

    imu.raw.gx = (int16_t)(mpu_buff[8]  << 8 | mpu_buff[9]);
    imu.raw.gy = (int16_t)(mpu_buff[10] << 8 | mpu_buff[11]);
    imu.raw.gz = (int16_t)(mpu_buff[12] << 8 | mpu_buff[13]);

    imu_calibrate_unit_convert();

    accel_low_pass_filter(&imu.rip);

    if(init_quaternion_flag==0){
        init_quaternion(imu.rip.mx, imu.rip.my);
        init_quaternion_flag=1;
    }
}
/**
  * @brief          ��ȡ���ٶȡ����ٶȵ���Ư
  * @author         ���˻�
  * @retval         ���ؿ�
  */
void get_mpu_accel_gyro_offset(void){
    for (int i = 0; i<300;i++){
        MPU_Read_Len(MPU_ADDR,MPU_ACCEL_XOUTH_REG,14,mpu_buff);

        imu.offset.ax +=  mpu_buff[0] << 8 | mpu_buff[1];
        imu.offset.ay +=  mpu_buff[2] << 8 | mpu_buff[3];
        imu.offset.az +=  mpu_buff[4] << 8 | mpu_buff[5];

        imu.offset.gx +=  mpu_buff[8]  << 8 | mpu_buff[9];
        imu.offset.gy +=  mpu_buff[10] << 8 | mpu_buff[11];
        imu.offset.gz +=  mpu_buff[12] << 8 | mpu_buff[13];

        delay_ms(2);
    }
    imu.offset.ax = imu.offset.ax / 300;
    imu.offset.ay = imu.offset.ay / 300;
    imu.offset.az = imu.offset.az / 300;
    imu.offset.gx = imu.offset.gx / 300;
    imu.offset.gy = imu.offset.gy / 300;
    imu.offset.gz = imu.offset.gz / 300;
}

/**
  * @brief          ���ټ��㷽��
  * @author         open source
  * @param[in]      ��Ҫ���㷽���ĵ����ȸ�����
  * @retval         ���ؿ�
  */
static float invSqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i>>1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}
/**
  * @brief          ��̬���㣬������Ԫ��
  * @author         http://x-io.co.uk/open-source-imu-and-ahrs-algorithms/
  * @param[in]      ת���ɹ��ʵ�λ�ļ��ٶȡ����ٶȵĽṹ��ָ��
  * @retval         ���ؿ�
  */
static void mahony_ahrs_updateIMU(imu_ripdata_t const *mpudata){
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    gx = mpudata->gx;
    gy = mpudata->gy;
    gz = mpudata->gz;
    ax = mpudata->ax;
    ay = mpudata->ay;
    az = mpudata->az;

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))){

        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Estimated direction of gravity and vector perpendicular to magnetic flux
        halfvx = q1 * q3 - q0 * q2;
        halfvy = q0 * q1 + q2 * q3;
        halfvz = q0 * q0 - 0.5f + q3 * q3;

        // Error is sum of cross product between estimated and measured direction of gravity
        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        // Compute and apply integral feedback if enabled
        if (twoKi > 0.0f){
          integralFBx += twoKi * halfex * (1.0f / sampleFreq); // integral error scaled by Ki
          integralFBy += twoKi * halfey * (1.0f / sampleFreq);
          integralFBz += twoKi * halfez * (1.0f / sampleFreq);
          gx += integralFBx; // apply integral feedback
          gy += integralFBy;
          gz += integralFBz;
        }
        else{
          integralFBx = 0.0f; // prevent integral windup
          integralFBy = 0.0f;
          integralFBz = 0.0f;
        }

        // Apply proportional feedback
        gx += twoKp * halfex;
        gy += twoKp * halfey;
        gz += twoKp * halfez;
    }

    // Integrate rate of change of quaternion
    gx *= (0.5f * (1.0f / sampleFreq)); // pre-multiply common factors
    gy *= (0.5f * (1.0f / sampleFreq));
    gz *= (0.5f * (1.0f / sampleFreq));
    qa = q0;
    qb = q1;
    qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    // Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}
/**
  * @brief          ��̬����, ������Ԫ��, this function takes 56.8us.(168M)
  * @author         http://x-io.co.uk/open-source-imu-and-ahrs-algorithms/
  * @param[in]      ת���ɹ��ʵ�λ�ļ��ٶȡ����ٶȡ�����ֵ�Ľṹ��ָ��
  * @retval         ���ؿ�
  */
static void mahony_ahrs_update(imu_ripdata_t const *mpudata){
    float recipNorm;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
    float hx, hy, bx, bz;
    float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    gx = mpudata->gx;
    gy = mpudata->gy;
    gz = mpudata->gz;
    ax = mpudata->ax;
    ay = mpudata->ay;
    az = mpudata->az;
    mx = mpudata->mx;
    my = mpudata->my;
    mz = mpudata->mz;

#if AXIS_6 == 1
    mx = 0;
    my = 0;
    mz = 0;
#endif

    if((mx == 0) && (my == 0) && (mz == 0)){
        mahony_ahrs_updateIMU(mpudata);
        return;
    }
    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {
        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Normalise magnetometer measurement
        recipNorm = invSqrt(mx * mx + my * my + mz * mz);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        q0q0 = q0 * q0;
        q0q1 = q0 * q1;
        q0q2 = q0 * q2;
        q0q3 = q0 * q3;
        q1q1 = q1 * q1;
        q1q2 = q1 * q2;
        q1q3 = q1 * q3;
        q2q2 = q2 * q2;
        q2q3 = q2 * q3;
        q3q3 = q3 * q3;

        // Reference direction of Earth's magnetic field
        hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
        hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
        bx = sqrt(hx * hx + hy * hy);
        bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

        // Estimated direction of gravity and magnetic field
        halfvx = q1q3 - q0q2;
        halfvy = q0q1 + q2q3;
        halfvz = q0q0 - 0.5f + q3q3;
        halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
        halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
        halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

        // Error is sum of cross product between estimated direction and measured direction of field vectors
        halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
        halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
        halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

        // Compute and apply integral feedback if enabled
        if (twoKi > 0.0f)
        {
            integralFBx += twoKi * halfex * (1.0f / sampleFreq); // integral error scaled by Ki
            integralFBy += twoKi * halfey * (1.0f / sampleFreq);
            integralFBz += twoKi * halfez * (1.0f / sampleFreq);
            gx += integralFBx; // apply integral feedback
            gy += integralFBy;
            gz += integralFBz;
        }
        else
        {
            integralFBx = 0.0f; // prevent integral windup
            integralFBy = 0.0f;
            integralFBz = 0.0f;
        }

        // Apply proportional feedback
        gx += twoKp * halfex;
        gy += twoKp * halfey;
        gz += twoKp * halfez;
    }

    // Integrate rate of change of quaternion
    gx *= (0.5f * (1.0f / sampleFreq)); // pre-multiply common factors
    gy *= (0.5f * (1.0f / sampleFreq));
    gz *= (0.5f * (1.0f / sampleFreq));
    qa = q0;
    qb = q1;
    qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    // Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}
/**
  * @brief          ���ݴ����Ƶ�ֵ��ʼ����Ԫ��
  * @author         DJI_RM
  * @param[in]      ��ȡ�����ƵĴ���ֵ�Ľṹ��
  * @retval         ���ؿ�
  */
void init_quaternion(float hx, float hy){

#if BOARD_DOWN
    if (hx < 0 && hy < 0) 
    {
        if (fabs(hx / hy) >=  1)
        {
            q0 = -0.005;
            q1 = -0.199;
            q2 = 0.979;
            q3 = -0.0089;
        }
        else
        {
            q0 = -0.008;
            q1 = -0.555;
            q2 = 0.83;
            q3 = -0.002;
        }
        
    }
    else if (hx < 0 && hy > 0)
    {
        if (fabs(hx / hy) >= 1)
        {
            q0 = 0.005;
            q1 = -0.199;
            q2 = -0.978;
            q3 = 0.012;
        }
        else
        {
            q0 = 0.005;
            q1 = -0.553;
            q2 = -0.83;
            q3 = -0.0023;
        }
        
    }
    else if (hx > 0 && hy > 0)
    {
        if (fabs(hx / hy) >=  1)
        {
            q0 = 0.0012;
            q1 = -0.978;
            q2 = -0.199;
            q3 = -0.005;
        }
        else
        {
            q0 = 0.0023;
            q1 = -0.83;
            q2 = -0.553;
            q3 = 0.0023;
        }
        
    }
    else if (hx > 0 && hy < 0)
    {
        if (fabs(hx / hy) >=  1)
        {
            q0 = 0.0025;
            q1 = 0.978;
            q2 = -0.199;
            q3 = 0.008;            
        }
        else
        {
            q0 = 0.0025;
            q1 = 0.83;
            q2 = -0.56;
            q3 = 0.0045;
        }        
    }
#else
        if (hx < 0 && hy < 0)
    {
        if (fabs(hx / hy) >= 1)
        {
            q0 = 0.195;
            q1 = -0.015;
            q2 = 0.0043;
            q3 = 0.979;
        }
        else
        {
            q0 = 0.555;
            q1 = -0.015;
            q2 = 0.006;
            q3 = 0.829;
        }
        
    }
    else if (hx < 0 && hy > 0)
    {
        if(fabs(hx / hy) >= 1)
        {
            q0 = -0.193;
            q1 = -0.009;
            q2 = -0.006;
            q3 = 0.979;
        }
        else
        {
            q0 = -0.552;
            q1 = -0.0048;
            q2 = -0.0115;
            q3 = 0.8313;
        }
        
    }
    else if (hx > 0 && hy > 0)
    {
        if(fabs(hx / hy) >= 1)
        {
            q0 = -0.9785;
            q1 = 0.008;
            q2 = -0.02;
            q3 = 0.195;
        }
        else
        {
            q0 = -0.9828;
            q1 = 0.002;
            q2 = -0.0167;
            q3 = 0.5557;
        }
        
    }
    else if (hx > 0 && hy < 0)
    {
        if(fabs(hx / hy) >= 1)
        {
            q0 = -0.979;
            q1 = 0.0116;
            q2 = -0.0167;
            q3 = -0.195;            
        }
        else
        {
            q0 = -0.83;
            q1 = 0.014;
            q2 = -0.012;
            q3 = -0.556;
        }        
    }
#endif
}
/**
  * @brief          ������Ԫ��������̬�ǣ�����yaw��������������
  * @author         ���˻�
  * @param[in]      NULL
  * @retval         ���ؿ�
  */

static void IMU_getYawPitchRoll(attitude_angle_t * atti)
{
    static float yaw_temp = 0,last_yaw_temp = 0;
    static int   yaw_count = 0;
    // yaw    -pi----pi
    atti->yaw = atan2(2.0f * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3)* 57.3;
    // pitch  -pi/2--- pi/2
    atti->pit = -asin(2.0f * (q1 * q3 - q0 * q2))* 57.3;
    // roll   -pi-----pi
    atti->rol = atan2(2.0f * (q0 * q1 + q2 * q3), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3)* 57.3;

    //yaw����������
    last_yaw_temp = yaw_temp;
    yaw_temp = atti->yaw; 
    if(yaw_temp - last_yaw_temp>= 300){
        yaw_count--;
    } 
    else if (yaw_temp - last_yaw_temp <= -300){
        yaw_count++;
    }
    atti->yaw = yaw_temp + yaw_count*360;

}


/**
  * @brief          mpu6500��ist8310�ĳ�ʼ���Լ����ٶȡ����ٶȡ�����ֵ����Ư��ͨ���궨������Ƿ���IMU�¿�
  * @author         ���˻�
  * @retval         ���ؿ�
  */
void imu_init(void){
    while(MPU_Init()){
        printf("MPU6050 init error��");//ʧ�ܽ�����ѭ��
        delay_ms(500);
    }
    printf("MPU6050 init done!");
    delay_ms(5);

    delay_ms(5);
//    get_mpu_accel_gyro_offset();
    delay_ms(5);
//    get_ist_mag_offset();

}
/**
  * @brief          IMU������
  * @author         ���˻�
  * @retval         ���ؿ�
  */
void imu_main(void){
    imu_get_data();
    mahony_ahrs_update(&imu.rip);
    IMU_getYawPitchRoll(&imu.atti);
    imu_yaw_angle=imu.atti.yaw;
    imu_yaw_angular_speed=imu.rip.gz*57.3f;
    imu_pitch_angular_speed=imu.rip.gy*57.3f;

#if 1
    printf("yaw_angle:%8.3lf   pit_angle:%8.3lf  rol_angle:%8.3lf\r\n", imu.atti.yaw, imu.atti.pit, imu.atti.rol);
    delay_ms(5);
#endif
#if 0
    printf("wx:%8.3lf  wy:%8.3lf  wz:%8.3lf\r\n", imu.rip.gx * 57.3f, imu.rip.gy * 57.3f, imu.rip.gz * 57.3f);
    delay_ms(5);
#endif
#if 0
    printf("%.3f,%.3f,%.3f\r\n",(float)imu.raw.mx/1000,(float)imu.raw.my/1000,(float)imu.raw.mz/1000);
    delay_ms(5);
#endif
}

float get_yaw_angle(void){
    return imu.atti.yaw;
}

float get_pit_angle(void){
    return imu.atti.pit;
}

float get_imu_wx(void){
    return imu.rip.gx * 57.3f;
}

float get_imu_wy(void){
    return imu.rip.gy * 57.3f;
}

float get_imu_wz(void){
    return imu.rip.gz * 57.3f;
}
