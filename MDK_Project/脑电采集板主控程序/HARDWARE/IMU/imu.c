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

volatile float IST8310_FIFO[3][6] = {0};    //[0]-[4]为最近5次数据 [5]为5次数据的平均值 
                                     //注：磁传感器的采样频率慢，所以单独列出


void init_quaternion(float hx, float hy);


 
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F407开发板
//MPU6050 驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2014/5/9
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	

//IIC连续写
//addr:器件地址 
//reg:寄存器地址
//len:写入长度
//buf:数据区
//返回值:0,正常
//    其他,错误代码
u8 MPU_Write_Len(u8 addr,u8 reg,u8 len,u8 *buf)
{
	u8 i; 
    IIC_Start(); 
	IIC_Send_Byte((addr<<1)|0);//发送器件地址+写命令	
	if(IIC_Wait_Ack())	//等待应答
	{
		IIC_Stop();		 
		return 1;		
	}
    IIC_Send_Byte(reg);	//写寄存器地址
    IIC_Wait_Ack();		//等待应答
	for(i=0;i<len;i++)
	{
		IIC_Send_Byte(buf[i]);	//发送数据
		if(IIC_Wait_Ack())		//等待ACK
		{
			IIC_Stop();	 
			return 1;		 
		}		
	}    
    IIC_Stop();	 
	return 0;	
} 
//IIC连续读
//addr:器件地址
//reg:要读取的寄存器地址
//len:要读取的长度
//buf:读取到的数据存储区
//返回值:0,正常
//    其他,错误代码
u8 MPU_Read_Len(u8 addr,u8 reg,u8 len,u8 *buf)
{ 
 	IIC_Start(); 
	IIC_Send_Byte((addr<<1)|0);//发送器件地址+写命令	
	if(IIC_Wait_Ack())	//等待应答
	{
		IIC_Stop();		 
		return 1;		
	}
    IIC_Send_Byte(reg);	//写寄存器地址
    IIC_Wait_Ack();		//等待应答
    IIC_Start();
	IIC_Send_Byte((addr<<1)|1);//发送器件地址+读命令	
    IIC_Wait_Ack();		//等待应答 
	while(len)
	{
		if(len==1)*buf=IIC_Read_Byte(0);//读数据,发送nACK 
		else *buf=IIC_Read_Byte(1);		//读数据,发送ACK  
		len--;
		buf++; 
	}    
    IIC_Stop();	//产生一个停止条件 
	return 0;	
}
//IIC写一个字节 
//reg:寄存器地址
//data:数据
//返回值:0,正常
//    其他,错误代码
u8 MPU_Write_Byte(u8 reg,u8 data) 				 
{ 
    IIC_Start(); 
	IIC_Send_Byte((MPU_ADDR<<1)|0);//发送器件地址+写命令	
	if(IIC_Wait_Ack())	//等待应答
	{
		IIC_Stop();		 
		return 1;		
	}
    IIC_Send_Byte(reg);	//写寄存器地址
    IIC_Wait_Ack();		//等待应答 
	IIC_Send_Byte(data);//发送数据
	if(IIC_Wait_Ack())	//等待ACK
	{
		IIC_Stop();	 
		return 1;		 
	}		 
    IIC_Stop();	 
	return 0;
}
//IIC读一个字节 
//reg:寄存器地址 
//返回值:读到的数据
u8 MPU_Read_Byte(u8 reg)
{
	u8 res;
    IIC_Start(); 
	IIC_Send_Byte((MPU_ADDR<<1)|0);//发送器件地址+写命令	
	IIC_Wait_Ack();		//等待应答 
    IIC_Send_Byte(reg);	//写寄存器地址
    IIC_Wait_Ack();		//等待应答
    IIC_Start();
	IIC_Send_Byte((MPU_ADDR<<1)|1);//发送器件地址+读命令	
    IIC_Wait_Ack();		//等待应答 
	res=IIC_Read_Byte(0);//读取数据,发送nACK 
    IIC_Stop();			//产生一个停止条件 
	return res;		
}
//设置MPU6050陀螺仪传感器满量程范围
//fsr:0,±250dps;1,±500dps;2,±1000dps;3,±2000dps
//返回值:0,设置成功
//    其他,设置失败 
u8 MPU_Set_Gyro_Fsr(u8 fsr)
{
	return MPU_Write_Byte(MPU_GYRO_CFG_REG,fsr<<3);//设置陀螺仪满量程范围  
}
//设置MPU6050加速度传感器满量程范围
//fsr:0,±2g;1,±4g;2,±8g;3,±16g
//返回值:0,设置成功
//    其他,设置失败 
u8 MPU_Set_Accel_Fsr(u8 fsr)
{
	return MPU_Write_Byte(MPU_ACCEL_CFG_REG,fsr<<3);//设置加速度传感器满量程范围  
}
//设置MPU6050的数字低通滤波器
//lpf:数字低通滤波频率(Hz)
//返回值:0,设置成功
//    其他,设置失败 
u8 MPU_Set_LPF(u16 lpf)
{
	u8 data=0;
	if(lpf>=188)data=1;
	else if(lpf>=98)data=2;
	else if(lpf>=42)data=3;
	else if(lpf>=20)data=4;
	else if(lpf>=10)data=5;
	else data=6; 
	return MPU_Write_Byte(MPU_CFG_REG,data);//设置数字低通滤波器  
}
//设置MPU6050的采样率(假定Fs=1KHz)
//rate:4~1000(Hz)
//返回值:0,设置成功
//    其他,设置失败 
u8 MPU_Set_Rate(u16 rate)
{
	u8 data;
	if(rate>1000)rate=1000;
	if(rate<4)rate=4;
	data=1000/rate-1;
	data=MPU_Write_Byte(MPU_SAMPLE_RATE_REG,data);	//设置数字低通滤波器
 	return MPU_Set_LPF(rate/2);	//自动设置LPF为采样率的一半
}

//得到温度值
//返回值:温度值(扩大了100倍)
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
//得到陀螺仪值(原始值)
//gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
//返回值:0,成功
//    其他,错误代码
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
//得到加速度值(原始值)
//gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
//返回值:0,成功
//    其他,错误代码
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




//初始化MPU6050
//返回值:0,成功
//    其他,错误代码
u8 MPU_Init(void)
{ 
	u8 res;
	IIC_Init();//初始化IIC总线
	MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X80);	//复位MPU6050
    delay_ms(100);
	MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X00);	//唤醒MPU6050 
	MPU_Set_Gyro_Fsr(3);					//陀螺仪传感器,±2000dps
	MPU_Set_Accel_Fsr(0);					//加速度传感器,±2g
	MPU_Set_Rate(50);						//设置采样率50Hz
	MPU_Write_Byte(MPU_INT_EN_REG,0X00);	//关闭所有中断
	MPU_Write_Byte(MPU_USER_CTRL_REG,0X00);	//I2C主模式关闭
	MPU_Write_Byte(MPU_FIFO_EN_REG,0X00);	//关闭FIFO
	MPU_Write_Byte(MPU_INTBP_CFG_REG,0X80);	//INT引脚低电平有效
	res=MPU_Read_Byte(MPU_DEVICE_ID_REG);
	if(res==MPU_ADDR)//器件ID正确
	{
		MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X01);	//设置CLKSEL,PLL X轴为参考
		MPU_Write_Byte(MPU_PWR_MGMT2_REG,0X00);	//加速度与陀螺仪都工作
		MPU_Set_Rate(50);						//设置采样率为50Hz
 	}else return 1;
	return 0;
}


/**
  * @brief          mpu和ist数据减去零漂，并转换为国际单位, 磁力值滑动平均滤波
  * @author         李运环
  * @param[in]      NULL
  * @retval         返回空
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
/* 摄氏度 */
    imu.rip.temp = imu.raw.temp * MPU6500_TEMPERATURE_FACTOR + MPU6500_TEMPERATURE_OFFSET;
}
/**
  * @brief          加速度计低通滤波
  * @author         李运环
  * @param[in]      转化为国际单位的加速度值的结构体指针
  * @retval         返回空
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
  * @brief          更新加速度、角速度、磁力值
  * @author         李运环
  * @retval         返回空
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
  * @brief          获取加速度、角速度的零漂
  * @author         李运环
  * @retval         返回空
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
  * @brief          快速计算方根
  * @author         open source
  * @param[in]      需要计算方根的单精度浮点数
  * @retval         返回空
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
  * @brief          姿态解算，更新四元数
  * @author         http://x-io.co.uk/open-source-imu-and-ahrs-algorithms/
  * @param[in]      转换成国际单位的加速度、角速度的结构体指针
  * @retval         返回空
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
  * @brief          姿态解算, 更新四元数, this function takes 56.8us.(168M)
  * @author         http://x-io.co.uk/open-source-imu-and-ahrs-algorithms/
  * @param[in]      转换成国际单位的加速度、角速度、磁力值的结构体指针
  * @retval         返回空
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
  * @brief          根据磁力计的值初始化四元数
  * @author         DJI_RM
  * @param[in]      读取磁力计的磁力值的结构体
  * @retval         返回空
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
  * @brief          根据四元数更新姿态角，并对yaw进行连续化处理
  * @author         李运环
  * @param[in]      NULL
  * @retval         返回空
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

    //yaw数据连续化
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
  * @brief          mpu6500和ist8310的初始化以及角速度、加速度、磁力值的零漂，通过宏定义决定是否开启IMU温控
  * @author         李运环
  * @retval         返回空
  */
void imu_init(void){
    while(MPU_Init()){
        printf("MPU6050 init error！");//失败进入死循环
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
  * @brief          IMU主函数
  * @author         李运环
  * @retval         返回空
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
