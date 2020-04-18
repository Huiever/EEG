#ifndef __SPI_H
#define __SPI_H
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F407开发板
//SPI 驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2014/5/6
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	
 	    													  
void SPI3_Init(void);			 //初始化SPI3口
void SPI3_SetSpeed(u8 SpeedSet); //设置SPI3速度   
u8 SPI3_ReadWriteByte(u8 TxData);//SPI3总线读写一个字节
u8 SPI3_ReadWriteBytes(u8 TxData,u8 Num);
unsigned char ADS1x9x_SPI_Data ( unsigned char Data);
#endif

