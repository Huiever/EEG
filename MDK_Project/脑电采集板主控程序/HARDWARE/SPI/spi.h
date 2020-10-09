#ifndef __SPI_H
#define __SPI_H
#include "sys.h"  

void SPI1_Init(void);			 //初始化SPI1口
void SPI1_SetSpeed(u8 SpeedSet); //设置SPI1速度   
uint8_t SPI_TX(uint8_t DATA);    //SPI1总线读写一个字节





#endif
