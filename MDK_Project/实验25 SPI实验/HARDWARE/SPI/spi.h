#ifndef __SPI_H
#define __SPI_H
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F407������
//SPI ��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2014/5/6
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	
 	    													  
void SPI3_Init(void);			 //��ʼ��SPI3��
void SPI3_SetSpeed(u8 SpeedSet); //����SPI3�ٶ�   
u8 SPI3_ReadWriteByte(u8 TxData);//SPI3���߶�дһ���ֽ�
u8 SPI3_ReadWriteBytes(u8 TxData,u8 Num);
unsigned char ADS1x9x_SPI_Data ( unsigned char Data);
#endif

