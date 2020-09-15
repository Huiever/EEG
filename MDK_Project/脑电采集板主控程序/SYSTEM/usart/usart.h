#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "stm32f4xx_conf.h"
#include "sys.h" 

#if 1

#define USART_REC_LEN  			200  	//定义最大接收字节数 200
#define EN_USART1_RX 			1		//使能（1）/禁止（0）串口1接收

#define USE_USART_DMA_RX        0
#define USE_USART_DMA_TX        1
#define DEBUG_USARTx            USART1

#define SEND_BUF_SIZE 33	//发送数据长度,最好等于sizeof(TEXT_TO_SEND)+2的整数倍.
#define SEND_BUF_SIZE_BIG 330

extern u8 uart_send_data[SEND_BUF_SIZE];
extern u8  USART_RX_BUF[USART_REC_LEN]; //接收缓冲,最大USART_REC_LEN个字节.末字节为换行符 
extern u16 USART_RX_STA;         		//接收状态标记	
//如果想串口中断接收，请不要注释以下宏定义
void uart_init(u32 bound);
void USART1_Print(uint8_t* ch, int len);
void MYDMA_Enable(DMA_Stream_TypeDef *DMA_Streamx,u16 ndtr);
#endif

#endif


