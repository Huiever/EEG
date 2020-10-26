#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "stm32f4xx_conf.h"
#include "sys.h" 

#define USE_BLE_USART           1

#define USART_REC_LEN  			200  	//定义最大接收字节数 200


#define USE_USART_DMA_RX        0
#define USE_USART_DMA_TX        1


#define PACKAGE_SIZE            33      //单个数据包的长度，根据openbci的协议，固定为33
#define TICKTICK                20      //一次发送数据包的个数

#define SEND_BUF_SIZE           PACKAGE_SIZE * TICKTICK	    //单次发送数据总长度


extern u8  uart_send_data[SEND_BUF_SIZE];

void uart_init(u32 bound);
void USARTX_Print(uint8_t* ch, int len);
void MYDMA_Enable(u16 ndtr);

#endif



