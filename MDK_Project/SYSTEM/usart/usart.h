#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "stm32f4xx_conf.h"
#include "sys.h" 

#define USE_BLE_USART           1

#define USART_REC_LEN  			200  	//�����������ֽ��� 200


#define USE_USART_DMA_RX        0
#define USE_USART_DMA_TX        1


#define PACKAGE_SIZE            33      //�������ݰ��ĳ��ȣ�����openbci��Э�飬�̶�Ϊ33
#define TICKTICK                20      //һ�η������ݰ��ĸ���

#define SEND_BUF_SIZE           PACKAGE_SIZE * TICKTICK	    //���η��������ܳ���


extern u8  uart_send_data[SEND_BUF_SIZE];

void uart_init(u32 bound);
void USARTX_Print(uint8_t* ch, int len);
void MYDMA_Enable(u16 ndtr);

#endif



