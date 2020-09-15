#ifndef __BLE_H
#define __BLE_H

#include "stdio.h"	
#include "stm32f4xx_conf.h"
#include "sys.h" 


#if 0

#define USART_REC_LEN  			200  	//�����������ֽ��� 200
#define EN_USART3_RX 			1		//ʹ�ܣ�1��/��ֹ��0������1����

#define USE_USART_DMA_RX        0
#define USE_USART_DMA_TX        1
#define DEBUG_USARTx            USART3

#define SEND_BUF_SIZE 33	//�������ݳ���,��õ���sizeof(TEXT_TO_SEND)+2��������.

extern u8 uart_send_data[SEND_BUF_SIZE];
extern u8  USART_RX_BUF[USART_REC_LEN]; //���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з� 
extern u16 USART_RX_STA;         		//����״̬���	
//����봮���жϽ��գ��벻Ҫע�����º궨��
void uart_init(u32 bound);
void USART3_Print(uint8_t* ch, int len);
void MYDMA_Enable(DMA_Stream_TypeDef *DMA_Streamx,u16 ndtr);

#endif
#endif