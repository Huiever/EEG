#ifndef __DELAY_H
#define __DELAY_H 			   
#include <sys.h>	  

void delay_init(u8 SYSCLK);
void delay_ms(u16 nms);
void delay_us(u32 nus);
#define TEST_LENGTH_SAMPLES  1000     /* 采样点数 */

#define STORE_BIN 1;

#define TEST_MCO_H				GPIO_SetBits	(GPIOA	,	GPIO_Pin_8)
#define TEST_MCO_L				GPIO_ResetBits(GPIOA	,	GPIO_Pin_8)

#endif

