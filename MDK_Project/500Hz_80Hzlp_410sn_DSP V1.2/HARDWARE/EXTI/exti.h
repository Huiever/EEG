#ifndef __EXTI_H
#define __EXIT_H	 
#include "sys.h" 
#include "arm_math.h"
extern float32_t ch_value1[1000][2];
extern float32_t ch_value2[1000][2];
extern volatile float compute_ram[1000][2];
extern volatile u8 buffer_choose;
extern volatile u8 buffer_full;
extern unsigned int c;
extern unsigned int e;
extern float32_t D;//滤波后的数
extern float32_t z[410];
extern float32_t z_n[410];
extern unsigned int w;
void EXTIX_Init(void);//外部中断初始化	
void PC12_TEST_Init(void);
//-----------------just for test-------------

#define TEST_PIN_H				GPIO_SetBits	(GPIOC	,	GPIO_Pin_12)
#define TEST_PIN_L				GPIO_ResetBits(GPIOC	,	GPIO_Pin_12)
//#define LENGTH_SAMPLES 64 /* 采样点数*/

#endif
