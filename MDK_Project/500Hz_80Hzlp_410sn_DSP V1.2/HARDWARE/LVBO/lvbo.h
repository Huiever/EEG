#ifndef __LVBO_H
#define __LVBO_H	 
#include "sys.h" 
#include "arm_math.h"
//extern float32_t ch_value1[1000][2];
//extern float32_t ch_value2[1000][2];
//extern volatile float compute_ram[1000][2];
//extern volatile u8 buffer_choose;
//extern volatile u8 buffer_full;
//extern unsigned int d;
//extern float32_t testInput_f32[800];
//void EXTIX_Init(void);	//�ⲿ�жϳ�ʼ��	
//void PC12_TEST_Init(void);
////-----------------just for test-------------

//#define TEST_PIN_H				GPIO_SetBits	(GPIOC	,	GPIO_Pin_12)
//#define TEST_PIN_L				GPIO_ResetBits(GPIOC	,	GPIO_Pin_12)
//#define LENGTH_SAMPLES 800 /* ��������*/
extern float32_t testOutput_ditong[410];
void arm_iir_f32_lp(void);

#define numStages2 4 /* 2��IIR�˲����ĸ���*/
#define LENGTH_SAMPLES 410 /* ��������*/

#endif




