/******************************************************************************
* @file    timer.c
* @author  Sky
* @version V1.4.0
* @date    10-September-2015
* @brief   This file contains timer3/timer4 init and interrupt process part.
*          timer3 controls left  stimulation.
*          timer4 controls right stimulation.
******************************************************************************/
#include "timer.h"
#include "ad5754r.h"
#include "delay.h"
#include "exti.h"
#include "ads129x.h"

extern u8 int_d;
extern u8 dat;
extern unsigned int compute_flag;
extern u8 DATA_Tmp[25];
extern unsigned char start_stim_left;
extern unsigned char start_stim_right;

extern unsigned int pwl_us;
extern unsigned int cyclel_ms;
extern unsigned int amp_l;
extern unsigned int pwr_us;
extern unsigned int cycler_ms;
extern unsigned int amp_r;
extern unsigned int rampupl_cycle;
extern unsigned int rampupr_cycle;
extern unsigned int rampdownl_cycle;
extern unsigned int rampdownr_cycle;
unsigned int time,timeout;//timeout:��ʱ�������

uint32_t stim_amp_l=0x0000;
uint32_t stim_amp_r=0x0000;
//======================================================================
//===========stimulation timer interruption
//======================================================================
//ͨ�ö�ʱ��3�жϳ�ʼ��
//arr���Զ���װֵ��
//psc��ʱ��Ԥ��Ƶ��
//��ʱ�����ʱ����㷽��:Tout=((arr+1)*(psc+1))/Ft us.
//Ft=��ʱ������Ƶ��,��λ:Mhz
//����ʹ�õ��Ƕ�ʱ��3!
void TIM3_Int_Init(u16 arr,u16 psc)  
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);  ///ʹ��TIM3ʱ��
	
  TIM_TimeBaseInitStructure.TIM_Period = arr; 	//�Զ���װ��ֵ
	TIM_TimeBaseInitStructure.TIM_Prescaler=psc;  //��ʱ����Ƶ
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStructure);//��ʼ��TIM3

	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE); 		//����ʱ��3�����ж�
	TIM_ClearITPendingBit(TIM3,TIM_IT_Update);  //����жϱ�־λ��ʦ�������û��
	TIM_Cmd(TIM3,ENABLE); //ʹ�ܶ�ʱ��3
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM3_IRQn; //��ʱ��3�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01; //��ռ���ȼ�1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x03; //�����ȼ�0
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
}
//---------------------------------------
//��ʱ��3�жϷ�����-------------------
//0x1999 0x3332 0x4ccb 0x6664 0x7ffd-----
//---------------------------------------
void TIM3_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM3,TIM_IT_Update)==SET) //����ж�
	{
		timeout++;
	}
	
	TIM_ClearITPendingBit(TIM3,TIM_IT_Update);  //����жϱ�־λ
	
}

void TIM4_Int_Init(u16 arr,u16 psc)  
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);  ///ʹ��TIM3ʱ��
	
  TIM_TimeBaseInitStructure.TIM_Period = arr; 	//�Զ���װ��ֵ
	TIM_TimeBaseInitStructure.TIM_Prescaler=psc;  //��ʱ����Ƶ
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseInitStructure);//��ʼ��TIM4

	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE); 		//����ʱ��4�����ж�
	TIM_ClearITPendingBit(TIM4,TIM_IT_Update);  //����жϱ�־λ
	TIM_Cmd(TIM4,ENABLE); //ʹ�ܶ�ʱ��4
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM4_IRQn; //��ʱ��4�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01; //��ռ���ȼ�1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x01; //�����ȼ�1
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
}

//��ʱ��4�жϷ�����
void TIM4_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM4,TIM_IT_Update)==SET) //����ж�
	{		
		//start stim_right
		if(start_stim_right==1)
		{
			if(rampupr_cycle==0)
			{
				stim_amp_r=amp_r;
			}
			else
			{
				stim_amp_r+=amp_r/rampupr_cycle;//0x028F=0.1V
				if(stim_amp_r>=amp_r) {stim_amp_r=amp_r;}	
			}				
				
			WriteToRAD5754RViaSpi(DAC_Set_Register | DAC_Channel_ALL | stim_amp_r);
			delay_us(pwr_us);
			WriteToRAD5754RViaSpi(DAC_Set_Register | (0x0000ffff & (~stim_amp_r+1)) | DAC_Channel_ALL);
			delay_us(pwr_us);
			WriteToRAD5754RViaSpi(DAC_Set_Register | DAC_Channel_ALL | 0x0000);									
		}							
		//stop_stim_right
		if(start_stim_right==0)
		{						
			if(rampdownr_cycle==0)
			{
				stim_amp_r=0x0000;
			}
			else
			{			
				if(stim_amp_r != 0x0000) {stim_amp_r-=amp_r/rampdownr_cycle;}//0x28F=0.1V		
				if(stim_amp_r<=amp_r/rampdownr_cycle) {stim_amp_r=0x0000;}
			}					
			
			WriteToRAD5754RViaSpi(DAC_Set_Register | DAC_Channel_ALL | stim_amp_r);
			delay_us(pwr_us);
			WriteToRAD5754RViaSpi(DAC_Set_Register | (0x0000ffff & (~stim_amp_r+1)) | DAC_Channel_ALL);
			delay_us(pwr_us);									
			WriteToRAD5754RViaSpi(DAC_Set_Register | DAC_Channel_ALL | 0x0000);									
		}	
	}
	TIM_ClearITPendingBit(TIM4,TIM_IT_Update);  //����жϱ�־λ
}
