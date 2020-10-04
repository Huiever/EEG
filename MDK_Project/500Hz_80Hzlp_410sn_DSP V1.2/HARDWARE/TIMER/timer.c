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
unsigned int time,timeout;//timeout:定时溢出次数

uint32_t stim_amp_l=0x0000;
uint32_t stim_amp_r=0x0000;
//======================================================================
//===========stimulation timer interruption
//======================================================================
//通用定时器3中断初始化
//arr：自动重装值。
//psc：时钟预分频数
//定时器溢出时间计算方法:Tout=((arr+1)*(psc+1))/Ft us.
//Ft=定时器工作频率,单位:Mhz
//这里使用的是定时器3!
void TIM3_Int_Init(u16 arr,u16 psc)  
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);  ///使能TIM3时钟
	
  TIM_TimeBaseInitStructure.TIM_Period = arr; 	//自动重装载值
	TIM_TimeBaseInitStructure.TIM_Prescaler=psc;  //定时器分频
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStructure);//初始化TIM3

	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE); 		//允许定时器3更新中断
	TIM_ClearITPendingBit(TIM3,TIM_IT_Update);  //清除中断标志位，师姐程序中没有
	TIM_Cmd(TIM3,ENABLE); //使能定时器3
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM3_IRQn; //定时器3中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01; //抢占优先级1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x03; //子优先级0
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
}
//---------------------------------------
//定时器3中断服务函数-------------------
//0x1999 0x3332 0x4ccb 0x6664 0x7ffd-----
//---------------------------------------
void TIM3_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM3,TIM_IT_Update)==SET) //溢出中断
	{
		timeout++;
	}
	
	TIM_ClearITPendingBit(TIM3,TIM_IT_Update);  //清除中断标志位
	
}

void TIM4_Int_Init(u16 arr,u16 psc)  
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);  ///使能TIM3时钟
	
  TIM_TimeBaseInitStructure.TIM_Period = arr; 	//自动重装载值
	TIM_TimeBaseInitStructure.TIM_Prescaler=psc;  //定时器分频
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseInitStructure);//初始化TIM4

	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE); 		//允许定时器4更新中断
	TIM_ClearITPendingBit(TIM4,TIM_IT_Update);  //清除中断标志位
	TIM_Cmd(TIM4,ENABLE); //使能定时器4
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM4_IRQn; //定时器4中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01; //抢占优先级1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x01; //子优先级1
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
}

//定时器4中断服务函数
void TIM4_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM4,TIM_IT_Update)==SET) //溢出中断
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
	TIM_ClearITPendingBit(TIM4,TIM_IT_Update);  //清除中断标志位
}
