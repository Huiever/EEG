/******************************************************************************
* @file    uart6.c
* @author  Sky
* @version V1.4.0
* @date    10-September-2015
* @brief   This file contains uart6_init and interrupt process part.
******************************************************************************/
#include "sys.h"
#include "usart6.h"
#include "ADS129x.h"
#include "ad5754r.h"
#include "exti.h"
#include "timer.h"

#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{ 	
	while((USART6->SR&0X40)==0);//循环发送,直到发送完毕   
	USART6->DR = (u8) ch;      
	return ch;
}
#endif


#if EN_USART6_RX 
//串口6中断服务程序
u16 USART_RX_STA6=0;

unsigned int RXBUF[17]={0x00};
unsigned char left_en,right_en,start_stim_left,start_stim_right;
unsigned int pwl_us,pwr_us,cyclel,cycler,amp_l,amp_r,rampupl_cycle,rampdownl_cycle,rampupr_cycle,rampdownr_cycle;
unsigned short int i,rec_gain=0;

void uart6_init(u32 bound)
{	
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6,ENABLE);
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_USART6);
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource7,GPIO_AF_USART6);
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOC,&GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate = bound;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;//no parity
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART6, &USART_InitStructure);
	
  USART_Cmd(USART6, ENABLE);
	
	USART_ClearFlag(USART6, USART_FLAG_TC);
	
#if EN_USART6_RX	
	USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);
	//Usart1 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x00;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0x00;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
#endif
}

/* for JEAN
Both 5Byte F-series && 17Byte E-series
*/

void USART6_IRQHandler(void)                	//串口6中断服务程序
{
	if(USART_GetITStatus(USART6, USART_IT_RXNE) != RESET)  //接收中断
	{
		i++;
    switch(i){
        case  1: RXBUF[0]  = USART_ReceiveData(USART6);break;//type
        case  2: RXBUF[1]  = USART_ReceiveData(USART6);break;//L_pwH   || pw
        case  3: RXBUF[2]  = USART_ReceiveData(USART6);break;//L_pwL   || freq
        case  4: RXBUF[3]  = USART_ReceiveData(USART6);break;//L_freqH || amp
			  case  5: RXBUF[4]  = USART_ReceiveData(USART6);break;//L_freqL || ramp
        case  6: RXBUF[5]  = USART_ReceiveData(USART6);break;//L_ampH
        case  7: RXBUF[6]  = USART_ReceiveData(USART6);break;//L_ampL
        case  8: RXBUF[7]  = USART_ReceiveData(USART6);break;//L_rampup
        case  9: RXBUF[8]  = USART_ReceiveData(USART6);break;//L_rampdown
        case 10: RXBUF[9]  = USART_ReceiveData(USART6);break;//R_pwH
			  case 11: RXBUF[10] = USART_ReceiveData(USART6);break;//R_pwL
        case 12: RXBUF[11] = USART_ReceiveData(USART6);break;//R_freqH	
        case 13: RXBUF[12] = USART_ReceiveData(USART6);break;//R_freqL			
        case 14: RXBUF[13] = USART_ReceiveData(USART6);break;//R_ampH
			  case 15: RXBUF[14] = USART_ReceiveData(USART6);break;//R_ampL
        case 16: RXBUF[15] = USART_ReceiveData(USART6);break;//R_ramp-up	
        case 17: RXBUF[16] = USART_ReceiveData(USART6);break;//R_ramp-down					
        default: break;
    }
		if((i==5) && ((RXBUF[0]&0xf0)==0xf0)) 
		{
			i=0;
			if(RXBUF[0]==0xf0){left_en=1;}
      else if(RXBUF[0]==0xf1){right_en=1;}
			else if(RXBUF[0]==0xf2){left_en=0;}
			else if(RXBUF[0]==0xf3){right_en=0;}
			else if(RXBUF[0]==0xf4){
													pwl_us=RXBUF[1]-0x14;
													cyclel=10000/RXBUF[2];//1/F*1000*10这里为了定时到小数点一位所以*10
				                  rampupl_cycle=(RXBUF[4]>>4)*1000/cyclel;//(RXBUF[4]>>4)*100/cyclel_ms  /10***
				                  rampdownl_cycle=(RXBUF[4]&0x0f)*1000/cyclel;				
				
													/*LEFT-protect pulse_width(1:150us) && frequency(1:200hz)*/
													if((pwl_us>=1) && (pwl_us<=150) && (cyclel>=50) && (cyclel<=10000))	
													{
														TIM3_Int_Init(cyclel-1,8400-1);
													}
													else
													{
														TIM_Cmd(TIM3,DISABLE);
													}
													amp_l=RXBUF[3]*32768/50;//对应关系==
													/*LEFT-protect voltage(0.1:4.9v)*/
													if((RXBUF[3]>=1) && (RXBUF[3]<=49))
													{
														start_stim_left=1;
													}
													else
													{
														start_stim_left=0;
													}
														 }
      else if(RXBUF[0]==0xf5){start_stim_left=0;}
			else if(RXBUF[0]==0xf6){
													pwr_us=RXBUF[1]-0x14;
													cycler=10000/RXBUF[2];//1/F*1000*10
								          rampupr_cycle=(RXBUF[4]>>4)*1000/cycler;
				                  rampdownr_cycle=(RXBUF[4]&0x0f)*1000/cycler;			
													/*LEFT-protect pulse_width(1:150us) && frequency(1:200hz)*/				
													if((pwr_us>=1) && (pwr_us<=150) && (cycler>=50) && (cycler<=10000))	
													{
														TIM4_Int_Init(cycler-1,8400-1);
													}
													else
													{
														TIM_Cmd(TIM4,DISABLE);
													}			
													amp_r=RXBUF[3]*32768/50;//对应关系==
													/*RIGHT-protect voltage(0.1:4.9v)*/
													if((RXBUF[3]>=1) && (RXBUF[3]<=49))
													{
														start_stim_right=1;
													}
													else
													{
														start_stim_right=0;
													}				
													   }
			else if(RXBUF[0]==0xf7){start_stim_right=0;}	
      else if(RXBUF[0]==0xf8){
				                      rec_gain=RXBUF[1];
				                      ADS_REG(WREG|CH1SET,rec_gain);
				                      ADS_REG(WREG|CH3SET,rec_gain);		
			                       }
		}
		
		if((i==17) && (RXBUF[0]&0xe0)==0xe0)
		{
				i=0;
				if(RXBUF[0]==0xE0){
														ADS_SPI(START);
														ADS_SPI(RDATAC);
													}
				else if(RXBUF[0]==0xE1)
													{
														ADS_SPI(SDATAC);
														ADS_SPI(STOP);
													}
				else if(RXBUF[0]==0xE4) //wait for this one ,just delay is OK
													{
														pwl_us=(RXBUF[1]<<8)+ RXBUF[2] - 0x14;//for correct the steup time of DAC

														/*prescale is 1302, so the unit is 15.5us*/
														/*|1Hz---------------------------250Hz----↑↑|
															|1000ms------------------------4ms------↓↓|
															|********Notice: 84MHz/1302=15.5us********|
															|64516-------------------------258-------0|
														*/
														cyclel=64516-((RXBUF[3]<<8)+RXBUF[4]);
														
														/*2580x15.5us = 40ms*/
														rampupl_cycle=RXBUF[7]*2580/cyclel;
														rampdownl_cycle=RXBUF[8]*2580/cyclel;	
					
														/*L-protect pulse_width(50:500us) && frequency(1:250hz)*/
														if((pwl_us>=50) && (pwl_us<=500) && (cyclel>=258) && (cyclel<=64516))	
														{
															TIM3_Int_Init(cyclel-1,1302-1);//84M/1302=15.5us
														}
														else
														{
															TIM_Cmd(TIM3,DISABLE);
														}
														/*L-protect voltage(0:4.5v)*/
														amp_l=(RXBUF[5]<<8)+RXBUF[6];											
														if((amp_l>=1) && (amp_l<=29439))
														{
															start_stim_left=1;
														}
														else
														{
															start_stim_left=0;
														}
													}
				else if(RXBUF[0]==0xE5){start_stim_left=0;}
				else if(RXBUF[0]==0xE2)
													{
														pwr_us=(RXBUF[9]<<8)+ RXBUF[10] - 0x14;//for correct the steup time of DAC

														/*prescale is 1302, so the unit is 15.5us*/
														/*|1Hz---------------------------250Hz----↑↑|
															|1000ms------------------------4ms------↓↓|
															|********Notice: 84MHz/1302=15.5us********|
															|64516-------------------------258-------0|
														*/
														cycler=64516-((RXBUF[11]<<8)+RXBUF[12]);
														
														/*2580x15.5us = 40ms*/
														rampupr_cycle=RXBUF[15]*2580/cycler;
														rampdownr_cycle=RXBUF[16]*2580/cycler;				
					
														/*R-protect pulse_width(50:500us) && frequency(1:250hz)*/
														if((pwr_us>=50) && (pwr_us<=500) && (cycler>=258) && (cycler<=64516))	
														{
															TIM4_Int_Init(cycler-1,1302-1);//84M/1302=15.5us
														}
														else
														{
															TIM_Cmd(TIM4,DISABLE);
														}
														/*R-protect voltage(0:4.5v)*/
														amp_r=(RXBUF[13]<<8)+RXBUF[14];											
														if((amp_r>=1) && (amp_r<=29439))
														{
															start_stim_right=1;
														}
														else
														{
															start_stim_right=0;
														}
												 }
				else if(RXBUF[0]==0xE3){start_stim_right=0;}	
				else if(RXBUF[0]==0xE6)
												 {
														pwl_us=(RXBUF[1]<<8)+ RXBUF[2] - 0x14;//for correct the steup time of DAC
														pwr_us=(RXBUF[9]<<8)+ RXBUF[10] - 0x14;//for correct the steup time of DAC

														/*prescale is 1302, so the unit is 15.5us*/
														/*|1Hz---------------------------250Hz----↑↑|
															|1000ms------------------------4ms------↓↓|
															|********Notice: 84MHz/1302=15.5us********|
															|64516-------------------------258-------0|
														*/
														cyclel=64516-((RXBUF[3]<<8)+RXBUF[4]);
														cycler=64516-((RXBUF[11]<<8)+RXBUF[12]);
														
														/*2580x15.5us = 40ms*/
														rampupl_cycle=RXBUF[7]*2580/cyclel;
														rampdownl_cycle=RXBUF[8]*2580/cyclel;	
														rampupr_cycle=RXBUF[15]*2580/cycler;
														rampdownr_cycle=RXBUF[16]*2580/cycler;													 
					
														/*L&R-protect pulse_width(50:500us) && frequency(1:250hz)*/
														if((pwl_us>=50) && (pwl_us<=500) && (cyclel>=258) && (cyclel<=64516) &&
															 (pwr_us>=50) && (pwr_us<=500) && (cycler>=258) && (cycler<=64516))	
														{
															TIM3_Int_Init(cyclel-1,1302-1);//84M/1302=15.5us
															TIM4_Int_Init(cycler-1,1302-1);//84M/1302=15.5us
														}
														else
														{
															TIM_Cmd(TIM3,DISABLE);
															TIM_Cmd(TIM4,DISABLE);
														}
														/*L&R-protect voltage(0:4.5v)*/
														amp_l=(RXBUF[5]<<8)+RXBUF[6];	
														amp_r=(RXBUF[13]<<8)+RXBUF[14];			
														if((amp_l>=1) && (amp_l<=29439) &&
															 (amp_r>=1) && (amp_r<=29439))
														{
															start_stim_left=1;
															start_stim_right=1;
														}
														else
														{
															start_stim_left=0;
															start_stim_right=0;
														}											
												 }
				else if(RXBUF[0]==0xE7){start_stim_left=0;start_stim_right=0;}				
				else if(RXBUF[0]==0xE8)
												 {
														rec_gain=RXBUF[1];
														ADS_REG(WREG|CH1SET,rec_gain&0x70);	//L_rec amplified x1 x2 x4 x6 x8 x12 x24
														ADS_REG(WREG|CH3SET,rec_gain&0x70);	//R_rec amplified x1 x2 x4 x6 x8 x12 x24			
												 }
		}
/****************************************************************************************
		*RECORDING CONTROL CODE
*****************************************************************************************/		
		if((left_en==1)||(right_en==1))
		{
			ADS_SPI(START);
			ADS_SPI(RDATAC);
		}
		else if(((RXBUF[0]&0xf0)==0xf0) && (left_en==0)&&(right_en==0))
		{
			ADS_SPI(SDATAC);
			ADS_SPI(STOP);
		}		
	}
} 
#endif	


void USART6_TxPacket(u8 *txbuf, u8 len)
{
	u8 u8_ctr;
	for(u8_ctr=0;u8_ctr<len;u8_ctr++)
	{
		USART_SendData(USART6, *txbuf++);
		while(USART_GetFlagStatus(USART6,USART_FLAG_TC)!=SET);
	}
}
