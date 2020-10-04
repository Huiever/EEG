/******************************************************************************
* @file    exti.c
* @author  Sky
* @version V1.4.0
* @date    10-September-2015
* @brief   This file contains ads1299 interrupt part.
******************************************************************************/
#include "math.h"
#include "exti.h"
#include "iwdg.h"
#include "delay.h"
#include "usart6.h"
#include "ads129x.h"
#include "ad5754r.h"
#include "timer.h"
#include "arm_math.h"
#include "stdio.h"
#include "lvbo.h"


/* Private variables ---------------------------------------------------------*/
uint8_t DATA_ADS[27]={0x00};
uint8_t DATA_Txd_EEG[9]={0xa5,0x5a,0x00,0x00,0x00,0x00,0x00,0x00,0xf7};
uint32_t HEX1[410];
uint32_t SHURU_EEG[800];
//uint32_t SHURU_EEG1[800];
//uint32_t SHURU_EEG11[800];
//uint32_t SHURU_EEG22[800];
unsigned int k=0;
unsigned int e;//mian()中的判断
float32_t z[410]={0.0};//滤波输入值
float32_t z_o[410]={0.0};//上一周期采集的电压值
float32_t z_n[410]={0.0};//本周期采集的电压值
float32_t b;//新来的数
float32_t D;//滤波后的数
unsigned int c;//滤波次数
unsigned int n;//总的滤波次数
unsigned int j=0;
int h=0;       //判断是否初始化

int f;
uint32_t HEX;//未转换的电压值


//外部中断初始化程序:初始化PB7-SPI_DRDY为中断输入
void EXTIX_Init(void) 
{
	NVIC_InitTypeDef   NVIC_InitStructure;
	EXTI_InitTypeDef   EXTI_InitStructure;
	
	//两个中断源IO口初始化
	GPIO_InitTypeDef  GPIO_InitStructure;
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//使能GPIOB时钟
	/* exti line gpio config */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7; //ADS129x的SPI_DRDY对应引脚PB7
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//普通输入模式
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;//浮空
  GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化GPIOB12
  /* Enable SYSCFG clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);//使能SYSCFG时钟
	EXTI_ClearITPendingBit(EXTI_Line7);//清除中断标志
  /* exti line (PB7,PB7) mode config */
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource7);//PB7 连接到中断线7
	
  /* 配置EXTI_Line7 */
  EXTI_InitStructure.EXTI_Line = EXTI_Line7;//LINE7
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;//中断事件
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; //下降沿中断
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;//使能LINE7
  EXTI_Init(&EXTI_InitStructure);//配置
	
	/* Enable the EXTI_PB7 gloabal Interrupt ,ADS1299*/
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;//外部中断7
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;//抢占优先级3 without DMA int
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;//子优先级1
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//使能外部中断通道
  NVIC_Init(&NVIC_InitStructure);//配置   
}

//--------------------------------------just for test PB13
void PC12_TEST_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);//使能GPIOG时钟
	
	GPIO_InitStructure.GPIO_Mode 		= GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType 	= GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Pin 		= GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_100MHz;
 	GPIO_Init(GPIOC, &GPIO_InitStructure);
}
/*******************************************************************************
* EXTI9_5_IRQHandler,ADS1299中断处理程序  mainly notice: Byte
		//0x5a 0xa5 
					 //   0x00 0x00 0x00
           //		0x00 0x00 0x00 
		       //   0x00 0x00 0x00
           //		0x00 0x00 0x00
    //0xf7
*******************************************************************************/
void EXTI9_5_IRQHandler(void)
{
	float32_t Data;
 	IWDG_Feed();//喂狗
	k++;
	if (EXTI_GetFlagStatus(EXTI_Line7)==SET)
	{
 		//TEST_PIN_H;
		EXTI_ClearITPendingBit(EXTI_Line7);
		ADS_Read(DATA_ADS);
//		DATA_Txd_EEG[2]=DATA_ADS[3];
//		DATA_Txd_EEG[3]=DATA_ADS[4];
//		DATA_Txd_EEG[4]=DATA_ADS[5];
//		DATA_Txd_EEG[5]=DATA_ADS[9];
//		DATA_Txd_EEG[6]=DATA_ADS[10];
//		DATA_Txd_EEG[7]=DATA_ADS[11];

		HEX=(DATA_ADS[6]<<16)|(DATA_ADS[7]<<8)|DATA_ADS[8];
		
		if (HEX & 0x800000)
			Data=(16777216-HEX)*(-4500.0)/8388607;
		else
			Data = HEX*(4500.0)/8388607;
			
		if (j<10)						 //采集数据
  	{
			z_n[h]=Data;
			h++;h=h%410;
		}
		else
		{
//			TIM3->CNT=0;//重设TIM3定时器的计数值
//			timeout=0;
			
			
			for (f=0;f<410;f++)//传递数据
			{
				z[f]=z_n[h];h++;h=h%410;
			}
			z_n[h]=Data;			 //防丢数据
			h++;h=h%410;
			
			j=0;//第二轮循环只赋值9次，保证数据不丢
			e=1;
			
//			k=TIM_GetCounter(TIM3);
//			time=TIM3->CNT+(u32)timeout*65536;
//			printf("timeout=%u\r\n", timeout); 
//			printf("time=%u\r\n", time); 

		}
		j++;	
//		n++;
			
//	USART6_TxPacket(DATA_Txd_EEG,9);
	}
}

