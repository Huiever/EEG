#include "exti.h"
#include "delay.h" 
#include "led.h" 
#include "ads1299.h"
#include "usart.h"
#include "spi.h"

//外部中断2服务程序
void EXTI9_5_IRQHandler(void)
{
	if (EXTI_GetFlagStatus(EXTI_Line7)==SET)
	{
        ads_data_process();//当DRDY拉低时获取ads1299的数据
    }
	EXTI_ClearITPendingBit(EXTI_Line7);//清除LINE2上的中断标志位 
}
	  
//外部中断初始化程序
//初始化PE2~4,PA0为中断输入.
void EXTIX_Init(void)
{
	NVIC_InitTypeDef   NVIC_InitStructure;
	EXTI_InitTypeDef   EXTI_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);//使能SYSCFG时钟
	
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource7);//PE2 连接到中断线2

    /* 配置EXTI_Line2 */
    EXTI_InitStructure.EXTI_Line = EXTI_Line7;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;//中断事件
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; //下降沿触发
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;//中断线使能
    EXTI_Init(&EXTI_InitStructure);//配置

    NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;//外部中断2
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;//抢占优先级1
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;//子优先级2
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//使能外部中断通道
    NVIC_Init(&NVIC_InitStructure);//配置
}
