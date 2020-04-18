#include "exti.h"
#include "delay.h" 
#include "led.h" 
#include "ads1299.h"
#include "usart.h"
#include "spi.h"
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
float z[410]={0.0};//滤波输入值
float z_o[410]={0.0};//上一周期采集的电压值
float z_n[410]={0.0};//本周期采集的电压值
float b;//新来的数
float D;//滤波后的数
unsigned int c;//滤波次数
unsigned int n;//总的滤波次数
unsigned int j=0;
int h=0;       //判断是否初始化
int f;
uint32_t HEX;//未转换的电压值
u8 Data = 0;

//外部中断2服务程序
void EXTI0_IRQHandler(void)
{
	if (EXTI_GetFlagStatus(EXTI_Line0)==SET)
	{
        ADS1299_CS=0; 
        ADS_SPI (RDATA);
//      TEST_PIN_H;
//		EXTI_ClearITPendingBit(EXTI_Line0);
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
        ADS1299_CS=1;         
    }
	EXTI_ClearITPendingBit(EXTI_Line0);//清除LINE2上的中断标志位 
}

	   
//外部中断初始化程序
//初始化PE2~4,PA0为中断输入.
void EXTIX_Init(void)
{
	NVIC_InitTypeDef   NVIC_InitStructure;
	EXTI_InitTypeDef   EXTI_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);//使能SYSCFG时钟
	
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource0);//PE2 连接到中断线2

    /* 配置EXTI_Line2 */
    EXTI_InitStructure.EXTI_Line = EXTI_Line0;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;//中断事件
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; //下降沿触发
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;//中断线使能
    EXTI_Init(&EXTI_InitStructure);//配置

    NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;//外部中断2
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;//抢占优先级1
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;//子优先级2
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//使能外部中断通道
    NVIC_Init(&NVIC_InitStructure);//配置
	   
}












