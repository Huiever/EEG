/******************************************************************************
* @file    main.c
* @author  Sky
* @version V1.4.0
* @date    10-September-2015
* @brief   This file contains the main profile.
******************************************************************************
*2015-09-23 function run
*add parameter adjust function
*2016-02-23 change protocol for JEAN
******************************************************************************/

#include "sys.h"
#include "delay.h"
#include "iwdg.h"
#include "usart6.h"
#include "exti.h"
#include "timer.h"
#include "ads129x.h"
#include "ad5754r.h"
#include "math.h" 
#include "arm_math.h" 
#include "lvbo.h"


int main(void)
{ 
	uint32_t i;
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	delay_init(168);
	
	/* IWDG Time */
	RCC_LSICmd(ENABLE);
	while(RCC_GetFlagStatus(RCC_FLAG_LSIRDY)==RESET){};
	
	//PC12_TEST_Init();
	/*250Hz sample rate*/
 	ADS_Init();
 	ADS_PowerOnInit();
	
  /* AD5754R_init */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	AD5754R_Init();	
	ConfigAD5754R();
	
	/* clear stimulation */
	WriteToLAD5754RViaSpi(DAC_Set_Register | DAC_Channel_ALL | 0x0000);	
	WriteToRAD5754RViaSpi(DAC_Set_Register | DAC_Channel_ALL | 0x0000);	
	
	uart6_init(256000);			//UART6 for serial port 
	
//	TIM3_Int_Init(65535,84-1);//1MHz计数频率,最长时间65ms
		
	printf("HELLO WORLD\n");

	EXTIX_Init();


 	//IWDG_Init(4,500); //watchdog
	while (1)
	{
		if (e==1)
		{ 		
			
			arm_iir_f32_lp();
			
//			TIM3->CNT=0;//重设TIM3定时器的计数值
//			timeout=0;
				
				for (i=0;i<10;i++)
			  {
					printf("%f %f\r\n",z[i+400],testOutput_ditong[i+400]);
				}

				e=0;
				
//				time=TIM3->CNT+(u32)timeout*65536;
//				printf("timeout=%u\r\n", timeout); 
//				printf("time=%u\r\n", time);
		}
	}
}

