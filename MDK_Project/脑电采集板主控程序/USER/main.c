#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "ble.h"
#include "led.h"
#include "spi.h"
#include "ads1299.h"
#include "exti.h"

	
int main(void)
{ 
    
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
	delay_init(168);     //初始化延时函数
    uart_init(115200);	//初始化串口波特率为115200
	LED_Init();					//初始化LED   
    ads1299_init();
	EXTIX_Init();
    
	while(1)
	{
        delay_ms(1);
	}       
}

