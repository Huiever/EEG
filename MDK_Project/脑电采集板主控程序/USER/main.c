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
    
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//����ϵͳ�ж����ȼ�����2
	delay_init(168);     //��ʼ����ʱ����
    uart_init(115200);	//��ʼ�����ڲ�����Ϊ115200
	LED_Init();					//��ʼ��LED   
    ads1299_init();
	EXTIX_Init();
    
	while(1)
	{
        delay_ms(1);
	}       
}

