#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "spi.h"
#include "key.h"
#include "ads1299.h"
#include "exti.h"
#include "imu.h"
#include "sdio_sdcard.h"
#include "malloc.h"

int main(void)
{ 
    u8 key;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//����ϵͳ�ж����ȼ�����2
	delay_init(168);     //��ʼ����ʱ����
    uart_init(115200);	//��ʼ�����ڲ�����Ϊ115200
	LED_Init();					//��ʼ��LED  
    ads1299_init();
    imu_init();
	EXTIX_Init();
    
    my_mem_init(SRAMIN);		//��ʼ���ڲ��ڴ�� 
	my_mem_init(SRAMCCM);		//��ʼ��CCM�ڴ��
	   
 	while(SD_Init())//��ⲻ��SD��
	{
		LED0=!LED0;//DS0��˸
	}
	show_sdcard_info();	//��ӡSD�������Ϣ

    
	while(1)
	{
        imu_main();
        key=KEY_Scan(0);
        switch(key){
//            case KEY0_PRES:
//                break;
            case KEY1_PRES:                
                ads_con_exchange();//���ĵ缫���ӷ�ʽ��LED1���м䣩����Ϊ��������������Ϊ˫������
                break;
            case KEY2_PRES:
                
                break;
//            case WKUP_PRES:
//                break;
            default:
                ;
        }
        delay_ms(1);
        //printf("yaw_angle:%8.3lf   pit_angle:%8.3f  rol_angle:%8.3lf\r\n", get_yaw_angle(), get_pit_angle(), get_rol_angle());
        delay_ms(5);
	}       
}

