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
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
	delay_init(168);     //初始化延时函数
    uart_init(115200);	//初始化串口波特率为115200
	LED_Init();					//初始化LED  
    ads1299_init();
    imu_init();
	EXTIX_Init();
    
    my_mem_init(SRAMIN);		//初始化内部内存池 
	my_mem_init(SRAMCCM);		//初始化CCM内存池
	   
 	while(SD_Init())//检测不到SD卡
	{
		LED0=!LED0;//DS0闪烁
	}
	show_sdcard_info();	//打印SD卡相关信息

    
	while(1)
	{
        imu_main();
        key=KEY_Scan(0);
        switch(key){
//            case KEY0_PRES:
//                break;
            case KEY1_PRES:                
                ads_con_exchange();//更改电极连接方式，LED1（中间）亮则为单极导联，否则为双极导联
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

