#include "ads1299.h"
#include "spi.h"
#include "sys.h"
#include "usart.h"
#include "delay.h"
void ADS1299_RREG(u8 ReadAddr, u8 pBuffer);
void ADS_PowerOnInit(void);
unsigned char ADS_SPI(unsigned char com);
unsigned char ADS_REG(unsigned char com,unsigned data);


void ads1299_gpio_init(){
    GPIO_InitTypeDef  GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);//使能GPIOC时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//使能GPIOB时钟

    //GPIOC
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;//PB14
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//输出
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
    GPIO_Init(GPIOC, &GPIO_InitStructure);//初始化

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//输入
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉输入
    GPIO_Init(GPIOC, &GPIO_InitStructure);//初始化
    
    ADS1299_PWDN = 0;
    ADS1299_RESET = 1;
    ADS1299_START = 0;
}

void ads1299_init()
{
    ads1299_gpio_init();
    SPI3_Init();		   			            //初始化SPI
    SPI3_SetSpeed(SPI_BaudRatePrescaler_2);		//设置为42M时钟,高速模式 
    ADS_PowerOnInit();

    ADS1299_CS=0; 
    ADS_SPI(START); //开启转换
    delay_us(1);
    ADS1299_CS = 1; 
}


/**对ADS129x内部寄存器进行操作 **/
unsigned char ADS_REG(unsigned char com,unsigned data)
{
	unsigned char i,data_return=0;
	for (i=0;i<45;i++);
	ADS_SPI(com);
	for (i=0;i<45;i++);
	ADS_SPI(0X00);
	for (i=0;i<45;i++);
	if ((com&0x20)==0x20)//判断是否为读寄存器指令
	{
		data_return=ADS_SPI(0X00);
		//for(i=0;i<45;i++);	
	}
	if ((com&0x40)==0x40)
	{
		data_return=ADS_SPI(data);
	}
	return (data_return);
}

/**通过SPI总线与ADS1292通信**/
unsigned char ADS_SPI(unsigned char com)
{
	while (SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_TXE) == 0);//检测发送寄存器是否为空，上一个数据发送完成
	SPI_I2S_SendData(SPI3, com);//发送数据      
	while (SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_RXNE) == 0);//检测接收标志是否非空，发送完成  
	return (unsigned char)(SPI_I2S_ReceiveData(SPI3));	//返回接收到的数据
}

/**ADS1299上电复位 **/
void ADS_PowerOnInit(void)
{	
    ADS1299_PWDN = 1;//上电
    delay_ms(1000);//wait for stable
    ADS1299_CS = 0; //选中芯片
    ADS_SPI(ADS_RESET);
    delay_ms(10);
    ADS_SPI(SDATAC);//RDATAC模式下，RREG会被忽略
    delay_ms(10);
    /*fc for bias test*/
    ADS_REG(WREG|CONFIG3,0Xec);	//使用内部参考电压，BIASREF使用内部产生（AVDD+AVSS）/2，使能BIAS buffer ec
    delay_ms(10);//等待内部参考电压稳定
    ADS_REG(WREG|CONFIG1,0x96);	//  250Hz 0x96;500hz 0x95;1k 0x94;2k 0x93;4k 0x92;8k 0x91;16k  0x90;
    
//    ADS_REG(WREG|CONFIG2,0xD0);	//测试信号内部产生，频率为f/(2^21)

    
//    ADS_REG(WREG|CONFIG2,0XD0);//测试信号内部产生
//    ADS_REG(WREG|CH1SET,0X00);	//amplified x1
//    ADS_REG(WREG|CH2SET,0X00);	//amplified x1
//    ADS_REG(WREG|CH3SET,0X00);	//amplified x1
//    ADS_REG(WREG|CH4SET,0X00);	//amplified x1
//    ADS_REG(WREG|CH5SET,0X00);	//amplified x1
//    ADS_REG(WREG|CH6SET,0X00);	//amplified x1
//    ADS_REG(WREG|CH7SET,0X00);	//amplified x1
//    ADS_REG(WREG|CH8SET,0X00);	//amplified x1
    ADS1299_CS = 1; //取消片选
}

/*读取72位的数据1100+LOFF_STAT[4:0]+GPIO[1:0]+13个0+2CHx24位，共9字节*/	
void ADS_Read(unsigned char *data)
{	unsigned char i;
	for (i=0;i<27;i++)
	{
		*data=SPI3_ReadWriteByte(0X00);
		data++;		
	}
}