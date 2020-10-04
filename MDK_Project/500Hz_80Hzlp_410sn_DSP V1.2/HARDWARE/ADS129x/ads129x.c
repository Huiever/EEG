/******************************************************************************
* @file    ads129x.c
* @author  Sky
* @version V1.4.0
* @date    10-September-2015
* @brief   This file contains ADC initialization process.
******************************************************************************/
#include "ads129x.h"
extern unsigned int rec_gain;


void ADS_Init(void) 
{
	GPIO_InitTypeDef 	GPIO_InitStructure_ADS;
	SPI_InitTypeDef 	SPI_InitStructure_ADS;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3,ENABLE);
	
	/* SPI pin mappings */
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_SPI3);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_SPI3);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_SPI3);
	
 	GPIO_InitStructure_ADS.GPIO_Mode 	= GPIO_Mode_IN;
 	GPIO_InitStructure_ADS.GPIO_Pin 	= ADS_DRDY;
 	GPIO_InitStructure_ADS.GPIO_PuPd  = GPIO_PuPd_NOPULL;
 	GPIO_InitStructure_ADS.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_Init(ADS_DRDY_G, &GPIO_InitStructure_ADS);
	
	GPIO_InitStructure_ADS.GPIO_Mode 	= GPIO_Mode_AF;
	GPIO_InitStructure_ADS.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure_ADS.GPIO_Pin 	= ADS_SCLK;
 	GPIO_InitStructure_ADS.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(ADS_SCLK_G, &GPIO_InitStructure_ADS);
	
	GPIO_InitStructure_ADS.GPIO_Mode 	= GPIO_Mode_AF;
	GPIO_InitStructure_ADS.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure_ADS.GPIO_Pin 	= ADS_DIN;
 	GPIO_InitStructure_ADS.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(ADS_DIN_G, &GPIO_InitStructure_ADS);
	
	GPIO_InitStructure_ADS.GPIO_Mode 	= GPIO_Mode_AF;
	GPIO_InitStructure_ADS.GPIO_Pin 	= ADS_DOUT;
	GPIO_InitStructure_ADS.GPIO_PuPd  = GPIO_PuPd_NOPULL;
 	GPIO_InitStructure_ADS.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(ADS_DOUT_G, &GPIO_InitStructure_ADS);

/*SPI接口初始化*/
	SPI_Cmd(SPI1,DISABLE);
	SPI_InitStructure_ADS.SPI_Direction = SPI_Direction_2Lines_FullDuplex; 	//双线双向全双工
	SPI_InitStructure_ADS.SPI_Mode 			= SPI_Mode_Master;									//SPI主模式
	SPI_InitStructure_ADS.SPI_DataSize 	= SPI_DataSize_8b;									//8bit数据
	SPI_InitStructure_ADS.SPI_CPOL 			= SPI_CPOL_Low;											//CLK空闲时为高电平          
	SPI_InitStructure_ADS.SPI_CPHA 			= SPI_CPHA_2Edge;										//数据捕获于第二个时钟沿
	SPI_InitStructure_ADS.SPI_NSS 			= SPI_NSS_Soft;											//片选用软件控制
	SPI_InitStructure_ADS.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;	//SPI频率APB2/8=84/8=10.5MHz（ADS129x的最大SPI时钟为20Mhz）  
	SPI_InitStructure_ADS.SPI_FirstBit 	= SPI_FirstBit_MSB;									//高位在前
	SPI_InitStructure_ADS.SPI_CRCPolynomial = 7;														//crc7，stm32spi带硬件ecc
	SPI_Init(SPI3, &SPI_InitStructure_ADS);
	SPI_Cmd(SPI3,ENABLE);
}


/**ADS1299上电复位 **/
void ADS_PowerOnInit(void)
{	
	delay_ms(1000);//wait for stable

	ADS_SPI(ADS_RESET);
	ADS_SPI(SDATAC);
	/*fc for bias test*/
	ADS_REG(WREG|CONFIG3,0Xec);	//使用内部参考电压，BIASREF使用内部产生（AVDD+AVSS）/2，使能BIAS buffer ec
	delay_ms(10);//等待内部参考电压稳定
	ADS_REG(WREG|CONFIG1,0x95);	//  250Hz 0x96;500hz,0x95
	//ADS_REG(WREG|CONFIG2,0XD0);//测试信号内部产生
	//sens=ADS_REG(RREG|CONFIG1,0x00);//读寄存器，第二个参数任意值均可

	ADS_REG(WREG|CH1SET,0X00);	//amplified x1
  ADS_REG(WREG|CH2SET,0X00);	//amplified x1
  ADS_REG(WREG|CH3SET,0X00);	//amplified x1
  ADS_REG(WREG|CH4SET,0X00);	//amplified x1
	ADS_REG(WREG|CH5SET,0X00);	//amplified x1
  ADS_REG(WREG|CH6SET,0X00);	//amplified x1
	ADS_REG(WREG|CH7SET,0X00);	//amplified x1
  ADS_REG(WREG|CH8SET,0X00);	//amplified x1
}

/**对ADS129x内部寄存器进行操作 **/
unsigned char ADS_REG(unsigned char com,unsigned data)
{
	unsigned char i,data_return;
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
	return (SPI_I2S_ReceiveData(SPI3));	//返回接收到的数据
}

/*读取72位的数据1100+LOFF_STAT[4:0]+GPIO[1:0]+13个0+2CHx24位，共9字节*/	
void ADS_Read(unsigned char *data)
{	unsigned char i;
	for (i=0;i<27;i++)
	{
		*data=ADS_SPI(0X00);
		data++;		
	}
}
