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

/*SPI�ӿڳ�ʼ��*/
	SPI_Cmd(SPI1,DISABLE);
	SPI_InitStructure_ADS.SPI_Direction = SPI_Direction_2Lines_FullDuplex; 	//˫��˫��ȫ˫��
	SPI_InitStructure_ADS.SPI_Mode 			= SPI_Mode_Master;									//SPI��ģʽ
	SPI_InitStructure_ADS.SPI_DataSize 	= SPI_DataSize_8b;									//8bit����
	SPI_InitStructure_ADS.SPI_CPOL 			= SPI_CPOL_Low;											//CLK����ʱΪ�ߵ�ƽ          
	SPI_InitStructure_ADS.SPI_CPHA 			= SPI_CPHA_2Edge;										//���ݲ����ڵڶ���ʱ����
	SPI_InitStructure_ADS.SPI_NSS 			= SPI_NSS_Soft;											//Ƭѡ���������
	SPI_InitStructure_ADS.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;	//SPIƵ��APB2/8=84/8=10.5MHz��ADS129x�����SPIʱ��Ϊ20Mhz��  
	SPI_InitStructure_ADS.SPI_FirstBit 	= SPI_FirstBit_MSB;									//��λ��ǰ
	SPI_InitStructure_ADS.SPI_CRCPolynomial = 7;														//crc7��stm32spi��Ӳ��ecc
	SPI_Init(SPI3, &SPI_InitStructure_ADS);
	SPI_Cmd(SPI3,ENABLE);
}


/**ADS1299�ϵ縴λ **/
void ADS_PowerOnInit(void)
{	
	delay_ms(1000);//wait for stable

	ADS_SPI(ADS_RESET);
	ADS_SPI(SDATAC);
	/*fc for bias test*/
	ADS_REG(WREG|CONFIG3,0Xec);	//ʹ���ڲ��ο���ѹ��BIASREFʹ���ڲ�������AVDD+AVSS��/2��ʹ��BIAS buffer ec
	delay_ms(10);//�ȴ��ڲ��ο���ѹ�ȶ�
	ADS_REG(WREG|CONFIG1,0x95);	//  250Hz 0x96;500hz,0x95
	//ADS_REG(WREG|CONFIG2,0XD0);//�����ź��ڲ�����
	//sens=ADS_REG(RREG|CONFIG1,0x00);//���Ĵ������ڶ�����������ֵ����

	ADS_REG(WREG|CH1SET,0X00);	//amplified x1
  ADS_REG(WREG|CH2SET,0X00);	//amplified x1
  ADS_REG(WREG|CH3SET,0X00);	//amplified x1
  ADS_REG(WREG|CH4SET,0X00);	//amplified x1
	ADS_REG(WREG|CH5SET,0X00);	//amplified x1
  ADS_REG(WREG|CH6SET,0X00);	//amplified x1
	ADS_REG(WREG|CH7SET,0X00);	//amplified x1
  ADS_REG(WREG|CH8SET,0X00);	//amplified x1
}

/**��ADS129x�ڲ��Ĵ������в��� **/
unsigned char ADS_REG(unsigned char com,unsigned data)
{
	unsigned char i,data_return;
	for (i=0;i<45;i++);
	ADS_SPI(com);
	for (i=0;i<45;i++);
	ADS_SPI(0X00);
	for (i=0;i<45;i++);
	if ((com&0x20)==0x20)//�ж��Ƿ�Ϊ���Ĵ���ָ��
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

/**ͨ��SPI������ADS1292ͨ��**/
unsigned char ADS_SPI(unsigned char com)
{
	while (SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_TXE) == 0);//��ⷢ�ͼĴ����Ƿ�Ϊ�գ���һ�����ݷ������
	SPI_I2S_SendData(SPI3, com);//��������      
	while (SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_RXNE) == 0);//�����ձ�־�Ƿ�ǿգ��������  
	return (SPI_I2S_ReceiveData(SPI3));	//���ؽ��յ�������
}

/*��ȡ72λ������1100+LOFF_STAT[4:0]+GPIO[1:0]+13��0+2CHx24λ����9�ֽ�*/	
void ADS_Read(unsigned char *data)
{	unsigned char i;
	for (i=0;i<27;i++)
	{
		*data=ADS_SPI(0X00);
		data++;		
	}
}
