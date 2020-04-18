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

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);//ʹ��GPIOCʱ��
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//ʹ��GPIOBʱ��

    //GPIOC
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;//PB14
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//���
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
    GPIO_Init(GPIOC, &GPIO_InitStructure);//��ʼ��

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//����
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//��������
    GPIO_Init(GPIOC, &GPIO_InitStructure);//��ʼ��
    
    ADS1299_PWDN = 0;
    ADS1299_RESET = 1;
    ADS1299_START = 0;
}

void ads1299_init()
{
    ads1299_gpio_init();
    SPI3_Init();		   			            //��ʼ��SPI
    SPI3_SetSpeed(SPI_BaudRatePrescaler_2);		//����Ϊ42Mʱ��,����ģʽ 
    ADS_PowerOnInit();

    ADS1299_CS=0; 
    ADS_SPI(START); //����ת��
    delay_us(1);
    ADS1299_CS = 1; 
}


/**��ADS129x�ڲ��Ĵ������в��� **/
unsigned char ADS_REG(unsigned char com,unsigned data)
{
	unsigned char i,data_return=0;
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
	return (unsigned char)(SPI_I2S_ReceiveData(SPI3));	//���ؽ��յ�������
}

/**ADS1299�ϵ縴λ **/
void ADS_PowerOnInit(void)
{	
    ADS1299_PWDN = 1;//�ϵ�
    delay_ms(1000);//wait for stable
    ADS1299_CS = 0; //ѡ��оƬ
    ADS_SPI(ADS_RESET);
    delay_ms(10);
    ADS_SPI(SDATAC);//RDATACģʽ�£�RREG�ᱻ����
    delay_ms(10);
    /*fc for bias test*/
    ADS_REG(WREG|CONFIG3,0Xec);	//ʹ���ڲ��ο���ѹ��BIASREFʹ���ڲ�������AVDD+AVSS��/2��ʹ��BIAS buffer ec
    delay_ms(10);//�ȴ��ڲ��ο���ѹ�ȶ�
    ADS_REG(WREG|CONFIG1,0x96);	//  250Hz 0x96;500hz 0x95;1k 0x94;2k 0x93;4k 0x92;8k 0x91;16k  0x90;
    
//    ADS_REG(WREG|CONFIG2,0xD0);	//�����ź��ڲ�������Ƶ��Ϊf/(2^21)

    
//    ADS_REG(WREG|CONFIG2,0XD0);//�����ź��ڲ�����
//    ADS_REG(WREG|CH1SET,0X00);	//amplified x1
//    ADS_REG(WREG|CH2SET,0X00);	//amplified x1
//    ADS_REG(WREG|CH3SET,0X00);	//amplified x1
//    ADS_REG(WREG|CH4SET,0X00);	//amplified x1
//    ADS_REG(WREG|CH5SET,0X00);	//amplified x1
//    ADS_REG(WREG|CH6SET,0X00);	//amplified x1
//    ADS_REG(WREG|CH7SET,0X00);	//amplified x1
//    ADS_REG(WREG|CH8SET,0X00);	//amplified x1
    ADS1299_CS = 1; //ȡ��Ƭѡ
}

/*��ȡ72λ������1100+LOFF_STAT[4:0]+GPIO[1:0]+13��0+2CHx24λ����9�ֽ�*/	
void ADS_Read(unsigned char *data)
{	unsigned char i;
	for (i=0;i<27;i++)
	{
		*data=SPI3_ReadWriteByte(0X00);
		data++;		
	}
}