#include "ads1299.h"
#include "spi.h"
#include "sys.h"
#include "usart.h"
#include "delay.h"

ads_data_t ads_data = {0x0a0d,0,0,0,0,0,0,0,0,0,0x0d0a};

void ADS1299_RREG(u8 ReadAddr, u8 pBuffer);
void ADS_PowerOnInit(void);
unsigned char ADS_SPI(unsigned char com);
unsigned char ADS_REG(unsigned char com,unsigned data);
void ads1299_gpio_init(void);


void ads1299_init()
{
    ads1299_gpio_init();//��ʼ��ads1299���õ�io��
    SPI3_Init();		   			            //��ʼ��SPI
    SPI3_SetSpeed(SPI_BaudRatePrescaler_2);		//����Ϊ42Mʱ��,����ģʽ 
    ADS_PowerOnInit(); //�ϵ��ʼ��

    ADS1299_CS=0; 
    ADS_SPI(START); //����ת��
    delay_us(1);
    ADS1299_CS = 1; 
}
void ads1299_gpio_init(void){
    GPIO_InitTypeDef  GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);//ʹ��GPIOCʱ��

    //GPIOC
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_12;//PB14
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
    delay_ms(1000);
}

/**ADS1299�ϵ縴λ **/
void ADS_PowerOnInit(void)
{	u8 buffer;
    ADS1299_PWDN = 1;//�ϵ�
    delay_ms(1000);//wait for stable
    ADS1299_CS = 0; //ѡ��оƬ
    ADS_SPI(ADS_RESET);
    delay_ms(10);
    ADS_SPI(SDATAC);//RDATACģʽ�£�RREG�ᱻ����
    delay_ms(10);
    ADS_SPI(SDATAC);//RDATACģʽ�£�RREG�ᱻ����
    delay_ms(10);
    /*fc for bias test*/
    ADS_REG(WREG|CONFIG3,0Xe0);	//ʹ���ڲ��ο���ѹ��BIASREFʹ���ڲ�������AVDD+AVSS��/2��ʹ��BIAS buffer ec
    delay_ms(10);//�ȴ��ڲ��ο���ѹ�ȶ�
    ADS_REG(WREG|CONFIG1,0x96);	//  250Hz 0x96;500hz 0x95;1k 0x94;2k 0x93;4k 0x92;8k 0x91;16k  0x90;
    //ADS_REG(WREG|CONFIG2,0xD0);	//�����ź��ڲ�������Ƶ��Ϊf/(2^21)
    ADS_REG(WREG|CONFIG2,0xC0);	
    ADS_REG(WREG|MISC1,0x20);	

    ADS_REG(WREG|CH1SET,0X03);	//amplified x1
    ADS_REG(WREG|CH2SET,0X03);	//amplified x1
    ADS_REG(WREG|CH3SET,0X03);	//amplified x1
    ADS_REG(WREG|CH4SET,0X03);	//amplified x1
    ADS_REG(WREG|CH5SET,0X80);	//amplified x1
    ADS_REG(WREG|CH6SET,0X80);	//amplified x1
    ADS_REG(WREG|CH7SET,0X80);	//amplified x1
    ADS_REG(WREG|CH8SET,0X80);	//amplified x1

    ADS1299_CS = 1; //ȡ��Ƭѡ
}
/**ͨ��SPI������ADS1292ͨ��**/
unsigned char ADS_SPI(unsigned char com)
{
	while (SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_TXE) == 0);//��ⷢ�ͼĴ����Ƿ�Ϊ�գ���һ�����ݷ������
	SPI_I2S_SendData(SPI3, com);//��������      
	while (SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_RXNE) == 0);//�����ձ�־�Ƿ�ǿգ��������  
	return (unsigned char)(SPI_I2S_ReceiveData(SPI3));	//���ؽ��յ�������
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

/**��ADS129x�ڲ��Ĵ������в��� **/
unsigned char ADS_REG(unsigned char com,unsigned data)
{
	unsigned char i,data_return=0;
	for (i=0;i<45;i++);//����sclkʱ��
	ADS_SPI(com);
	for (i=0;i<45;i++);
	ADS_SPI(0X00);
	for (i=0;i<45;i++);
	if ((com & RREG)== RREG)//�ж��Ƿ�Ϊ���Ĵ���ָ��
	{
		data_return=ADS_SPI(0X00);
		//for(i=0;i<45;i++);	
	}
	if ((com & WREG)== WREG)
	{
		data_return=ADS_SPI(data);
	}
	return (data_return);
}

void ads_data_process()
{ 
    int j = 0;
    static uint8_t DATA_ADS[27]={0x00};
    static uint32_t HEX[8] = {0};//δת���ĵ�ѹֵu16 Data = 0;

    ADS1299_CS = 0;
    ADS_SPI (RDATA);
    ADS_Read(DATA_ADS);

//    for(int i = 0; i < 8; i++){
//        j = (i+1) * 3;
//        HEX[i] = (DATA_ADS[j]<<16)|(DATA_ADS[j+1]<<8)|DATA_ADS[j+2];		
//        if (HEX[i] & 0x800000)
//            ads_data.data[i] = (16777216 - HEX[i]) * (-4500.0)/8388607;
//        else
//            ads_data.data[i] = HEX[i] * (4500.0)/8388607;	
//    }
    char d[2]={0x0d,0x0a};
    char dd[2] = {0x0a,0x0d};
    USART2_Print((uint8_t*)&d, 2);
    USART2_Print(DATA_ADS, 27);
    USART2_Print((uint8_t*)&dd, 2);
    
    //USART1_Print((uint8_t*)&ads_data, 22);
    ADS1299_CS = 1; 
    
}