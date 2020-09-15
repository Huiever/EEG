#include "ads1299.h"
#include "spi.h"
#include "sys.h"
#include "usart.h"
#include "delay.h"
#include "led.h"
#include "ble.h"
#include "string.h"
ads_data_t ads_data = {0xa0,0,
                       0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
                       0,0,0,0,0,0,
                       0xc0};
u8 stream_data = 0;         
void ADS1299_RREG(u8 ReadAddr, u8 pBuffer);
void ADS_PowerOnInit(void);
unsigned char ADS_SPI(unsigned char com);
unsigned char ADS_REG(unsigned char com,unsigned data);
void ads1299_gpio_init(void);

void ads1299_reset(void);
              
void ads1299_init()
{
    ads1299_gpio_init();//��ʼ��ads1299���õ�io��
    SPI3_Init();		   			            //��ʼ��SPI,42Mʱ��
    delay_ms(10);
    ADS_PowerOnInit(); //�ϵ��ʼ��
    ads1299_reset();
    ADS1299_CS = 0; 
    ADS_SPI(START); //����ת��
    ADS1299_CS = 1; 
    
    LED0 = 0;
}
void ads1299_gpio_init(void){
    GPIO_InitTypeDef  GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);//ʹ��GPIOBʱ��
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);//ʹ��GPIOBʱ��
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3;//PB14
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//���
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
    GPIO_Init(GPIOE, &GPIO_InitStructure);//��ʼ��

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//����
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//��������
    GPIO_Init(GPIOD, &GPIO_InitStructure);//��ʼ��
      
    ADS1299_PWDN = 0;
    ADS1299_RESET = 1;
    ADS1299_START = 0;
    delay_ms(10);
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

    while(buffer != 0x3e){
        SPI3_Init();		   			            //��ʼ��SPI,42Mʱ��
        buffer = ADS_REG(RREG|ID,0X00);
        delay_ms(50);
        printf("The divece ID is : %x\r\n",buffer);
    }

    /*fc for bias test*/
    ADS_REG(WREG|CONFIG3,0Xe0);	//ʹ���ڲ��ο���ѹ��BIASREFʹ���ڲ�������AVDD+AVSS��/2��ʹ��BIAS buffer ec
    delay_ms(10);//�ȴ��ڲ��ο���ѹ�ȶ�
    ADS_REG(WREG|MISC1,0x20);	//SRB1
    while(ADS_REG(RREG|MISC1,0X00) != 0x20){
        ADS_REG(WREG|MISC1,0x20);	//SRB1
    };
    
    ADS_REG(WREG|CONFIG1,0x96);	//  250Hz 0x96;500hz 0x95;1k 0x94;2k 0x93;4k 0x92;8k 0x91;16k  0x90;
    while(ADS_REG(RREG|CONFIG1,0X00) != 0x96){
        ADS_REG(WREG|CONFIG1,0x96);	//amplified x1
    };
//    ADS_REG(WREG|CONFIG2,0xD0);	//�����ź��ڲ�������Ƶ��Ϊf/(2^21)

    while(ADS_REG(RREG|CONFIG2,0X00) != 0xC0){
        ADS_REG(WREG|CONFIG2,0xC0);	//amplified x1
    };
    
//��ͨ������    
    while(ADS_REG(RREG|CH1SET,0X00) != 0x60){
        ADS_REG(WREG|CH1SET,0X60);	//off
    };    
    while(ADS_REG(RREG|CH2SET,0X00) != 0x80){
        ADS_REG(WREG|CH2SET,0X80);	//on
    };   
    while(ADS_REG(RREG|CH3SET,0X00) != 0x80){
        ADS_REG(WREG|CH3SET,0X80);	//on
    };
    while(ADS_REG(RREG|CH4SET,0X00) != 0x60){
        ADS_REG(WREG|CH4SET,0X60);	//off
    };    
    while(ADS_REG(RREG|CH5SET,0X00) != 0x80){
        ADS_REG(WREG|CH5SET,0X80);	//off
    };   
    while(ADS_REG(RREG|CH6SET,0X00) != 0x80){
        ADS_REG(WREG|CH6SET,0X80);	//off
    };
    while(ADS_REG(RREG|CH7SET,0X00) != 0x80){
        ADS_REG(WREG|CH7SET,0X80);	//off
    };   
    while(ADS_REG(RREG|CH8SET,0X00) != 0x80){
        ADS_REG(WREG|CH8SET,0X80);	//off
    };

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
    static int count = 0;
    
    static int ticktick = 0;
    static uint8_t DATA_ADS[27]={0x00};
    static uint32_t HEX[8] = {0};//δת���ĵ�ѹֵu16 Data = 0;
    static uint8_t DATA_REC[27] = {0x00};
    ADS1299_CS = 0;
    ADS_SPI (RDATA);
    
    ADS_Read(DATA_REC);
    memcpy(ads_data.eeg_data, DATA_REC+3,24);

    
//    if(ticktick < 30){
//        memcpy(uart_send_data + ticktick * 33, &ads_data, 33);
//        ticktick++;
//    }
//    else{
//        if(stream_data != 0){
//            MYDMA_Enable(DMA1_Stream3,SEND_BUF_SIZE);
//        }
//        ticktick = 0;
//    }
     
    memcpy(uart_send_data, &ads_data, 33);
    if(stream_data != 0){
        MYDMA_Enable(DMA2_Stream7,SEND_BUF_SIZE);
    }
    
    ads_data.sample_number = count++;

//    for(int i = 0; i < 8; i++){
//        j = (i+1) * 3;
//        HEX[i] = (DATA_ADS[j]<<16)|(DATA_ADS[j+1]<<8)|DATA_ADS[j+2];		
//        if (HEX[i] & 0x800000)
//            ads_data.data[i] = (16777216 - HEX[i]) * (-4500.0)/8388607;
//        else
//            ads_data.data[i] = HEX[i] * (4500.0)/8388607;	
//    }

//    memcpy(eeg_data_buff_big + count*31,&ads_data,SEND_BUF_SIZE);  
//    count++;
//    if(count >= 10){
//        count = 0;
//        USART1_Print(eeg_data_buff_big, SEND_BUF_SIZE_BIG);
//    }

//    USART1_Print((uint8_t*)&ads_data, SEND_BUF_SIZE);
    

   
    ADS1299_CS = 1;    
}

void ads1299_reset(void)
{
        printf("OpenBCI V3 8-16 channel\r\n");
        printf("On Board ADS1299 Device ID: 0x3E\r\n");
        printf("LIS3DH Device ID: 0x00\r\n");
        printf("Firmware: v0.0.1\r\n");
        printf("$$$");
}

void processChar(char character){
    switch(character){
        case 'v':
            ads1299_reset();
            break;
        case 'b':
            stream_data = 1;
            break;
        default:
            ;
            
    }
}