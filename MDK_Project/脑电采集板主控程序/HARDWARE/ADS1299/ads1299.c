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
                       
uint8_t SPI_TX(uint8_t DATA)
{
    // SEND DATA
    SPI1->DR = (uint8_t)DATA;

    // WAIT UNTIL TRANSMIT IS COMPLETED
    while (!(SPI1->SR & SPI_I2S_FLAG_TXE));
    // WAIT UNTIL DATA RECIEVE IS COMPLETED
    while (!(SPI1->SR & SPI_I2S_FLAG_RXNE));
    // WAIT UNTIL SPI IS NOT BUSY
    while (SPI1->SR & SPI_I2S_FLAG_BSY);
    // WAIT 2uS - Give the chip some time 2 sort things out
    delay_us(2);
    // RETURN RECIEVED DATA
    return (SPI1->DR);
}  
void ads1299_write_reg(uint8_t ADDR, uint8_t VAL)
{
    
    // SEND FIRST BYTE: 0x2 | ADDR
    SPI_TX(WREG | ADDR);
    // SEND SECOND BYTE: NUMBER_TO_WRITE
    SPI_TX(0x00);
    // SEND VALUE TO WRITE
    SPI_TX(VAL);
    // WAIT 2*TCLK'S = 888ns = 1us
    delay_us(1);
    ;
}
 uint8_t ads1299_read_reg(uint8_t ADDR)
{
    
    // SEND FIRST BYTE: 0x2 | ADDR
    SPI_TX(RREG|ADDR);
    // SEND SECOND BYTE: NUMBER_TO_READ -- pretty much always 0x0
    SPI_TX(0x00);
    // SEND A DUMMY BYTE TO RECIEVE DATA
    uint8_t RESP = SPI_TX(0x00);

    // WAIT 2*TCLK'S = 888ns = 1us
    delay_us(2);
    // Return read value
    ;
    return (RESP);
}

   void ads1299_reset(void);        
void ads1299_init()
{
    SPI1_Init();		   			            //��ʼ��SPI,42Mʱ��
    ads1299_gpio_init();//��ʼ��ads1299���õ�io��
    ADS_PowerOnInit(); //�ϵ��ʼ��
    ads1299_reset(); 
    ADS_SPI(START); //����ת��   
    LED0 = 0;
}
void ads1299_gpio_init(void){
    GPIO_InitTypeDef  GPIO_InitStructure;


    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);//ʹ��GPIOBʱ��
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//ʹ��GPIOBʱ��
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);//ʹ��GPIOBʱ��
    
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3;//PB14
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//���
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
    GPIO_Init(GPIOE, &GPIO_InitStructure);//��ʼ��
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;//PB14
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//���
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
    GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;//PB14
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//���
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
    GPIO_Init(GPIOD, &GPIO_InitStructure);//��ʼ��

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//����
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//��������
    GPIO_Init(GPIOD, &GPIO_InitStructure);//��ʼ��
    
    
    ADS1299_CS = 0; 
    ADS1299_PWDN = 0;
    delay_ms(1000);//wait for stable
    ADS1299_RESET = 0;
    delay_us(4);
    ADS1299_RESET = 1;
    delay_us(20);
    ADS1299_START = 0;
}

/**ADS1299�ϵ縴λ **/
void ADS_PowerOnInit(void)
{	u8 buffer;
    
//    delay_ms(40);//wait for stable
//    ADS1299_PWDN = 0;//�ϵ�
//    delay_us(2);//wait for stable
//    ADS1299_PWDN = 1;//�ϵ�
//    delay_us(10);//wait for stable
//    
//        delay_ms(40);//wait for stable
//    ADS1299_PWDN = 0;//�ϵ�
//    delay_us(2);//wait for stable
//    ADS1299_PWDN = 1;//�ϵ�
//    delay_us(10);//wait for stable
    
    ADS1299_PWDN = 1;//�ϵ�
    delay_ms(1000);//wait for stable

    delay_ms(100);//wait for stable
    SPI_TX(ADS_RESET);
    delay_us(12);   
    SPI_TX(SDATAC);//RDATACģʽ�£�RREG�ᱻ���� 
    delay_ms(110);
    
    while(buffer != 0x3e){
        //SPI1_Init();		   			            //��ʼ��SPI,42Mʱ��
        //delay_ms(50);
        buffer = ads1299_read_reg(ID);   
        printf("The divece ID is : %x\r\n",buffer);
    }
    
    /*fc for bias test*/
    while(ads1299_read_reg(RREG+CONFIG3)!=0xe0){
        ads1299_write_reg(WREG+CONFIG3,0Xe0);	//ʹ���ڲ��ο���ѹ��BIASREFʹ���ڲ�������AVDD+AVSS��/2��ʹ��BIAS buffer ec
    };
    delay_ms(10);//�ȴ��ڲ��ο���ѹ�ȶ�
    
    while(ads1299_read_reg(RREG|MISC1) != 0x20){
        ads1299_write_reg(WREG|MISC1,0x20);	//SRB1
    };
    
    ADS_REG(WREG|CONFIG1,0x96);	//  250Hz 0x96;500hz 0x95;1k 0x94;2k 0x93;4k 0x92;8k 0x91;16k  0x90;
    while(ads1299_read_reg(RREG|CONFIG1) != 0x96){
        ads1299_write_reg(WREG|CONFIG1,0x96);	//amplified x1
    };
//    ADS_REG(WREG|CONFIG2,0xD0);	//�����ź��ڲ�������Ƶ��Ϊf/(2^21)

    while(ads1299_read_reg(RREG|CONFIG2) != 0xC0){
        ads1299_write_reg(WREG|CONFIG2,0xC0);	//amplified x1
    };
    
//��ͨ������    
    while(ads1299_read_reg(RREG|CH1SET) != 0x60){
        ads1299_write_reg(WREG|CH1SET,0X60);	//off
    };    
    while(ads1299_read_reg(RREG|CH2SET) != 0x80){
        ads1299_write_reg(WREG|CH2SET,0X80);	//on
    };   
    while(ads1299_read_reg(RREG|CH3SET) != 0x80){
        ads1299_write_reg(WREG|CH3SET,0X80);	//on
    };
    while(ads1299_read_reg(RREG|CH4SET) != 0x60){
        ads1299_write_reg(WREG|CH4SET,0X60);	//off
    };    
    while(ads1299_read_reg(RREG|CH5SET) != 0x80){
        ads1299_write_reg(WREG|CH5SET,0X80);	//off
    };   
    while(ads1299_read_reg(RREG|CH6SET) != 0x80){
        ads1299_write_reg(WREG|CH6SET,0X80);	//off
    };
    while(ads1299_read_reg(RREG|CH7SET) != 0x80){
        ads1299_write_reg(WREG|CH7SET,0X80);	//off
    };   
    while(ads1299_read_reg(RREG|CH8SET) != 0x80){
        ads1299_write_reg(WREG|CH8SET,0X80);	//off
    };   
    ADS_SPI(START); //����ת��
    delay_ms(100);

}
/**ͨ��SPI������ADS1292ͨ��**/
unsigned char ADS_SPI(unsigned char com)
{
    
    u8 rec = 0;
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == 0);//��ⷢ�ͼĴ����Ƿ�Ϊ�գ���һ�����ݷ������
	SPI_I2S_SendData(SPI1, com);//��������      
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == 0);//�����ձ�־�Ƿ�ǿգ��������  
	rec = SPI_I2S_ReceiveData(SPI1);	//���ؽ��յ�������
    ;
    return rec;
}

void ADS_SPI_SEND(unsigned char com)
{
    
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == 0);//��ⷢ�ͼĴ����Ƿ�Ϊ�գ���һ�����ݷ������
	SPI_I2S_SendData(SPI1, com);//��������     
    ;
}

unsigned char ADS_SPI_RECV(void)
{
    
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == 0);//��ⷢ�ͼĴ����Ƿ�Ϊ�գ���һ�����ݷ������
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == 0);//�����ձ�־�Ƿ�ǿգ��������  
	return SPI_I2S_ReceiveData(SPI1);	//���ؽ��յ�������
    ;
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
	unsigned char i,data_return;
//    ADS_SPI_SEND(com);
//    ADS_SPI_SEND(0x00);
//    if ((com&0x20)==0x20)//�ж��Ƿ�Ϊ���Ĵ���ָ��
//	{
//		data_return=ADS_SPI_RECV();
//		//for(i=0;i<45;i++);	
//	}
//    if ((com&0x40)==0x40)
//	{
//		ADS_SPI_SEND(data);
//	}
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


void ads_data_process()
{ 
    static int count = 0;
    
    static int ticktick = 0;
    static uint8_t DATA_ADS[27]={0x00};
    static uint32_t HEX[8] = {0};//δת���ĵ�ѹֵu16 Data = 0;
    static uint8_t DATA_REC[27] = {0x00};

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
        //MYDMA_Enable(DMA2_Stream7,SEND_BUF_SIZE);
        USART1_Print((uint8_t*)&ads_data, SEND_BUF_SIZE);
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