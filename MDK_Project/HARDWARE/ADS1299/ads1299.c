#include "ads1299.h"
#include "spi.h"
#include "sys.h"
#include "usart.h"
#include "delay.h"
#include "led.h"
#include "string.h"
#include "imu.h"

ads_data_t ads_data = {0xa0,0,
                       0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
                       0,0,0,0,0,0,
                       0xc0};
u8 stream_data = 0; 
                       
void ADS_PowerOnInit(void);
void ads1299_gpio_init(void);                      
uint8_t ads1299_read_reg(uint8_t ADDR);
void ads1299_write_reg(uint8_t ADDR, uint8_t VAL);
void show_ads_info(void);  

                       
void ads1299_init(void)
{     
    SPI1_Init();		//初始化SPI 
    ads1299_gpio_init();//初始化ads1299所用的io口 
    ADS_PowerOnInit(); //上电初始化
    show_ads_info();
    SPI_TX(START); //开启转换    
    LED0 = 0;
}
void ads1299_gpio_init(void){
    GPIO_InitTypeDef  GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);//使能GPIOB时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//使能GPIOB时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);//使能GPIOB时钟
       
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3;//PB14
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//输出
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
    GPIO_Init(GPIOE, &GPIO_InitStructure);//初始化
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;//PB14
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//输出
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
    GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_6;//PB14
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//输出
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
    GPIO_Init(GPIOD, &GPIO_InitStructure);//初始化

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//输入
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉输入
    GPIO_Init(GPIOD, &GPIO_InitStructure);//初始化
    
    ADS1299_CLKS = 0;
    ADS1299_CS = 1; 
    ADS1299_CLKS = 1;
    ADS1299_PWDN = 0;
    ADS1299_RESET = 0;
    delay_ms(150);
    ADS1299_RESET = 1;
    delay_us(20);
    ADS1299_START = 0;
    ADS1299_CS = 0; 
}

/**ADS1299上电复位 **/
void ADS_PowerOnInit(void)
{	
    u8 buffer;   
    ADS1299_PWDN = 1;        //上电
    delay_ms(1000);          //wait for stable, no less than 150ms
    SPI_TX(ADS_RESET);
    delay_ms(1);   
    SPI_TX(SDATAC);//RDATAC模式下，RREG会被忽略 
    
    buffer = ads1299_read_reg(ID);
    while(buffer != 0x3e){
        buffer = ads1299_read_reg(ID);   
        printf("The divece ID is : %x\r\n",buffer);
    }
    
    /*fc for bias test*/
    while(ads1299_read_reg(CONFIG3)!=0xe0){
        ads1299_write_reg(CONFIG3,0Xe0);	//使用内部参考电压，BIASREF使用内部产生（AVDD+AVSS）/2，使能BIAS buffer ec
    };
    
    delay_ms(150);//等待内部参考电压稳定
    
    while(ads1299_read_reg(MISC1) != 0x20){
        ads1299_write_reg(MISC1,0x20);	//SRB1
        LED1 = 0;//亮
    };
    
    ads1299_write_reg(CONFIG1,0x96);	//  250Hz 0x96;500hz 0x95;1k 0x94;2k 0x93;4k 0x92;8k 0x91;16k  0x90;
    while(ads1299_read_reg(CONFIG1) != 0x96){
        ads1299_write_reg(CONFIG1,0x96);	//amplified x1
    };
    

    while(ads1299_read_reg(CONFIG2) != 0xC0){
        ads1299_write_reg(CONFIG2,0xC0);	//amplified x1
    };
    
    //ads1299_write_reg(CONFIG2,0xD0);	//测试信号内部产生，频率为f/(2^21)

//各通道配置    
    while(ads1299_read_reg(CH1SET) != 0x60){
        ads1299_write_reg(CH1SET,0x60);	//off
    };    
    while(ads1299_read_reg(CH2SET) != 0x60){
        ads1299_write_reg(CH2SET,0x60);	//on
    };   
    while(ads1299_read_reg(CH3SET) != 0x60){
        ads1299_write_reg(CH3SET,0x60);	//on
    };
    while(ads1299_read_reg(CH4SET) != 0x60){
        ads1299_write_reg(CH4SET,0x60);	//off
    };    
    while(ads1299_read_reg(CH5SET) != 0x60){
        ads1299_write_reg(CH5SET,0X60);	//off
    };   
    while(ads1299_read_reg(CH6SET) != 0x60){
        ads1299_write_reg(CH6SET,0X60);	//off
    };
    while(ads1299_read_reg(CH7SET) != 0x60){
        ads1299_write_reg(CH7SET,0X60);	//off
    };   
    while(ads1299_read_reg(CH8SET) != 0x60){
        ads1299_write_reg(CH8SET,0X60);	//off
    };   
}

void ads1299_write_reg(uint8_t ADDR, uint8_t VAL)
{ 
    //ADS1299_CS = 0;  
    // SEND FIRST BYTE: 0x2 | ADDR
    SPI_TX(WREG | ADDR);
    // SEND SECOND BYTE: NUMBER_TO_WRITE
    SPI_TX(0x00);
    // SEND VALUE TO WRITE
    SPI_TX(VAL);
    // WAIT 2*TCLK'S = 888ns = 1us
    delay_us(1);
    //ADS1299_CS = 1;  
}
uint8_t ads1299_read_reg(uint8_t ADDR)
{
    //ADS1299_CS = 0;  
    // SEND FIRST BYTE: 0x2 | ADDR
    SPI_TX(RREG|ADDR);
    // SEND SECOND BYTE: NUMBER_TO_READ -- pretty much always 0x0
    SPI_TX(0x00);
    // SEND A DUMMY BYTE TO RECIEVE DATA
    uint8_t RESP = SPI_TX(0x00);
    // WAIT 2*TCLK'S = 888ns = 1us
    delay_us(2);
    // Return read value
    //ADS1299_CS = 1;  
    return (RESP);
}

/*读取72位的数据1100+LOFF_STAT[4:0]+GPIO[1:0]+13个0+2CHx24位，共9字节*/	
void ADS_Read(unsigned char *data)
{	
    unsigned char i; 
	for (i=0;i<27;i++)
	{
		*data=SPI_TX(0X00);
		data++;		
	}
}


void ads_data_process()
{ 
    static int count = 0;
    static int ticktick = 0;
    
//    static uint8_t DATA_ADS[27]={0x00};
//    static uint32_t HEX[8] = {0};//未转换的电压值u16 Data = 0;
    static uint8_t DATA_REC[27] = {0x00}; 
    SPI_TX (RDATA);
    ADS_Read(DATA_REC);
    
    memcpy(ads_data.eeg_data, DATA_REC+3,24);

    if(ticktick < TICKTICK){
        memcpy(uart_send_data + ticktick * 33, &ads_data, 33);
        ticktick++;
    }
    else{
        if(stream_data != 0){
            #if USE_USART_DMA_TX
            MYDMA_Enable(SEND_BUF_SIZE);
            #else
            USARTX_Print((uint8_t*)&ads_data, SEND_BUF_SIZE);
            #endif
        }
        ticktick = 0;
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
}

void show_ads_info(void)
{
    printf("OpenBCI V3 8-16 channel\r\n");
    printf("On Board ADS1299 Device ID: 0X%X\r\n",ads1299_read_reg(ID));
    //printf("LIS3DH Device ID: 0x00\r\n");
    printf("Firmware: v0.0.1\r\n");
    printf("$$$\r\n");
}

void processChar(char character){
    switch(character){
        case OPENBCI_MISC_SOFT_RESET:
            ads1299_init(); //软复位
            break;
        case OPENBCI_STREAM_START:
            stream_data = 1; //打开数据流
            break;
        case OPENBCI_STREAM_STOP:
            stream_data = 0; //关闭数据流
            break;
        default:
            ;          
    }
}

void ads_con_exchange(void){
    static uint8_t mode = 0;
    if(mode == 1){
        while(ads1299_read_reg(MISC1) != 0x20){
            ads1299_write_reg(MISC1,0x20);	//使用SRB1
            LED1 = 0;//亮
        };
    }
    else{
        while(ads1299_read_reg(MISC1) != 0x00){
            ads1299_write_reg(MISC1,0x00);	//不使用SRB1
            LED1 = 1;//灭
        };    
    }
    mode = !mode;
}
