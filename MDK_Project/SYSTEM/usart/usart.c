#include "sys.h"
#include "usart.h"	
#include "ads1299.h"


#if USE_BLE_USART
    #define USARTX    USART3
#else
    #define USARTX    USART1
#endif

u8 uart_send_data[SEND_BUF_SIZE];
//注意,读取USARTx->SR能避免莫名其妙的错误   	
u8 USART_RX_BUF[USART_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
u16 USART_RX_STA=0;       //接收状态标记

//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{ 	
	while((USARTX->SR&0X40)==0);//循环发送,直到发送完毕   
	USARTX->DR = (u8) ch;      
	return ch;
}

void USARTX_sendChar(uint8_t ch) {
    while ((USARTX->SR & 0X40) == 0);
    USARTX->DR = (u8)ch;
}
void USARTX_Print(uint8_t* ch, int len) {
    int i = 0;
    for (i = 0; i < len; i++) {
        USARTX_sendChar((u8)ch[i]);
    }
}
#if USE_BLE_USART 

void ble_init(void){
    GPIO_InitTypeDef GPIO_InitStructure;
    
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //使能GPIOA时钟

    //USART3端口配置
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12; 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//输出
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
    GPIO_Init(GPIOA,&GPIO_InitStructure); //初始化PA11，PA12
    
    GPIO_SetBits(GPIOA,GPIO_Pin_11); //下降沿唤醒
    GPIO_ResetBits(GPIOA,GPIO_Pin_12); //下降沿唤醒
}

//串口3中断服务程序
static void USART3_DMA_Tx_Config(u32 mar,u16 ndtr);

//初始化IO 串口3 
//bound:波特率
void uart_init(u32 bound){
    //GPIO端口设置
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE); //使能GPIOC时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);//使能USART3时钟

    //串口1对应引脚复用映射
    GPIO_PinAFConfig(GPIOD,GPIO_PinSource8,GPIO_AF_USART3); //GPIOC9复用为USART3
    GPIO_PinAFConfig(GPIOD,GPIO_PinSource9,GPIO_AF_USART3); //GPIOC10复用为USART3

    //USART3端口配置
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9; //GPIOC9与GPIOC10
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
    GPIO_Init(GPIOD,&GPIO_InitStructure); //初始化PA9，PA10

    //USART3 初始化设置
    USART_InitStructure.USART_BaudRate = bound;//波特率设置
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
    USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
    USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
    USART_Init(USART3, &USART_InitStructure); //初始化串口1

    USART_Cmd(USART3, ENABLE);  //使能串口1 
    USART_ClearFlag(USART3, USART_FLAG_TC);
	
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//开启相关中断

    //USART3 NVIC 配置
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;//串口1中断通道
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//抢占优先级3
    NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//子优先级3
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
    NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、

#if USE_USART_DMA_TX   
    USART3_DMA_Tx_Config((u32)uart_send_data,SEND_BUF_SIZE);// 开启串口DMA发送
    USART_DMACmd(USART3,USART_DMAReq_Tx,ENABLE);  //使能串口1的DMA发送
#endif
    
    ble_init();
}


void USART3_IRQHandler(void)                	//串口1中断服务程序
{
	u8 Res;
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
	{
		Res =USART_ReceiveData(USART3);//(USART3->DR);	//读取接收到的数据
		processChar(Res);		 
  } 

} 

#if USE_USART_DMA_TX
static void USART3_DMA_Tx_Config(u32 mar,u16 ndtr)
{
    DMA_InitTypeDef DMA_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    
    // 开启DMA时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);//DMA1时钟使能 
    DMA_DeInit(DMA1_Stream3);

    while (DMA_GetCmdStatus(DMA1_Stream3) != DISABLE){}//等待DMA可配置 

    /* 配置 DMA Stream */
    DMA_InitStructure.DMA_Channel = DMA_Channel_4;  //通道选择
    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART3->DR;//DMA外设地址
    DMA_InitStructure.DMA_Memory0BaseAddr = mar;//DMA 存储器0地址
    DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;//存储器到外设模式
    DMA_InitStructure.DMA_BufferSize = ndtr;//数据传输量 
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设非增量模式
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//存储器增量模式
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//外设数据长度:8位
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//存储器数据长度:8位
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;// 使用循环模式 
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;//中等优先级
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;//存储器突发单次传输
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//外设突发单次传输
    DMA_Init(DMA1_Stream3, &DMA_InitStructure);//初始化DMA Stream

    // 清除DMA所有标志
    DMA_ClearFlag(DMA1_Stream3,DMA_FLAG_TCIF3);
    DMA_ITConfig(DMA1_Stream3, DMA_IT_TE, ENABLE);
    
    /* Enable the DMA Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream3_IRQn;   // 发送DMA通道的中断配置 
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;  // 优先级设置 
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; 
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
    NVIC_Init(&NVIC_InitStructure); 
    DMA_ITConfig(DMA1_Stream3, DMA_IT_TC, ENABLE);// 开启DMA通道传输完成中断 
    // 不使能DMA
    DMA_Cmd (DMA1_Stream3,DISABLE);
}

//开启一次DMA传输
//DMA_Streamx:DMA数据流,DMA1_Stream0~7/DMA2_Stream0~7 
//ndtr:数据传输量  
void MYDMA_Enable(u16 ndtr)
{
    DMA_Stream_TypeDef *DMA_Streamx = DMA1_Stream3;
    
//	DMA_Cmd(DMA_Streamx, DISABLE);                      //关闭DMA传输 
//	

    while (DMA_GetCmdStatus(DMA_Streamx) != DISABLE){

    }	//确保DMA可以被设置  
		
	DMA_SetCurrDataCounter(DMA_Streamx,ndtr);          //数据传输量  
 
	DMA_Cmd(DMA_Streamx, ENABLE);                      //开启DMA传输 
}	


//DMA 发送应用源码 
void DMA1_Stream3_IRQHandler(void) 
{      
    if(DMA_GetITStatus(DMA1_Stream3, DMA_IT_TCIF3)) 
    { 
//        printf("DMA1中断\r\n"); 
        DMA_ClearFlag(DMA1_Stream3, DMA_IT_TCIF3);// 清除标志 
        DMA_Cmd(DMA1_Stream3, DISABLE);           // 关闭DMA通道 
    } 
} 

#endif

#else 

	

static void USART1_DMA_Tx_Config(u32 mar,u16 ndtr);

//初始化IO 串口1 
//bound:波特率
void uart_init(u32 bound){
    //GPIO端口设置
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //使能GPIOA时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//使能USART1时钟

    //串口1对应引脚复用映射
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1); //GPIOA9复用为USART1
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); //GPIOA10复用为USART1

    //USART1端口配置
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; //GPIOA9与GPIOA10
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
    GPIO_Init(GPIOA,&GPIO_InitStructure); //初始化PA9，PA10

    //USART1 初始化设置
    USART_InitStructure.USART_BaudRate = bound;//波特率设置
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
    USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
    USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
    USART_Init(USART1, &USART_InitStructure); //初始化串口1

    USART_Cmd(USART1, ENABLE);  //使能串口1 

    USART_ClearFlag(USART1, USART_FLAG_TC);

	
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启相关中断

    //Usart1 NVIC 配置
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;//串口1中断通道
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//抢占优先级3
    NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//子优先级3
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
    NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、


#if USE_USART_DMA_TX   
    USART1_DMA_Tx_Config((u32)uart_send_data,SEND_BUF_SIZE);// 开启串口DMA发送
    USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);  //使能串口1的DMA发送
#endif
	
}


void USART1_IRQHandler(void)                	//串口1中断服务程序
{
	u8 Res;
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
	{
		Res =USART_ReceiveData(USART1);//(USART1->DR);	//读取接收到的数据
		processChar(Res);		 
  } 

} 



#if USE_USART_DMA_TX
static void USART1_DMA_Tx_Config(u32 mar,u16 ndtr)
{
    DMA_InitTypeDef DMA_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    
    // 开启DMA时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);//DMA1时钟使能 
    DMA_DeInit(DMA2_Stream7);

    while (DMA_GetCmdStatus(DMA2_Stream7) != DISABLE){}//等待DMA可配置 

    /* 配置 DMA Stream */
    DMA_InitStructure.DMA_Channel = DMA_Channel_4;  //通道选择
    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART1->DR;//DMA外设地址
    DMA_InitStructure.DMA_Memory0BaseAddr = mar;//DMA 存储器0地址
    DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;//存储器到外设模式
    DMA_InitStructure.DMA_BufferSize = ndtr;//数据传输量 
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设非增量模式
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//存储器增量模式
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//外设数据长度:8位
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//存储器数据长度:8位
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;// 使用循环模式 
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;//中等优先级
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;//存储器突发单次传输
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//外设突发单次传输
    DMA_Init(DMA2_Stream7, &DMA_InitStructure);//初始化DMA Stream

    // 清除DMA所有标志
    DMA_ClearFlag(DMA2_Stream7,DMA_FLAG_TCIF7);
    DMA_ITConfig(DMA2_Stream7, DMA_IT_TE, ENABLE);
    
    /* Enable the DMA Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream7_IRQn;   // 发送DMA通道的中断配置 
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;  // 优先级设置 
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; 
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
    NVIC_Init(&NVIC_InitStructure); 
    DMA_ITConfig(DMA2_Stream7, DMA_IT_TC, ENABLE);// 开启DMA通道传输完成中断 
    // 不使能DMA
    DMA_Cmd (DMA2_Stream7,DISABLE);
}

//开启一次DMA传输 DMA2_Stream7
//ndtr:数据传输量  
void MYDMA_Enable(u16 ndtr)
{
    DMA_Stream_TypeDef *DMA_Streamx = DMA2_Stream7;
    
//	DMA_Cmd(DMA_Streamx, DISABLE);                      //关闭DMA传输 
//	

    while (DMA_GetCmdStatus(DMA_Streamx) != DISABLE){

    }	//确保DMA可以被设置  
		
	DMA_SetCurrDataCounter(DMA_Streamx,ndtr);          //数据传输量  
 
	DMA_Cmd(DMA_Streamx, ENABLE);                      //开启DMA传输 
}	


//DMA 发送应用源码 
void DMA2_Stream7_IRQHandler(void) 
{      
    if(DMA_GetITStatus(DMA2_Stream7, DMA_IT_TCIF7)) 
    { 
//        printf("DMA1中断\r\n"); 
        DMA_ClearFlag(DMA2_Stream7, DMA_IT_TCIF7);// 清除标志 
        DMA_Cmd(DMA2_Stream7, DISABLE);           // 关闭DMA通道 
    } 
} 

#endif
#endif
