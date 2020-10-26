#include "sys.h"
#include "usart.h"	
#include "ads1299.h"


#if USE_BLE_USART
    #define USARTX    USART3
#else
    #define USARTX    USART1
#endif

u8 uart_send_data[SEND_BUF_SIZE];
//ע��,��ȡUSARTx->SR�ܱ���Ī������Ĵ���   	
u8 USART_RX_BUF[USART_REC_LEN];     //���ջ���,���USART_REC_LEN���ֽ�.
u16 USART_RX_STA=0;       //����״̬���

//�������´���,֧��printf����,������Ҫѡ��use MicroLIB	  
#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//�ض���fputc���� 
int fputc(int ch, FILE *f)
{ 	
	while((USARTX->SR&0X40)==0);//ѭ������,ֱ���������   
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
    
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //ʹ��GPIOAʱ��

    //USART3�˿�����
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12; 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//���
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
    GPIO_Init(GPIOA,&GPIO_InitStructure); //��ʼ��PA11��PA12
    
    GPIO_SetBits(GPIOA,GPIO_Pin_11); //�½��ػ���
    GPIO_ResetBits(GPIOA,GPIO_Pin_12); //�½��ػ���
}

//����3�жϷ������
static void USART3_DMA_Tx_Config(u32 mar,u16 ndtr);

//��ʼ��IO ����3 
//bound:������
void uart_init(u32 bound){
    //GPIO�˿�����
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE); //ʹ��GPIOCʱ��
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);//ʹ��USART3ʱ��

    //����1��Ӧ���Ÿ���ӳ��
    GPIO_PinAFConfig(GPIOD,GPIO_PinSource8,GPIO_AF_USART3); //GPIOC9����ΪUSART3
    GPIO_PinAFConfig(GPIOD,GPIO_PinSource9,GPIO_AF_USART3); //GPIOC10����ΪUSART3

    //USART3�˿�����
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9; //GPIOC9��GPIOC10
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
    GPIO_Init(GPIOD,&GPIO_InitStructure); //��ʼ��PA9��PA10

    //USART3 ��ʼ������
    USART_InitStructure.USART_BaudRate = bound;//����������
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
    USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
    USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
    USART_Init(USART3, &USART_InitStructure); //��ʼ������1

    USART_Cmd(USART3, ENABLE);  //ʹ�ܴ���1 
    USART_ClearFlag(USART3, USART_FLAG_TC);
	
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//��������ж�

    //USART3 NVIC ����
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;//����1�ж�ͨ��
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//��ռ���ȼ�3
    NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//�����ȼ�3
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
    NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ�����

#if USE_USART_DMA_TX   
    USART3_DMA_Tx_Config((u32)uart_send_data,SEND_BUF_SIZE);// ��������DMA����
    USART_DMACmd(USART3,USART_DMAReq_Tx,ENABLE);  //ʹ�ܴ���1��DMA����
#endif
    
    ble_init();
}


void USART3_IRQHandler(void)                	//����1�жϷ������
{
	u8 Res;
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)  //�����ж�(���յ������ݱ�����0x0d 0x0a��β)
	{
		Res =USART_ReceiveData(USART3);//(USART3->DR);	//��ȡ���յ�������
		processChar(Res);		 
  } 

} 

#if USE_USART_DMA_TX
static void USART3_DMA_Tx_Config(u32 mar,u16 ndtr)
{
    DMA_InitTypeDef DMA_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    
    // ����DMAʱ��
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);//DMA1ʱ��ʹ�� 
    DMA_DeInit(DMA1_Stream3);

    while (DMA_GetCmdStatus(DMA1_Stream3) != DISABLE){}//�ȴ�DMA������ 

    /* ���� DMA Stream */
    DMA_InitStructure.DMA_Channel = DMA_Channel_4;  //ͨ��ѡ��
    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART3->DR;//DMA�����ַ
    DMA_InitStructure.DMA_Memory0BaseAddr = mar;//DMA �洢��0��ַ
    DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;//�洢��������ģʽ
    DMA_InitStructure.DMA_BufferSize = ndtr;//���ݴ����� 
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//���������ģʽ
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//�洢������ģʽ
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//�������ݳ���:8λ
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//�洢�����ݳ���:8λ
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;// ʹ��ѭ��ģʽ 
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;//�е����ȼ�
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;//�洢��ͻ�����δ���
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//����ͻ�����δ���
    DMA_Init(DMA1_Stream3, &DMA_InitStructure);//��ʼ��DMA Stream

    // ���DMA���б�־
    DMA_ClearFlag(DMA1_Stream3,DMA_FLAG_TCIF3);
    DMA_ITConfig(DMA1_Stream3, DMA_IT_TE, ENABLE);
    
    /* Enable the DMA Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream3_IRQn;   // ����DMAͨ�����ж����� 
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;  // ���ȼ����� 
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; 
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
    NVIC_Init(&NVIC_InitStructure); 
    DMA_ITConfig(DMA1_Stream3, DMA_IT_TC, ENABLE);// ����DMAͨ����������ж� 
    // ��ʹ��DMA
    DMA_Cmd (DMA1_Stream3,DISABLE);
}

//����һ��DMA����
//DMA_Streamx:DMA������,DMA1_Stream0~7/DMA2_Stream0~7 
//ndtr:���ݴ�����  
void MYDMA_Enable(u16 ndtr)
{
    DMA_Stream_TypeDef *DMA_Streamx = DMA1_Stream3;
    
//	DMA_Cmd(DMA_Streamx, DISABLE);                      //�ر�DMA���� 
//	

    while (DMA_GetCmdStatus(DMA_Streamx) != DISABLE){

    }	//ȷ��DMA���Ա�����  
		
	DMA_SetCurrDataCounter(DMA_Streamx,ndtr);          //���ݴ�����  
 
	DMA_Cmd(DMA_Streamx, ENABLE);                      //����DMA���� 
}	


//DMA ����Ӧ��Դ�� 
void DMA1_Stream3_IRQHandler(void) 
{      
    if(DMA_GetITStatus(DMA1_Stream3, DMA_IT_TCIF3)) 
    { 
//        printf("DMA1�ж�\r\n"); 
        DMA_ClearFlag(DMA1_Stream3, DMA_IT_TCIF3);// �����־ 
        DMA_Cmd(DMA1_Stream3, DISABLE);           // �ر�DMAͨ�� 
    } 
} 

#endif

#else 

	

static void USART1_DMA_Tx_Config(u32 mar,u16 ndtr);

//��ʼ��IO ����1 
//bound:������
void uart_init(u32 bound){
    //GPIO�˿�����
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //ʹ��GPIOAʱ��
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//ʹ��USART1ʱ��

    //����1��Ӧ���Ÿ���ӳ��
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1); //GPIOA9����ΪUSART1
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); //GPIOA10����ΪUSART1

    //USART1�˿�����
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; //GPIOA9��GPIOA10
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
    GPIO_Init(GPIOA,&GPIO_InitStructure); //��ʼ��PA9��PA10

    //USART1 ��ʼ������
    USART_InitStructure.USART_BaudRate = bound;//����������
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
    USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
    USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
    USART_Init(USART1, &USART_InitStructure); //��ʼ������1

    USART_Cmd(USART1, ENABLE);  //ʹ�ܴ���1 

    USART_ClearFlag(USART1, USART_FLAG_TC);

	
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//��������ж�

    //Usart1 NVIC ����
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;//����1�ж�ͨ��
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//��ռ���ȼ�3
    NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//�����ȼ�3
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
    NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ�����


#if USE_USART_DMA_TX   
    USART1_DMA_Tx_Config((u32)uart_send_data,SEND_BUF_SIZE);// ��������DMA����
    USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);  //ʹ�ܴ���1��DMA����
#endif
	
}


void USART1_IRQHandler(void)                	//����1�жϷ������
{
	u8 Res;
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //�����ж�(���յ������ݱ�����0x0d 0x0a��β)
	{
		Res =USART_ReceiveData(USART1);//(USART1->DR);	//��ȡ���յ�������
		processChar(Res);		 
  } 

} 



#if USE_USART_DMA_TX
static void USART1_DMA_Tx_Config(u32 mar,u16 ndtr)
{
    DMA_InitTypeDef DMA_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    
    // ����DMAʱ��
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);//DMA1ʱ��ʹ�� 
    DMA_DeInit(DMA2_Stream7);

    while (DMA_GetCmdStatus(DMA2_Stream7) != DISABLE){}//�ȴ�DMA������ 

    /* ���� DMA Stream */
    DMA_InitStructure.DMA_Channel = DMA_Channel_4;  //ͨ��ѡ��
    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART1->DR;//DMA�����ַ
    DMA_InitStructure.DMA_Memory0BaseAddr = mar;//DMA �洢��0��ַ
    DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;//�洢��������ģʽ
    DMA_InitStructure.DMA_BufferSize = ndtr;//���ݴ����� 
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//���������ģʽ
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//�洢������ģʽ
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//�������ݳ���:8λ
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//�洢�����ݳ���:8λ
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;// ʹ��ѭ��ģʽ 
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;//�е����ȼ�
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;//�洢��ͻ�����δ���
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//����ͻ�����δ���
    DMA_Init(DMA2_Stream7, &DMA_InitStructure);//��ʼ��DMA Stream

    // ���DMA���б�־
    DMA_ClearFlag(DMA2_Stream7,DMA_FLAG_TCIF7);
    DMA_ITConfig(DMA2_Stream7, DMA_IT_TE, ENABLE);
    
    /* Enable the DMA Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream7_IRQn;   // ����DMAͨ�����ж����� 
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;  // ���ȼ����� 
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; 
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
    NVIC_Init(&NVIC_InitStructure); 
    DMA_ITConfig(DMA2_Stream7, DMA_IT_TC, ENABLE);// ����DMAͨ����������ж� 
    // ��ʹ��DMA
    DMA_Cmd (DMA2_Stream7,DISABLE);
}

//����һ��DMA���� DMA2_Stream7
//ndtr:���ݴ�����  
void MYDMA_Enable(u16 ndtr)
{
    DMA_Stream_TypeDef *DMA_Streamx = DMA2_Stream7;
    
//	DMA_Cmd(DMA_Streamx, DISABLE);                      //�ر�DMA���� 
//	

    while (DMA_GetCmdStatus(DMA_Streamx) != DISABLE){

    }	//ȷ��DMA���Ա�����  
		
	DMA_SetCurrDataCounter(DMA_Streamx,ndtr);          //���ݴ�����  
 
	DMA_Cmd(DMA_Streamx, ENABLE);                      //����DMA���� 
}	


//DMA ����Ӧ��Դ�� 
void DMA2_Stream7_IRQHandler(void) 
{      
    if(DMA_GetITStatus(DMA2_Stream7, DMA_IT_TCIF7)) 
    { 
//        printf("DMA1�ж�\r\n"); 
        DMA_ClearFlag(DMA2_Stream7, DMA_IT_TCIF7);// �����־ 
        DMA_Cmd(DMA2_Stream7, DISABLE);           // �ر�DMAͨ�� 
    } 
} 

#endif
#endif
