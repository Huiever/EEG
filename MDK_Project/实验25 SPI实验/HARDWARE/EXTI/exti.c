#include "exti.h"
#include "delay.h" 
#include "led.h" 
#include "ads1299.h"
#include "usart.h"
#include "spi.h"
/* Private variables ---------------------------------------------------------*/
uint8_t DATA_ADS[27]={0x00};
uint8_t DATA_Txd_EEG[9]={0xa5,0x5a,0x00,0x00,0x00,0x00,0x00,0x00,0xf7};
uint32_t HEX1[410];
uint32_t SHURU_EEG[800];
//uint32_t SHURU_EEG1[800];
//uint32_t SHURU_EEG11[800];
//uint32_t SHURU_EEG22[800];
unsigned int k=0;
unsigned int e;//mian()�е��ж�
float z[410]={0.0};//�˲�����ֵ
float z_o[410]={0.0};//��һ���ڲɼ��ĵ�ѹֵ
float z_n[410]={0.0};//�����ڲɼ��ĵ�ѹֵ
float b;//��������
float D;//�˲������
unsigned int c;//�˲�����
unsigned int n;//�ܵ��˲�����
unsigned int j=0;
int h=0;       //�ж��Ƿ��ʼ��
int f;
uint32_t HEX;//δת���ĵ�ѹֵ
u8 Data = 0;

//�ⲿ�ж�2�������
void EXTI0_IRQHandler(void)
{
	if (EXTI_GetFlagStatus(EXTI_Line0)==SET)
	{
        ADS1299_CS=0; 
        ADS_SPI (RDATA);
//      TEST_PIN_H;
//		EXTI_ClearITPendingBit(EXTI_Line0);
		ADS_Read(DATA_ADS);
//		DATA_Txd_EEG[2]=DATA_ADS[3];
//		DATA_Txd_EEG[3]=DATA_ADS[4];
//		DATA_Txd_EEG[4]=DATA_ADS[5];
//		DATA_Txd_EEG[5]=DATA_ADS[9];
//		DATA_Txd_EEG[6]=DATA_ADS[10];
//		DATA_Txd_EEG[7]=DATA_ADS[11];

		HEX=(DATA_ADS[6]<<16)|(DATA_ADS[7]<<8)|DATA_ADS[8];
		
		if (HEX & 0x800000)
			Data=(16777216-HEX)*(-4500.0)/8388607;
		else
			Data = HEX*(4500.0)/8388607;
			
		if (j<10)						 //�ɼ�����
        {
			z_n[h]=Data;
			h++;h=h%410;
		}
		else
		{
//			TIM3->CNT=0;//����TIM3��ʱ���ļ���ֵ
//			timeout=0;
			
			
			for (f=0;f<410;f++)//��������
			{
				z[f]=z_n[h];h++;h=h%410;
			}
			z_n[h]=Data;			 //��������
			h++;h=h%410;
			
			j=0;//�ڶ���ѭ��ֻ��ֵ9�Σ���֤���ݲ���
			e=1;
			
//			k=TIM_GetCounter(TIM3);
//			time=TIM3->CNT+(u32)timeout*65536;
//			printf("timeout=%u\r\n", timeout); 
//			printf("time=%u\r\n", time); 

		}
		j++;
        ADS1299_CS=1;         
    }
	EXTI_ClearITPendingBit(EXTI_Line0);//���LINE2�ϵ��жϱ�־λ 
}

	   
//�ⲿ�жϳ�ʼ������
//��ʼ��PE2~4,PA0Ϊ�ж�����.
void EXTIX_Init(void)
{
	NVIC_InitTypeDef   NVIC_InitStructure;
	EXTI_InitTypeDef   EXTI_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);//ʹ��SYSCFGʱ��
	
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource0);//PE2 ���ӵ��ж���2

    /* ����EXTI_Line2 */
    EXTI_InitStructure.EXTI_Line = EXTI_Line0;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;//�ж��¼�
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; //�½��ش���
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;//�ж���ʹ��
    EXTI_Init(&EXTI_InitStructure);//����

    NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;//�ⲿ�ж�2
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;//��ռ���ȼ�1
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;//�����ȼ�2
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//ʹ���ⲿ�ж�ͨ��
    NVIC_Init(&NVIC_InitStructure);//����
	   
}












