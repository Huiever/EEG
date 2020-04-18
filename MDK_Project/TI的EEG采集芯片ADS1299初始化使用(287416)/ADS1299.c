#define __ADS1299_C__


#include "ADS1299.h"
#include "user_coreconfig.h"
#include "User_handle.h"
#include "global.h"
#include "delay.h"
#include "printf.h"
#include "spi.h"
#include "queue.h"

void ADS1299Init(void)
{
	ADS1299_GPIO_init();
	Init_ADS1299_Stop_Key_Interrupt();
	SPIx_Init();
	Init_ADS1x9x ();
	Disable_ADS1x9x_Interrupt ();
	Soft_Stop_ADS1x9x ();           //停止AD转换
	Stop_Read_Data_Continuous ();   //停止连续转换
	
}

void ADS1299_GPIO_init(void)
{

  //STM32F103控制ADS1298引脚定义
  GPIO_InitTypeDef GPIO_InitStruct;
     //总线时钟允许
  /* Enable GPIOA GPIOB GPIOC clock */
 
  //PWDN --PD10	  
  GPIO_InitStruct.GPIO_Pin = ADS1299_PWDN_PIN;  
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_Mode	= GPIO_Mode_Out_PP;	//作为输出端口
  GPIO_Init(ADS1299_PWDN_PORT, &GPIO_InitStruct) ;

  //START --PB0
  GPIO_InitStruct.GPIO_Pin = ADS1299_START_PIN;  
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_Mode	= GPIO_Mode_Out_PP;	//作为输出端口
  GPIO_Init(ADS1299_START_PORT, &GPIO_InitStruct) ;

  //RESET --PA3
  GPIO_InitStruct.GPIO_Pin = ADS1299_RESET_PIN;  
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_Mode	= GPIO_Mode_Out_PP;	//作为输出端口
  GPIO_Init(ADS1299_RESET_PORT, &GPIO_InitStruct) ;

  //CLKSEL --PC4
  GPIO_InitStruct.GPIO_Pin = ADS1299_CLKSEL_PIN;  //PC4
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_Mode	= GPIO_Mode_Out_PP;	//作为输出端口
  GPIO_Init(ADS1299_CLKSEL_PORT, &GPIO_InitStruct) ;

  /* Configure I/O for Flash Chip select */
  GPIO_InitStruct.GPIO_Pin = ADS1299_CS_PIN;  //SPI CS
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;  //复用推挽输出
  GPIO_Init(ADS1299_CS_PORT, &GPIO_InitStruct);

}


void Init_ADS1x9x_Data_Ready_Interrupt (void)
{
	
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	GPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Speed	= GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode	= GPIO_Mode_IPU;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOD,GPIO_PinSource8);

	#if 0
	 /* Configure the EXTI9*/ 
	EXTI_InitStruct_PD8.EXTI_Line	= EXTI_Line8;
	EXTI_InitStruct_PD8.EXTI_Mode	= EXTI_Mode_Interrupt;
	EXTI_InitStruct_PD8.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStruct_PD8.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStruct_PD8);
	#endif
	/* Enable the EXTI Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
}

void Enable_ADS1x9x_Interrupt (void)
{
	EXTI_ClearITPendingBit(EXTI_Line8);
	EXTI_InitStruct_PD8.EXTI_Line	 = EXTI_Line8;
	EXTI_InitStruct_PD8.EXTI_Mode	 = EXTI_Mode_Interrupt;
	EXTI_InitStruct_PD8.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStruct_PD8.EXTI_LineCmd = ENABLE; 
	EXTI_Init(&EXTI_InitStruct_PD8);
}

void Disable_ADS1x9x_Interrupt (void)
{
	EXTI_ClearITPendingBit(EXTI_Line8);
	EXTI_InitStruct_PD8.EXTI_Line	 = EXTI_Line8;
	EXTI_InitStruct_PD8.EXTI_Mode	 = EXTI_Mode_Interrupt;
	EXTI_InitStruct_PD8.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStruct_PD8.EXTI_LineCmd = DISABLE; 
	EXTI_Init(&EXTI_InitStruct_PD8);
}

void Init_ADS1299_Stop_Key_Interrupt (void)
{
	
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	EXTI_InitTypeDef EXTI_InitStruct;

	//GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable , ENABLE);
	
	GPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed	= GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode	= GPIO_Mode_IPU;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOD,GPIO_PinSource9);
	
	 /* Configure the EXTI9*/ 
	EXTI_InitStruct.EXTI_Line	= EXTI_Line9;
	EXTI_InitStruct.EXTI_Mode	= EXTI_Mode_Interrupt;
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;

	EXTI_ClearITPendingBit(EXTI_Line9);
	
	EXTI_Init(&EXTI_InitStruct);

	/* Enable the EXTI Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
}


void Set_ADS1x9x_Start_Pin (unsigned char state)
{
	if(state==RESET)
	{
		ADS1298_StartLow();			
	}
	else if(state==SET)
    {
    	ADS1298_StartHigh();    	
    }
}	

void Init_ADS1x9x_IO (void)
{
  // LD TODO:  This could probably be modularized in a better way (less repeated code)
       
    // Power Down Pin (ACTIVE LOW)
    ADS1298_PowerDown();                                       // HIGH, LOW
   
    // RESET Pin (ACTIVE LOW)
    ADS1298_ResetHigh();                                       // HIGH, LOW 

	//Chip select High
	ADS1298_CS_High();

    // Start (ACTIVE High)
    ADS1298_StartLow();                                        // HIGH, LOW  

    // Clock Select (Low -> External Clock, High -> Internal Clock
    ADS1298_ClkselLow();                                        // HIGH, LOW  
}

void POR_Reset_ADS1x9x (void)
{
    
 
    //LD TODO: Use universal clock settings to set delays

    // Small Delay 
    ADS1298_ResetHigh();                                       // Reset HIGH
    //for (i = 0xFFFF; i > 0; i--)
    //{
	   //Delay_1us(20);
    //}
    Delay_1ms(1000);
    ADS1298_ResetLow();                                        // Reset LOW    
	Delay_1us(20);                                // Small Delay   
    ADS1298_ResetHigh();                                       // Reset HIGH
    Delay_1us(40);       
}

void Power_Down_ADS1x9x (void)
{
    ADS1298_PowerDown();
    ADS1298_ClkselLow();

}

void Power_Up_ADS1x9x (void)
{    
    ADS1298_PowerUp();
	ADS1298_StartHigh();

}


void Set_ADS1x9x_Chip_Enable (void)                        // ADS1x9x module uses GPIO as the SPI CS
{
    //此处片选是由SPI总线自动产生

	ADS1298_CS_Low();
	
	//delayUS(1);                                                   // Short Delay before invoking the SPI Port
}

void Clear_ADS1x9x_Chip_Enable (void)                      // ADS1x9x uses GPIO for SPI CS
{
    //此处片选是由SPI总线自动产生  
	ADS1298_CS_High(); 

	//delayUS(1);
}

unsigned char ADS1x9x_SPI_Data ( unsigned char Data)  // Complements the SPI_Address command 
{
    //清发送中断标志
    USR_SPI_ClearFlag(SPI1,SPI_FLAG_TXE);
    //清接收中断标志
    USR_SPI_ClearFlag(SPI1,SPI_FLAG_RXNE); 

	USR_SPI_SendData(SPI1,Data);                              // Send the data sitting at the pointer Data to the TX Buffer
	while (USR_SPI_GetFlagStatus(SPI1,SPI_FLAG_TXE) == RESET);
    while (USR_SPI_GetFlagStatus(SPI1,SPI_FLAG_RXNE) == RESET);
     //清发送中断标志
    USR_SPI_ClearFlag(SPI1,SPI_FLAG_TXE);
    //清接收中断标志
    USR_SPI_ClearFlag(SPI1,SPI_FLAG_RXNE);     

	//delayUS(10);
	//delayUS(10);
//    Clear_ADS1x9x_Chip_Enable (device_slot);                                    // Clear the Chip ENABLE to terminate the SPI transaction
    return (USR_SPI_ReceiveData(SPI1));                             // Return Status Data or Requested Read Data
}

 

void Soft_Start_ReStart_ADS1x9x (void)
{
    ADS1298_StartLow();                                                 // Start Pin LOW   
    //delayUS(20);  
	Set_ADS1x9x_Chip_Enable ();  
	delayUS(10);                                              // Small Delay   
    ADS1x9x_SPI_Data (START_RESTART_CONVERSION);                   // Send 0x08 to the ADS1x9x
    delayUS(40);
	Clear_ADS1x9x_Chip_Enable (); 
	delayUS(10);                                                      
}

void Soft_Reset_ADS1x9x (void)
{
    Set_ADS1x9x_Chip_Enable (); 
	Delay_1us(10);
	ADS1x9x_SPI_Data (RESET_CONVERTER);                            // Send 0x06 to the ADS1x9x
	Delay_1us(70);
	Clear_ADS1x9x_Chip_Enable ();
 	Delay_1us(40);
}

void Start_Read_Data_Continuous (void)
{
   	Set_ADS1x9x_Chip_Enable (); 
	delayUS(10); 
	ADS1x9x_SPI_Data (SET_READ_DATA_CONTINUOUSLY);       // Send 0x06 to the ADS1x9x
	delayUS(70);
	Clear_ADS1x9x_Chip_Enable ();
 	delayUS(40);
}


void Stop_Read_Data_Continuous (void)
{
    Set_ADS1x9x_Chip_Enable ();
	Delay_1us(10);
    ADS1x9x_SPI_Data ( STOP_READ_DATA_CONTINUOUSLY);
	Delay_1us(40);
	Clear_ADS1x9x_Chip_Enable ();
	Delay_1us(20);
}

void Wake_Up_ADS1x9x (void)
{
    Set_ADS1x9x_Chip_Enable(); 
    ADS1x9x_SPI_Data (WAKE_CONVERTER_FROM_SLEEP);                  // Send 0x02 to the ADS1x9x                                                      
}

void Put_ADS1x9x_In_Sleep (void)
{
    Set_ADS1x9x_Chip_Enable (); 
    ADS1x9x_SPI_Data (PLACE_CONVERTER_IN_SLEEP_MODE);              // Send 0x04 to the ADS1x9x
}

U8 Enable_ADS1x9x_Test_Mode (U8 Test_Signal_Source, U8 Test_Signal_Reference, U8 Test_Signal_Type)
{
    U8 Verify_status = ADS_1x9x_INIT_SUCCESS;                                      // Error state set Clear

	printf("\r\n Enable_ADS1x9x_Test_Mode %x %x %x ", Test_Signal_Source, Test_Signal_Reference, Test_Signal_Type);
	
    //Enable_ADS1x9x_Interrupt();
    
// Load Values set in the ADS1x9x
    ADS1x9x_SPI_Address_Byte_Count (READ_CONFIG_2_REGISTER, SINGLE_BYTE_READ_WRITE);          // Read Device ID, Single Byte the Part Number
    ADS1x9x_Config_2 = ADS1x9x_SPI_Data (SPI_TEST_DATA);                                 // Send Dummy variable (0x55) to return the part number (Chip Select Cleared automatically)
	printf("\r\n ADS1x9x_Config_2 %x ", ADS1x9x_Config_2);
//  Modify Values for the ADS1x9x     
    ADS1x9x_Config_2_register->Test_Signal_Frequency = Test_Signal_Type;                      // DEFAULT_PULSED_AT_CLOCK_FREQUENCY_DIVIDED_BY_2_TO_THE_21ST
                                                                                              // PULSED_AT_CLOCK_FREQUENCY_DIVIDED_BY_2_TO_THE_20TH
    printf("\r\n ADS1x9x_Config_2_register %x ", ADS1x9x_Config_2_register->Test_Signal_Frequency);	                                                                                          // AT_DC 
    ADS1x9x_Config_2_register->Test_Signal_Amplitude = Test_Signal_Reference;                 // DEFAULT_PLUS_MINUS_1_MV_TIMES_VREF_DIVIDED_BY_2_4
    printf("\r\n ADS1x9x_Config_2_register %x ", ADS1x9x_Config_2_register->Test_Signal_Amplitude);	                                                                                          // PLUS_MINUS_2_MV_TIMES_VREF_DIVIDED_BY_2_4
                                                                                              // PLUS_MINUS_10_MV_TIMES_VREF_DIVIDED_BY_2_4
                                                                                              // PLUS_MINUS_1V_TIMES_VREF_DIVIDED_BY_2_4 
    ADS1x9x_Config_2_register->Test_Source = Test_Signal_Source;                              // DEFAULT_TEST_SIGNALS_ARE_DRIVEN_EXTERNALLY, TEST_SIGNALS_ARE_DRIVEN_INTERNALLY
	printf("\r\n ADS1x9x_Config_2_register %x ", ADS1x9x_Config_2_register->Test_Source);	
	printf("\r\n ADS1x9x_Config_2 %x ", ADS1x9x_Config_2);																						  printf("\r\n ADS1x9x_Config_2 %x ", ADS1x9x_Config_2);

// Program Values into ADS1x9x
    ADS1x9x_SPI_Address_Byte_Count (WRITE_CONFIG_2_REGISTER, SINGLE_BYTE_READ_WRITE);         // Read Device ID, Single Byte the Part Number
    ADS1x9x_SPI_data = ADS1x9x_SPI_Data (ADS1x9x_Config_2);                                   // Send Dummy variable (0x55) to return the part number (Chip Select Cleared automatically)

//#ifdef VERIFY
    //  Read Back Register    
    ADS1x9x_SPI_Address_Byte_Count (READ_CONFIG_2_REGISTER, SINGLE_BYTE_READ_WRITE);          // Read Device ID, Single Byte the Part Number
    ADS1x9x_SPI_data = ADS1x9x_SPI_Data (SPI_TEST_DATA);  
    
	printf("\r\n ADS1x9x_SPI_data %x ", ADS1x9x_SPI_data);

    // Read the Value from the SPI port   
    if (ADS1x9x_SPI_data != ADS1x9x_Config_2)
    {
        Verify_status = ADS_1x9x_VERIFY_ERROR;
    }        
//  -----------------------------------
//#endif /* VERIFY */    
    return Verify_status;
}

void ADS1x9x_SPI_Address_Byte_Count ( unsigned char Read_Write_Address, unsigned char Number_Of_Bytes)
{ 
	U8 ucData = 0;
	//printf("\r\n ADS1x9x_SPI_Address_Byte_Count %x, %x", Read_Write_Address, Number_Of_Bytes);
#if 1
	while (USR_SPI_GetFlagStatus(SPI1, SPI_FLAG_TXE) == RESET); 		  
	USR_SPI_SendData(SPI1, Read_Write_Address); 
	while (USR_SPI_GetFlagStatus(SPI1, SPI_FLAG_RXNE) == RESET);
	ucData = USR_SPI_ReceiveData(SPI1);

	while (USR_SPI_GetFlagStatus(SPI1, SPI_FLAG_TXE) == RESET); 		  
	USR_SPI_SendData(SPI1, Number_Of_Bytes); 
	while (USR_SPI_GetFlagStatus(SPI1, SPI_FLAG_RXNE) == RESET);
	ucData = USR_SPI_ReceiveData(SPI1);
	//return i;
#else
	
	//清发送中断标志
	USR_SPI_ClearFlag(SPI1,SPI_FLAG_TXE);
	//清接收中断标志
	USR_SPI_ClearFlag(SPI1,SPI_FLAG_RXNE);
	USR_SPI_SendData(SPI1,Read_Write_Address);				 // Transmit the Funtion/Address payload
	while (USR_SPI_GetFlagStatus(SPI1,SPI_FLAG_TXE) == RESET);
	while (USR_SPI_GetFlagStatus(SPI1,SPI_FLAG_RXNE) == RESET);
	ucData = USR_SPI_ReceiveData(SPI1);
	//清发送中断标志
	USR_SPI_ClearFlag(SPI1,SPI_FLAG_TXE);
	//清接收中断标志
	USR_SPI_ClearFlag(SPI1,SPI_FLAG_RXNE);
	//delayUS(40);
	//此处片选是由SPI总线自动产生
	USR_SPI_SendData(SPI1,Number_Of_Bytes); 				 // Transmit the Number of Bytes payload
	while (USR_SPI_GetFlagStatus(SPI1,SPI_FLAG_TXE) == RESET);
	while (USR_SPI_GetFlagStatus(SPI1,SPI_FLAG_RXNE) == RESET);
	ucData = USR_SPI_ReceiveData(SPI1);
	//清发送中断标志
	USR_SPI_ClearFlag(SPI1,SPI_FLAG_TXE);
	//清接收中断标志
	USR_SPI_ClearFlag(SPI1,SPI_FLAG_RXNE);
#endif
//	delayUS(10);
}



unsigned char ADS1x9x_Read_Version (void)  
{
    unsigned char Version_ID = 0;
	
	Set_ADS1x9x_Chip_Enable ();
	Delay_1us(10);
    ADS1x9x_SPI_Address_Byte_Count (READ_DEVICE_ID, SINGLE_BYTE_READ_WRITE);    // Read Device ID, Single Byte the Part Number
	//delayUS(40);
	Version_ID = ADS1x9x_SPI_Data (SPI_TEST_DATA); // Send Dummy variable (0x55) to return the part number (Chip Select Cleared automatically)
	//Version_ID = ADS1x9x_SPI_Data (device_slot, SPI_TEST_DATA); // Send Dummy variable (0x55) to return the part number (Chip Select Cleared automatically)
	Delay_1us(40);
	Clear_ADS1x9x_Chip_Enable ();
	Delay_1us(20);
	return Version_ID;
}

U8 ADS1x9x_Read_Addr(U8 addr)  
{
    unsigned char Version_ID = 0;
	
	Set_ADS1x9x_Chip_Enable ();
	Delay_1us(10);
    ADS1x9x_SPI_Address_Byte_Count (addr, SINGLE_BYTE_READ_WRITE);    // Read Device ID, Single Byte the Part Number
	//delayUS(40);
	Version_ID = ADS1x9x_SPI_Data ( SPI_TEST_DATA); // Send Dummy variable (0x55) to return the part number (Chip Select Cleared automatically)
	//Version_ID = ADS1x9x_SPI_Data (device_slot, SPI_TEST_DATA); // Send Dummy variable (0x55) to return the part number (Chip Select Cleared automatically)
	Delay_1us(40);
	Clear_ADS1x9x_Chip_Enable ();
	Delay_1us(20);
	return Version_ID;
}

void ADS1299_SPI_Read_Str(U8 *str,U8 Addr, U8 Num)
{
	#if 0
	U8 device_slot = 0;
	
	Set_ADS1x9x_Chip_Enable (device_slot);
	Delay_1us(10);
    ADS1x9x_SPI_Address_Byte_Count (device_slot, READ_CONFIG_1_REGISTER, 2);    // Read Device ID, Single Byte the Part Number
	Delay_1ms(40);
	device_slot = ADS1x9x_SPI_Data (device_slot, SPI_TEST_DATA); // Send Dummy variable (0x55) to return the part number (Chip Select Cleared automatically)
	printf("\r\n device_slot %x", device_slot);
	device_slot = ADS1x9x_SPI_Data (device_slot, SPI_TEST_DATA); // Send Dummy variable (0x55) to return the part number (Chip Select Cleared automatically)
	Delay_1us(40);
	Clear_ADS1x9x_Chip_Enable (device_slot);
	Delay_1us(20);
	
	printf("\r\n device_slot %x", device_slot);
	#else
	WORD i = 0;
	ADS1298_CS_Low();
	Delay_1us(10);
    ADS1x9x_SPI_Address_Byte_Count ( Addr, Num - 1);
	Delay_1us(40);
	for(i = 0; i < Num; i++)
	{
		*(str + i) = ADS1x9x_SPI_Data (SPI_TEST_DATA);
	}
	Delay_1us(40);
	ADS1298_CS_High(); 
	Delay_1us(20);

	#endif
}

void ADS1299_SPI_Write_Str(U8 * str,U8 Addr,U8 Num)
{
	U8 i = 0, ucData = 0;
	ADS1298_CS_Low();
	Delay_1us(10);
	ADS1x9x_SPI_Address_Byte_Count (WRITE_CONFIG_1_REGISTER, 2);
	Delay_1us(40);
	for(i = 0; i < Num; i++)
	{
		ucData = SPIx_ReadWriteByte(*(str + i));
	}
	Delay_1us(40);
	ADS1298_CS_High();
	Delay_1us(20);
}


U8 Init_ADS1x9x (void)
{
    U8 Verify_Check = 0;
    U8 Module_Present = 0;
	U8 ADS1x9x_Version_ID_Number=0;
    U8 number_of_retries = 10;
    U8 str[3] = {0,0,0};
	//WORD i = 0;
	Puts("\r\n init_ADS1x9x_Data_Ready_Interrupt");
    Init_ADS1x9x_Data_Ready_Interrupt ();     // Set up the ADS1x9x Interrupt pin

    Init_ADS1x9x_IO ();    
    //Puts("\r\n Delay_1us");
    Delay_1us(10);
    Power_Up_ADS1x9x ();                 // Power up Digital portion of the ADS1x9x                                              
	
	//Puts("\r\n Power_Up_ADS1x9x");

    POR_Reset_ADS1x9x ();
    //Puts("\r\n Soft_Reset_ADS1x9x");
	//ADS1298软复位
	Soft_Reset_ADS1x9x();

	//停止连续读数
	//Puts("\r\n Stop_Read_Data_Continuous");
	Stop_Read_Data_Continuous ();
	//Puts("\r\n Module_Present");
	while (!Module_Present)                                                     // Wait for Module to be present
	{
	    if (number_of_retries)
	    {
			ADS1x9x_Version_ID_Number = ADS1x9x_Read_Version ();

			printf("\r\n ADS1x9x_Version_ID_Number %x", ADS1x9x_Version_ID_Number);
			if (ADS1x9x_Version_ID_Number == ADS1x9x_VERSION_ID)                  // (0x22 for old board, 0x42 for new one)                 
			{
				Module_Present = SET;  
			}
			
			number_of_retries--;
	    }
	    else
	    {
			Puts("\r\n ADS_1x9x_NOT_FOUND");
	        return ADS_1x9x_NOT_FOUND;
	    }
	} 

	ADS1299_SPI_Read_Str(str, READ_CONFIG_1_REGISTER, 3);
	printf("\r\n str %x", str[0]);
	printf("\r\n str %x", str[1]);
	printf("\r\n str %x", str[2]);

	str[0] = 0x91;
	str[1] = 0xC1;
	str[2] = 0x62;
	ADS1299_SPI_Write_Str(str, WRITE_CONFIG_1_REGISTER, 3);

	ADS1299_SPI_Read_Str(str, READ_CONFIG_1_REGISTER, 3);
	printf("\r\n str %x", str[0]);
	printf("\r\n str %x", str[1]);
	printf("\r\n str %x", str[2]);
	
	//从AT24C01空间读取配置参数

	 // AT24CXX_Read(Parmnent_Addr,	ADS1298_Parment,Parmnent_Length);
   	  //Verify_Check = Initialize_ADS1x9x_Registers (device_slot, (unsigned char*) ADS1x9x_download_pointer);
	  //Verify_Check = Initialize_ADS1x9x_Registers (device_slot, ADS1298_Parment);

#if 0
	Puts("\r\n 350");
    init_ADS1x9x_Via_Constant_Table (device_slot, (unsigned char*) ADS1x9x_download_pointer);
    Puts("\r\n 352");
    
    Verify_Check = verify_ADS1x9x_Registers (device_slot, (unsigned char*) ADS1x9x_download_pointer);
    Puts("\r\n 354");
    
    Verify_Check = Initialize_ADS1x9x_Data_Rate (device_slot, (*ADS1x9x_download_pointer)&0x7f/*MODULATION_FREQUENCY_DIVIDED_BY_1024*/);    // DEFAULT_MODULATION_FREQUENCY_DIVIDED_BY_16
    //#else                                                                                         // MODULATION_FREQUENCY_DIVIDED_BY_512
    Puts("\r\n 357");                                                                                // MODULATION_FREQUENCY_DIVIDED_BY_1024
    if((*ADS1x9x_download_pointer)>>7)
	{
    	Verify_Check = Initialize_ADS1x9x_Mode (device_slot, HIGH_RESOLUTION_MODE);                         // DEFAULT_LOW_POWER_MODE, HIGH_RESOLUTION_MODE
    }
	else
	{

	   Verify_Check = Initialize_ADS1x9x_Mode (device_slot, DEFAULT_LOW_POWER_MODE);
	}
	Puts("\r\n 367");
    for (i = 0; i < ECG_Num_Channels; i++)
    {
        Verify_Check = Initialize_ADS1x9x_Channel                             //  Context Save will store the previous setting of the channel 
        (
            device_slot, 
            i + 1,                                                              // references channels 1 - 8
            DEFAULT_ADS1x9x_ELECTRODE_INPUT,                                    // DEFAULT_ADS1x9x_ELECTRODE_INPUT, ADS1x9x_INPUT_SHORTED, ADS1x9x_RIGHT_LEG_DETECT, ADS1x9x_ONE_HALF_DIGITAL_SUPPLY
                                                                                // ADS1x9x_TEMPERATURE_SENSOR, ADS1x9x_CALIBRATION_SIGNAL, ADS1x9x_RIGHT_LEG_DETECT_POSITIVE, ADS1x9x_RIGHT_LEG_DETECT_NEGATIVE
            DEFAULT_GAIN_OF_6,                                                  // DEFAULT_GAIN_OF_6, GAIN_OF_1, GAIN_OF_2, GAIN_OF_3, GAIN_OF_4, GAIN_OF_8, GAIN_OF_12
            DEFAULT_DISABLE_POWER_DOWN,                                         // DEFAULT_DISABLE_POWER_DOWN, ENABLE_POWER_DOWN
            IGNORE_PREVIOUS_STATE                                               // CONTEXT_SAVE_CHANNEL, IGNORE_PREVIOUS_STATE
       		
	    ); 
        
        if (Verify_Check == ADS_1x9x_VERIFY_ERROR)
        {
            break;                                                              // exit loop and report verify error
        }
    }
#endif
   
    Disable_ADS1x9x_Interrupt ();
    printf("\r\n Verify_Check %x", Verify_Check);
    return Verify_Check;
}


void Enable_ADS1x9x_Conversion (void)
{    
    
	//Enable_ADS1x9x_Interrupt ();                                     // Interrupts must be enabled to transmit and receive data

	//执行RDATAC命令行将忽略操作寄存器命令
    Start_Read_Data_Continuous ();                                   // Enable continuous conversion mode
    
	//停止连续读数
//	Stop_Read_Data_Continuous (device_slot);
//	Hard_Start_ReStart_ADS1x9x (device_slot);                                   // Pull START pin high
//    delayUS(40);

    Soft_Start_ReStart_ADS1x9x ();
    
	Set_ADS1x9x_Start_Pin(SET);
    ADS1x9x_Status_Flags.ADC_Data_Ready = RESET;                                // Clear Data Ready flag
	//Set_ADS1x9x_Chip_Enable ();                                      // latch the gpio values on the module
    //Clear_ADS1x9x_Chip_Enable ();
    SetDelayStartReadyIT(20);
}

unsigned char ADS1x9x_SPI_Burst ( unsigned char Data) // Complements the SPI_Address command 
{   
     //清发送中断标志
    USR_SPI_ClearFlag(SPI1,SPI_FLAG_TXE);;                                                                        //  But allows multiple transactions (no Clear Chip ENABLE)
    //清接收中断标志
    USR_SPI_ClearFlag(SPI1,SPI_FLAG_RXNE);
    USR_SPI_SendData(SPI1,Data);                              // Send the data sitting at the pointer Data to the TX Buffer

    //while (USR_SPI_GetFlagStatus(SPI1,SPI_FLAG_TXE) == RESET);
    while (USR_SPI_GetFlagStatus(SPI1,SPI_FLAG_RXNE) == RESET);

    //delayUS(40);   
    return (USR_SPI_ReceiveData(SPI1));                             // Return Status Data or Requested Read Data
}


void Soft_Stop_ADS1x9x (void)
{  	
    ADS1298_StartLow();                                                 // Start Pin LOW   
    delayUS(20);                                                   // Small Delay   
    Set_ADS1x9x_Chip_Enable (); 
    delayUS(40);
	ADS1x9x_SPI_Data (STOP_CONVERSION);                            // Send 0x0A to the ADS1x9x
	delayUS(40);
}


/*********************************************************************************************************/
/**********************************************************************************************************
* Initialize the ADS1x9x using a Constants Table of known good settings                        *
**********************************************************************************************************/

void Init_ADS1x9x_Via_Constant_Table (unsigned char* constant_pointer)
{
    unsigned char i, j;

    ADS1x9x_SPI_Address_Byte_Count (DEFAULT_WRITE_NUMBER_OF_REGISTERS, ADS1x9x_TOP_REGISTER_SIZE);
    
    for (i = 0; i <= ADS1x9x_SPI_WRITE_DELAY; i++);                             // Delay added between SPI Writes 
                                    
    for (i = 0; i <= ADS1x9x_TOP_REGISTER_SIZE; i++)                            // Loop through registers to load the hex data value pairs
    {                              
        ADS1x9x_SPI_data = ADS1x9x_SPI_Burst (constant_pointer[i]);
        for (j = 0; j <= ADS1x9x_SPI_WRITE_DELAY; j++);                         // Delay added between SPI Writes 
    }

    Clear_ADS1x9x_Chip_Enable ();                                    // Clear the Chip ENABLE to terminate any previous SPI transactions

    ADS1x9x_SPI_Address_Byte_Count ((DEFAULT_WRITE_NUMBER_OF_REGISTERS + ADS1x9x_TOP_REGISTER_SIZE + 2), ADS1x9x_BOTTOM_REGISTER_SIZE);

    for (i = 0; i <= ADS1x9x_SPI_WRITE_DELAY; i++);                             // Delay added between SPI Writes 

    for (i = 0; i < ADS1x9x_BOTTOM_REGISTER_SIZE; i++)                          // Loop through registers to load the hex data value pairs
    {                              
        ADS1x9x_SPI_data = ADS1x9x_SPI_Burst (constant_pointer[ADS1x9x_REGISTER_OFFSET]);
        for (j = 0; j <= ADS1x9x_SPI_WRITE_DELAY; j++);                         // Delay added between SPI Writes 
    }

    Clear_ADS1x9x_Chip_Enable ();                                    // Clear the Chip ENABLE to terminate any previous SPI transactions
}


/**********************************************************************************************************
* Verify the ADS1x9x using a Constants Table of known good settings                        *
**********************************************************************************************************/
unsigned char Verify_ADS1x9x_Registers (unsigned char* constant_pointer)
{   
    unsigned char i;
    unsigned char error = ADS_1x9x_INIT_SUCCESS;                                // Set Error as defaulted clear

    ADS1x9x_SPI_Address_Byte_Count (DEFAULT_READ_NUMBER_OF_REGISTERS, ADS1x9x_TOP_REGISTER_SIZE);

    for (i = 0; i <= ADS1x9x_TOP_REGISTER_SIZE; i++)                            // Loop through registers to load the hex data value pairs
    {                              
        ADS1x9x_SPI_data = ADS1x9x_SPI_Burst (SPI_TEST_DATA);
          
        if (ADS1x9x_SPI_data != constant_pointer [i])                           // Do the values match?  Assuming the Addresses match
        {
            error = ADS_1x9x_VERIFY_ERROR;                                      // Set the Error
        }
    }
    Clear_ADS1x9x_Chip_Enable ();                                    // Clear the Chip ENABLE to terminate any previous SPI transactions

    ADS1x9x_SPI_Address_Byte_Count ((DEFAULT_READ_NUMBER_OF_REGISTERS + ADS1x9x_TOP_REGISTER_SIZE + 2), ADS1x9x_BOTTOM_REGISTER_SIZE);
    
    for (i = 0; i < ADS1x9x_BOTTOM_REGISTER_SIZE; i++)                          // Loop through registers to load the hex data value pairs
    {                              
        ADS1x9x_SPI_data = ADS1x9x_SPI_Burst (SPI_TEST_DATA);
          
        if (ADS1x9x_SPI_data != constant_pointer [ADS1x9x_REGISTER_OFFSET])     // Do the values match?  Assuming the Addresses match
        {
            error = ADS_1x9x_VERIFY_ERROR;                                      // Set the Error
        }
    }
    Clear_ADS1x9x_Chip_Enable ();                                    // Clear the Chip ENABLE to terminate any previous SPI transactions

    return error;                                                               // Return the error back from the routine
}


/**********************************************************************************************************
*               ADS1x9x Data Rate                                                                         *
**********************************************************************************************************/
unsigned char Initialize_ADS1x9x_Data_Rate ( unsigned char Modulation_Frequency_Divider)
{
    unsigned char Verify_status = ADS_1x9x_INIT_SUCCESS;                                    // Error state set Clear
    
    // Load Values set in the ADS1x9x
    ADS1x9x_SPI_Address_Byte_Count (READ_CONFIG_1_REGISTER, SINGLE_BYTE_READ_WRITE);        // Read Device ID, Single Byte the Part Number
    ADS1x9x_Config_1 = ADS1x9x_SPI_Data ( SPI_TEST_DATA);                                    // Send Dummy variable (0x55) to return the part number (Chip Select Cleared automatically)

    //  Modify Values for the ADS1x9x
    ADS1x9x_Config_1_register->Output_Data_Rate = Modulation_Frequency_Divider;             // DEFAULT_MODULATION_FREQUENCY_DIVIDED_BY_16
                                                                                            // MODULATION_FREQUENCY_DIVIDED_BY_32
                                                                                            // MODULATION_FREQUENCY_DIVIDED_BY_64 
                                                                                            // MODULATION_FREQUENCY_DIVIDED_BY_128
                                                                                            // MODULATION_FREQUENCY_DIVIDED_BY_256
                                                                                            // MODULATION_FREQUENCY_DIVIDED_BY_512
                                                                                            // MODULATION_FREQUENCY_DIVIDED_BY_1024

    // Program Values into ADS1x9x
    ADS1x9x_SPI_Address_Byte_Count ( WRITE_CONFIG_1_REGISTER, SINGLE_BYTE_READ_WRITE);      // Read Device ID, Single Byte the Part Number
    ADS1x9x_SPI_data = ADS1x9x_SPI_Data ( ADS1x9x_Config_1);                                // Send Dummy variable (0x55) to return the part number (Chip Select Cleared automatically)
  
  
#ifdef VERIFY
    //  Read Back Register    
    ADS1x9x_SPI_Address_Byte_Count (READ_CONFIG_1_REGISTER, SINGLE_BYTE_READ_WRITE);       // Read Device ID, Single Byte the Part Number
    ADS1x9x_SPI_data = ADS1x9x_SPI_Data (SPI_TEST_DATA);                                   // Read the Value from the SPI port   
    if (ADS1x9x_SPI_data != ADS1x9x_Config_1)
    {
        Verify_status = ADS_1x9x_VERIFY_ERROR;
    }        
//  -----------------------------------
#endif /* VERIFY */  
    return Verify_status;   
}

