#ifndef _AD5754R_H
#define _AD5754R_H
#include "stm32f4xx.h"

/*L_AD5754R 接口宏定义*/
	#define L_AD_nCLR_G				GPIOE
	#define L_AD_nLDAC_G			GPIOE
	#define L_AD_nSYNC_G			GPIOE
  #define L_AD_SCLK_G				GPIOE
	#define L_AD_SDO_G				GPIOE
	#define L_AD_SDIN_G				GPIOE

	#define L_AD_nCLR_P	 			GPIO_Pin_15
	#define L_AD_nLDAC_P			GPIO_Pin_14
	#define L_AD_nSYNC_P			GPIO_Pin_11
	#define L_AD_SCLK_P				GPIO_Pin_12	
	#define L_AD_SDO_P  			GPIO_Pin_10			//STM32 MISO
	#define L_AD_SDIN_P				GPIO_Pin_13   	//STM32 MOSI

	#define L_AD_nCLR_H				GPIO_SetBits	(L_AD_nCLR_G	,	L_AD_nCLR_P)
	#define L_AD_nCLR_L				GPIO_ResetBits(L_AD_nCLR_G	,	L_AD_nCLR_P)
	
	#define L_AD_nLDAC_H			GPIO_SetBits	(L_AD_nLDAC_G	,	L_AD_nLDAC_P)
	#define L_AD_nLDAC_L			GPIO_ResetBits(L_AD_nLDAC_G	,	L_AD_nLDAC_P)
	
	#define L_AD_nSYNC_H			GPIO_SetBits	(L_AD_nSYNC_G	,	L_AD_nSYNC_P)
	#define L_AD_nSYNC_L			GPIO_ResetBits(L_AD_nSYNC_G	,	L_AD_nSYNC_P)
	
  #define L_AD_SCLK_H				GPIO_SetBits	(L_AD_SCLK_G	,	L_AD_SCLK_P)
	#define L_AD_SCLK_L				GPIO_ResetBits(L_AD_SCLK_G	,	L_AD_SCLK_P)

	#define L_AD_SDIN_H				GPIO_SetBits	(L_AD_SDIN_G	,	L_AD_SDIN_P)
	#define L_AD_SDIN_L				GPIO_ResetBits(L_AD_SDIN_G	,	L_AD_SDIN_P)

/*R_AD5754R 接口宏定义*/
	#define R_AD_nCLR_G				GPIOG
	#define R_AD_nLDAC_G			GPIOG
	#define R_AD_nSYNC_G			GPIOD
  #define R_AD_SCLK_G				GPIOG
	#define R_AD_SDO_G				GPIOD
	#define R_AD_SDIN_G				GPIOG

	#define R_AD_nCLR_P	 			GPIO_Pin_5
	#define R_AD_nLDAC_P			GPIO_Pin_4
	#define R_AD_nSYNC_P			GPIO_Pin_15
	#define R_AD_SCLK_P				GPIO_Pin_2
	#define R_AD_SDO_P  			GPIO_Pin_14			//STM32 MISO
	#define R_AD_SDIN_P				GPIO_Pin_3	   	//STM32 MOSI

	#define R_AD_nCLR_H				GPIO_SetBits	(R_AD_nCLR_G	,	R_AD_nCLR_P)
	#define R_AD_nCLR_L				GPIO_ResetBits(R_AD_nCLR_G	,	R_AD_nCLR_P)
	
	#define R_AD_nLDAC_H			GPIO_SetBits	(R_AD_nLDAC_G	,	R_AD_nLDAC_P)
	#define R_AD_nLDAC_L			GPIO_ResetBits(R_AD_nLDAC_G	,	R_AD_nLDAC_P)
	
	#define R_AD_nSYNC_H			GPIO_SetBits	(R_AD_nSYNC_G	,	R_AD_nSYNC_P)
	#define R_AD_nSYNC_L			GPIO_ResetBits(R_AD_nSYNC_G	,	R_AD_nSYNC_P)
	
  #define R_AD_SCLK_H				GPIO_SetBits	(R_AD_SCLK_G	,	R_AD_SCLK_P)
	#define R_AD_SCLK_L				GPIO_ResetBits(R_AD_SCLK_G	,	R_AD_SCLK_P)

	#define R_AD_SDIN_H				GPIO_SetBits	(R_AD_SDIN_G	,	R_AD_SDIN_P)
	#define R_AD_SDIN_L				GPIO_ResetBits(R_AD_SDIN_G	,	R_AD_SDIN_P)
	
	
	
	#define DB0  0x000001
	#define DB1  0x000002
	#define DB2  0x000004	
	#define DB3  0x000008
	#define DB4  0x000010
	#define DB5  0x000020
	#define DB6  0x000040	
	#define DB7  0x000080
	#define DB8  0x000100
	#define DB9  0x000200
	#define DB10 0x000400	
	#define DB11 0x000800
	#define DB12 0x001000	
	#define DB13 0x002000
	#define DB14 0x004000	
	#define DB15 0x008000
	#define DB16 0x010000
	#define DB17 0x020000
	#define DB18 0x040000	
	#define DB19 0x080000
	#define DB20 0x100000	
	#define DB21 0x200000
	#define DB22 0x400000	
	#define DB23 0x800000
	
	#define DAC_Set_Register 							0x000000
	#define Output_Range_Select_Register 	DB19
	#define Power_Control_Register 				DB20
	#define DAC_Control_Register 					DB20 | DB19
	/* DAC_Set_Register */
	#define DAC_Channel_A  				0x000000
	#define DAC_Channel_B  				DB16
	#define DAC_Channel_C  				DB17
	#define DAC_Channel_D  				DB17 | DB16
  #define DAC_Channel_ALL 			DB18
	/* Output_Range_Select_Register */
	#define Output_Range_P_5 			0x000000
 	#define Output_Range_P_10 		DB0
	#define Output_Range_P_10_8 	DB1
	#define Output_Range_PN_5 		DB1 | DB0
	#define Output_Range_PN_10    DB2
	#define Output_Range_PN_10_8  DB2 | DB0
	/* Power_Control_Register  */
	#define PUA 	DB0
	#define PUB 	DB1
	#define PUC 	DB2
	#define PUD 	DB3
	#define PUREF DB4
	#define TSD 	DB5
	#define OCA 	DB7
	#define OCB 	DB8
	#define OCC 	DB9
	#define OCD 	DB10
	#define PowerUp_ALL 	DB3|DB2|DB1|DB0
	/* DAC_Control_Register */
	#define TSD_ENABLE		DB3
	#define Limit_ENABLE  DB2
	#define CLR_ENABLE		DB1
	#define SDO_DISABLE 	DB0
	#define NOP   				DB20|DB19
	#define CLR						DB20|DB19|DB18
	#define LOAD					DB20|DB19|DB18|DB16
	
	void ConfigAD5754R(void);
	void Delay_W5100(unsigned int);
	void AD5754R_Init(void);
	void L_SPI_Init(void);
	void R_SPI_Init(void);
	void AD5754R_nLDAC_Init(void);
	void AD5754R_nCLR_Init(void);
	void AD5754R_nSYNC_Init(void);

	void WriteToLAD5754RViaSpi(uint32_t);
	void WriteToRAD5754RViaSpi(uint32_t);
	uint32_t ReadFromLAD5754RViaSpi(uint32_t);
	uint32_t ReadFromRAD5754RViaSpi(uint32_t);

#endif
