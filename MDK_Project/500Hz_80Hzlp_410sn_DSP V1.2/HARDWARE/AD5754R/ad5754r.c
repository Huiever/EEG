/*now is L_AD5754R+R_AD5754R*/
#include "ad5754r.h"
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"

void Delay_W5100(unsigned int x)
{
        unsigned int i;
        for(i=0;i<x;i++)
        ;
}

void AD5754R_Init(void)
{
        L_SPI_Init();
				R_SPI_Init();
	
        AD5754R_nLDAC_Init();
        AD5754R_nCLR_Init();
        AD5754R_nSYNC_Init();
	
        L_AD_nCLR_H;
        L_AD_nLDAC_L;
        L_AD_SCLK_L;
				L_AD_nSYNC_H;
	
	      R_AD_nCLR_H;
        R_AD_nLDAC_L;
        R_AD_SCLK_L;
				R_AD_nSYNC_H;
}

void L_SPI_Init(void)					//GPIO used to simulate SPI
{
				GPIO_InitTypeDef 	GPIO_InitStructure_AD;
				RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE);
				
				GPIO_InitStructure_AD.GPIO_Mode 	= GPIO_Mode_OUT;			//复用推挽输出
				GPIO_InitStructure_AD.GPIO_OType 	= GPIO_OType_PP;
				GPIO_InitStructure_AD.GPIO_Pin 		= L_AD_SCLK_P;
				GPIO_InitStructure_AD.GPIO_Speed 	= GPIO_Speed_100MHz;
				GPIO_Init(L_AD_SCLK_G, &GPIO_InitStructure_AD);
				
				GPIO_InitStructure_AD.GPIO_Mode 	= GPIO_Mode_OUT;			//复用推挽输出 mosi
				GPIO_InitStructure_AD.GPIO_OType 	= GPIO_OType_PP;
				GPIO_InitStructure_AD.GPIO_Pin 		= L_AD_SDIN_P;
				GPIO_InitStructure_AD.GPIO_Speed 	= GPIO_Speed_100MHz;
				GPIO_Init(L_AD_SDIN_G, &GPIO_InitStructure_AD);
				
				GPIO_InitStructure_AD.GPIO_Mode 	= GPIO_Mode_IN;				//浮空输入  miso
				GPIO_InitStructure_AD.GPIO_Pin 		= L_AD_SDO_P;
				GPIO_InitStructure_AD.GPIO_PuPd  	= GPIO_PuPd_NOPULL;
				GPIO_InitStructure_AD.GPIO_Speed 	= GPIO_Speed_100MHz;
				GPIO_Init(L_AD_SDO_G, &GPIO_InitStructure_AD);
				
				GPIO_SetBits(GPIOE,L_AD_SCLK_P|L_AD_SDIN_P|L_AD_SDO_P);
}

void R_SPI_Init(void)					//GPIO used to simulate SPI
{
				GPIO_InitTypeDef 	GPIO_InitStructure_AD;
				RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOG,ENABLE);
				
				GPIO_InitStructure_AD.GPIO_Mode 	= GPIO_Mode_OUT;			//复用推挽输出
				GPIO_InitStructure_AD.GPIO_OType 	= GPIO_OType_PP;
				GPIO_InitStructure_AD.GPIO_Pin 		= R_AD_SCLK_P;
				GPIO_InitStructure_AD.GPIO_Speed 	= GPIO_Speed_100MHz;
				GPIO_Init(R_AD_SCLK_G, &GPIO_InitStructure_AD);
				
				GPIO_InitStructure_AD.GPIO_Mode 	= GPIO_Mode_OUT;			//复用推挽输出 mosi
				GPIO_InitStructure_AD.GPIO_OType 	= GPIO_OType_PP;
				GPIO_InitStructure_AD.GPIO_Pin 		= R_AD_SDIN_P;
				GPIO_InitStructure_AD.GPIO_Speed 	= GPIO_Speed_100MHz;
				GPIO_Init(R_AD_SDIN_G, &GPIO_InitStructure_AD);
				
				GPIO_InitStructure_AD.GPIO_Mode 	= GPIO_Mode_IN;				//浮空输入  miso
				GPIO_InitStructure_AD.GPIO_Pin 		= R_AD_SDO_P;
				GPIO_InitStructure_AD.GPIO_PuPd  	= GPIO_PuPd_NOPULL;
				GPIO_InitStructure_AD.GPIO_Speed 	= GPIO_Speed_100MHz;
				GPIO_Init(R_AD_SDO_G, &GPIO_InitStructure_AD);
				
				GPIO_SetBits(GPIOG,R_AD_SCLK_P|R_AD_SDIN_P);
				GPIO_SetBits(GPIOD,R_AD_SDO_P);
}

void AD5754R_nLDAC_Init(void)
{
				GPIO_InitTypeDef  GPIO_InitStructure_AD;
				RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);	
				/*AD5754R用到的IO口初始化*/
				GPIO_InitStructure_AD.GPIO_Mode 	= GPIO_Mode_OUT;			//推挽输出
				GPIO_InitStructure_AD.GPIO_OType 	= GPIO_OType_PP;
				GPIO_InitStructure_AD.GPIO_Pin 		= L_AD_nLDAC_P;
				GPIO_InitStructure_AD.GPIO_Speed 	= GPIO_Speed_100MHz;
				GPIO_Init(L_AD_nLDAC_G, &GPIO_InitStructure_AD);

				RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);	
				/*AD5754R用到的IO口初始化*/
				GPIO_InitStructure_AD.GPIO_Mode 	= GPIO_Mode_OUT;			//推挽输出
				GPIO_InitStructure_AD.GPIO_OType 	= GPIO_OType_PP;
				GPIO_InitStructure_AD.GPIO_Pin 		= R_AD_nLDAC_P;
				GPIO_InitStructure_AD.GPIO_Speed 	= GPIO_Speed_100MHz;
				GPIO_Init(R_AD_nLDAC_G, &GPIO_InitStructure_AD);	
	
}
void AD5754R_nCLR_Init(void)
{
				GPIO_InitTypeDef  GPIO_InitStructure_AD;
				RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);	
				/*AD5754R用到的IO口初始化*/
				GPIO_InitStructure_AD.GPIO_Mode 	= GPIO_Mode_OUT;			//推挽输出
				GPIO_InitStructure_AD.GPIO_OType 	= GPIO_OType_PP;
				GPIO_InitStructure_AD.GPIO_Pin 		= L_AD_nCLR_P;
				GPIO_InitStructure_AD.GPIO_Speed 	= GPIO_Speed_100MHz;
				GPIO_Init(L_AD_nCLR_G, &GPIO_InitStructure_AD);
	
				RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);	
				/*AD5754R用到的IO口初始化*/
				GPIO_InitStructure_AD.GPIO_Mode 	= GPIO_Mode_OUT;			//推挽输出
				GPIO_InitStructure_AD.GPIO_OType 	= GPIO_OType_PP;
				GPIO_InitStructure_AD.GPIO_Pin 		= R_AD_nCLR_P;
				GPIO_InitStructure_AD.GPIO_Speed 	= GPIO_Speed_100MHz;
				GPIO_Init(R_AD_nCLR_G, &GPIO_InitStructure_AD);
}

void AD5754R_nSYNC_Init(void)
{
				GPIO_InitTypeDef  GPIO_InitStructure_AD;
				RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);	
				/*AD5754R用到的IO口初始化*/
				GPIO_InitStructure_AD.GPIO_Mode 	= GPIO_Mode_OUT;			//推挽输出
				GPIO_InitStructure_AD.GPIO_OType 	= GPIO_OType_PP;
				GPIO_InitStructure_AD.GPIO_Pin 		= L_AD_nSYNC_P;
				GPIO_InitStructure_AD.GPIO_Speed 	= GPIO_Speed_100MHz;
				GPIO_Init(L_AD_nSYNC_G, &GPIO_InitStructure_AD);
	
				RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);	
				/*AD5754R用到的IO口初始化*/
				GPIO_InitStructure_AD.GPIO_Mode 	= GPIO_Mode_OUT;			//推挽输出
				GPIO_InitStructure_AD.GPIO_OType 	= GPIO_OType_PP;
				GPIO_InitStructure_AD.GPIO_Pin 		= R_AD_nSYNC_P;
				GPIO_InitStructure_AD.GPIO_Speed 	= GPIO_Speed_100MHz;
				GPIO_Init(R_AD_nSYNC_G, &GPIO_InitStructure_AD);
}

void WriteToLAD5754RViaSpi(uint32_t Data)
{
				uint8_t  i;
				L_AD_nSYNC_H;
		//Delay_W5100(10);
				L_AD_nSYNC_L;         //bring CS low
		//Delay_W5100(1);
				for(i=0; i<24; i++)
				{
							L_AD_SCLK_H;  
	  //Delay_W5100(5);					
							if(0x800000 == (Data & 0x800000))
							{
										L_AD_SDIN_H;
							}
							else
							{
										L_AD_SDIN_L;                                                                                                                                                                                   
							}
			//Delay_W5100(5);
							L_AD_SCLK_L;
			//Delay_W5100(5);
							Data = Data << 1;
	    //Delay_W5100(5);							
				}
				L_AD_nSYNC_H;
    //Delay_W5100(10);				
}

void WriteToRAD5754RViaSpi(uint32_t Data)
{
				uint8_t  i;
				R_AD_nSYNC_H;
			//Delay_W5100(10);
				R_AD_nSYNC_L;         //bring CS low
			//Delay_W5100(1);
				for(i=0; i<24; i++)
				{
							R_AD_SCLK_H; 
			//Delay_W5100(5);	
							if(0x800000 == (Data & 0x800000))
							{
										R_AD_SDIN_H;
		//Delay_W5100(5);
							}
							else
							{
										R_AD_SDIN_L;                                                                                                                                                                                   
							}
		//Delay_W5100(5);
							R_AD_SCLK_L;
		//Delay_W5100(5);							
							Data = Data << 1;
		//Delay_W5100(5);							
				}
				R_AD_nSYNC_H;		
		//Delay_W5100(10);				
}

void ConfigAD5754R(void)
{
        uint8_t  i;
        uint32_t ins[3];

		    ins[0] = (~DB2) & (DAC_Control_Register | DB16 | DB3);
				ins[1] = Output_Range_Select_Register | Output_Range_PN_5 | DAC_Channel_ALL ;	
	      ins[2] = Power_Control_Register | PUA | PUB | PUC | PUD;
        
        for(i=0; i<3; i++)
        { 
							WriteToLAD5754RViaSpi(ins[i]);
							WriteToRAD5754RViaSpi(ins[i]);//same as LAD5754R
					Delay_W5100(200);
        }
/*test very important*/	
	ReadFromLAD5754RViaSpi(0x800000 | DAC_Control_Register | DB16);
	ReadFromLAD5754RViaSpi(0x800000 | Output_Range_Select_Register);
	ReadFromLAD5754RViaSpi(0x800000 | Power_Control_Register);

}                  

uint32_t ReadFromLAD5754RViaSpi(uint32_t Data)   //Data should be 800000|regsiter address
{
        uint8_t i;
        uint32_t R_Data = 0;
        uint32_t Noop = NOP;

        WriteToLAD5754RViaSpi(Data);//first write
	
        L_AD_nSYNC_H;
        Delay_W5100(100);
        L_AD_nSYNC_L;
        Delay_W5100(20);
	
        for(i=0; i<24; i++)				 //second write
        {
						L_AD_SCLK_H;
						Delay_W5100(20);
						//////////////////////////////////////////////////right for MSB shift
						if(0x800000 == (Noop & 0x800000))       
						{
									L_AD_SDIN_H;          //Send one to SDI pin
						}
						else
						{
									L_AD_SDIN_L;          //Send zero to SDI pin
						}
						Noop = Noop << 1;
						R_Data = R_Data << 1;
						Delay_W5100(20);
						L_AD_SCLK_L;
						Delay_W5100(20);
						if(GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_10) == 0x01)
						{
									R_Data = R_Data | 0x01;
						}							
						Delay_W5100(30);
        } 
        L_AD_nSYNC_H;         
        Delay_W5100(20);
        return R_Data;
}

uint32_t ReadFromRAD5754RViaSpi(uint32_t Data)   //Data should be 800000|regsiter address
{
        uint8_t i;
        uint32_t R_Data = 0;
        uint32_t Noop = NOP;

        WriteToRAD5754RViaSpi(Data);//first write
	
        R_AD_nSYNC_H;
        Delay_W5100(100);
        R_AD_nSYNC_L;
        Delay_W5100(20);
	
        for(i=0; i<24; i++)				 //second write
        {
						R_AD_SCLK_H;
						Delay_W5100(20);
						//////////////////////////////////////////////////right for MSB shift
						if(0x800000 == (Noop & 0x800000))       
						{
									R_AD_SDIN_H;          //Send one to SDI pin
						}
						else
						{
									R_AD_SDIN_L;          //Send zero to SDI pin
						}
						Noop = Noop << 1;
						R_Data = R_Data << 1;
						Delay_W5100(20);
						R_AD_SCLK_L;
						Delay_W5100(20);
						if(GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_14) == 0x01)
						{
									R_Data = R_Data | 0x01;
						}							
						Delay_W5100(30);
        } 
        R_AD_nSYNC_H;         
        Delay_W5100(20);
        return R_Data;
}

