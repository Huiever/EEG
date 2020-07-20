#ifndef __ADS1299_H
#define __ADS1299_H
#include "sys.h"

//寄存器地址表
#define ID              0x00
#define CONFIG1         0x01
#define CONFIG2         0x02
#define CONFIG3			0x03
#define LOFF			0x04
#define CH1SET          0x05
#define CH2SET			0x06
#define CH3SET			0x07
#define CH4SET			0x08
#define CH5SET			0x09
#define CH6SET			0x0a
#define CH7SET			0x0b
#define CH8SET			0x0c
#define BIAS_SENSP      0x0d
#define BIAS_SENSN      0x0e
#define LOFF_SENSP      0x0f
#define LOFF_SENSN      0x10
#define LOFF_FLIP       0x11
#define LOFF_STATP      0x12
#define LOFF_STATN      0x13
#define GPIO			0x14
#define MISC1 			0x15
#define MISC2			0x16
#define CONFIG4			0x17
//
#define WAKE_UP         0x02
#define STANDBY         0x04
#define ADS_RESET       0x06
#define START           0x08
#define STOP            0x0a
#define RDATAC          0x10
#define SDATAC          0x11
#define RDATA           0x12
//

#define	RREG			0X20	//读取001r rrrr 000n nnnn  这里定义的只有高八位，低8位在发送命令时设置
#define WREG			0X40	//写入010r rrrr 000n nnnn

#define	ADS1299_DRDY 		PDin(7)  		//
#define	ADS1299_RESET 		PCout(6)  		//
#define	ADS1299_PWDN 		PEout(2)  		//
#define	ADS1299_CS 		    PEout(3)  		//
#define	ADS1299_START 		PCout(9)  		//

typedef struct{
    uint16_t start_code;
    uint16_t status;
    float data[8];
    uint16_t end_code;
}ads_data_t;

void ads1299_init();
void ads_data_process();

#endif
