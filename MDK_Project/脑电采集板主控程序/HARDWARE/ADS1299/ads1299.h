#ifndef __ADS1299_H
#define __ADS1299_H
#include "sys.h"

//�Ĵ�����ַ��
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

#define	RREG			0X20	//��ȡ001r rrrr 000n nnnn  ���ﶨ���ֻ�и߰�λ����8λ�ڷ�������ʱ����
#define WREG			0X40	//д��010r rrrr 000n nnnn

#define	ADS1299_DRDY 		PDin(7)  		//
#define	ADS1299_RESET 		PDout(0)  		//
#define	ADS1299_PWDN 		PEout(2)  		//
#define	ADS1299_CS 		    PEout(3)  		//
#define	ADS1299_START 		PBout(6)  		//

typedef struct{
    uint8_t header;
    uint8_t sample_number;
    uint8_t eeg_data[24];
    uint8_t aux_data[6];
    uint8_t footer;
}ads_data_t;
extern ads_data_t ads_data;
void ads1299_init();
void ads_data_process();
void processChar(char character);
#endif
