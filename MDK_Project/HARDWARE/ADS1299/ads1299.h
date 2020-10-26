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

#define	ADS1299_CLKS 		PDout(6)  		//
#define	ADS1299_DRDY 		PDin(7)  		//
#define	ADS1299_RESET 		PDout(0)  		//
#define	ADS1299_PWDN 		PEout(2)  		//
#define	ADS1299_CS 		    PEout(3)  		//
#define	ADS1299_START 		PBout(6)  		//

// OPENBCI_COMMANDS
/** Turning channels off */
#define OPENBCI_CHANNEL_OFF_1 '1'
#define OPENBCI_CHANNEL_OFF_2 '2'
#define OPENBCI_CHANNEL_OFF_3 '3'
#define OPENBCI_CHANNEL_OFF_4 '4'
#define OPENBCI_CHANNEL_OFF_5 '5'
#define OPENBCI_CHANNEL_OFF_6 '6'
#define OPENBCI_CHANNEL_OFF_7 '7'
#define OPENBCI_CHANNEL_OFF_8 '8'
#define OPENBCI_CHANNEL_OFF_9 'q'
#define OPENBCI_CHANNEL_OFF_10 'w'
#define OPENBCI_CHANNEL_OFF_11 'e'
#define OPENBCI_CHANNEL_OFF_12 'r'
#define OPENBCI_CHANNEL_OFF_13 't'
#define OPENBCI_CHANNEL_OFF_14 'y'
#define OPENBCI_CHANNEL_OFF_15 'u'
#define OPENBCI_CHANNEL_OFF_16 'i'

/** Turn channels on */
#define OPENBCI_CHANNEL_ON_1 '!'
#define OPENBCI_CHANNEL_ON_2 '@'
#define OPENBCI_CHANNEL_ON_3 '#'
#define OPENBCI_CHANNEL_ON_4 '$'
#define OPENBCI_CHANNEL_ON_5 '%'
#define OPENBCI_CHANNEL_ON_6 '^'
#define OPENBCI_CHANNEL_ON_7 '&'
#define OPENBCI_CHANNEL_ON_8 '*'
#define OPENBCI_CHANNEL_ON_9 'Q'
#define OPENBCI_CHANNEL_ON_10 'W'
#define OPENBCI_CHANNEL_ON_11 'E'
#define OPENBCI_CHANNEL_ON_12 'R'
#define OPENBCI_CHANNEL_ON_13 'T'
#define OPENBCI_CHANNEL_ON_14 'Y'
#define OPENBCI_CHANNEL_ON_15 'U'
#define OPENBCI_CHANNEL_ON_16 'I'

/** Test Signal Control Commands
 * 1x - Voltage will be 1 * (VREFP - VREFN) / 2.4 mV
 * 2x - Voltage will be 2 * (VREFP - VREFN) / 2.4 mV
 */
#define OPENBCI_TEST_SIGNAL_CONNECT_TO_DC            'p'
#define OPENBCI_TEST_SIGNAL_CONNECT_TO_GROUND        '0'
#define OPENBCI_TEST_SIGNAL_CONNECT_TO_PULSE_1X_FAST '='
#define OPENBCI_TEST_SIGNAL_CONNECT_TO_PULSE_1X_SLOW '-'
#define OPENBCI_TEST_SIGNAL_CONNECT_TO_PULSE_2X_FAST ']'
#define OPENBCI_TEST_SIGNAL_CONNECT_TO_PULSE_2X_SLOW '['

/** Channel Setting Commands */
#define OPENBCI_CHANNEL_CMD_ADC_Normal      '0'
#define OPENBCI_CHANNEL_CMD_ADC_Shorted     '1'
#define OPENBCI_CHANNEL_CMD_ADC_BiasDRP     '6'
#define OPENBCI_CHANNEL_CMD_ADC_BiasDRN     '7'
#define OPENBCI_CHANNEL_CMD_ADC_BiasMethod  '2'
#define OPENBCI_CHANNEL_CMD_ADC_MVDD        '3'
#define OPENBCI_CHANNEL_CMD_ADC_Temp        '4'
#define OPENBCI_CHANNEL_CMD_ADC_TestSig     '5'
#define OPENBCI_CHANNEL_CMD_BIAS_INCLUDE    '1'
#define OPENBCI_CHANNEL_CMD_BIAS_REMOVE     '0'
#define OPENBCI_CHANNEL_CMD_CHANNEL_1       '1'
#define OPENBCI_CHANNEL_CMD_CHANNEL_2       '2'
#define OPENBCI_CHANNEL_CMD_CHANNEL_3       '3'
#define OPENBCI_CHANNEL_CMD_CHANNEL_4       '4'
#define OPENBCI_CHANNEL_CMD_CHANNEL_5       '5'
#define OPENBCI_CHANNEL_CMD_CHANNEL_6       '6'
#define OPENBCI_CHANNEL_CMD_CHANNEL_7       '7'
#define OPENBCI_CHANNEL_CMD_CHANNEL_8       '8'
#define OPENBCI_CHANNEL_CMD_CHANNEL_9       'Q'
#define OPENBCI_CHANNEL_CMD_CHANNEL_10      'W'
#define OPENBCI_CHANNEL_CMD_CHANNEL_11      'E'
#define OPENBCI_CHANNEL_CMD_CHANNEL_12      'R'
#define OPENBCI_CHANNEL_CMD_CHANNEL_13      'T'
#define OPENBCI_CHANNEL_CMD_CHANNEL_14      'Y'
#define OPENBCI_CHANNEL_CMD_CHANNEL_15      'U'
#define OPENBCI_CHANNEL_CMD_CHANNEL_16      'I'
#define OPENBCI_CHANNEL_CMD_GAIN_1          '0'
#define OPENBCI_CHANNEL_CMD_GAIN_2          '1'
#define OPENBCI_CHANNEL_CMD_GAIN_4          '2'
#define OPENBCI_CHANNEL_CMD_GAIN_6          '3'
#define OPENBCI_CHANNEL_CMD_GAIN_8          '4'
#define OPENBCI_CHANNEL_CMD_GAIN_12         '5'
#define OPENBCI_CHANNEL_CMD_GAIN_24         '6'
#define OPENBCI_CHANNEL_CMD_LATCH           'X'
#define OPENBCI_CHANNEL_CMD_POWER_OFF       '1'
#define OPENBCI_CHANNEL_CMD_POWER_ON        '0'
#define OPENBCI_CHANNEL_CMD_SET             'x'
#define OPENBCI_CHANNEL_CMD_SRB1_CONNECT    '1'
#define OPENBCI_CHANNEL_CMD_SRB1_DISCONNECT '0'
#define OPENBCI_CHANNEL_CMD_SRB2_CONNECT    '1'
#define OPENBCI_CHANNEL_CMD_SRB2_DISCONNECT '0'

/** Default Channel Settings */
#define OPENBCI_CHANNEL_DEFAULT_ALL_SET 'd'
#define OPENBCI_CHANNEL_DEFAULT_ALL_REPORT 'D'

/** LeadOff Impedance Commands */
#define OPENBCI_CHANNEL_IMPEDANCE_LATCH                'Z'
#define OPENBCI_CHANNEL_IMPEDANCE_SET                  'z'
#define OPENBCI_CHANNEL_IMPEDANCE_TEST_SIGNAL_APPLIED    '1'
#define OPENBCI_CHANNEL_IMPEDANCE_TEST_SIGNAL_APPLIED_NOT '0'

/** SD card Commands */
#define OPENBCI_SD_LOG_FOR_HOUR_1    'G'
#define OPENBCI_SD_LOG_FOR_HOUR_2    'H'
#define OPENBCI_SD_LOG_FOR_HOUR_4    'J'
#define OPENBCI_SD_LOG_FOR_HOUR_12   'K'
#define OPENBCI_SD_LOG_FOR_HOUR_24   'L'
#define OPENBCI_SD_LOG_FOR_MIN_5     'A'
#define OPENBCI_SD_LOG_FOR_MIN_15    'S'
#define OPENBCI_SD_LOG_FOR_MIN_30    'F'
#define OPENBCI_SD_LOG_FOR_SEC_14    'a'
#define OPENBCI_SD_LOG_STOP        'j'

/** Stream Data Commands */
#define OPENBCI_STREAM_START  'b'
#define OPENBCI_STREAM_STOP   's'

/** Miscellaneous */
#define OPENBCI_MISC_QUERY_REGISTER_SETTINGS '?'
#define OPENBCI_MISC_SOFT_RESET              'v'

/** 16 Channel Commands */
#define OPENBCI_CHANNEL_MAX_NUMBER_8    'c'
#define OPENBCI_CHANNEL_MAX_NUMBER_16   'C'

#define OPENBCI_BOARD_MODE_SET '/'

#define OPENBCI_GET_VERSION 'V'

/** Set sample rate */
#define OPENBCI_SAMPLE_RATE_SET '~'

/** Insert marker into the stream */
#define OPENBCI_INSERT_MARKER '`'

/** Sync Clocks */
#define OPENBCI_TIME_SET '<'
#define OPENBCI_TIME_STOP '>'

/** Wifi Stuff */
#define OPENBCI_WIFI_ATTACH '{'
#define OPENBCI_WIFI_REMOVE '}'
#define OPENBCI_WIFI_STATUS ':'
#define OPENBCI_WIFI_RESET ';'

/** Possible number of channels */
#define OPENBCI_NUMBER_OF_CHANNELS_DAISY 16
#define OPENBCI_NUMBER_OF_CHANNELS_DEFAULT 8

/** Helpful numbers */
#define OPENBCI_NUMBER_OF_BOARD_SETTINGS 1
#define OPENBCI_NUMBER_OF_CHANNEL_SETTINGS 6
#define OPENBCI_NUMBER_OF_LEAD_OFF_SETTINGS 2

/** Possible Sample Rates*/
#define OPENBCI_SAMPLE_RATE_125 125
#define OPENBCI_SAMPLE_RATE_250 250

/** Time out for multi char commands **/
#define MULTI_CHAR_COMMAND_TIMEOUT_MS 1000

/** Packet Size */
#define OPENBCI_PACKET_SIZE 33

#define OPENBCI_NUMBER_BYTES_PER_ADS_SAMPLE 24
#define OPENBCI_NUMBER_CHANNELS_PER_ADS_SAMPLE 24

/** Impedance Calculation Variables */
#define OPENBCI_LEAD_OFF_DRIVE_AMPS 0.000000006
#define OPENBCI_LEAD_OFF_FREQUENCY_HZ 31

#define OPENBCI_TIME_OUT_MS_1 1
#define OPENBCI_TIME_OUT_MS_3 3

#define OPENBCI_NUMBER_OF_BYTES_SETTINGS_CHANNEL 9
#define OPENBCI_NUMBER_OF_BYTES_SETTINGS_LEAD_OFF 5

#define OPENBCI_NUMBER_OF_BYTES_AUX 6

#define OPENBCI_FIRMWARE_VERSION_V1 1
#define OPENBCI_FIRMWARE_VERSION_V2 1

#define OPENBCI_ADS_BYTES_PER_CHAN 3
#define OPENBCI_ADS_CHANS_PER_BOARD 8

/** BLE Packet Information */
#define BLE_BYTES_PER_PACKET 20
#define BLE_BYTES_PER_SAMPLE 6
#define BLE_SAMPLES_PER_PACKET 3
#define BLE_TOTAL_DATA_BYTES 18
#define BLE_RING_BUFFER_SIZE 50

typedef struct{
    uint8_t header;
    uint8_t sample_number;
    uint8_t eeg_data[24];
    uint8_t aux_data[6];
    uint8_t footer;
}ads_data_t;
extern ads_data_t ads_data;
void ads1299_init(void);
void ads_data_process(void);
void processChar(char character);
void ads_con_exchange(void);
#endif
