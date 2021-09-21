/*
 * Lora.h
 *
 *  Created on: Mar 2, 2021
 *      Author: chad.smith
 */

#ifndef INC_LORA_DRIVER_H_
#define INC_LORA_DRIVER_H_

//#include "stm32f407xx.h"

//**********************************************************MAcros to make help change with MCU AND HAL******************************************************************//
#define LORA_CHIPSELECT_GPIO GPIOA
#define LORA_CHIPSELECT_PIN  GPIO_PIN_11

// Resetting the Lora
#define LORA_RESET_GPIO  GPIOA
#define LORA_RESET_PIN   GPIO_PIN_10

// RX/ TX SWITCHES ON THE LORA MODULE
#define LORA_TX_SWITCH_GPIO GPIOA
#define LORA_RX_SWITCH_GPIO GPIOA
#define LORA_TX_SWITCH_PIN  GPIO_PIN_8
#define LORA_RX_SWITCH_PIN  GPIO_PIN_9

// dio2
#define LORA_DIO2_GPIO  GPIOB
#define LORA_DIO2_PIN  GPIO_PIN_7

// monitor busy
#define LORA_BUSY_GPIO GPIOA
#define LORA_BUSY_PIN GPIO_PIN_12

// message waiting dio1
#define LORA_MESSAGE_WAITING_GPIO GPIOB
#define LORA_MESSAGE_WAITING_PIN GPIO_PIN_6


// Commands  (OP CODES)

// OPERATIONAL MODES
#define OP_SET_SLEEP				0x84  // PUT CHIP IN SLEEP MODE
#define OP_SET_STANDBY				0x80  // SET CHIP IN STDBY_RC OR STBY_XOSC
#define OP_SET_FS					0xC1  // SET CHIN IN FREQUENCY SYNTHESIS
#define OP_SET_TX					0x83  // SET CHIP IN TX MODE
#define OP_SET_RX					0x82  // SET CHIP IN RX MODE
#define OP_SET_STOPR				0x9F  // rx period and sleep period
#define OP_SET_RX_DUTY_CYCLE		0x94  // rXPERIOD / Sleep Period
#define OP_SET_CAD					0xC5  // SET CHIP INTO RX MODE WITH PASSED CAD PARAMETERS
#define OP_SET__TX_CONTINUOUSWAVE 	0xD1  // SET CHIP INTO TX MODE WITH INF CARRIER WAVE SETTING
#define	OP_SET_TX_INF_PR			0xD2  // SET CHIP INTO TX MODE WITH INF PREAMBLE
#define OP_SET_REGULATOR_MODE		0x96  //
#define OP_SET_CALIBRATE			0x89 // CALIBRATE
#define OP_SET_CALIBRATE_IMAGE		0x98 // LAUNCHES AN IMAGE CALIBRATION AT THE GIVEN FREQS
#define OP_SET_PA_CONFIG			0x95 // CONFIGURE THE DUTY CYCLE MAX OUTPUT PWER
#define OP_SET_RXTX_FALLBACK_MODE 	0x93 // WHICH MODE THE CHIP GOES INTO AFTER TX/RX DONE


// COMMANDS FOR ACCESING THE RADIO REGISTER AND FIFO
#define OP_WRITEREGISTER			0x0D // Write into one or several registers
#define OP_READREGISTER				0x1D
#define OP_WRITEBUFFER				0x0E  // WRIET DATA TO FIFO
#define OP_READBUFFER				0x1E	// READ DATAT FROM FIFO


// COMMANDS CONTROLLING THE RADIO INTERRUPS
#define OP_SET_DIO_IRQ_PARAMS			0x08 // CONFIGURE THE IRQ AND DIOS ATTACHED TO EACH IRQ
#define OP_GET_IRQ_STATUS				0x12 // GET THE VALUES OF THE TRIGGERED IRQS
#define OP_CLEAR_IRQ_STATUS				0x02 // CLEAR ON OR SEVERAL OF THE IRQS
#define OP_SET_DIO2_AS_RF_SWITCHCTRL	0x9D // CONFIGURE RADIO TO CONTROL AN RF SWITCH FROM DIO2
#define OP_Set_DIO3_As_Tcxo_Ctrl		0x97 // CONFIGURE THE RADIO TO USE A TCXO CONTROLLED BY DI03

// RF MODULATION AND PACKET COMMANDS
#define OP_SET_RF_FREQUENCY				0x86 // SET THE RF FREQUENCY OF THE RADIO
#define OP_SET_PACKET_TYPE				0x8A // SLECT PACKET TYPE CORRESPONDING TO MODEM
#define OP_GET_PACKE_TYPE				0x11 // GT THE CURRENT PACKET CONFIG FOR DEVIC
#define OP_SET_TX_PARAMS				0x8E // SET OUTPUT POWER AND RAMP TIME FOR THE PA
#define OP_SET_MODULATION_PARAMS		0x8B // COMPUTE AND SET VALUES IN SELECTED PROTOCOL MODEM FOR GIVE MODULATION PARAMS
#define OP_SET_PACKET_PARAMS			0x8C // SET VALUES ON SELECTED PROTOCOL MODEM FOR GIVEN PACKET PARAMETERS
#define OP_SET_CAD_PARAMS				0x88 // SET THE PARAMETERS WHICH ARE USED FOR PERFORMAG CAD (LORA ONLY)
#define OP_SET_BUFFER_BASEADDR			0x8F // STORE TX AND RX BASE ADDRESS IN REGISTER OF SELECTED PROTOCOL MODEL
#define OP_SET_LORA_SYMB_NUMT_IMEOUT 	0xA0 // SET THE NUMBER OF SYMBOL THE MODEM HAS TO WAIT TO VALIDATE A LOCK

// COMMANDS RETURNING THE RADIO STATUS
#define OP_GET_STATUS					0xC0 // RETURNS CURRENTS STATUS OF DEVICE
#define OP_GET_RSSI_LNST				0x15 // RETURNS INSTANT RSSI
#define OP_GET_RX_BUFFER_STATUS			0x13 // RETURNS PAYLOAD LENGTH
#define OP_GET_PACKET_STATUS			0x14 // RETURNS RSSIAVG,RSSISYNC,PSTATUS2,PSTAUS3,
#define OP_GET_DEVICE_ERRORS			0x17 // RETURNS THE ERROR WHICH HAS OCCURED
#define OP_CLEAR_DEVICE_ERRORS			0x07 // CLEAR ALL THE ERRORS
#define OP_GET_STATS					0x10 // RETURNS STATS ON THE LAST FEW RECIEVED PACKETS
#define OP_RESET_STATS					0x00 // RESETS THE STATS

// packet sizes for different commands
#define SET_RX_DUTY_CYCLE_SIZE 		 	7 // HOW MANY BYTES FOR SETTING LORA TO SNIFF MODE
#define SET_RX_SIZE					 	4
#define PACKET_SIZE_SETDIOIRQPARAMS     9 // 9 BYTES IN SETDIOiRQPARAMS
#define PACKET_SIZE_CLEARIRQ		 	3
#define PACKET_SIZE_SETCADPARAMS        7
#define PACKET_SIZE_GETPACKETTYPE		3
#define PACKET_SIZE_SETTX				4
#define PACKET_SIZE_GETIRQSTATUS		4
#define PACKET_SIZE_GETDEVICEERRORS		4
#define PACKET_SIZE_CLEARDEVICEERRORS	2
#define PACKET_SIZE_SETSTDBY			2
#define PACKET_SIZE_SETPACKETTYPE		2
#define PACKET_SIZE_SETRFFREQ			5
#define PACKET_SIZE_SETPACONFIG			5
#define PACKET_SIZE_SETTXPARAMS			3
#define PACKET_SIZE_SETBUFFERBASEADDR	3
#define PACKET_SIZE_GETSTATS			8
#define PACKET_SIZE_GETRSSI				3
#define PACKET_SIZE_PACKETPARAMS		7
#define PACKET_SIZE_LORAMODPARAMS		5
#define PACKET_SIZE_GETRXBUFFERSTATUS   4
#define PACKET_SIZE_GETPACKETSTATUS		5


/********************************************REGISTER ADDRESSES AS NEEDED*************************************************************/
#define RX_GAIN_ADDR					0x08AC  // DEFAULT IS 0x94

/********************************************CONFIGURING DIFFERENT OP CODES*********************************************************/


// STANDBY 0X80
#define STDBY_RC					0 // DEVICE RUNNING ON 13 mHZ
#define STDBY_XOSC					1 // DEVICE RUNNING ON 32 MHZ

// STOP TIME ON PREAMBLE SPI TRANSACTION 0X9F
#define STOP_ON_syncword			0x00 // timer stops on sync word
#define STOP_ON_PREAMBLE			0x01 // TIMER STOPS ON PREAMBLE

// FALL BACK SPI TRANSACTION
#define FALLBACK_FS  				0x40
#define FALLBACK_XOSC  				0x30
#define FALLBACK_RC					0x20


// PACKET TYPES 0X8A
#define PACKET_TYPE_GFSK 			0x00
#define PACKET_TYPE_LORA 			0x01

// Ramp times
#define SET_RAMP_10U 				0x00 // 10 uS
#define SET_RAMP_20U 				0x01
#define SET_RAMP_40U 				0x02
#define SET_RAMP_80U 				0x03
#define SET_RAMP_200U 				0x04
#define SET_RAMP_800U 				0x05
#define SET_RAMP_1700U 				0x06
#define SET_RAMP_3400U 				0x07

// power
#define power_14   					0x0E;
#define power_22					0x16;

// LORA modulation params 0x8b 1 2 3 4 5 6 7 8 bytes
// byte 1 modparam Speading factor
#define SF5   						0x05
#define SF6   						0x06
#define SF7   						0x07
#define SF8   						0x08
#define SF9   						0x09
#define SF10   						0x0A
#define SF11  						0x0B
#define SF12  						0x0C

// BYTE 2 MODPARAM BANDWIDTH
#define LORA_BW_7 					0x00 // 7.81 KHZ
#define LORA_BW_10 					0x08 // 10.42 KHZ
#define LORA_BW_15					0x01 // 15.63 KHZ
#define LORA_BW_20 					0x09 // 7.81 KHZ
#define LORA_BW_31 					0x02 // 31.25 KHZ
#define LORA_BW_41 					0x0A // 7.81 KHZ
#define LORA_BW_62 					0x03 // 62.50 KHZ
#define LORA_BW_125 				0x04 // 125 KHZ
#define LORA_BW_250 				0x05 // 250 KHZ
#define LORA_BW_500 				0x06 // 500 KHZ

// BYTE 3 MODPARAM CODING RATE
#define LORA_CR_4_5					0x01
#define LORA_CR_4_6					0x02
#define LORA_CR_4_7					0x03
#define LORA_CR_4_8					0x04

// byte 4 modparam Low Data Rate optimize
#define LowDataRateOptimizeOFF 		0x00
#define LowDataRateOptimizeON  		0x01

//freqs
#define RF_FREQ_902        			0x38600000
#define RF_FREQ_903        			0x38700000
#define RF_FREQ_904        			0x38800000
#define RF_FREQ_905        			0x38900000
#define RF_FREQ_906        			0x38a00000
#define RF_FREQ_907       			0x38b00000
#define RF_FREQ_908        			0x38c00000
#define RF_FREQ_909        			0x38d00000
#define RF_FREQ_910        			0x38e00000
#define RF_FREQ_911        			0x38f00000
#define RF_FREQ_912        			0x39000000
#define RF_FREQ_913       			0x39100000
#define RF_FREQ_914        			0x39200000
#define RF_FREQ_915        			0x39300000
#define RF_FREQ_916        			0x39400000
#define RF_FREQ_917       		 	0x39500000
#define RF_FREQ_918        			0x39600000
#define RF_FREQ_919        			0x39700000
#define RF_FREQ_920        			0x39800000
#define RF_FREQ_921        			0x39900000
#define RF_FREQ_922        			0x39a00000
#define RF_FREQ_923        			0x39b00000
#define RF_FREQ_924        			0x39c00000
#define RF_FREQ_925        			0x39d00000
#define RF_FREQ_926        			0x39e00000
#define RF_FREQ_927        			0x39f00000
#define RF_FREQ_928					0x3A000000

// macros for optimized Pa Config refer to table 13-21 for optimal setting ONLY FOR SX1262
// dutycycle
#define pa_DutyCycle_LOW  			0x02
#define pa_DutyCycle_MED  			0x03
#define pa_DutyCycle_HIGH  			0x04

// SIZE OF THE PA IN THE SX1262 VALID RANGE BETWEEN OX00 AND 0X07
#define hpMax_14					0x02 // 14 dBm
#define hpMax_17					0x03 // 17 dBM
#define hpMax_20					0x05
#define hpMax_22					0x07

// Device seletc goes between sx1262 and 1261
#define device_1262  				0x00
#define device_1261 				0x01


// paLut

#define paLut_set 					0x01  //paLut is reserved byte in PACONFIG and always has the value 0x01.

#define TX_BASE_ADDR  				0x80
#define RX_BASE_ADDR   				0x00


// macros for packet type Lora
// Cyclical Redundancy Check CRC

#define CRC_LORA_OFF   				0x00
#define CRC_LORA_ON    				0x01

// Header type Lora
#define HEADER_TYPE_FIXED  			0x01
#define HEADER_TYPE_VARIABLE		0x00

#define IQ_STANDARD					0x00
#define IQ_INVERTED					0x01

// IRQ REGISTER BIT POSITIONS
#define IRQ_TX_DONE			  		0
#define IRQ_RX_DONE					1
#define IRQ_PREAMBLE_DETECTED 		2
#define IRQ_SYNC_WORD_Valid			3
#define IRQ_HEADER_VALID			4
#define IRQ_HEADER_ERR				5
#define IRQ_CRC_ERR					6
#define IRQ_CAD_DONE 				7
#define IRQ_CAD_DETETECTED			8
#define IRQ_TIMEOUT					9

// cad number of symbol definition # of symbols used
#define CAD_ON_1_SYMB			0X00
#define CAD_ON_2_SYMB			0X01
#define CAD_ON_4_SYMB			0X02
#define CAD_ON_8_SYMB			0X03
#define CAD_ON_16_SYMB			0X04

// CAD DET PEAK RELATED TO SPREADING FACTOR SF
#define cadDetPeak_SF5          18
#define cadDetPeak_SF6          19
#define cadDetPeak_SF7          20
#define cadDetPeak_SF8          21
#define cadDetPeak_SF9          22
#define cadDetPeak_SF10         23
#define cadDetPeak_SF11         24
#define cadDetPeak_SF12         25

#define CADDETMIN				10

#define CAD_EXIT_MODE_CAD_ONLY  0X00  // GOES TO STDBY AFTER DETECTION
#define CAD_EXIT_MODE_CAD_RX	0X01  // STAYS IN RXMODE UNTIL PACKET IS DETECTED
#define N_B						96

typedef struct
{
	uint8_t Opcode;
	uint8_t cadSymbolNum;
	uint8_t cadDetPeak;
	uint8_t cadDetMin;
	uint8_t cadExitMode;
	uint16_t cadTimeout;
}Cad_Params_t;

typedef struct
{
	uint8_t OpCode;
	uint8_t DutyCycle;
	uint8_t HpMax;
	uint8_t deviceSel;
	uint8_t paLut;

}Pa_config_t;

typedef struct
{
	uint8_t OpCode;
	uint8_t StdbyConfig;

}STDBY_config_t;

typedef struct
{
	uint8_t OpCode;
	uint8_t packet_config;

}Packet_type_config_t;

typedef struct
{
	uint8_t OpCode;
	uint32_t RF_freq;

}RF_freq_config_t;

typedef struct
{
	uint8_t OpCode;
	uint8_t power;
	uint8_t RampTime;

}Tx_Params_config_t;

typedef struct
{
	uint8_t OpCode;
	uint8_t Tx_Base_Addr;
	uint8_t RX_Base_Addr;

}Buffer_Base_Addr_config_t;

typedef struct
{
	uint8_t OpCode;
	uint8_t offset;
	uint8_t *buf;
	uint8_t num_bytes;

}Write_Buffer_config_t;

typedef struct{

	uint8_t SpreadingFactor;
	uint8_t BandWidth;
	uint8_t CodingRate;
	uint8_t LdOpt;  			// Low data rate optimize pg 84 and page 39

}Lora_ModParams_config_t;

typedef struct{

	uint8_t BR[3];
	uint8_t PulseShape;			// different from Lora Bandwidths
	uint8_t BandWidth;
	uint8_t Fdev[3];				// frequency deviation
	uint8_t LdOpt;  			// Low data rate optimize pg 84 and page 39

}GFSK_ModParams_config_t;

typedef struct
{
	Lora_ModParams_config_t Lora;
	GFSK_ModParams_config_t GFSK;
	uint8_t PacketType;
	uint8_t OpCode;


}ModParams_config_t;

/***********************************Packet Parameter Definitions*********************************************/
typedef struct{

	uint16_t PreambleLen;
	uint8_t HeaderType;
	uint8_t PayLoadLen;
	uint8_t CRCType;
	uint8_t InvertIQ;

}Lora_PacketParams_config_t;

typedef struct{

	uint8_t PreambleLen;
	uint8_t PreambDetectorLen;
	uint8_t SyncWordLen;
	uint8_t AddrComp;
	uint8_t PacketType;
	uint8_t PayLoadLen;
	uint8_t CRCType;

}GFSK_PacketParams_config_t;

typedef struct
{
	Lora_PacketParams_config_t 	Lora;
	GFSK_PacketParams_config_t 	GFSK;
	uint8_t 					PacketType;
	uint8_t 					OpCode;


}PacketParams_config_t;

typedef struct
{
	uint8_t offset;
	uint8_t numbytes;

}Read_buffer_config_t;

typedef struct
{
	uint8_t SetDioIrqParams;
	uint16_t IrqMask;
	uint16_t Dio1Mask;
	uint16_t Dio2Mask;
	uint16_t Dio3Mask;
}DioIrq_Params_config_t;


typedef struct
{
	STDBY_config_t 				STDBY;
	Packet_type_config_t 		PacketType;
	RF_freq_config_t 			RfFrequency;
	Pa_config_t 				Pa_Config;
	Tx_Params_config_t			TxParams;
	Buffer_Base_Addr_config_t 	BufferBaseAddr;
	Write_Buffer_config_t 		WriteBuffer;
	ModParams_config_t			ModulationParams;
	PacketParams_config_t 		PacketParams;
	uint8_t 					SyncWord;
	uint8_t 					SetTx;
	DioIrq_Params_config_t      DioIrq;



}Lora_Config_t;



/******************************* SET PACKET PARAMETERS ****************************************************/

// OPCODE 0X8C BYTES 1 2 3 4 5 6 7 8 9

//
/******************************************** API *******************************************************/

void Lora_ClearDeviceErros(SPI_HandleTypeDef*hspi);

void Lora_Configure(SPI_HandleTypeDef*hspi,Lora_Config_t*pLora);
void Lora_Set_STDBY(SPI_HandleTypeDef*hspi,STDBY_config_t* stdby);
void Lora_Set_PacketType(SPI_HandleTypeDef*hspi,Packet_type_config_t *packet_type);
void Lora_Set_RFFreq(SPI_HandleTypeDef*hspi,RF_freq_config_t * rf_freq);
void Lora_Set_PaConfig(SPI_HandleTypeDef*hspi,Pa_config_t *pa_config);
void Lora_Set_TxParams(SPI_HandleTypeDef*hspi,Tx_Params_config_t *TxParams,Pa_config_t *pa_config);
void Lora_Set_BufferBaseAddr(SPI_HandleTypeDef*hspi,Buffer_Base_Addr_config_t *buffer_base_addr);
void Lora_WriteBuffer(SPI_HandleTypeDef*hspi,Write_Buffer_config_t *pwrite_buffer,uint8_t []);
//void Lora_WriteBuffer(SPI_HandleTypeDef*hspi,Write_Buffer_config_t *pwrite_buffer);
void Lora_ReadBuffer(SPI_HandleTypeDef*hspi,Read_buffer_config_t *pread_buff,uint8_t * rx_buff);
void Lora_Set_ModulationParams(SPI_HandleTypeDef*hspi, ModParams_config_t *modparams);
void Lora_Set_PacketParams(SPI_HandleTypeDef*hspi, PacketParams_config_t *packetparams);
void Lora_Set_DioIrqParams(SPI_HandleTypeDef*hspi,DioIrq_Params_config_t*pDioIrq);
void Lora_Set_Tx(SPI_HandleTypeDef*hspi,uint32_t timeout);
void Lora_SetCad(SPI_HandleTypeDef*hspi);
void Lora_SetCadParams(SPI_HandleTypeDef*hspi,Cad_Params_t *pCadParams);

// Get Functios
void Lora_Get_Status(SPI_HandleTypeDef*hspi,uint8_t *pstatus);
//void Lora_Get_PacketType(SPI_HandleTypeDef*hspi,uint8_t *ppackettype);
void Lora_Get_PacketType(SPI_HandleTypeDef*hspi);
void Lora_Get_IrqStatus(SPI_HandleTypeDef*hspi, uint8_t *pspi_rxbuf);
void Lora_Set_RxDutyCycle(SPI_HandleTypeDef*hspi,uint32_t rxPeriod,uint32_t sleepPeriod);
void Lora_Set_Rx(SPI_HandleTypeDef*hspi,uint32_t timeout);
void Lora_ClearIrqStatus(SPI_HandleTypeDef*hspi,uint16_t ClearIrqParams);
void Lora_GetDeviceErrors(SPI_HandleTypeDef*hspi);
void Lora_GetStats(SPI_HandleTypeDef*hspi);
void Lora_GetRssi(SPI_HandleTypeDef*hspi,uint8_t *pRssi);
void Lora_GetRxBufferStatus(SPI_HandleTypeDef*hspi,uint8_t * payload_info);
void Lora_GetPacketStatus(SPI_HandleTypeDef*hspi,uint8_t * pPacket_status);


#endif /* INC_LORA_DRIVER_H_ */
