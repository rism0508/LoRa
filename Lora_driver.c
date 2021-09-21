/*
 * Lora_driver.c
 *
 *  Created on: Mar 2, 2021
 *      Author: chad.smith
 */

#include "../../../Lora_driver/inc/Lora_driver.h"

#include<stdlib.h>

/*********************** API's support by this driver*************************************************/

void Lora_Configure(SPI_HandleTypeDef*hspi,Lora_Config_t*pLora)
{

	Lora_Set_STDBY(hspi,&pLora->STDBY);

	Lora_Set_PaConfig(hspi,&pLora->Pa_Config);

	Lora_Set_PacketType(hspi,&pLora->PacketType);

	Lora_Set_RFFreq(hspi,&pLora->RfFrequency);

	Lora_Set_TxParams(hspi, &pLora->TxParams, &pLora->Pa_Config);

	Lora_Set_BufferBaseAddr(hspi,&pLora->BufferBaseAddr);

	Lora_Set_ModulationParams(hspi,&pLora->ModulationParams);

	Lora_Set_PacketParams(hspi,&pLora->PacketParams);

	Lora_Set_DioIrqParams(hspi, &pLora->DioIrq);









}
void Lora_Set_STDBY(SPI_HandleTypeDef*hspi,STDBY_config_t* stdby)
{
	uint8_t spi_buf[PACKET_SIZE_SETSTDBY] = {0};
	spi_buf[0] = OP_SET_STANDBY;

	uint8_t spi_rx[PACKET_SIZE_SETSTDBY] = {0};

	if(stdby->StdbyConfig == STDBY_XOSC)
	{

		monitor_busyline();
		HAL_GPIO_WritePin(LORA_CHIPSELECT_GPIO, LORA_CHIPSELECT_PIN, GPIO_PIN_RESET);

		HAL_SPI_TransmitReceive(hspi, spi_buf, spi_rx, PACKET_SIZE_SETSTDBY,1);

		HAL_GPIO_WritePin(LORA_CHIPSELECT_GPIO, LORA_CHIPSELECT_PIN, GPIO_PIN_SET);



	}else
	{

		monitor_busyline();
		HAL_GPIO_WritePin(LORA_CHIPSELECT_GPIO, LORA_CHIPSELECT_PIN, GPIO_PIN_RESET);

		HAL_SPI_TransmitReceive(hspi, spi_buf, spi_rx, PACKET_SIZE_SETSTDBY,1);

		HAL_GPIO_WritePin(LORA_CHIPSELECT_GPIO, LORA_CHIPSELECT_PIN, GPIO_PIN_SET);
	}

}

void Lora_Set_PacketType(SPI_HandleTypeDef*hspi,Packet_type_config_t *packet_type)
{
	/*
	packet_type->OpCode = OP_SET_PACKET_TYPE;
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspi, &packet_type->OpCode, 1, 100);
	HAL_SPI_Transmit(hspi, &packet_type->packet_config, 1, 100);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	*/
	uint8_t spi_buf[PACKET_SIZE_SETPACKETTYPE] = {0};
	spi_buf[0] = OP_SET_PACKET_TYPE;
	spi_buf[1] = packet_type->packet_config;
	uint8_t spi_rx[PACKET_SIZE_SETPACKETTYPE] = {0};

	monitor_busyline();

	HAL_GPIO_WritePin(LORA_CHIPSELECT_GPIO, LORA_CHIPSELECT_PIN, GPIO_PIN_RESET);


	HAL_SPI_TransmitReceive(hspi, spi_buf, spi_rx, PACKET_SIZE_SETPACKETTYPE,1);


	HAL_GPIO_WritePin(LORA_CHIPSELECT_GPIO, LORA_CHIPSELECT_PIN, GPIO_PIN_SET);

}



	/********************************* Set RF frequency send 5 bytes********************************************************/

void Lora_Set_RFFreq(SPI_HandleTypeDef*hspi,RF_freq_config_t * rf_freq)
{
	uint8_t spi_buf[PACKET_SIZE_SETRFFREQ] = {0};
	spi_buf[0] = OP_SET_RF_FREQUENCY;
	spi_buf[1] = (rf_freq->RF_freq >> 24) & 0xFF; // grab first byte msb mask out others
	spi_buf[2] = (rf_freq->RF_freq >> 16) & 0xFF;
	spi_buf[3] = (rf_freq->RF_freq >> 8) & 0xFF;
	spi_buf[4] = (rf_freq->RF_freq >> 0) & 0xFF;

	uint8_t spi_rx[PACKET_SIZE_SETRFFREQ] = {0};
	monitor_busyline();
	HAL_GPIO_WritePin(LORA_CHIPSELECT_GPIO, LORA_CHIPSELECT_PIN, GPIO_PIN_RESET);

	HAL_SPI_TransmitReceive(hspi, spi_buf, spi_rx, PACKET_SIZE_SETRFFREQ,1);

	HAL_GPIO_WritePin(LORA_CHIPSELECT_GPIO, LORA_CHIPSELECT_PIN, GPIO_PIN_SET);




}

void Lora_Set_PaConfig(SPI_HandleTypeDef*hspi,Pa_config_t *pa_config)
{
	uint8_t spi_buf[PACKET_SIZE_SETPACONFIG] = {0};
	spi_buf[0] = OP_SET_PA_CONFIG;
	spi_buf[2] = pa_config->HpMax;
	spi_buf[3] = device_1262;
	spi_buf[4] = paLut_set;

	uint8_t spi_rx[PACKET_SIZE_SETPACONFIG] = {0};

	if(pa_config->HpMax == hpMax_14)
	{
		spi_buf[1] = pa_DutyCycle_LOW; // duty_cycle


	}else if (pa_config->HpMax == hpMax_17)
	{
		spi_buf[1] =pa_DutyCycle_LOW;


	}else if (pa_config->HpMax == hpMax_20)
	{
		spi_buf[1] = pa_DutyCycle_MED;


	}else if (pa_config->HpMax == hpMax_22)
	{
		spi_buf[1] = pa_DutyCycle_HIGH;

	}else // if we don't want to configure because we are only in RX mode or something
	{
		return;
	}
	monitor_busyline();
	HAL_GPIO_WritePin(LORA_CHIPSELECT_GPIO, LORA_CHIPSELECT_PIN, GPIO_PIN_RESET);

	HAL_SPI_TransmitReceive(hspi, spi_buf, spi_rx, PACKET_SIZE_SETPACONFIG,1);

	HAL_GPIO_WritePin(LORA_CHIPSELECT_GPIO, LORA_CHIPSELECT_PIN, GPIO_PIN_SET);


}
	// Set tx parameters 3 bytes opcode power Ramptime
void Lora_Set_TxParams(SPI_HandleTypeDef*hspi,Tx_Params_config_t *TxParams,Pa_config_t *pa_config)
{
	uint8_t spi_buf[PACKET_SIZE_SETTXPARAMS]  = {0};
	spi_buf[0] = OP_SET_TX_PARAMS;
	spi_buf[2] = TxParams->RampTime;

	uint8_t spi_rx[PACKET_SIZE_SETTXPARAMS] = {0};

	if(pa_config->HpMax == hpMax_14)
	{
		spi_buf[1] = power_14;

	}else if ((pa_config->HpMax) == hpMax_17)
	{
		spi_buf[1] = power_22;

	}else if (pa_config->HpMax == hpMax_20)
	{
		spi_buf[1] = power_22;

	}else if (pa_config->HpMax == hpMax_22)
	{
		spi_buf[1] = power_22;

	}else
	{
		return;
	}
	monitor_busyline();
	HAL_GPIO_WritePin(LORA_CHIPSELECT_GPIO, LORA_CHIPSELECT_PIN, GPIO_PIN_RESET);

	HAL_SPI_TransmitReceive(hspi, spi_buf, spi_rx, PACKET_SIZE_SETTXPARAMS,1);

	HAL_GPIO_WritePin(LORA_CHIPSELECT_GPIO, LORA_CHIPSELECT_PIN, GPIO_PIN_SET);


}

void Lora_Set_BufferBaseAddr(SPI_HandleTypeDef*hspi,Buffer_Base_Addr_config_t *buffer_base_addr)
{
	uint8_t spi_buf[PACKET_SIZE_SETBUFFERBASEADDR] = {0};
	spi_buf[0] = OP_SET_BUFFER_BASEADDR;
	spi_buf[1] = buffer_base_addr->Tx_Base_Addr;
	spi_buf[2] = buffer_base_addr->RX_Base_Addr;

	uint8_t spi_rx[PACKET_SIZE_SETBUFFERBASEADDR] = {0};
	monitor_busyline();
	HAL_GPIO_WritePin(LORA_CHIPSELECT_GPIO, LORA_CHIPSELECT_PIN, GPIO_PIN_RESET);

	HAL_SPI_TransmitReceive(hspi, spi_buf, spi_rx, PACKET_SIZE_SETBUFFERBASEADDR,1);

	HAL_GPIO_WritePin(LORA_CHIPSELECT_GPIO, LORA_CHIPSELECT_PIN, GPIO_PIN_SET);



}

void Lora_WriteBuffer(SPI_HandleTypeDef*hspi,Write_Buffer_config_t *pwrite_buffer,uint8_t array[])
{
	pwrite_buffer->OpCode = OP_WRITEBUFFER;
	monitor_busyline();
	HAL_GPIO_WritePin(LORA_CHIPSELECT_GPIO, LORA_CHIPSELECT_PIN, GPIO_PIN_RESET);

	HAL_SPI_Transmit(hspi, &pwrite_buffer->OpCode, 1, 100);
	HAL_SPI_Transmit(hspi, &pwrite_buffer->offset, 1, 100);
	HAL_SPI_Transmit(hspi, pwrite_buffer->buf, pwrite_buffer->num_bytes, 100);
	//HAL_SPI_Transmit(hspi, array, pwrite_buffer->num_bytes, 100);


	HAL_GPIO_WritePin(LORA_CHIPSELECT_GPIO, LORA_CHIPSELECT_PIN, GPIO_PIN_SET);


}

void Lora_ReadBuffer(SPI_HandleTypeDef*hspi,Read_buffer_config_t *pread_buff,uint8_t * rx_buff)
{

	uint8_t temp[8] = {0};
	uint8_t spi_buf[3] = {0};
	spi_buf[0] = OP_READBUFFER;
	spi_buf[1] = pread_buff->offset;




	monitor_busyline();

	HAL_GPIO_WritePin(LORA_CHIPSELECT_GPIO, LORA_CHIPSELECT_PIN, GPIO_PIN_RESET);


	HAL_SPI_Transmit(hspi, spi_buf,3, 100); // transmit opcode and offset
	//HAL_SPI_Receive(hspi, rx_buff, pread_buff->numbytes, 100);
	//HAL_SPI_TransmitReceive(hspi, spi_buf, spi_rx,8,100);
	HAL_SPI_TransmitReceive(hspi,temp, rx_buff, pread_buff->numbytes, 100);


	HAL_GPIO_WritePin(LORA_CHIPSELECT_GPIO, LORA_CHIPSELECT_PIN, GPIO_PIN_SET);





}

void Lora_Set_ModulationParams(SPI_HandleTypeDef*hspi,ModParams_config_t *modparams)
{
	uint8_t spi_buf[PACKET_SIZE_LORAMODPARAMS] = {0};
	spi_buf[0] = OP_SET_MODULATION_PARAMS;
	spi_buf[1] = modparams->Lora.SpreadingFactor;
	spi_buf[2] = modparams->Lora.BandWidth;
	spi_buf[3] = modparams->Lora.CodingRate;
	spi_buf[4] = modparams->Lora.LdOpt;

	uint8_t spi_rx[PACKET_SIZE_LORAMODPARAMS] = {0};

	monitor_busyline();

	HAL_GPIO_WritePin(LORA_CHIPSELECT_GPIO, LORA_CHIPSELECT_PIN, GPIO_PIN_RESET);


	HAL_SPI_TransmitReceive(hspi, spi_buf, spi_rx, PACKET_SIZE_LORAMODPARAMS, 100);


	HAL_GPIO_WritePin(LORA_CHIPSELECT_GPIO, LORA_CHIPSELECT_PIN, GPIO_PIN_SET);






}

void Lora_Set_PacketParams(SPI_HandleTypeDef*hspi, PacketParams_config_t *packetparams)
{
	uint8_t spi_buf[PACKET_SIZE_PACKETPARAMS] = {0};
	spi_buf[0] = OP_SET_PACKET_PARAMS;
	spi_buf[1] = (packetparams->Lora.PreambleLen >> 8 ) & 0x00FF;
	spi_buf[2] = (packetparams->Lora.PreambleLen >> 0 ) & 0x00FF;
	spi_buf[3] = (packetparams->Lora.HeaderType);
	spi_buf[4] = packetparams->Lora.PayLoadLen;
	spi_buf[5] = packetparams->Lora.CRCType;
	spi_buf[6] = packetparams->Lora.InvertIQ;

	uint8_t spi_rx[PACKET_SIZE_PACKETPARAMS] = {0};

	packetparams->OpCode = OP_SET_PACKET_PARAMS;
	if(packetparams->PacketType == PACKET_TYPE_LORA)
	{
		monitor_busyline();

		HAL_GPIO_WritePin(LORA_CHIPSELECT_GPIO, LORA_CHIPSELECT_PIN, GPIO_PIN_RESET);


		HAL_SPI_TransmitReceive(hspi, spi_buf, spi_rx, PACKET_SIZE_PACKETPARAMS, 100);


		HAL_GPIO_WritePin(LORA_CHIPSELECT_GPIO, LORA_CHIPSELECT_PIN, GPIO_PIN_SET);


	}else if (packetparams->PacketType == PACKET_TYPE_GFSK)
	{
		// need to code
		return;
	}
}


void Lora_Get_Status(SPI_HandleTypeDef*hspi,uint8_t *pstatus)
{

	uint8_t spi_buf[2] = {0};
	spi_buf[0] = OP_GET_STATUS;

	//uint8_t spi_rx[2] ={0};
	uint16_t size = 2;

	monitor_busyline();

	HAL_GPIO_WritePin(LORA_CHIPSELECT_GPIO, LORA_CHIPSELECT_PIN, GPIO_PIN_RESET);


	HAL_SPI_TransmitReceive(hspi, spi_buf, pstatus, size, 1000);


	HAL_GPIO_WritePin(LORA_CHIPSELECT_GPIO, LORA_CHIPSELECT_PIN, GPIO_PIN_SET);


}

//void Lora_Get_PacketType(SPI_HandleTypeDef*hspi,uint8_t *ppackettype)
void Lora_Get_PacketType(SPI_HandleTypeDef*hspi)
{

	uint8_t spi_buf[PACKET_SIZE_GETPACKETTYPE] = {0};
	spi_buf[0] = OP_GET_PACKE_TYPE;

	uint8_t spi_rx[PACKET_SIZE_GETPACKETTYPE] = {0};


	monitor_busyline();

	HAL_GPIO_WritePin(LORA_CHIPSELECT_GPIO, LORA_CHIPSELECT_PIN, GPIO_PIN_RESET);

	HAL_SPI_TransmitReceive(hspi, spi_buf, spi_rx, PACKET_SIZE_GETPACKETTYPE, 1);

	HAL_GPIO_WritePin(LORA_CHIPSELECT_GPIO, LORA_CHIPSELECT_PIN, GPIO_PIN_SET);

}


void Lora_Set_Fs(SPI_HandleTypeDef*hspi)
{
	uint8_t opcode = 0xC1;

	monitor_busyline();

	HAL_GPIO_WritePin(LORA_CHIPSELECT_GPIO, LORA_CHIPSELECT_PIN, GPIO_PIN_RESET);


	HAL_SPI_Transmit(hspi, &opcode, 1, 100);


	HAL_GPIO_WritePin(LORA_CHIPSELECT_GPIO, LORA_CHIPSELECT_PIN, GPIO_PIN_SET);
}

void Lora_Set_Tx(SPI_HandleTypeDef*hspi,uint32_t timeout)
{
	uint8_t spi_buf[PACKET_SIZE_SETTX] = {0};
	spi_buf[0] = OP_SET_TX;
	spi_buf[1] = (timeout >> 16) & 0xFF;
	spi_buf[2] = (timeout >> 8) & 0xFF;
	spi_buf[3] = (timeout >> 0) & 0xFF;

	uint8_t spi_rx[PACKET_SIZE_SETTX] = {0};

	monitor_busyline();

	HAL_GPIO_WritePin(LORA_CHIPSELECT_GPIO, LORA_CHIPSELECT_PIN, GPIO_PIN_RESET);


	HAL_SPI_TransmitReceive(hspi, spi_buf, spi_rx, PACKET_SIZE_SETTX, 100);


	HAL_GPIO_WritePin(LORA_CHIPSELECT_GPIO, LORA_CHIPSELECT_PIN, GPIO_PIN_SET);

}

void Lora_Get_IrqStatus(SPI_HandleTypeDef*hspi, uint8_t *pspi_rxbuf)
{
	uint8_t spi_buf[PACKET_SIZE_GETIRQSTATUS] = {0};
	spi_buf[0] = OP_GET_IRQ_STATUS;

	//uint8_t spi_rx[PACKET_SIZE_GETIRQSTATUS] = {0};
	monitor_busyline();

	HAL_GPIO_WritePin(LORA_CHIPSELECT_GPIO, LORA_CHIPSELECT_PIN, GPIO_PIN_RESET);

	HAL_SPI_TransmitReceive(hspi, spi_buf, pspi_rxbuf, PACKET_SIZE_GETIRQSTATUS, 1);


	HAL_GPIO_WritePin(LORA_CHIPSELECT_GPIO, LORA_CHIPSELECT_PIN, GPIO_PIN_SET);

}

void Lora_Set_RxDutyCycle(SPI_HandleTypeDef*hspi,uint32_t rxPeriod,uint32_t sleepPeriod)

{
	uint8_t spi_buf[SET_RX_DUTY_CYCLE_SIZE];
	spi_buf[0] = OP_SET_RX_DUTY_CYCLE;
	spi_buf[1]=  (rxPeriod >> 16) & 0xFF;
	spi_buf[2] = (rxPeriod >> 8) & 0xFF;
	spi_buf[3] = (rxPeriod>> 0) & 0xFF;
	spi_buf[4]=  (sleepPeriod >> 16) & 0xFF;
	spi_buf[5] = (sleepPeriod >> 8) & 0xFF;
	spi_buf[6] = (sleepPeriod>> 0) & 0xFF;

	monitor_busyline();

	HAL_GPIO_WritePin(LORA_CHIPSELECT_GPIO, LORA_CHIPSELECT_PIN, GPIO_PIN_RESET);


	HAL_SPI_Transmit(hspi, spi_buf, SET_RX_DUTY_CYCLE_SIZE, 100);


	HAL_GPIO_WritePin(LORA_CHIPSELECT_GPIO, LORA_CHIPSELECT_PIN, GPIO_PIN_SET);

}

void Lora_Set_Rx(SPI_HandleTypeDef*hspi,uint32_t timeout)
{
	uint8_t spi_buf[SET_RX_SIZE];
	spi_buf[0] = OP_SET_RX;
	spi_buf[1]=  (timeout >> 16) & 0xFF;
	spi_buf[2] = (timeout >> 8) & 0xFF;
	spi_buf[3] = (timeout>> 0) & 0xFF;

	uint8_t spi_rx[SET_RX_SIZE] = {0};

	monitor_busyline();

	HAL_GPIO_WritePin(LORA_CHIPSELECT_GPIO, LORA_CHIPSELECT_PIN, GPIO_PIN_RESET);


	HAL_SPI_TransmitReceive(hspi, spi_buf, spi_rx, SET_RX_SIZE, 1);


	HAL_GPIO_WritePin(LORA_CHIPSELECT_GPIO, LORA_CHIPSELECT_PIN, GPIO_PIN_SET);

}

void Lora_Set_DioIrqParams(SPI_HandleTypeDef*hspi,DioIrq_Params_config_t*pDioIrq)
{
	uint8_t spi_buf[PACKET_SIZE_SETDIOIRQPARAMS];
	spi_buf[0] = OP_SET_DIO_IRQ_PARAMS;
	spi_buf[1] = (pDioIrq->IrqMask >>8) & 0xFF;
	spi_buf[2] = (pDioIrq->IrqMask >>0) & 0xFF;
	spi_buf[3] = (pDioIrq->Dio1Mask >>8) & 0xFF;
	spi_buf[4] = (pDioIrq->Dio1Mask >>0) & 0xFF;
	spi_buf[5] = (pDioIrq->Dio2Mask >>8) & 0xFF;
	spi_buf[6] = (pDioIrq->Dio2Mask >>0) & 0xFF;
	spi_buf[7] = (pDioIrq->Dio3Mask >>8) & 0xFF;
	spi_buf[8] = (pDioIrq->Dio3Mask >>0) & 0xFF;

	uint8_t spi_rx[PACKET_SIZE_SETDIOIRQPARAMS] = {0};

	monitor_busyline();

	HAL_GPIO_WritePin(LORA_CHIPSELECT_GPIO, LORA_CHIPSELECT_PIN, GPIO_PIN_RESET);

	HAL_SPI_TransmitReceive(hspi, spi_buf, spi_rx,PACKET_SIZE_SETDIOIRQPARAMS,1);

	HAL_GPIO_WritePin(LORA_CHIPSELECT_GPIO, LORA_CHIPSELECT_PIN, GPIO_PIN_SET);

}

void Lora_ClearIrqStatus(SPI_HandleTypeDef*hspi,uint16_t ClearIrqParams)
{
	uint8_t spi_buf[PACKET_SIZE_CLEARIRQ];
	spi_buf[0] = OP_CLEAR_IRQ_STATUS;
	spi_buf[1] = (ClearIrqParams >>8) & 0xFF;
	spi_buf[2] = (ClearIrqParams >> 0) & 0xFF;

	uint8_t spi_rx[PACKET_SIZE_CLEARIRQ] = {0};

	monitor_busyline();

	HAL_GPIO_WritePin(LORA_CHIPSELECT_GPIO, LORA_CHIPSELECT_PIN, GPIO_PIN_RESET);

	HAL_SPI_TransmitReceive(hspi, spi_buf, spi_rx,PACKET_SIZE_CLEARIRQ,1);

	HAL_GPIO_WritePin(LORA_CHIPSELECT_GPIO, LORA_CHIPSELECT_PIN, GPIO_PIN_SET);
}

void Lora_SetCad(SPI_HandleTypeDef*hspi)
{
	uint8_t spi_buf = OP_SET_CAD;

	monitor_busyline();

	HAL_GPIO_WritePin(LORA_CHIPSELECT_GPIO, LORA_CHIPSELECT_PIN, GPIO_PIN_RESET);

	HAL_SPI_Transmit(hspi, &spi_buf, 1, 100);

	HAL_GPIO_WritePin(LORA_CHIPSELECT_GPIO, LORA_CHIPSELECT_PIN, GPIO_PIN_SET);
}

void Lora_SetCadParams(SPI_HandleTypeDef*hspi,Cad_Params_t *pCadParams)
{
	uint8_t spi_buf[PACKET_SIZE_SETCADPARAMS];
	spi_buf[0] = OP_SET_CAD_PARAMS;
	spi_buf[1] = pCadParams->cadSymbolNum;
	spi_buf[2] = pCadParams->cadDetPeak;
	spi_buf[3] = pCadParams->cadDetMin;
	spi_buf[4] = pCadParams->cadExitMode;
	spi_buf[5] = (pCadParams->cadSymbolNum >> 8 ) & 0xFF;
	spi_buf[6] = (pCadParams->cadSymbolNum >> 0 ) & 0xFF;

	monitor_busyline();

	HAL_GPIO_WritePin(LORA_CHIPSELECT_GPIO, LORA_CHIPSELECT_PIN, GPIO_PIN_RESET);

	HAL_SPI_Transmit(hspi, spi_buf, PACKET_SIZE_SETCADPARAMS, 100);

	HAL_GPIO_WritePin(LORA_CHIPSELECT_GPIO, LORA_CHIPSELECT_PIN, GPIO_PIN_SET);
}

void Lora_GetDeviceErrors(SPI_HandleTypeDef*hspi)
{
	uint8_t spi_txbuf[PACKET_SIZE_GETDEVICEERRORS] = {0};
	spi_txbuf[0] = OP_GET_DEVICE_ERRORS;

	uint8_t spi_rx[PACKET_SIZE_GETDEVICEERRORS] = {0};

	monitor_busyline();

	HAL_GPIO_WritePin(LORA_CHIPSELECT_GPIO, LORA_CHIPSELECT_PIN, GPIO_PIN_RESET);

	HAL_SPI_TransmitReceive(hspi, spi_txbuf, spi_rx,PACKET_SIZE_GETDEVICEERRORS, 1);

	HAL_GPIO_WritePin(LORA_CHIPSELECT_GPIO, LORA_CHIPSELECT_PIN, GPIO_PIN_SET);

}
void Lora_ClearDeviceErros(SPI_HandleTypeDef*hspi)
{
	uint8_t spi_buf[PACKET_SIZE_CLEARDEVICEERRORS] = {0};
	spi_buf[0] = OP_CLEAR_DEVICE_ERRORS;

	uint8_t spi_rx[PACKET_SIZE_CLEARDEVICEERRORS] = {0};

	monitor_busyline();

	HAL_GPIO_WritePin(LORA_CHIPSELECT_GPIO, LORA_CHIPSELECT_PIN, GPIO_PIN_RESET);

	HAL_SPI_TransmitReceive(hspi, spi_buf, spi_rx, PACKET_SIZE_CLEARDEVICEERRORS,1);

	HAL_GPIO_WritePin(LORA_CHIPSELECT_GPIO, LORA_CHIPSELECT_PIN, GPIO_PIN_SET);
}

void Lora_GetStats(SPI_HandleTypeDef*hspi)
{
	uint8_t spi_buf[PACKET_SIZE_GETSTATS] = {0};
	spi_buf[0] = OP_GET_STATS;

	uint8_t spi_rx[PACKET_SIZE_GETSTATS] = {0};

	monitor_busyline();

	HAL_GPIO_WritePin(LORA_CHIPSELECT_GPIO, LORA_CHIPSELECT_PIN, GPIO_PIN_RESET);

	HAL_SPI_TransmitReceive(hspi, spi_buf, spi_rx, PACKET_SIZE_GETSTATS,1);

	HAL_GPIO_WritePin(LORA_CHIPSELECT_GPIO, LORA_CHIPSELECT_PIN, GPIO_PIN_SET);



}

void Lora_GetRssi(SPI_HandleTypeDef*hspi,uint8_t *pRssi)
{
	uint8_t spi_buf[PACKET_SIZE_GETRSSI] = {0};
	spi_buf[0] = OP_GET_RSSI_LNST;

	uint8_t spi_rx[PACKET_SIZE_GETRSSI] = {0};

	monitor_busyline();

	HAL_GPIO_WritePin(LORA_CHIPSELECT_GPIO, LORA_CHIPSELECT_PIN, GPIO_PIN_RESET);

	HAL_SPI_TransmitReceive(hspi, spi_buf, pRssi, PACKET_SIZE_GETRSSI,100);
	//HAL_SPI_TransmitReceive(hspi, temp, pRssi, 1 ,100);

	HAL_GPIO_WritePin(LORA_CHIPSELECT_GPIO, LORA_CHIPSELECT_PIN, GPIO_PIN_SET);

}

void Lora_GetRxBufferStatus(SPI_HandleTypeDef*hspi,uint8_t * payload_info)
{
	uint8_t spi_buf[PACKET_SIZE_GETRXBUFFERSTATUS-2] = {0};
	uint8_t temp[2] = {0};
	spi_buf[0] = OP_GET_RX_BUFFER_STATUS;

	uint8_t spi_rx[PACKET_SIZE_GETRXBUFFERSTATUS-2] = {0};

	monitor_busyline();

	HAL_GPIO_WritePin(LORA_CHIPSELECT_GPIO, LORA_CHIPSELECT_PIN, GPIO_PIN_RESET);

	HAL_SPI_TransmitReceive(hspi, spi_buf, spi_rx, PACKET_SIZE_GETRXBUFFERSTATUS-2 ,100);
	HAL_SPI_TransmitReceive(hspi, temp, payload_info, 2, 100);

	HAL_GPIO_WritePin(LORA_CHIPSELECT_GPIO, LORA_CHIPSELECT_PIN, GPIO_PIN_SET);


}

void Lora_GetPacketStatus(SPI_HandleTypeDef*hspi,uint8_t * pPacket_status)
{
	uint8_t spi_buf[PACKET_SIZE_GETPACKETSTATUS] = {0};
	spi_buf[0] = OP_GET_PACKET_STATUS;


	HAL_GPIO_WritePin(LORA_CHIPSELECT_GPIO, LORA_CHIPSELECT_PIN, GPIO_PIN_RESET);

	HAL_SPI_TransmitReceive(hspi, spi_buf, pPacket_status, PACKET_SIZE_GETPACKETSTATUS ,100);

	HAL_GPIO_WritePin(LORA_CHIPSELECT_GPIO, LORA_CHIPSELECT_PIN, GPIO_PIN_SET);
}

// helper function
void monitor_busyline()
{
	while(HAL_GPIO_ReadPin(LORA_BUSY_GPIO, LORA_BUSY_PIN) == 1) ;
	HAL_Delay(1);

	return;

}






