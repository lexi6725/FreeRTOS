/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"

/* Library includes. */
#include "stm32f1xx.h"
#include "stm32f1xx_hal_conf.h"
#include "stm32f1xx_hal.h"
#include "nrf24l01.h"
#include "stm3210e_bit3.h"

#include "serial.h"

const uint8_t nRF_TX_ADDRESS[nRF_TX_ADR_WIDTH] = {0x59, 0x12, 0x67, 0x67, 0x67};
const uint8_t nRF_RX_ADDRESS[nRF_RX_ADR_WIDTH] = {0x59, 0x12, 0x67, 0x67, 0x67};

uint8_t nRF_Check(void)
{
	uint8_t buf[5] = {0xA5, 0xA5, 0xA5, 0xA5, 0xA5};
	uint8_t i;

	nRF_SPI_IO_WriteData(nRF_WRITE_REG+nRF_TX_ADDR, buf, 5);
	nRF_SPI_IO_ReadData(nRF_TX_ADDR, buf, 5);

	for (i = 0; i < 5; ++i)
		if(buf[i] != 0xA5)
			return HAL_ERROR;
	return HAL_OK;
}

void nRF_RX_Mode(void)
{
	/*!< Select the nRF: Chip Select low */
	nRF_SPI_CS_LOW();

	nRF_SPI_IO_WriteData(nRF_WRITE_REG+nRF_RX_ADDR_P0, (const uint8_t *)nRF_RX_ADDRESS, nRF_RX_ADR_WIDTH);

	nRF_SPI_IO_WriteReg(nRF_WRITE_REG+nRF_EN_AA, 0x01);
	nRF_SPI_IO_WriteReg(nRF_WRITE_REG+nRF_EN_RXADDR, 0x01);
	nRF_SPI_IO_WriteReg(nRF_WRITE_REG+nRF_RF_CH, 40);
	nRF_SPI_IO_WriteReg(nRF_WRITE_REG+nRF_RX_PW_P0, nRF_RX_PLOAD_WIDTH);
	nRF_SPI_IO_WriteReg(nRF_WRITE_REG+nRF_RF_SETUP, 0x0F);
	nRF_SPI_IO_WriteReg(nRF_WRITE_REG+nRF_CONFIG, 0x0F);
	
	/*!< Deselect the nRF: Chip Select high */
	nRF_SPI_CS_HIGH();
}


void nRF_TX_Mode(void)
{
	/*!< Select the nRF: Chip Select low */
	nRF_CSN_LOW();

	nRF_SPI_IO_WriteData(nRF_WRITE_REG+nRF_TX_ADDR, (const uint8_t *)nRF_TX_ADDRESS, nRF_TX_ADR_WIDTH);

	nRF_SPI_IO_WriteReg(nRF_WRITE_REG+nRF_EN_AA, 0x01);
	nRF_SPI_IO_WriteReg(nRF_WRITE_REG+nRF_EN_RXADDR, 0x01);
	nRF_SPI_IO_WriteReg(nRF_WRITE_REG+nRF_SETUP_RETR, 0x1A);
	nRF_SPI_IO_WriteReg(nRF_WRITE_REG+nRF_RF_CH, 40);
	nRF_SPI_IO_WriteReg(nRF_WRITE_REG+nRF_RF_SETUP, 0x0F);
	nRF_SPI_IO_WriteReg(nRF_WRITE_REG+nRF_CONFIG, 0x0E);
	
	/*!< Deselect the nRF: Chip Select high */
	nRF_CSN_HIGH();
}

uint8_t nRF_TxPacket(uint8_t *txbuf)
{
	uint8_t status;

	/*!< Select the nRF: Chip Select low */
	nRF_CSN_LOW();

	nRF_SPI_IO_WriteData(nRF_WR_TX_PLOAD, txbuf, nRF_TX_PLOAD_WIDTH);
	
	/*!< Deselect the nRF: Start Send */
	nRF_CSN_HIGH();

	while(nRF_IRQ_STATE());

	status = nRF_SPI_IO_ReadReg(nRF_STATUS);
	nRF_SPI_IO_WriteReg(nRF_WRITE_REG+nRF_STATUS, status);

	if (status & nRF_MAX_TX)
	{
		nRF_SPI_IO_WriteReg(nRF_FLUSH_TX, 0xFF);
		return nRF_MAX_TX;
	}
	if (status & nRF_TX_OK)
	{
		return nRF_TX_OK;
	}

	return 0xFF;
}

uint8_t nRF_RxPacket(uint8_t *rxbuf)
{
	uint8_t status;

	status = nRF_SPI_IO_ReadReg(nRF_STATUS);
	nRF_SPI_IO_WriteReg(nRF_WRITE_REG+nRF_STATUS, status);
	if (status & nRF_RX_OK)
	{
		nRF_SPI_IO_ReadData(nRF_RD_RX_PLOAD, rxbuf, nRF_RX_PLOAD_WIDTH);
		nRF_SPI_IO_WriteReg(nRF_FLUSH_RX, 0xFF);
		return 0;
	}

	return 1;
}

/* The task that is created three times. */
static portTASK_FUNCTION_PROTO( vnRFTask, pvParameters );

/*-----------------------------------------------------------*/

void vStartnRFTasks( UBaseType_t uxPriority )
{
	xTaskCreate( vnRFTask, "nRF", configMINIMAL_STACK_SIZE, NULL, uxPriority, ( TaskHandle_t * ) NULL );
}
/*-----------------------------------------------------------*/

static portTASK_FUNCTION( vnRFTask, pvParameters )
{
TickType_t xRate, xLastTime;

	/* The parameters are not used. */
	( void ) pvParameters;

	nRF_SPI_IO_Init();
	xRate = 300;
	/* We will turn the LED on and off again in the delay period, so each
	delay is only half the total period. */
	xRate /= ( TickType_t ) 2;

	/* We need to initialise xLastFlashTime prior to the first call to 
	vTaskDelayUntil(). */
	xLastTime = xTaskGetTickCount();

	for(;;)
	{
		vTaskDelayUntil( &xLastTime, xRate );
		if (nRF_Check() == HAL_OK)
		{
			UART_PutString("nRF24L01 Connected ...\n", 23);
		}
	}
} /*lint !e715 !e818 !e830 Function definition must be standard for task creation. */

