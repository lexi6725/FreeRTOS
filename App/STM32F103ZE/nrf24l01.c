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

//#define TX_SEND

const uint8_t nRF_TX_ADDRESS[nRF_TX_ADR_WIDTH] = {0x59, 0x12, 0x67, 0x67, 0x25};
const uint8_t nRF_RX_ADDRESS[nRF_RX_ADR_WIDTH] = {0x59, 0x12, 0x67, 0x67, 0x25};

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

	nRF_SPI_IO_WriteData(nRF_WRITE_REG+nRF_TX_ADDR, (const uint8_t *)nRF_TX_ADDRESS, nRF_TX_ADR_WIDTH);
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
	nRF_SPI_IO_WriteData(nRF_WRITE_REG+nRF_RX_ADDR_P0, (const uint8_t *)nRF_RX_ADDRESS, nRF_RX_ADR_WIDTH);

	nRF_SPI_IO_WriteReg(nRF_WRITE_REG+nRF_EN_AA, 0x01);
	nRF_SPI_IO_WriteReg(nRF_WRITE_REG+nRF_EN_RXADDR, 0x01);
	nRF_SPI_IO_WriteReg(nRF_WRITE_REG+nRF_SETUP_RETR, 0x0A);
	nRF_SPI_IO_WriteReg(nRF_WRITE_REG+nRF_RF_CH, 40);
	nRF_SPI_IO_WriteReg(nRF_WRITE_REG+nRF_RF_SETUP, 0x0F);
	nRF_SPI_IO_WriteReg(nRF_WRITE_REG+nRF_RX_PW_P0, nRF_RX_PLOAD_WIDTH);
	nRF_SPI_IO_WriteReg(nRF_WRITE_REG+nRF_CONFIG, 0x0E);
	
	/*!< Deselect the nRF: Chip Select high */
	nRF_CSN_HIGH();
	
}

uint8_t nRF_TxPacket(uint8_t *txbuf)
{
	uint8_t status;

	/*!< Select the nRF: Chip Select low */
	nRF_CSN_LOW();
	
	nRF_SPI_IO_WriteReg(nRF_FLUSH_TX, 0x00);

	nRF_SPI_IO_WriteData(nRF_WR_TX_PLOAD, txbuf, nRF_TX_PLOAD_WIDTH);
	
	/*!< Deselect the nRF: Start Send */
	nRF_CSN_HIGH();

	while(nRF_IRQ_STATE());

	status = nRF_SPI_IO_ReadReg(nRF_STATUS);
	nRF_SPI_IO_WriteReg(nRF_WRITE_REG+nRF_STATUS, status);

	if (status & nRF_MAX_TX)
	{
		nRF_CSN_LOW();
		nRF_SPI_IO_WriteReg(nRF_FLUSH_TX, 0x00);
		nRF_CSN_HIGH();
		return nRF_MAX_TX;
	}
	if (status & nRF_TX_OK)
	{
		nRF_SPI_IO_WriteReg(nRF_WRITE_REG+nRF_STATUS, status);
		return nRF_TX_OK;
	}

	return status;
}

uint8_t nRF_RxPacket(uint8_t *rxbuf)
{
	uint8_t status;

	status = nRF_SPI_IO_ReadReg(nRF_STATUS);
	nRF_SPI_IO_WriteReg(nRF_WRITE_REG+nRF_STATUS, status);
	if (status & nRF_RX_OK)
	{
		nRF_SPI_IO_ReadData(nRF_RD_RX_PLOAD, rxbuf, nRF_RX_PLOAD_WIDTH);
		nRF_CSN_LOW();
		nRF_SPI_IO_WriteReg(nRF_FLUSH_RX, 0xFF);
		nRF_CSN_HIGH();
		return 0;
	}
	nRF_CSN_LOW();
	nRF_SPI_IO_WriteReg(nRF_FLUSH_RX, 0xFF);
	nRF_CSN_HIGH();
	status = nRF_SPI_IO_ReadReg(nRF_FIFO_STATUS);

	return status;
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	uint8_t RegValue;
	RegValue = nRF_SPI_IO_ReadReg(nRF_STATUS);
	nRF_SPI_IO_WriteReg(nRF_WRITE_REG+nRF_STATUS, RegValue);
	RegValue = nRF_SPI_IO_ReadReg(nRF_FIFO_STATUS);
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
uint8_t	retValue;
#if defined(TX_SEND)
uint8_t buf[33] = "Tx Send ...\n";
#else
uint8_t buf[33];
#endif

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
		//if (nRF_Check() == HAL_OK)
		{
			#if defined(TX_SEND)
			nRF_TX_Mode();
			if ((retValue = nRF_TxPacket(buf))== nRF_TX_OK)
			{
				BSP_LED_Toggle(LED2);
				UART_PutString(buf, 12);\
			}
			else if (retValue == nRF_MAX_TX)
			{
				UART_PutString("Max TX\n", 7);
			}
			//vTaskDelayUntil( &xLastTime, 1);
			//nRF_RX_Mode();
			//if (nRF_RxPacket(buf) == 0)
			//{
			//	BSP_LED_Off(LED2);
			//	UART_PutString(buf, 12);
			//}
			#else
			nRF_RX_Mode();
			vTaskDelayUntil(&xLastTime, 1);
			memset(buf, 0, 33);
			if ((retValue = nRF_RxPacket(buf)) == 0)
			{
				BSP_LED_Toggle(LED2);
				UART_PutString(buf, 12);
				//vTaskDelayUntil( &xLastTime, xRate);
			}
			else
			{
				//vTaskDelayUntil( &xLastTime, xRate);
				UART_PutString("Rev Err\n", 8);
			}
			//vTaskDelayUntil( &xLastTime, 1);
			//nRF_TX_Mode();
			//if (nRF_TxPacket(buf)== nRF_TX_OK)
			//{
			//	BSP_LED_Off(LED2);
			//	UART_PutString(buf, 12);
			//}
			#endif
		}
	}
} /*lint !e715 !e818 !e830 Function definition must be standard for task creation. */

