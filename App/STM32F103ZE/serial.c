/*
    FreeRTOS V8.2.1 - Copyright (C) 2015 Real Time Engineers Ltd.
    All rights reserved

    VISIT http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.

    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation >>!AND MODIFIED BY!<< the FreeRTOS exception.

    ***************************************************************************
    >>!   NOTE: The modification to the GPL is included to allow you to     !<<
    >>!   distribute a combined work that includes FreeRTOS without being   !<<
    >>!   obliged to provide the source code for proprietary components     !<<
    >>!   outside of the FreeRTOS kernel.                                   !<<
    ***************************************************************************

    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE.  Full license text is available on the following
    link: http://www.freertos.org/a00114.html

    ***************************************************************************
     *                                                                       *
     *    FreeRTOS provides completely free yet professionally developed,    *
     *    robust, strictly quality controlled, supported, and cross          *
     *    platform software that is more than just the market leader, it     *
     *    is the industry's de facto standard.                               *
     *                                                                       *
     *    Help yourself get started quickly while simultaneously helping     *
     *    to support the FreeRTOS project by purchasing a FreeRTOS           *
     *    tutorial book, reference manual, or both:                          *
     *    http://www.FreeRTOS.org/Documentation                              *
     *                                                                       *
    ***************************************************************************

    http://www.FreeRTOS.org/FAQHelp.html - Having a problem?  Start by reading
    the FAQ page "My application does not run, what could be wrong?".  Have you
    defined configASSERT()?

    http://www.FreeRTOS.org/support - In return for receiving this top quality
    embedded software for free we request you assist our global community by
    participating in the support forum.

    http://www.FreeRTOS.org/training - Investing in training allows your team to
    be as productive as possible as early as possible.  Now you can receive
    FreeRTOS training directly from Richard Barry, CEO of Real Time Engineers
    Ltd, and the world's leading authority on the world's leading RTOS.

    http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
    including FreeRTOS+Trace - an indispensable productivity tool, a DOS
    compatible FAT file system, and our tiny thread aware UDP/IP stack.

    http://www.FreeRTOS.org/labs - Where new FreeRTOS products go to incubate.
    Come and try FreeRTOS+TCP, our new open source TCP/IP stack for FreeRTOS.

    http://www.OpenRTOS.com - Real Time Engineers ltd. license FreeRTOS to High
    Integrity Systems ltd. to sell under the OpenRTOS brand.  Low cost OpenRTOS
    licenses offer ticketed support, indemnification and commercial middleware.

    http://www.SafeRTOS.com - High Integrity Systems also provide a safety
    engineered and independently SIL3 certified version for use in safety and
    mission critical applications that require provable dependability.

    1 tab == 4 spaces!
*/

/*
	BASIC INTERRUPT DRIVEN SERIAL PORT DRIVER FOR UART0.
*/

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"

/* Library includes. */
#include "stm32f1xx.h"
#include "stm32f1xx_hal_dma.h"
#include "stm32f1xx_hal_uart.h"
#include "stm32f1xx_hal_conf.h"

/* Demo application includes. */
#include "serial.h"
#include "main.h"
/*-----------------------------------------------------------*/

/* Misc defines. */
#define serINVALID_QUEUE				( ( QueueHandle_t ) 0 )
#define serNO_BLOCK						( ( TickType_t ) 0 )
#define serTX_BLOCK_TIME				( 10 / portTICK_PERIOD_MS )

/*-----------------------------------------------------------*/

/* The queue used to hold received characters. */
static QueueHandle_t xRxedChars;
static QueueHandle_t xCharsForTx;

UART_HandleTypeDef	UartHandle;
portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

/*-----------------------------------------------------------*/

/* UART interrupt handler. */
void vUARTInterruptHandler( void );

/*-----------------------------------------------------------*/

/*
 * See the serial2.h header file.
 */
xComPortHandle xSerialPortInitMinimal( unsigned long ulWantedBaud, unsigned portBASE_TYPE uxQueueLength )
{
xComPortHandle xReturn;

	/* Create the queues used to hold Rx/Tx characters. */
	//xRxedChars = xQueueCreate( uxQueueLength, ( unsigned portBASE_TYPE ) sizeof( signed char ) );
	//xCharsForTx = xQueueCreate( uxQueueLength + 1, ( unsigned portBASE_TYPE ) sizeof( signed char ) );
	
	/* If the queue/semaphore was created correctly then setup the serial port
	hardware. */
	//if( ( xRxedChars != serINVALID_QUEUE ) && ( xCharsForTx != serINVALID_QUEUE ) )
	{
		/* ##-1- Enable peripherals and GPIO Clocks */
		/* Enable USART1 clock */
		#if 0
		__HAL_RCC_USART1_CLK_ENABLE();

		/* Enable GPIOA Clock */
		__HAL_RCC_GPIOA_CLK_ENABLE();

		/* ##-2- Configure Peripheral GPIO */
		/* Configure USART1 Rx (PA10) as input floating */
		GPIO_InitStructure.Pin = GPIO_PIN_10;
		GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStructure.Pull = GPIO_PULLUP;
		GPIO_InitStructure.Speed     = GPIO_SPEED_HIGH;
		HAL_GPIO_Init( GPIOA, &GPIO_InitStructure );
		
		/* Configure USART1 Tx (PA9) as alternate function push-pull */
		GPIO_InitStructure.Pin 	= GPIO_PIN_9;
		GPIO_InitStructure.Mode	= GPIO_MODE_AF_PP;
		HAL_GPIO_Init( GPIOA, &GPIO_InitStructure );

		/* ##-3- Configure the NVIC for UART */
		HAL_NVIC_SetPriority(USART1_IRQn, 4, 0);
		HAL_NVIC_EnableIRQ(USART1_IRQn);
		#endif
		/* ##-4- Configure USART1 Init Structure */
		UartHandle.Instance			= USART1;
		UartHandle.Init.BaudRate	= 115200;
		UartHandle.Init.WordLength	= UART_WORDLENGTH_8B;
		UartHandle.Init.StopBits	= UART_STOPBITS_1;
		UartHandle.Init.Parity		= UART_PARITY_NONE;
		UartHandle.Init.HwFlowCtl	= UART_HWCONTROL_NONE;
		UartHandle.Init.Mode		= UART_MODE_TX_RX;

		if (HAL_UART_DeInit(&UartHandle) != HAL_OK)
		{
			while(1);
		}
		if (HAL_UART_Init(&UartHandle) != HAL_OK)
		{
			while(1);		// Error
		}
				
	}
	//else
	//{
	//	xReturn = ( xComPortHandle ) 0;
	//}

	/* This demo file only supports a single port but we have to return
	something to comply with the standard demo header file. */
	return xReturn;
}
/*-----------------------------------------------------------*/
#if 0
signed portBASE_TYPE xSerialGetChar( xComPortHandle pxPort, signed char *pcRxedChar, TickType_t xBlockTime )
{
	/* The port handle is not required as this driver only supports one port. */
	( void ) pxPort;

	/* Get the next character from the buffer.  Return false if no characters
	are available, or arrive before xBlockTime expires. */
	if( xQueueReceive( xRxedChars, pcRxedChar, xBlockTime ) )
	{
		return pdTRUE;
	}
	else
	{
		return pdFALSE;
	}
}
#endif
/*-----------------------------------------------------------*/

void vSerialPutString( xComPortHandle pxPort, const signed char * const pcString, unsigned short usStringLength )
{
signed char *pxNext;

	/* A couple of parameters that this port does not use. */
	( void ) usStringLength;
	( void ) pxPort;

	/* NOTE: This implementation does not handle the queue being full as no
	block time is used! */

	/* The port handle is not required as this driver only supports UART1. */
	( void ) pxPort;

	/* Send each character in the string, one at a time. */
	pxNext = ( signed char * ) pcString;

	if (HAL_UART_Transmit_DMA(&UartHandle, (uint8_t *)pcString, usStringLength) != HAL_OK)
	{
		while(1);
	}
	while (HAL_UART_GetState(&UartHandle) != HAL_UART_STATE_READY)
	{
	}
	//while( *pxNext )
	//{
	//	xSerialPutChar( pxPort, *pxNext, serTX_BLOCK_TIME );
	//	pxNext++;
	//}
}
/*-----------------------------------------------------------*/
#if 0
signed portBASE_TYPE xSerialPutChar( xComPortHandle pxPort, signed char cOutChar, TickType_t xBlockTime )
{
signed portBASE_TYPE xReturn;

	if( xQueueSend( xCharsForTx, &cOutChar, xBlockTime ) == pdPASS )
	{
		xReturn = pdPASS;
		__HAL_UART_ENABLE_IT(&UartHandle, UART_IT_TXE);
		//HAL_UART_Transmit_IT(&UartHandle,(uint8_t *)&cOutChar, 1);
	}
	else
	{
		xReturn = pdFAIL;
	}

	return xReturn;
}
#endif
/*-----------------------------------------------------------*/

void vSerialClose( xComPortHandle xPort )
{
	/* Not supported as not required by the demo application. */
}
/*-----------------------------------------------------------*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	//char cChar;

	/*if ((HAL_UART_Receive_IT(huart,(uint8_t *)&cChar, 1)) == HAL_OK)
	{
		xQueueSendFromISR( xRxedChars, &cChar, &xHigherPriorityTaskWoken);
	}*/
	
	vParTestToggleLED(1);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	/*char cChar;
	if (xQueueReceiveFromISR( xCharsForTx, &cChar, &xHigherPriorityTaskWoken) == pdTRUE)
	{
		HAL_UART_Transmit_IT(huart,(uint8_t *)&cChar, 1);
	}
	else
	{
		__HAL_UART_DISABLE_IT(huart, UART_IT_TXE);
	}*/
	vParTestToggleLED(1);
}	

void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle)
{
	while(1);
}

void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	static DMA_HandleTypeDef hdma_tx;
	static DMA_HandleTypeDef hdma_rx;
	
	__HAL_RCC_USART1_CLK_ENABLE();

	/* Enable GPIOA Clock */
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/* Enable DMA Clock */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* ##-2- Configure Peripheral GPIO */
	/* Configure USART1 Rx (PA10) as input floating */
	GPIO_InitStructure.Pin = GPIO_PIN_10;
	GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	GPIO_InitStructure.Speed     = GPIO_SPEED_HIGH;
	HAL_GPIO_Init( GPIOA, &GPIO_InitStructure );
	
	/* Configure USART1 Tx (PA9) as alternate function push-pull */
	GPIO_InitStructure.Pin 	= GPIO_PIN_9;
	GPIO_InitStructure.Mode	= GPIO_MODE_AF_PP;
	GPIO_InitStructure.Pull = GPIO_PULLUP;
	HAL_GPIO_Init( GPIOA, &GPIO_InitStructure );

	/* ##-3- Configure the DMA */
	hdma_tx.Instance			= USARTx_DMA_TX_Channel;
	hdma_tx.Init.Direction		= DMA_MEMORY_TO_PERIPH;
	hdma_tx.Init.PeriphInc		= DMA_PINC_DISABLE;
	hdma_tx.Init.MemInc			= DMA_MINC_ENABLE;
	hdma_tx.Init.PeriphDataAlignment	= DMA_PDATAALIGN_BYTE;
	hdma_tx.Init.MemDataAlignment		= DMA_MDATAALIGN_BYTE;
	hdma_tx.Init.Mode			= DMA_NORMAL;
	hdma_tx.Init.Priority		= DMA_PRIORITY_LOW;

	HAL_DMA_Init(&hdma_tx);

	/* Associate the initialized DMA handle to the UART handle */
	__HAL_LINKDMA(huart, hdmatx, hdma_tx);


	/* ##-4- Configure the NVIC for DMA */
	/* NVIC Configure for DMA transfer complete interrupt */
	HAL_NVIC_SetPriority(USARTx_DMA_TX_IRQn, 0, 1);
	HAL_NVIC_EnableIRQ(USARTx_DMA_TX_IRQn);
	HAL_NVIC_SetPriority(USARTx_DMA_RX_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(USARTx_DMA_RX_IRQn);
	
	HAL_NVIC_SetPriority(USART1_IRQn, 0, 1);
	HAL_NVIC_EnableIRQ(USART1_IRQn);
	
}

void HAL_UART_MspDeInit(UART_HandleTypeDef *huart)
{
  /*##-1- Reset peripherals ##################################################*/
  __HAL_RCC_USART1_FORCE_RESET();
  __HAL_RCC_USART1_RELEASE_RESET();

  /*##-2- Disable peripherals and GPIO Clocks #################################*/
  /* Configure UART Tx as alternate function  */
  HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);
  
  /*##-3- Disable the NVIC for UART ##########################################*/
  HAL_NVIC_DisableIRQ(USART1_IRQn);
}

