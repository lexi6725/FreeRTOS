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
 * Creates all the demo application tasks, then starts the scheduler.  The WEB
 * documentation provides more details of the standard demo application tasks.
 * In addition to the standard demo tasks, the following tasks and tests are
 * defined and/or created within this file:
 *
 * "Fast Interrupt Test" - A high frequency periodic interrupt is generated
 * using a free running timer to demonstrate the use of the
 * configKERNEL_INTERRUPT_PRIORITY configuration constant.  The interrupt
 * service routine measures the number of processor clocks that occur between
 * each interrupt - and in so doing measures the jitter in the interrupt timing.
 * The maximum measured jitter time is latched in the ulMaxJitter variable, and
 * displayed on the LCD by the 'Check' task as described below.  The
 * fast interrupt is configured and handled in the timertest.c source file.
 *
 * "LCD" task - the LCD task is a 'gatekeeper' task.  It is the only task that
 * is permitted to access the display directly.  Other tasks wishing to write a
 * message to the LCD send the message on a queue to the LCD task instead of
 * accessing the LCD themselves.  The LCD task just blocks on the queue waiting
 * for messages - waking and displaying the messages as they arrive.
 *
 * "Check" task -  This only executes every five seconds but has the highest
 * priority so is guaranteed to get processor time.  Its main function is to
 * check that all the standard demo tasks are still operational.  Should any
 * unexpected behaviour within a demo task be discovered the 'check' task will
 * write an error to the LCD (via the LCD task).  If all the demo tasks are
 * executing with their expected behaviour then the check task writes PASS
 * along with the max jitter time to the LCD (again via the LCD task), as
 * described above.
 *
 */

/* Standard includes. */
#include <stdio.h>
#include <math.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "main.h"

/* Library includes. */
#include "stm32f4xx_it.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_pwr.h"

/* Demo app includes. */
#include "stm324xg_eval.h"
#include "flash.h"
#include "serial.h"
#include "serial.h"
#include "nrf24l01.h"

/* Task priorities. */
#define mainFLASH_TASK_PRIORITY				( tskIDLE_PRIORITY + 1 )
#define mainRF_TASK_PRIORITY				( tskIDLE_PRIORITY + 2 )


/*-----------------------------------------------------------*/
void SystemClock_Config(void);

int main( void )
{
#ifdef DEBUG
  debug();
#endif
	HAL_Init();
	SystemClock_Config();
	BSP_LED_Init(LED1);
	BSP_LED_Init(LED2);
	BSP_LED_Init(LED3);
	BSP_LED_Init(LED4);
	UART_Init(115200);

	vStartLEDFlashTasks( mainFLASH_TASK_PRIORITY );
	vStartnRFTasks(mainRF_TASK_PRIORITY);

	/* Start the scheduler. */
	vTaskStartScheduler();

	/* Will only get here if there was not enough heap space to create the
	idle task. */
	return 0;
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 72000000
  *            HCLK(Hz)                       = 72000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 2
  *            APB2 Prescaler                 = 1
  *            HSE Frequency(Hz)              = 8000000
  *            HSE PREDIV1                    = 1
  *            PLLMUL                         = 9
  *            Flash Latency(WS)              = 2
  * @param  None
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();

  /* The voltage scaling allows optimizing the power consumption when the device is 
     clocked below the maximum system frequency, to update the voltage scaling value 
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  
  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);
 
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
  clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;  
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);

  /* STM32F405x/407x/415x/417x Revision Z devices: prefetch is supported  */
  if (HAL_GetREVID() == 0x1001)
  {
    /* Enable the Flash prefetch */
    __HAL_FLASH_PREFETCH_BUFFER_ENABLE();
  }
}

#if 0
static portTASK_FUNCTION( vHMC5883LTask, pvParameters )
{
	TickType_t xRate, xLastTime;
	uint8_t isConnect = 0;
	HMC_Measure Measure_data;
	uint8_t buf[12];
	uint16_t x, y, z;

	/* The parameters are not used. */
	( void ) pvParameters;
	
	xRate = 1000;
	xRate /= portTICK_PERIOD_MS;
	
	/* We need to initialise xLastFlashTime prior to the first call to 
	vTaskDelayUntil(). */
	xLastTime = xTaskGetTickCount();

	isConnect = 0;

	for(;;)
	{
		/* Delay for half the flash period then turn the LED on. */
		vTaskDelayUntil( &xLastTime, xRate );
		if (isConnect)
		{
			HMC5883L_ReadAngle(0x3C, &Measure_data);

			x = abs(Measure_data.x);
			buf[0] = 'x';
			buf[1] = '=';
			if (Measure_data.x > 0)
				buf[2] = '+';
			else
				buf[2] = '-';
			buf[3] = (x%10000)/1000+'0';
			buf[4] = (x%1000)/100+'0';
			buf[5] = (x%100)/10+'0';
			buf[6] = x%10+'0';
			buf[7] = '\n';
			UART_PutString(buf, 8);
			
			vTaskDelayUntil( &xLastTime, 2 );
			y = abs(Measure_data.y);
			buf[0] = 'y';
			buf[1] = '=';
			if (Measure_data.y > 0)
				buf[2] = '+';
			else
				buf[2] = '-';
			buf[3] = (y%10000)/1000+'0';
			buf[4] = (y%1000)/100+'0';
			buf[5] = (y%100)/10+'0';
			buf[6] = y%10+'0';
			buf[7] = '\n';
			UART_PutString(buf, 8);
			
			vTaskDelayUntil( &xLastTime, 2 );
			z = abs(Measure_data.z);
			buf[0] = 'z';
			buf[1] = '=';
			if (Measure_data.z > 0)
				buf[2] = '+';
			else
				buf[2] = '-';
			buf[3] = (z%10000)/1000+'0';
			buf[4] = (z%1000)/100+'0';
			buf[5] = (z%100)/10+'0';
			buf[6] = z%10+'0';
			buf[7] = '\n';
			UART_PutString(buf, 8);

			vTaskDelayUntil( &xLastTime, 2 );
			Measure_data.angle = (int16_t)(atan2((double)Measure_data.y,(double)Measure_data.x)*(180/3.1415926)+180);
			memcpy(buf, "angle=", 6);
			buf[6] = (Measure_data.angle%1000)/100+'0';
			buf[7] = (Measure_data.angle%100)/10+'0';
			buf[8] = (Measure_data.angle%10)+'0';
			buf[9] = '\n';
			buf[10] = '\n';
			UART_PutString(buf, 11);
			BSP_LED_Toggle(LED2);
		}
		else
		{
			vTaskDelayUntil(&xLastTime, 1000-xRate);
			if (HMC5883L_IsReady(0x3C, 10))
			{
				HMC5883L_Init(0x3C);
				isConnect = 1;
				UART_PutString("HMC5883L Init ... \n", 19);
			}
			UART_PutString("HMC5883L not Connect ...\n", 25);
		}
	}
}
#endif

static void Error_Handle(void)
{
	BSP_LED_On(LED1);
}

#ifdef  DEBUG
/* Keep the linker happy. */
void assert_failed( unsigned char* pcFile, unsigned long ulLine )
{
	for( ;; )
	{
	}
}
#endif
