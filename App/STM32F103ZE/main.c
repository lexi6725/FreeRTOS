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
#include "event_groups.h"
#include "main.h"

/* Library includes. */
#include "stm32f1xx_it.h"
#include "stm32f1xx_hal.h"

/* Demo app includes. */
#include "stm3210e_bit3.h"
#include "flash.h"
#include "serial.h"
#include "hmc5883l.h"
#include "pwm.h"
#include "serial.h"
#include "nrf24l01.h"
#include "mpu9050.h"

/* Task priorities. */
#define mainFLASH_TASK_PRIORITY				( tskIDLE_PRIORITY + 1 )
#define mainRF_TASK_PRIORITY				( tskIDLE_PRIORITY + 2 )

static portTASK_FUNCTION_PROTO( vHMC5883LTask, pvParameters );
static portTASK_FUNCTION_PROTO( vMPU9050Task, pvParameters );

/*-----------------------------------------------------------*/
void SystemClock_Config(void);

// EventGroup
EventGroupHandle_t xEventGruop = NULL;

int main( void )
{
#ifdef DEBUG
  debug();
#endif
	HAL_Init();
	SystemClock_Config();
	BSP_LED_Init(LED1);
	BSP_LED_Init(LED2);
	//xSerialPortInitMinimal(115200, 20);
	PWM_GPIO_Init();
	UART_Init(115200);
	//BSP_LCD_Init();
	//MPU9050_Init();

	if ((xEventGruop = xEventGroupCreate()) != NULL)
	{
		vStartLEDFlashTasks( mainFLASH_TASK_PRIORITY );
		xTaskCreate( vHMC5883LTask, "HMC5883L", configMINIMAL_STACK_SIZE, NULL, 1, ( TaskHandle_t * ) NULL );
		//xTaskCreate( vMPU9050Task, "MPU", configMINIMAL_STACK_SIZE, NULL, 1, ( TaskHandle_t * ) NULL );
		vStartnRFTasks(mainRF_TASK_PRIORITY);
		
		/* Start the scheduler. */
		vTaskStartScheduler();
	}
	else
	{
		BSP_LED_On(LED2);
		while(1);
	}

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
  RCC_ClkInitTypeDef clkinitstruct = {0};
  RCC_OscInitTypeDef oscinitstruct = {0};
  
  /* Enable HSE Oscillator and activate PLL with HSE as source */
  oscinitstruct.OscillatorType  = RCC_OSCILLATORTYPE_HSE;
  oscinitstruct.HSEState        = RCC_HSE_ON;
  oscinitstruct.HSEPredivValue  = RCC_HSE_PREDIV_DIV1;
  oscinitstruct.PLL.PLLState    = RCC_PLL_ON;
  oscinitstruct.PLL.PLLSource   = RCC_PLLSOURCE_HSE;
  oscinitstruct.PLL.PLLMUL      = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&oscinitstruct)!= HAL_OK)
  {
    /* Initialization Error */
    while(1);
  }

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  clkinitstruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  clkinitstruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  clkinitstruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  clkinitstruct.APB2CLKDivider = RCC_HCLK_DIV1;
  clkinitstruct.APB1CLKDivider = RCC_HCLK_DIV2;  
  if (HAL_RCC_ClockConfig(&clkinitstruct, FLASH_LATENCY_2)!= HAL_OK)
  {
    /* Initialization Error */
    while(1);
  }
}

static portTASK_FUNCTION( vHMC5883LTask, pvParameters )
{
	TickType_t xRate, xLastTime;
	uint8_t isConnect;
	HMC_Data_t hmc_data;

	/* The parameters are not used. */
	( void ) pvParameters;
	
	xRate = 500;
	xRate /= portTICK_PERIOD_MS;
	
	/* We need to initialise xLastFlashTime prior to the first call to 
	vTaskDelayUntil(). */
	xLastTime = xTaskGetTickCount();

	isConnect = 10;

	for(;;)
	{
		if (isConnect < 10)
		{
			if (HMC5883L_ReadAngle(&hmc_data))
			{
				printf("\nx: %d\ty: %d\tz: %d\t", hmc_data.direct.x, hmc_data.direct.y, hmc_data.direct.z);
				printf("angle_x: %.2f\tangle_y: %.2f\tangle_z: %.2f\n", hmc_data.angle.x, hmc_data.angle.y, hmc_data.angle.z);
				BSP_LED_Toggle(LED2);
			}
			else
				isConnect++;
		}
		else
		{
			vTaskDelayUntil(&xLastTime, 500);
			if (HMC5883L_IsReady(10))
			{
				HMC5883L_Init();
				isConnect = 0;
				printf("HMC5883L Init ... \n");
			}
			printf("HMC5883L not Connect ...\n");
		}
	}
}


static portTASK_FUNCTION( vMPU9050Task, pvParameters )
{
	mpu9050_t mpu9050;

	/* The parameters are not used. */
	( void ) pvParameters;

	for(;;)
	{
		if (MPU9050_Read(&mpu9050))
		{
			printf("accel:\nx= %d\ny= %d\nz= %d\n", mpu9050.accel.x, mpu9050.accel.y, mpu9050.accel.z);
			printf("gyro:\nx= %d\ny= %d\nz= %d\n", mpu9050.gyro.x, mpu9050.gyro.y, mpu9050.gyro.z);
		}
	}
}

int fputc(int ch, FILE *f)
{
	UART_PutString((uint8_t *)&ch , 1);
	
	return ch;
}

int GetKey(void)
{
	int ch;
	UART_GetString((uint8_t *)&ch, 1);

	return ch;
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
