/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"

/* Library includes. */
#include "stm32f4xx.h"
#include "stm32f4xx_hal_conf.h"
#include "stm32f4xx_hal.h"
#include "key.h"
#include "nrf24l01.h"

static uint16_t KeyPinC[KEYNUMPC] = {
	KEY_K2_PIN,
	KEY_K3_PIN,
	JOY_LEFT_PIN,
	JOY_UP_PIN,
	JOY_DOWN_PIN,
	KEY_K1_PIN
};

static uint16_t KeyPinA[KEYNUMPA] = {
	JOY_SEL_PIN,
	JOY_RIGHT_PIN
};

static uint8_t KeyStatus = 0;

extern EventGroupHandle_t xEventGruop;;

/**
  * @brief Configure JOY BUTTON GPIO and EXTI Line.
  * @param  None
  * @retval None
  */
void Key_Init(void)
{
	GPIO_InitTypeDef gpioinitstruct = {0};

	/* Enable the corresponding JOY Button Clock */
	JOY_SEL_RIGHT_CLK_ENABLE();
	JOY_LEFT_UP_DOWN_CLK_ENABLE();

	/* Configure Button pin as input */
	gpioinitstruct.Pin		= JOY_SEL_PIN|JOY_RIGHT_PIN;
	gpioinitstruct.Pull		= GPIO_NOPULL;
	gpioinitstruct.Speed	= GPIO_SPEED_HIGH;
	gpioinitstruct.Mode		= GPIO_MODE_INPUT;
	HAL_GPIO_Init(JOY_SEL_RIGHT_GPIO_PORT, &gpioinitstruct);

	gpioinitstruct.Pin		= JOY_LEFT_PIN|JOY_UP_PIN|JOY_DOWN_PIN|KEY_K1_PIN|KEY_K2_PIN|KEY_K3_PIN;
	gpioinitstruct.Pull		= GPIO_NOPULL;
	gpioinitstruct.Speed	= GPIO_SPEED_HIGH;
	gpioinitstruct.Mode		= GPIO_MODE_INPUT;
	HAL_GPIO_Init(JOY_LEFT_UP_DOWN_GPIO_PORT, &gpioinitstruct);
}

void Key_Scan(void)
{
	uint8_t index;
	uint8_t flag = 0;
	
	/* Scan PC[0:4], PC13*/
	for (index = 0; index < KEYNUMPC; ++index)
	{
		if (HAL_GPIO_ReadPin(KEY_K123_GPIO, KeyPinC[index]) == GPIO_PIN_RESET)
		{
			if(!(KeyStatus&(0x01<<index)))
			{
				KeyStatus |= (0x01<<index);
				if (index < (KEYNUMPC-1))	// K1作为功能键，激发第二功能
					flag = 1;
			}
		}
		else
		{
			if (KeyStatus & (0x01<<index))
			{
				KeyStatus &= ~(0x01<<index);
				//flag = 1;
			}
		}
	}

	/* Scan PA[0:1] */
	for (index = 0; index < KEYNUMPA; ++index)
	{
		if (HAL_GPIO_ReadPin(JOY_SEL_RIGHT_GPIO_PORT, KeyPinA[index]) == GPIO_PIN_RESET)
		{
			if (!(KeyStatus & (0x01<<(index+KEYNUMPC))))
			{
				KeyStatus |= (0x01<<(index+KEYNUMPC));
				flag = 1;
			}
		}
		else
		{
			if (KeyStatus & (0x01<<(index+KEYNUMPC)))
			{
				KeyStatus &= ~(0x01<<(index+KEYNUMPC));
				//flag = 1;
			}
		}
	}
	if (flag && xEventGruop)
		xEventGroupSetBits(xEventGruop, Key_State_Down);
}

uint8_t Get_Key_Status(void)
{
	return KeyStatus;
}

static portTASK_FUNCTION( vnKeyTask, pvParameters )
{
TickType_t xRate, xLastTime;

	/* The parameters are not used. */
	( void ) pvParameters;

	Key_Init();
	xRate = configTICK_RATE_HZ/KEY_SCAN_FREQ;

	/* We need to initialise xLastFlashTime prior to the first call to 
	vTaskDelayUntil(). */
	xLastTime = xTaskGetTickCount();
	
	for(;;)
	{
		vTaskDelayUntil( &xLastTime, xRate );
		portENTER_CRITICAL();
		Key_Scan();
		portEXIT_CRITICAL();
	}
} /*lint !e715 !e818 !e830 Function definition must be standard for task creation. */

void vStartKeyTasks( UBaseType_t uxPriority )
{
	xTaskCreate( vnKeyTask, "Key", configMINIMAL_STACK_SIZE, NULL, uxPriority, ( TaskHandle_t * ) NULL );
}

