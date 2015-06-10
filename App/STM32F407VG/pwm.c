#include <stdlib.h>

#include "FreeRTOS.h"
#include "task.h"
#include "pwm.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_conf.h"

TIM_Base_InitTypeDef	TimHandle;


void PWM_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_TIM4_CLK_ENABLE();
	__HAL_RCC_AFIO_CLK_ENABLE();

	/* GPIOB Configure: TIM4 Channel 3 and 4 alternate function push-pull */
	GPIO_InitStructure.Pin		= GPIO_PIN_8 | GPIO_PIN_9;
	GPIO_InitStructure.Mode		= GPIO_MODE_AF_PP;
	//GPIO_InitStructure.Pull		= GPIO_PULLUP;
	GPIO_InitStructure.Speed	= GPIO_SPEED_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void PWM_TIM_Config(uint32_t Freq, PWM_Rate_Type rate)
{
	TIM_HandleTypeDef htim4;
	TIM_MasterConfigTypeDef sMasterConfig;
	TIM_OC_InitTypeDef sConfigOC;

	if (rate.left_rate > PWM_PERIOD)
		rate.left_rate = PWM_PERIOD;
	if (rate.right_rate > PWM_PERIOD)
		rate.right_rate = PWM_PERIOD;

	htim4.Instance			= TIM4;
	htim4.Init.Prescaler	= (uint16_t)(SystemCoreClock/Freq)-1;
	htim4.Init.Period		= PWM_PERIOD;
	htim4.Init.CounterMode	= TIM_COUNTERMODE_UP;
	htim4.Init.ClockDivision= TIM_CLOCKDIVISION_DIV1;
	HAL_TIM_PWM_Init(&htim4);

	sMasterConfig.MasterOutputTrigger	= TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode		= TIM_MASTERSLAVEMODE_DISABLE;
	HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig);

	sConfigOC.OCMode		= TIM_OCMODE_PWM1;
	sConfigOC.OCPolarity	= TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode	= TIM_OCFAST_DISABLE;
	sConfigOC.OCNPolarity 	= TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCNIdleState	= TIM_OCNIDLESTATE_RESET;
	sConfigOC.OCIdleState	= TIM_OCIDLESTATE_RESET;
	
	sConfigOC.Pulse			= rate.left_rate;
	HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3);

	sConfigOC.Pulse			= rate.right_rate;
	HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4);

	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
}

