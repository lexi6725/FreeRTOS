#include <stdlib.h>

#include "FreeRTOS.h"
#include "task.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_conf.h"

#include "nrf24l01.h"
#include "pwm.h"

TIM_HandleTypeDef 	htim4;
PWM_Rate_Type		pwm_rate = {0, 0};
uint8_t pwmState 	= 0;

void PWM_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	PWM_CLK_ENABLE();
	PWM_TIM_CLK_ENABLE();
	PWM_CLK_AF_CLK_ENABLE();

	/* GPIOB Configure: TIM4 Channel 3 and 4 alternate function push-pull */
	GPIO_InitStructure.Pin		= PWM_LEFT_PIN | PWM_RITHT_PIN;
	GPIO_InitStructure.Mode		= GPIO_MODE_AF_PP;
	//GPIO_InitStructure.Pull		= GPIO_PULLUP;
	GPIO_InitStructure.Speed	= GPIO_SPEED_HIGH;
	HAL_GPIO_Init(PWM_GPIO_PORT, &GPIO_InitStructure);

	/* GPIOD Configure Direct Ctr */
	GPIO_InitStructure.Pin		= PWM_CTR_LEFT_PIN | PWM_CTR_RIGHT_PIN;
	GPIO_InitStructure.Mode		= GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Pull		= GPIO_NOPULL;
	GPIO_InitStructure.Speed	= GPIO_SPEED_HIGH;
	HAL_GPIO_Init(PWM_CTR_GPIO_PORT, &GPIO_InitStructure);
}

void PWM_TIM_Config(PWM_Rate_Type rate)
{
	TIM_MasterConfigTypeDef sMasterConfig;
	TIM_OC_InitTypeDef sConfigOC;

	if (rate.left_rate > PWM_PERIOD)
		rate.left_rate = PWM_PERIOD;
	if (rate.right_rate > PWM_PERIOD)
		rate.right_rate = PWM_PERIOD;
	if (rate.left_rate < PWM_MIN_RATE)
		rate.left_rate = PWM_MIN_RATE;
	if (rate.right_rate < PWM_MIN_RATE)
		rate.right_rate = PWM_MIN_RATE;

	htim4.Instance			= PWM_TIM;
	htim4.Init.Prescaler	= (uint16_t)(SystemCoreClock/PWM_FREQ)-1;
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

	pwm_rate.left_rate = rate.left_rate;
	pwm_rate.right_rate = rate.right_rate;
	
	sConfigOC.Pulse			= rate.left_rate;
	HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3);

	sConfigOC.Pulse			= rate.right_rate;
	HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4);

	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
	pwmState |= PWM_START;
}

uint8_t PWM_Ctr_Dir(PWM_Ctr_Type* ctr)
{
	if (ctr->type&0xF0 != DataType_Key)
		return HAL_ERROR;

	if (ctr->data[0] &(DIR_UP|Key_K1) == (DIR_UP|Key_K1))
	{
		HAL_GPIO_WritePin(PWM_CTR_GPIO_PORT, PWM_CTR_LEFT_PIN, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(PWM_CTR_GPIO_PORT, PWM_CTR_RIGHT_PIN, GPIO_PIN_SET);
		pwm_rate.left_rate += PWM_PERIOD/20;
		pwm_rate.right_rate += PWM_PERIOD/20;
		PWM_TIM_Config(pwm_rate);
	}
	else if (ctr->data[0] & (DIR_DOWN|Key_K1) == (DIR_DOWN|Key_K1))
	{
		HAL_GPIO_WritePin(PWM_CTR_GPIO_PORT, PWM_CTR_LEFT_PIN, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(PWM_CTR_GPIO_PORT, PWM_CTR_RIGHT_PIN, GPIO_PIN_SET);
		pwm_rate.left_rate -= PWM_PERIOD/20;
		pwm_rate.right_rate -= PWM_PERIOD/20;
		PWM_TIM_Config(pwm_rate);
	}
	else if(ctr->data[0] & (DIR_LEFT|Key_K1) == (DIR_LEFT|Key_K1))
	{
		HAL_GPIO_WritePin(PWM_CTR_GPIO_PORT, PWM_CTR_LEFT_PIN, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(PWM_CTR_GPIO_PORT, PWM_CTR_RIGHT_PIN, GPIO_PIN_SET);
		if (pwm_rate.left_rate > pwm_rate.right_rate)
			pwm_rate.right_rate = pwm_rate.left_rate;
		else
			pwm_rate.left_rate = pwm_rate.right_rate;
		PWM_TIM_Config(pwm_rate);
	}
	else if(ctr->data[0] & (DIR_RIGHT|Key_K1) == (DIR_RIGHT|Key_K1))
	{
		HAL_GPIO_WritePin(PWM_CTR_GPIO_PORT, PWM_CTR_LEFT_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(PWM_CTR_GPIO_PORT, PWM_CTR_RIGHT_PIN, GPIO_PIN_RESET);
		if (pwm_rate.left_rate > pwm_rate.right_rate)
			pwm_rate.right_rate = pwm_rate.left_rate;
		else
			pwm_rate.left_rate = pwm_rate.right_rate;
		PWM_TIM_Config(pwm_rate);
	}
	else if (ctr->data[0] & DIR_UP)
	{
		HAL_GPIO_WritePin(PWM_CTR_GPIO_PORT, PWM_CTR_LEFT_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(PWM_CTR_GPIO_PORT, PWM_CTR_RIGHT_PIN, GPIO_PIN_SET);
		if (pwm_rate.left_rate > pwm_rate.right_rate)
			pwm_rate.right_rate = pwm_rate.left_rate;
		else
			pwm_rate.left_rate = pwm_rate.right_rate;
		PWM_TIM_Config(pwm_rate);
	}
	else if (ctr->data[0] & DIR_DOWN)
	{
		HAL_GPIO_WritePin(PWM_CTR_GPIO_PORT, PWM_CTR_LEFT_PIN, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(PWM_CTR_GPIO_PORT, PWM_CTR_RIGHT_PIN, GPIO_PIN_RESET);
		if (pwm_rate.left_rate > pwm_rate.right_rate)
			pwm_rate.right_rate = pwm_rate.left_rate;
		else
			pwm_rate.left_rate = pwm_rate.right_rate;
		PWM_TIM_Config(pwm_rate);
	}
	else if (ctr->data[0] & DIR_LEFT)
	{
		HAL_GPIO_WritePin(PWM_CTR_GPIO_PORT, PWM_CTR_LEFT_PIN, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(PWM_CTR_GPIO_PORT, PWM_CTR_RIGHT_PIN, GPIO_PIN_SET);
		pwm_rate.left_rate -= PWM_PERIOD/20;
		PWM_TIM_Config(pwm_rate);
	}
	else if (ctr->data[0] & DIR_RIGHT)
	{
		HAL_GPIO_WritePin(PWM_CTR_GPIO_PORT, PWM_CTR_LEFT_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(PWM_CTR_GPIO_PORT, PWM_CTR_RIGHT_PIN, GPIO_PIN_RESET);
		pwm_rate.right_rate -= PWM_PERIOD/20;
		PWM_TIM_Config(pwm_rate);
	}
	else if (ctr->data[0] & Key_K2)
	{
		if (pwmState & PWM_START)
		{
			HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
			HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4);
		}
		else
		{
			HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
			HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
		}
	}
	return HAL_OK;
}

