#ifndef __PWM_H_
#define __PWM_H_

#define PWM_PERIOD		100				// PWMÊä³öÆµÂÊ
#define PWM_TIM			TIM4			// Use TIMx for PWM

typedef struct
{
	uint16_t	left_rate;
	uint16_t	right_rate;
} PWM_Rate_Type;

extern void PWM_GPIO_Init(void);
extern void PWM_TIM_Config(uint32_t Freq, PWM_Rate_Type rate);

#endif
