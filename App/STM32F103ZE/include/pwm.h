#ifndef __PWM_H_
#define __PWM_H_

#define PWM_FREQ		100000
#define PWM_PERIOD		100				// PWMÊä³öÆµÂÊ
#define PWM_TIM			TIM4			// Use TIMx for PWM
#define PWM_MIN_RATE	10

#define PWM_START		(0x01<<0)
#define PWM_DOWN		(0x01<<0)

#define PWM_LEFT_PIN		GPIO_PIN_8
#define PWM_RITHT_PIN		GPIO_PIN_9
#define PWM_GPIO_PORT		GPIOB
#define PWM_CLK_ENABLE()	__HAL_RCC_GPIOB_CLK_ENABLE()
#define PWM_CLK_DISABEL()	__HAL_RCC_GPIOB_CLK_DISABLE()
#define PWM_CLK_AF_CLK_ENABLE()	__HAL_RCC_AFIO_CLK_ENABLE()
#define PWM_CLK_AF_CLK_DISABLE()	__HAL_RCC_AFIO_CLK_DISABLE()
#define PWM_TIM_CLK_ENABLE()	__HAL_RCC_TIM4_CLK_ENABLE()
#define PWM_TIM_CLK_DISABLE()	__HAL_RCC_TIM4_CLK_DISABLE()

#define PWM_CTR_LEFT1_PIN		GPIO_PIN_0
#define PWM_CTR_LEFT2_PIN		GPIO_PIN_1
#define PWM_CTR_RIGHT1_PIN		GPIO_PIN_2
#define PWM_CTR_RIGHT2_PIN		GPIO_PIN_3
#define PWM_CTR_GPIO_PORT		GPIOC
#define PWM_CTR_CLK_ENABLE()	__HAL_RCC_GPIOC_CLK_ENABLE()
#define PWM_CTR_CLK_DISABLE()	__HAL_RCC_GPIOC_CLK_DISABLE()

/* KeyStatus Flag */
#define KEY_BRK				(0x01<<0)
#define KEY_SPD				(0x01<<1)
#define KEY_LEFT			(0x01<<2)
#define KEY_UP				(0x01<<3)
#define KEY_DOWN			(0x01<<4)
#define KEY_Fn				(0x01<<5)
#define KEY_SEL				(0x01<<6)
#define KEY_RIGHT			(0x01<<7)

#define KEY_SPD_ADD			(KEY_SPD)
#define KEY_SPD_RDC			(KEY_Fn|KEY_SPD)

typedef struct
{
	uint16_t	left_rate;
	uint16_t	right_rate;
} PWM_Rate_Type;

typedef struct 
{
	uint8_t type;
	uint8_t data[4];
} PWM_Ctr_Type;

extern void PWM_GPIO_Init(void);
extern void PWM_TIM_Config(PWM_Rate_Type rate);
uint8_t PWM_Ctr_Dir(PWM_Ctr_Type * ctr);

#endif
