#ifndef __KEY_H_
#define __KEY_H_

/**
  * @brief Joystick Pins definition 
  */ 
#define JOY_SEL_PIN					GPIO_PIN_0
#define JOY_RIGHT_PIN				GPIO_PIN_1
#define JOY_SEL_RIGHT_GPIO_PORT		GPIOA
#define JOY_SEL_RIGHT_CLK_ENABLE()	__GPIOA_CLK_ENABLE()
#define JOY_SEL_RIGHT_CLK_DISABLE()	__GPIOA_CLK_DISABLE()
#define KEYNUMPA					2

#define JOY_LEFT_PIN				GPIO_PIN_2
#define JOY_UP_PIN					GPIO_PIN_3
#define JOY_DOWN_PIN				GPIO_PIN_4
#define JOY_LEFT_UP_DOWN_GPIO_PORT	GPIOC
#define JOY_LEFT_UP_DOWN_CLK_ENABLE()	__GPIOC_CLK_ENABLE()
#define JOY_LEFT_UP_DOWN_CLK_DISABLE()	__GPIOC_CLK_DISABLE()

#define KEY_K1_PIN					GPIO_PIN_13
#define KEY_K2_PIN					GPIO_PIN_0
#define KEY_K3_PIN					GPIO_PIN_1
#define KEY_K123_GPIO				GPIOC
#define KEY_K123_CLK_ENABLE()		__GPIOC_CLK_ENABLE()
#define KEY_K123_CLK_DISABLE()		__GPIOC_CLK_DISABLE()
#define KEYNUMPC					6

#define KEY_SCAN_FREQ				200

/* KeyStatus Flag */
#define KEY_BRK				(0x01<<0)
#define KEY_SPD				(0x01<<1)
#define KEY_LEFT			(0x01<<2)
#define KEY_UP				(0x01<<3)
#define KEY_DOWN			(0x01<<4)
#define KEY_Fn				(0x01<<5)
#define KEY_SEL				(0x01<<6)
#define KEY_RIGHT			(0x01<<7)


void vStartKeyTasks( UBaseType_t uxPriority );
uint8_t Get_Key_Status(void);


#endif
