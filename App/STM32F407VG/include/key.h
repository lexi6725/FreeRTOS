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

#define KEY_SCAN_FREQ				500

/* KeyStatus Flag */
#define KeyStatus_K2				(0x01<<0)
#define KeyStatus_K3				(0x01<<1)
#define KeyStatus_LEFT				(0x01<<2)
#define KeyStatus_UP				(0x01<<3)
#define KeyStatus_DOWN				(0x01<<4)
#define KeyStatus_K1				(0x01<<5)
#define KeyStatus_SEL				(0x01<<6)
#define KeyStatus_RIGHT				(0x01<<7)

void vStartKeyTasks( UBaseType_t uxPriority );
uint8_t Get_Key_Status(void);


#endif
