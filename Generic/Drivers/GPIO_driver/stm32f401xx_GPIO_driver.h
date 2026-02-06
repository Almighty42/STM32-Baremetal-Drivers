#ifndef STM32F401RE_GPIO_DRIVER_H
#define STM32F401RE_GPIO_DRIVER_H

#include "../../stm32f401xx.h"

// NOTE: Configuration structure for a GPIO pin

typedef struct {
	uint8_t GPIO_Pin_Number;		// Possible values from @GPIO_PIN_NUMBERS
	uint8_t GPIO_Pin_Mode;			// Possible values from @GPIO_PIN_MODES
	uint8_t GPIO_Pin_Speed;			// Possible values from @GPIO_PIN_SPEED
	uint8_t GPIO_Pin_PuPd_Control;		// Possible values from @GPIO_PIN_PUPD
	uint8_t GPIO_Pin_OTP_Type;		// Possible values from @GPIO_PIN_TYPE
	uint8_t GPIO_Pin_Alt_Fun_Mode;
} GPIO_Pin_Config_t;

// NOTE: Handle structure for a GPIO pin

typedef struct {
	GPIO_TypeDef *p_GPIOx;			// Holds the base address of the GPIO port to which the pin belongs
	GPIO_Pin_Config_t GPIO_Pin_Config;	// Holds GPIO pin configuration settings
} GPIO_Handle_t;

// NOTE: @GPIO_PIN_NUMBERS

#define GPIO_PIN_NO_0		0
#define GPIO_PIN_NO_1		1
#define GPIO_PIN_NO_2		2
#define GPIO_PIN_NO_3		3
#define GPIO_PIN_NO_4		4
#define GPIO_PIN_NO_5		5
#define GPIO_PIN_NO_6		6
#define GPIO_PIN_NO_7		7
#define GPIO_PIN_NO_8		8
#define GPIO_PIN_NO_9		9
#define GPIO_PIN_NO_10		10
#define GPIO_PIN_NO_11		11
#define GPIO_PIN_NO_12		12
#define GPIO_PIN_NO_13		13
#define GPIO_PIN_NO_14		14
#define GPIO_PIN_NO_15		15

// NOTE: GPIO_PIN_MODES

#define GPIO_MODE_IN		0
#define GPIO_MODE_OUT		1
#define GPIO_MODE_ALT		2
#define GPIO_MODE_ANALOG	3
#define GPIO_MODE_IT_FT		4
#define GPIO_MODE_IT_RT		5
#define GPIO_MODE_IT_RFT	6

// NOTE: GPIO_PIN_TYPE

#define GPIO_OP_TYPE_PP		0
#define GPIO_OP_TYPE_OD		1

// NOTE: GPIO_PIN_SPEED

#define GPIO_SPEED_LOW		0
#define GPIO_SPEED_MED		1
#define GPIO_SPEED_FAST		2
#define GPIO_SPEED_HIGH		3

// NOTE: GPIO_PIN_PUPD

#define GPIO_NO_PUPD		0
#define GPIO_PIN_PU		1
#define GPIO_PIN_PD		2

// NOTE: --- APIs supported by this driver ---

// NOTE: Peripheral clock setup
void GPIO_peri_clk_control(GPIO_TypeDef *p_GPIOx, uint8_t EN_or_DI);

// NOTE: Init / de-init
void GPIO_init(GPIO_Handle_t *p_GPIO_handle);
void GPIO_de_init(GPIO_TypeDef *p_GPIOx);

// NOTE: Data read/write
uint8_t GPIO_read_input_pin(GPIO_TypeDef *p_GPIOx, uint8_t pin_n);
uint16_t GPIO_read_input_port(GPIO_TypeDef *p_GPIOx);
void GPIO_write_output_pin(GPIO_TypeDef *p_GPIOx, uint8_t pin_n, uint8_t val);
void GPIO_write_output_port(GPIO_TypeDef *p_GPIOx, uint8_t val);
void GPIO_toggle_output_pin(GPIO_TypeDef *p_GPIOx, uint8_t pin_n);

// NOTE: IRQ configuration and ISR handling
void GPIO_irq_interrupt_config(uint8_t irq_n,  uint8_t EN_or_DI);
void GPIO_irq_priority_config(uint8_t irq_n ,uint8_t irq_prio);
void GPIO_irq_handling(uint8_t pin_n);

#endif
