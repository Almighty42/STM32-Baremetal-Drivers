#include "stm32f401RE_header.h"
#include <stdint.h>

void GPIO_Clock_Enable(void);
void GPIO_Pin_Init_LED(void);
void GPIO_Pin_Init_Button(void);
void wait_ms(uint32_t ms);
uint8_t is_button_pressed();

int main(void)
{
	// Enables RCC clock for GPIO_A
	GPIO_Clock_Enable();

	// Prepares the GPIO_A for digital output and sets other settings
	GPIO_Pin_Init_LED();
	GPIO_Pin_Init_Button();

	while (1) {
		if (is_button_pressed()) {
			TOGGLE_BIT(GPIOA->ODR, 5);
		}
	}
}

void GPIO_Clock_Enable(void)
{
	SET_BIT(RCC->AHB1ENR, 0);
	SET_BIT(RCC->AHB1ENR, 2);
}

void GPIO_Pin_Init_LED(void)
{
	// Set mode as digital output
	CLEAR_FIELD_2BIT(GPIOA->MODER, 10);
	SET_BIT(GPIOA->MODER, 10);

	// Set output type as push-pull
	CLEAR_BIT(GPIOA->OTYPER, 5);

	// Set output speed as low
	// GPIOB->OSPEEDR &= ~(3UL << 4);
	CLEAR_FIELD_2BIT(GPIOA->OSPEEDR, 10);

	// Set no pull up, no pull down
	CLEAR_FIELD_2BIT(GPIOA->PUPDR, 10);
}

void GPIO_Pin_Init_Button(void)
{
	// Set mode as digital output
	CLEAR_FIELD_2BIT(GPIOC->MODER, 26);

	// Set output type as push-pull
	// CLEAR_BIT(GPIOA->OTYPER, 5);

	// Set output speed as low
	// GPIOB->OSPEEDR &= ~(3UL << 4);
	// CLEAR_FIELD_2BIT(GPIOA->OSPEEDR, 10);

	// Set no pull up, no pull down
	CLEAR_FIELD_2BIT(GPIOA->PUPDR, 26);
}

void wait_ms(uint32_t ms)
{
	for (uint32_t i = 0; i < ms; i++)
		for (uint32_t j = 0; j < 255; j++)
			;
}

uint8_t is_button_pressed()
{
	if (READ_BIT(GPIOC->IDR, 13) == 0UL)
		return 0;
	uint32_t counter = 0;
	for (uint32_t i = 0; i < 10; i++) {
		wait_ms(5);
		if (READ_BIT(GPIOC->IDR, 13) == 0UL)
			counter = 0;
		else {
			counter += 1;
			if (counter >= 4)
				return 1;
		}
	}
	return 0;
}
