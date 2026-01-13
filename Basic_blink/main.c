#include "stm32f401RE_header.h"
#include <stdint.h>

void GPIO_Clock_Enable(void);
void GPIO_Pin_Init(void);

int main(void)
{
	// Enables RCC clock for GPIO_A
	GPIO_Clock_Enable();

	// Prepares the GPIO_A for digital output and sets other settings
	GPIO_Pin_Init();

	while (1) {
		// LED ON
		SET_BIT(GPIOA->ODR, 5);
		for (uint32_t i = 0; i < 100000; i++)
			;
		// LED OFF
		CLEAR_BIT(GPIOA->ODR, 5);
		for (uint32_t i = 0; i < 100000; i++)
			;
	}
}

void GPIO_Clock_Enable(void)
{
	SET_BIT(RCC->AHB1ENR, 0);
}

void GPIO_Pin_Init(void)
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
