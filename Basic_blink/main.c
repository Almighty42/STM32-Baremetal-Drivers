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

void SysTick_Init(uint32_t ticks)
{
	// Disable SysTick IRQ and SysTick timer
	SysTick->CTRL = 0;

	// Set reload register
	SysTick->LOAD = ticks - 1;

	// Set interrupt priority of SysTick
	// Make SysTick less urgent (highest priority number)
	// __NVIC PRIO_BITS: number of bits for priority levels, defined in
	// CMSIS NVIC_SetPriority (SysTick_IRQn, (1 << __NVIC_PRIO_BITS) - 1);

	// Reset the SysTick counter value
	SysTick->VAL = 0;

	// Select processor clock
	// 1 = processor clock, 0 = external clock
	SET_BIT(SysTick->CTRL, 2);

	// Enables SysTick exception request
	// 1 = counting down to zero asserts the SysTick exception request
	// 0 = counting down to zero does NOT assert the SysTick exception
	// request
	SET_BIT(SysTick->CTRL, 1);

	// Enable the SysTick timer
	SET_BIT(SysTick->CTRL, 0);
}

void SysTick_Handler(void)
{
	// Time delay is a global variable declared as volatile
	if (TimeDelay > 0) // Prevent it from being negative
		TimeDelay--;
}

void Delay(uint32_t nTime)
{
	// nTime - specifies the delay time length
	TimeDelay = nTime; // Time delay must be declared as volatile
	while (TimeDelay != 0)
		; // Busy wait
}
