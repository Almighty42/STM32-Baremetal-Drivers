#include "stm32f401RE_header.h"
#include <stdint.h>

#define DELAY_MS 500

volatile uint32_t time_delay = 0;

void GPIO_Clock_Enable(void);
void GPIO_Pin_Init(void);
void SysTick_Init(uint32_t ticks);
void Delay(uint32_t time);

int main(void)
{
	// Enables RCC clock for GPIO_A
	GPIO_Clock_Enable();

	// Prepares the GPIO_A for digital output and sets other settings
	GPIO_Pin_Init();

	// Initialize SysTick for 16MHz
	SysTick_Init(16000000U);

	while (1) {
		// LED ON
		SET_BIT(GPIOA->ODR, 5);
		Delay(DELAY_MS);
		// LED OFF
		CLEAR_BIT(GPIOA->ODR, 5);
		Delay(DELAY_MS);
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
	// Disable SysTick IRQ and SysTick counter
	SysTick->CTRL = 0;
	// Set reload register
	uint32_t reload = ticks / 1000U - 1U;
	SysTick->LOAD = reload;
	// Set interrupt priority of SysTick
	NVIC_Set_IRQ_Priority(SysTick_IRQn, (1 << __NVIC_PRIO_BITS) - 1);
	// Reset the SysTick counter value
	SysTick->VAL = 0;
	// Processor clock
	SET_BIT(SysTick->CTRL, 2);
	// Enables SysTick exception request
	SET_BIT(SysTick->CTRL, 1);
	// Enables SysTick timer
	SET_BIT(SysTick->CTRL, 0);
}

void Delay(uint32_t time)
{
	time_delay = time;
	while (time_delay != 0)
		;
}

void SysTick_Handler(void)
{
	if (time_delay > 0)
		time_delay--;
}
