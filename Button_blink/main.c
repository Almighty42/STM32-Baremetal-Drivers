#include "stm32f401RE_header.h"
#include <stdint.h>

#define DEBOUNCE_MS 100U

volatile uint32_t g_ms_ticks = 0;

void GPIO_Clock_Enable(void);
void GPIO_Pin_Init_LED(void);
void GPIO_Pin_Init_Button(void);
void EXTI_Init(void);
void SysTick_Init(uint32_t ticks);

int main(void)
{
	// Enables RCC clock for GPIO_A
	GPIO_Clock_Enable();

	// Prepares the GPIO_A for digital output and sets other settings
	GPIO_Pin_Init_LED();
	GPIO_Pin_Init_Button();

	SysTick_Init(16000000U);
	EXTI_Init();

	while (1) {
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
	CLEAR_FIELD_2BIT(GPIOC->PUPDR, 26);
}

void EXTI_Init(void)
{
	// Enable SYSCFG clock
	SET_BIT(RCC->APB2ENR, 14);

	// Select PC13 as trigger source for EXTI 13
	CLEAR_FIELD_4BIT(SYSCFG->EXTICR4, 4);
	SET_BIT(SYSCFG->EXTICR4, 5);

	// Enable rising edge trigger for EXTI 13
	SET_BIT(EXTI->RTSR, 13);

	// Disable falling edge trigger for EXTI 13
	CLEAR_BIT(EXTI->FTSR, 13);

	// Enable EXTI 13 interrupt
	SET_BIT(EXTI->IMR, 13);

	// Set EXTI 13 priority to 1
	NVIC_Set_IRQ_Priority(10, 1);

	// Enable EXTI 13 interrupt in NVIC
	NVIC_Enable_IRQ(10);
}

void SysTick_Init(uint32_t ticks)
{
	uint32_t reload = ticks / 1000U - 1U;
	SysTick->LOAD = reload;
	SysTick->VAL = 0;
	SET_BIT(SysTick->CTRL, 2);
	SET_BIT(SysTick->CTRL, 1);
	SET_BIT(SysTick->CTRL, 0);
}

void EXTI15_10_IRQHandler(void)
{
	static uint32_t last_event_ms = 0;

	// Check for EXTI 3 interrupt flag
	if (IS_BIT_SET(EXTI->PR, 13)) {
		uint32_t now = g_ms_ticks;
		if ((now - last_event_ms) >= DEBOUNCE_MS) {
			last_event_ms = now;
			// Toggle LED
			if (IS_BIT_SET(GPIOA->ODR, 5))
				CLEAR_BIT(GPIOA->ODR, 5);
			else
				SET_BIT(GPIOA->ODR, 5);
		}
		// Clear interrupt pending request
		SET_BIT(EXTI->PR, 13);
	}
}

void SysTick_Handler(void)
{
	g_ms_ticks++;
}
