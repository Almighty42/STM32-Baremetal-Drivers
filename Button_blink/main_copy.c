// #include "stm32f401RE_header.h"
#include <stdint.h>

#define RCC_BASE (0x40023800UL)
#define RCC_AHB1ENR (0x40023830UL)
#define RCC_APB2ENR (0x40023844UL)

#define GPIOA_BASE    0x40020000UL
#define GPIOA_MODER   (GPIOA_BASE + 0x00)
#define GPIOA_OTYPER  (GPIOA_BASE + 0x04)
#define GPIOA_OSPEEDR (GPIOA_BASE + 0x08)
#define GPIOA_PUPDR   (GPIOA_BASE + 0x0C)
#define GPIOA_ODR     (GPIOA_BASE + 0x14)

#define GPIOC_BASE    0x40020800UL
#define GPIOC_MODER   (GPIOC_BASE + 0x00)
#define GPIOC_OTYPER  (GPIOC_BASE + 0x04)
#define GPIOC_OSPEEDR (GPIOC_BASE + 0x08)
#define GPIOC_PUPDR   (GPIOC_BASE + 0x0C)
#define GPIOC_ODR     (GPIOC_BASE + 0x14)

#define SYSCFG_BASE (0x40013800)
#define SYSCFG_EXTICR4 (0x40013814)

#define EXTI_BASE (0x40013C00)
#define EXTI_RTSR (0x40013C08)
#define EXTI_FTSR (0x40013C0C)
#define EXTI_IMR (0x40013C00)
#define EXTI_PR (EXTI_BASE + 0x14)

#define NVIC_BASE (0xE000E100)
#define NVIC_IPR_10 (NVIC_BASE + 0x400 + 0x4 * 10)
#define NVIC_ISER_1 (NVIC_BASE + 0x100 + 0x4 * 1)

uint32_t state = 1;


void GPIO_Clock_Enable(void);
void GPIO_Pin_Init_LED(void);
void GPIO_Pin_Init_Button(void);
void EXTI_Init(void);

void led_on(void);
void led_off(void);

 void NVIC_Set_IRQ_Priority(void);
 void NVIC_Enable_IRQ(void);

int main(void)
{
	// Enables RCC clock for GPIO_A
	GPIO_Clock_Enable();


	// Prepares the GPIO_A for digital output and sets other settings
	GPIO_Pin_Init_LED();
	GPIO_Pin_Init_Button();
	
	led_on();

	EXTI_Init();

	while (1) {
	}
}

void GPIO_Clock_Enable(void)
{
	state++;
	volatile uint32_t* rcc_ahb1enr = (volatile uint32_t*)RCC_AHB1ENR;
	*rcc_ahb1enr |= (1UL << 0);
	*rcc_ahb1enr |= (1UL << 2);
	// SET_BIT(RCC->AHB1ENR, 0);
	// SET_BIT(RCC->AHB1ENR, 2);
}

void GPIO_Pin_Init_LED(void)
{
	state++;
	volatile uint32_t* gpioa_moder = (volatile uint32_t*)GPIOA_MODER;
	volatile uint32_t* gpioa_otyper = (volatile uint32_t*)GPIOA_OTYPER;
	volatile uint32_t* gpioa_ospeedr = (volatile uint32_t*)GPIOA_OSPEEDR;
	volatile uint32_t* gpioa_pupdr = (volatile uint32_t*)GPIOA_PUPDR;

	// Set mode as digital output
	*gpioa_moder &= ~(1UL << 10);
	*gpioa_moder &= ~(1UL << 11);
	// CLEAR_FIELD_2BIT(GPIOA->MODER, 10);
	*gpioa_moder |= (1UL << 10);
	// SET_BIT(GPIOA->MODER, 10);

	// Set output type as push-pull
	*gpioa_otyper &= ~(1UL << 5);
	// CLEAR_BIT(GPIOA->OTYPER, 5)

	// Set output speed as low
	// GPIOB->OSPEEDR &= ~(3UL << 4);
	*gpioa_ospeedr &= ~(1UL << 10);
	*gpioa_ospeedr &= ~(1UL << 11);
	// CLEAR_FIELD_2BIT(GPIOA->OSPEEDR, 10);

	// Set no pull up, no pull down
	*gpioa_pupdr &= ~(1UL << 10);
	*gpioa_pupdr &= ~(1UL << 11);
	// CLEAR_FIELD_2BIT(GPIOA->PUPDR, 10);
}

void GPIO_Pin_Init_Button(void)
{
	state++;
    volatile uint32_t* gpioc_moder   = (volatile uint32_t*)GPIOC_MODER;
    volatile uint32_t* gpioc_otyper  = (volatile uint32_t*)GPIOC_OTYPER;
    volatile uint32_t* gpioc_ospeedr = (volatile uint32_t*)GPIOC_OSPEEDR;
    volatile uint32_t* gpioc_pupdr   = (volatile uint32_t*)GPIOC_PUPDR;

    // Set mode as digital input
    *gpioc_moder   &= ~(3UL << 26);  // PC13 input

    // Optional but harmless for input
    *gpioc_otyper  &= ~(1UL << 13);  // push-pull
    *gpioc_ospeedr &= ~(3UL << 26);  // low speed

    // Enable pull-up on PC13
    *gpioc_pupdr   &= ~(3UL << 26);
    *gpioc_pupdr   |=  (1UL << 26);  // pull-up
}

 void NVIC_Set_IRQ_Priority(void) {
	state++;
	volatile uint32_t* nvic_ipr = (volatile uint32_t*)NVIC_IPR_10;
	*nvic_ipr &= ~(0x000000FFUL);
	*nvic_ipr |= (1UL << 4);
}

 void NVIC_Enable_IRQ(void) {
	state++;
	volatile uint32_t* nvic_iser = (volatile uint32_t*)NVIC_ISER_1;
	*nvic_iser |= (1UL << 8);
}

void EXTI_Init(void)
{
	state++;
	volatile uint32_t* rcc_apb2enr = (volatile uint32_t*)RCC_APB2ENR;
	volatile uint32_t* syscfg_exticr4 = (volatile uint32_t*)SYSCFG_EXTICR4;
	volatile uint32_t* exti_rtsr = (volatile uint32_t*)EXTI_RTSR;
	volatile uint32_t* exti_ftsr = (volatile uint32_t*)EXTI_FTSR;
	volatile uint32_t* exti_imr = (volatile uint32_t*)EXTI_IMR;

	// Enable SYSCFG clock
	*rcc_apb2enr |= (1UL << 14);
	
	// Select PC13 as trigger for EXTI13
	*syscfg_exticr4 &= ~(15UL << 4);
	*syscfg_exticr4 |= (1UL << 5);
	
	// Enable rising edge trigger for EXTI13
	// *exti_rtsr |= (1UL << 13);
	*exti_rtsr &= ~(1UL << 13);
	
	// Disable falling edge trigger for EXTI13
	// *exti_ftsr &= ~(1UL << 13);
	*exti_ftsr |= (1UL << 13);
	
	// Enable EXTI13 interrupt
	*exti_imr |= (1UL << 13);
	
	// Set EXTI13 priority to 1
	NVIC_Set_IRQ_Priority();
	
	// Enable EXTI15 interrupt
	NVIC_Enable_IRQ();
}

void EXTI15_10_IRQHandler(void)
{
	state++;
	volatile uint32_t *exti_pr = (volatile uint32_t*)EXTI_PR;
	if (*exti_pr & (1U << 13)) {
		*exti_pr = (1U << 13);
		led_on();
	}
}

void led_on(void) {

	state++;
	// SET_BIT(GPIOA->ODR, 5);
	volatile uint32_t* gpioa_odr = (volatile uint32_t*)GPIOA_ODR;
	*gpioa_odr |= (1UL << 5);
}

void led_off(void) {
	state++;
	// CLEAR_BIT(GPIOA->ODR, 5);
	volatile uint32_t* gpioa_odr = (volatile uint32_t*)GPIOA_ODR;
	*gpioa_odr &= ~(1UL << 5);
}

