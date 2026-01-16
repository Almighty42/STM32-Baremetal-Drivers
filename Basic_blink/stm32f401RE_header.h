#ifndef STM32F401RE_HEADER
#define STM32F401RE_HEADER

#include <stdint.h>

#define __I volatile const // Read only
#define __O volatile       // Write only
#define __IO volatile      // Read / Write

// INFO: SysTick

typedef struct
{
	__IO uint32_t CTRL;
	__IO uint32_t LOAD;
	__IO uint32_t VAL;
	__IO uint32_t CALIB;
} SysTick_Type;

#define SysTick ((SysTick_Type*)0xE000E010)

// INFO: GPIO

typedef struct
{
	__IO uint32_t MODER;
	__IO uint32_t OTYPER;
	__IO uint32_t OSPEEDR;
	__IO uint32_t PUPDR;
	__IO uint32_t IDR;
	__IO uint32_t ODR;
	__IO uint32_t BSRR;
	__IO uint32_t LCKR;
	__IO uint32_t AFRL;
	__IO uint32_t AFRH;
} GPIO_TypeDef;

#define GPIOA ((GPIO_TypeDef*)0x40020000)
#define GPIOB ((GPIO_TypeDef*)0x40020400)
#define GPIOC ((GPIO_TypeDef*)0x40020800)
#define GPIOD ((GPIO_TypeDef*)0x40020C00)
#define GPIOE ((GPIO_TypeDef*)0x40021000)
#define GPIOH ((GPIO_TypeDef*)0x40021C00)

// INFO: RCC

typedef struct
{
	__IO uint32_t CR;
	__IO uint32_t PLLCFGR;
	__IO uint32_t CFGR;
	__IO uint32_t CIR;
	__IO uint32_t AHB1RSTR;
	__IO uint32_t AHB2RSTR;
	uint32_t res0;
	uint32_t res1;
	__IO uint32_t APB1RSTR;
	__IO uint32_t APB2RSTR;
	uint32_t res2;
	uint32_t res3;
	__IO uint32_t AHB1ENR;
	__IO uint32_t AHB2ENR;
	uint32_t res4;
	uint32_t res5;
	__IO uint32_t APB1ENR;
	__IO uint32_t APB2ENR;
	uint32_t res6;
	uint32_t res7;
	__IO uint32_t AHB1LPENR;
	__IO uint32_t AHB2LPENR;
	uint32_t res8;
	uint32_t res9;
	__IO uint32_t APB1LPENR;
	__IO uint32_t APB2LPENR;
	uint32_t res10;
	uint32_t res11;
	__IO uint32_t BDCR;
	__IO uint32_t CSR;
	uint32_t res12;
	uint32_t res13;
	__IO uint32_t SSCGR;
	__IO uint32_t PLLI2SCFGR;
	__IO uint32_t DCKCFGR;
} RCC_TypeDef;
#define RCC ((RCC_TypeDef*)0x40023800)

// INFO: NVIC

typedef struct
{
	uint32_t res0;
	__I uint32_t ICTR;
	uint32_t res1[28];
	__IO uint32_t ISER[8];
	uint32_t res2[24];
	__IO uint32_t ICER[8];
	uint32_t res3[24];
	__IO uint32_t ISPR[8];
	uint32_t res4[24];
	__IO uint32_t ICPR[8];
	uint32_t res5[24];
	__I uint32_t IABR[8];
	uint32_t res6[36];
	__IO uint32_t IPR[60];
	uint32_t res7[824];
	__O uint32_t STIR;

} NVIC_Type;
#define NVIC ((NVIC_Type*)0xE000E000);
#define __NVIC_PRIO_BITS 4

static inline void NVIC_Set_IRQ_Priority(uint32_t IRQn, uint32_t priority)
{
	uint32_t IPRn = (uint32_t)IRQn >> 2;          // IRQn / 4
	uint32_t shift = ((uint32_t)IRQn & 0x3U) * 8; // (IRQn % 4) * 8
	uint32_t priority_field = (priority & ((1U << __NVIC_PRIO_BITS) - 1U))
	                          << (8U - __NVIC_PRIO_BITS);
	uint32_t reg = NVIC->IPR[IPRn];
}

typedef struct
{
	__I uint32_t CPUID;
	__IO uint32_t ICSR;
	__IO uint32_t VTOR;
	__IO uint32_t AIRCR;
	__IO uint32_t SCR;
	__IO uint32_t CCR;
	__IO uint32_t SHPR1;
	__IO uint32_t SHPR2;
	__IO uint32_t SHPR3;
	__IO uint32_t SHCSR;
	__IO uint32_t CFSR;
	__IO uint32_t HFSR;
	uint32_t res0;
	__IO uint32_t MMAR;
	__IO uint32_t BFAR;
	__IO uint32_t AFSR;

} SCB_Type;
#define SCB ((SCB_Type*)0xE000ED00);

#define SET_BIT(reg, bit) ((reg) |= (1UL << (bit)))
#define CLEAR_BIT(reg, bit) ((reg) &= ~(1UL << (bit)))
#define TOGGLE_BIT(reg, bit) ((reg) ^= (1UL << (bit)))
#define READ_BIT(reg, bit) ((reg) & (1UL << (bit)))
#define IS_BIT_SET(reg, bit) (READ_BIT(reg, bit) != 0UL)
#define CLEAR_FIELD_2BIT(reg, pos) ((reg) &= ~(3U << (pos)))

#endif
