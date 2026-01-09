#include <stdint.h>

#define RCC_BASE 0x40023800
#define RCC_AHB1ENR (RCC_BASE + 0x30)

#define GPIOA_BASE 0x40020000
#define GPIOA_MODER (GPIOA_BASE + 0x00)
#define GPIOA_ODR (GPIOA_BASE + 0x14)

#define SET_BIT(REG, BIT) ((REG) |= (1U << (BIT)))
#define CLEAR_BIT(REG, BIT) ((REG) &= ~(1U << (BIT)))

int main(void)
{
	// Enabling GPIOA Clock ( RCC )
	volatile uint32_t* const gpioa_clock =
	    (volatile uint32_t*)(RCC_AHB1ENR);
	*gpioa_clock |= (1u << 0);

	// Configuring PA5 as OUTPUT
	volatile uint32_t* const gpioa_moder =
	    (volatile uint32_t*)(GPIOA_MODER);
	*gpioa_moder &= ~(3u << 10);
	*gpioa_moder |= (1u << 10);

	// Flipping PA5 value
	volatile uint32_t* const gpioa_odr = (volatile uint32_t*)(GPIOA_ODR);
	while (1) {
		// Sets PA5 to ON
		*gpioa_odr |= (1u << 5);
		for (volatile uint32_t i = 0; i < 1000000; i++) {
		}
		// Sets PA5 to OFF
		*gpioa_odr &= ~(1u << 5);
		for (volatile uint32_t i = 0; i < 1000000; i++) {
		}
	}
}
