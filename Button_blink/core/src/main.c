#include "stm32f401xx.h"
#include "stm32f401xx_GPIO_driver.h"

int main(void)
{
	GPIO_Handle_t gpio_led = {
	    .p_GPIOx = GPIOA,
	    .GPIO_Pin_Config.GPIO_Pin_Number = GPIO_PIN_NO_5,
	    .GPIO_Pin_Config.GPIO_Pin_Mode = GPIO_MODE_OUT,
	    .GPIO_Pin_Config.GPIO_Pin_Speed = GPIO_SPEED_FAST,
	    .GPIO_Pin_Config.GPIO_Pin_OTP_Type = GPIO_OP_TYPE_PP,
	    .GPIO_Pin_Config.GPIO_Pin_PuPd_Control = GPIO_NO_PUPD};

	GPIO_peri_clk_control(GPIOA, ENABLE);
	GPIO_init(&gpio_led);

	GPIO_Handle_t gpio_btn = {
	    .p_GPIOx = GPIOC,
	    .GPIO_Pin_Config.GPIO_Pin_Number = GPIO_PIN_NO_13,
	    .GPIO_Pin_Config.GPIO_Pin_Mode = GPIO_MODE_IT_FT,
	    .GPIO_Pin_Config.GPIO_Pin_Speed = GPIO_SPEED_FAST,
	    .GPIO_Pin_Config.GPIO_Pin_PuPd_Control = GPIO_PIN_PU};

	GPIO_peri_clk_control(GPIOC, ENABLE);
	GPIO_init(&gpio_btn);

	GPIO_irq_priority_config(IRQ_NO_EXTI15_10, NVIC_IRQ_PRIO_15);
	GPIO_irq_interrupt_config(IRQ_NO_EXTI15_10, ENABLE);

	while (1) {
	}
}

void EXTI15_10_IRQHandler(void)
{
	GPIO_irq_handling(GPIO_PIN_NO_13);
	GPIO_toggle_output_pin(GPIOA, GPIO_PIN_NO_5);
}
