#include "utils.h"
#include "stm32f401xx.h"
#include "stm32f401xx_GPIO_driver.h"

_Bool led_init(void)
{
	GPIO_Handle_t gpio_led = {
	    .p_GPIOx = GPIOA,
	    .GPIO_Pin_Config.GPIO_Pin_Number = GPIO_PIN_NO_5,
	    .GPIO_Pin_Config.GPIO_Pin_Mode = GPIO_MODE_OUT,
	    .GPIO_Pin_Config.GPIO_Pin_Speed = GPIO_SPEED_FAST,
	    .GPIO_Pin_Config.GPIO_Pin_OTP_Type = GPIO_OP_TYPE_PP,
	    .GPIO_Pin_Config.GPIO_Pin_PuPd_Control = GPIO_NO_PUPD};

	GPIO_status_t status_1 = GPIO_peri_clk_control(GPIOA, ENABLE);

	GPIO_status_t status_2 = GPIO_init(&gpio_led);

	if (status_1 == GPIO_OK && status_2 == GPIO_OK)
		return TRUE;
	else
		return FALSE;
}

void led_on(void)
{
	if (!IS_BIT_SET(GPIOA->ODR, GPIO_PIN_NO_5))
		GPIO_toggle_output_pin(GPIOA, GPIO_PIN_NO_5);
}

void led_off(void)
{
	if (IS_BIT_SET(GPIOA->ODR, GPIO_PIN_NO_5))
		GPIO_toggle_output_pin(GPIOA, GPIO_PIN_NO_5);
}

void led_toggle(void)
{
	GPIO_toggle_output_pin(GPIOA, GPIO_PIN_NO_5);
}
