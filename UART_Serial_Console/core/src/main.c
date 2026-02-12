// Core
#include "stm32f401xx.h"
// Peripheral drivers
#include "stm32f401xx_GPIO_driver.h"
#include "stm32f401xx_USART_driver.h"
// Stdlib
#include <stdint.h>

// Transmitter buffer
const uint8_t* msg[] = {(const uint8_t*)"Ex 1\r\n", (const uint8_t*)"Ex 2\r\n",
                        (const uint8_t*)"Ex 3\r\n"};
uint8_t msg_num = 0;

// Receiver buffer
char rx_buffer[1024];

// USART2 handle
USART_Handle_t usart2;

void GPIO_btn_init(void);
void USART2_GPIO_init(void);
void USART2_init(void);
uint32_t strlen(const char* str);

int main(void)
{
	GPIO_btn_init();

	USART2_GPIO_init();

	USART2_init();

	USART_irq_interrupt_config(IRQ_NO_USART2, ENABLE);

	USART_peri_control(USART2, ENABLE);

	while (1) {
	}
}

void GPIO_btn_init(void)
{
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
}

void USART2_GPIO_init(void)
{
	GPIO_peri_clk_control(GPIOA, ENABLE);

	GPIO_Handle_t p_usart2_gpio = {
	    .p_GPIOx = GPIOA,
	    .GPIO_Pin_Config.GPIO_Pin_Mode = GPIO_MODE_ALT,
	    .GPIO_Pin_Config.GPIO_Pin_OTP_Type = GPIO_OP_TYPE_PP,
	    .GPIO_Pin_Config.GPIO_Pin_PuPd_Control = GPIO_PIN_PU,
	    .GPIO_Pin_Config.GPIO_Pin_Alt_Fun_Mode = GPIO_ALT_MODE_AF7,
	    .GPIO_Pin_Config.GPIO_Pin_Speed = GPIO_SPEED_FAST,
	};

	// Tx
	p_usart2_gpio.GPIO_Pin_Config.GPIO_Pin_Number = GPIO_PIN_NO_2;
	GPIO_init(&p_usart2_gpio);

	// Rx
	p_usart2_gpio.GPIO_Pin_Config.GPIO_Pin_Number = GPIO_PIN_NO_3;
	GPIO_init(&p_usart2_gpio);
}

void USART2_init(void)
{
	usart2.p_USARTx = USART2;
	usart2.USART_Pin_Config.USART_baud = USART_STD_BAUD_115200;
	usart2.USART_Pin_Config.USART_hw_flow_control = USART_HW_FLOW_CTRL_NONE;
	usart2.USART_Pin_Config.USART_mode = USART_MODE_TXRX;
	usart2.USART_Pin_Config.USART_n_stop_bits = USART_STOPBITS_1;
	usart2.USART_Pin_Config.USART_word_len = USART_WORDLEN_8BITS;
	usart2.USART_Pin_Config.USART_parity_control = USART_PARITY_DISABLE;

	USART_init(&usart2);
}

uint32_t strlen(const char* str)
{
	uint32_t len = 0;
	while (*str++ != '\0')
		len++;
	return len;
}

void EXTI15_10_IRQHandler(void)
{
	GPIO_irq_handling(GPIO_PIN_NO_13);

	USART_send_data(&usart2, msg[msg_num], strlen((char*)msg[msg_num]));
	msg_num++;
}
