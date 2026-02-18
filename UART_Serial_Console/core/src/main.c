// Core
#include "stm32f401xx.h"
// Peripheral drivers
#include "stm32f401xx_GPIO_driver.h"
#include "stm32f401xx_USART_driver.h"
// Stdlib
#include <stdint.h>

#define LINE_BUFFER_SIZE 80

// USART2 handle
USART_Handle_t usart2;

static void USART2_init(void);
static void USART_write(char str[]);

// Helper functions
static uint32_t str_len(const uint8_t* str);

int main(void)
{
	USART2_init();

	// USART_write("\r\n");
	// USART_write("========================\r\n");
	// USART_write("STM32F401RE UART Console\r\n");
	// USART_write("========================\r\n");
	// USART_write("Type 'help' for available commands\r\n");
	// USART_write("> ");

	// USART_receive_data_it(&usart2, rx_buff, 1);

	while (1) {
	}
}

void USART2_init(void)
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

	usart2.p_USARTx = USART2;
	usart2.USART_Pin_Config.USART_baud = USART_STD_BAUD_115200;
	usart2.USART_Pin_Config.USART_hw_flow_control = USART_HW_FLOW_CTRL_NONE;
	usart2.USART_Pin_Config.USART_mode = USART_MODE_TXRX;
	usart2.USART_Pin_Config.USART_n_stop_bits = USART_STOPBITS_1;
	usart2.USART_Pin_Config.USART_word_len = USART_WORDLEN_8BITS;
	usart2.USART_Pin_Config.USART_parity_control = USART_PARITY_DISABLE;

	USART_init(&usart2);

	USART_irq_interrupt_config(IRQ_NO_USART2, ENABLE);

	USART_peri_control(USART2, ENABLE);
}

static void USART_write(char str[])
{
	// TODO:
	// THIS FUNCTION WRITES TO TX BUFFER
	// THEN THE TX-EMPTY INTERRUPT PULLS FROM THE BUFFER AND FEEDS USART
	// UNTIL EMPTY
}

static uint32_t str_len(const uint8_t* str)
{
	uint32_t len = 0;
	while (*str++ != '\0')
		len++;
	return len;
}

void USART2_IRQHandler(void)
{
}

void USART_application_event_callback(USART_Handle_t* p_USART_handle,
                                      USART_AppEvent_t app_ev)
{
}
