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

void USART2_init(void);
static void USART_write(const char* str);
static uint32_t str_len(const uint8_t* str);
static void handle_command(char line_buff[]);

// FIX: TEMPORARY
uint8_t rx_buff[1024];

char line_buff[LINE_BUFFER_SIZE];
uint8_t line_buff_len = 0;

int main(void)
{
	USART2_init();

	USART_write("\r\n");
	USART_write("========================\r\n");
	USART_write("STM32F401RE UART Console\r\n");
	USART_write("========================\r\n");
	USART_write("Type 'help' for available commands\r\n");
	USART_write("> ");

	USART_receive_data_it(&usart2, rx_buff, 1);

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

static void USART_write(const char* str)
{
	uint32_t len = str_len((const uint8_t*)str);
	USART_status_t status;

	do {
		status = USART_send_data_it(&usart2, (uint8_t*)str, len);
	} while (status == USART_BUSY_IN_TX);
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
	USART_irq_handling(&usart2);
}

void USART_application_event_callback(USART_Handle_t* p_USART_handle,
                                      USART_AppEvent_t app_ev)
{
	if (app_ev == USART_EVENT_RX_CMPLT) {
		uint8_t ch = rx_buff[0];

		// Echo back
		if (ch == '\r') {
			// EOF
			if (line_buff_len < LINE_BUFFER_SIZE)
				line_buff[line_buff_len] = '\0';
			else
				line_buff[line_buff_len - 1] = '\0';

			// const char c1[] = "\r\n";
			// USART_send_data_it(p_USART_handle, (uint8_t*)c1, 2);
			USART_write("\r\n");

			handle_command(line_buff);

			line_buff_len = 0;
			USART_write("> ");
			// const char c2[] = "> ";
			// USART_send_data_it(p_USART_handle, (uint8_t*)c2, 2);
		}
		else if (ch == '\b') {
			if (line_buff_len > 0) {
				line_buff_len--;
				// TODO: CHECK THIS
				USART_write("\b \b");
			}
		}
		else {
			if (line_buff_len < LINE_BUFFER_SIZE - 1) {
				// TODO: HANDLE BUFFER CLEARING
				line_buff[line_buff_len++] = (char)ch;
				USART_send_data_it(p_USART_handle, &ch, 1);
			}
		}

		// Arm next RX
		USART_receive_data_it(p_USART_handle, rx_buff, 1);
	}
}

static void handle_command(char line_buff[])
{
	// if (line_buff[0] == '\0')
	// 	return;
	// else
	// 	USART_write("Handling done!");
}
