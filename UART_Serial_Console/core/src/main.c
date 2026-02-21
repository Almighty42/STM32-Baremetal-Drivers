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
static void process_line(char* line);

// Helper functions
static uint32_t str_cmp(const char* s1, const char* s2);
static uint32_t str_len(const uint8_t* str);
static void USART_write(char str[]);

static char line_buffer[LINE_BUFFER_SIZE];
static uint32_t line_len = 0;

int main(void)
{
	USART2_init();

	USART_write("\r\n");
	USART_write("========================\r\n");
	USART_write("STM32F401RE UART Console\r\n");
	USART_write("========================\r\n");
	USART_write("Type 'help' for available commands\r\n");
	USART_write("> ");

	uint8_t ch;

	while (1) {
		if (USART_read_byte(&usart2, &ch)) {
			// Backspace handling
			if (ch == '\b' || ch == 0x7F) {
				if (line_len > 0) {
					line_len--;
					USART_write("\b \b");
				}
				continue;
			}

			USART_write_byte(&usart2, &ch, 1);

			if (ch == '\r' || ch == '\n') {
				USART_write("\r\n");
				if (line_len < LINE_BUFFER_SIZE)
					line_buffer[line_len] = '\0';
				else
					line_buffer[LINE_BUFFER_SIZE - 1] =
					    '\0';
				process_line(line_buffer);
				line_len = 0;
				USART_write("> ");
			}
			else {
				if (line_len < LINE_BUFFER_SIZE - 1)
					line_buffer[line_len++] = (char)ch;
				else {
					// Line overflow
				}
			}
		}
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

static void process_line(char* line)
{
	while (*line == ' ' || *line == '\t')
		line++;
	if (*line == '\0')
		return;
	if (str_cmp(line, "help") == 0) {
		USART_write("Available commands:\r\n");
		USART_write(" led on\t- Turn LED on\r\n");
		USART_write(" led off\t- Turn LED off\r\n");
		USART_write(" led toggle\t- Toggle LED state\r\n");
		USART_write(" status\t- Show system status\r\n");
		USART_write(" help\t- Show this help\r\n");
	}
	else {
		USART_write("ERROR: Unknown command\r\n");
		USART_write("Type 'help' for available commands\r\n");
	}
}

static uint32_t str_len(const uint8_t* str)
{
	uint32_t len = 0;
	while (*str++ != '\0')
		len++;
	return len;
}

static uint32_t str_cmp(const char* s1, const char* s2)
{
	while (*s1 && (*s1 == *s2)) {
		s1++;
		s2++;
	}
	return (uint32_t)((unsigned char)*s1) - (uint32_t)((unsigned char)*s2);
}

static void USART_write(char str[])
{
	uint32_t len = str_len((uint8_t*)str);
	uint32_t sent = 0;

	while (sent < len) {
		sent += USART_write_byte(&usart2, (const uint8_t*)&str[sent],
		                         len - sent);
	}
}

void USART2_IRQHandler(void)
{
	USART_irq_handling(&usart2);
}
