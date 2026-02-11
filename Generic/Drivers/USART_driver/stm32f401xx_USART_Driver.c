#include "stm32f401xx_USART_Driver.h"
#include "stm32f401xx.h"
#include "stm32f401xx_RCC_driver.h"
#include <stdint.h>

#define POLLING_TIMEOUT 100000

/********************************************************************************
 *
 * TODO: Future plans for this driver:
 * 1. Implement DMA USART in future
 *
 *******************************************************************************/

/********************************************************************************
 * @fn				- USART_peri_clk_control
 *
 * @brief			- Enables or disables peripheral clock for USART
 *
 * @param[*p_USART_x]		- Base address of the USART peripheral
 * @param[EN_or_DI]		- ENABLE or DISABLE macros
 *
 * @return			- Success / Failure status of the function
 *
 * @Note			- None
 *******************************************************************************/

USART_status_t USART_peri_clk_control(USART_TypeDef* p_USART_x,
                                      uint8_t EN_or_DI)
{
	VALIDATE_EN_DI(EN_or_DI, USART_ERROR_INVALID_STATE);

	USART_status_t status = USART_ERROR_INVALID_PORT;

	if (p_USART_x == USART1) {
		if (EN_or_DI == ENABLE) {
			USART1_PCLK_EN();
		}
		else
			USART1_PCLK_DI();
		status = USART_OK;
	}
	else if (p_USART_x == USART2) {
		if (EN_or_DI == ENABLE) {
			USART2_PCLK_EN();
		}
		else
			USART2_PCLK_DI();
		status = USART_OK;
	}
	else if (p_USART_x == USART6) {
		if (EN_or_DI == ENABLE) {
			USART6_PCLK_EN();
		}
		else
			USART6_PCLK_DI();
		status = USART_OK;
	}

	return status;
}

/********************************************************************************
 * @fn				- USART_init
 *
 * @brief			- Initializes a USART peripheral
 *
 * @param[*p_USART_handle]	- Handle structure of a USART peripheral
 *
 * @return			- Success / Failure status of the function
 *
 * @Note			- None
 *******************************************************************************/

USART_status_t USART_init(USART_Handle_t* p_USART_handle)
{
	VALIDATE_PTR(p_USART_handle, USART_ERROR_NULL_PTR);

	USART_TypeDef* port = p_USART_handle->p_USARTx;
	VALIDATE_USART_PORT(port);

	volatile uint32_t* cr1 = &p_USART_handle->p_USARTx->CR1;
	volatile uint32_t* cr2 = &p_USART_handle->p_USARTx->CR2;
	volatile uint32_t* cr3 = &p_USART_handle->p_USARTx->CR3;

	// Clock
	USART_peri_clk_control(port, ENABLE);

	// Check if USART is disabled before configuring
	CLEAR_BIT(*cr1, USART_CR1_UE);

	// Mode
	uint8_t mode = p_USART_handle->USART_Pin_Config.USART_mode;

	VALIDATE_USART_MODE(mode);

	CLEAR_FIELD_2BIT(*cr1, 2);
	if (mode == USART_MODE_ONLY_RX) {
		SET_BIT(*cr1, 2);
	}
	else if (mode == USART_MODE_ONLY_TX) {
		SET_BIT(*cr1, 3);
	}
	else if (mode == USART_MODE_TXRX) {
		SET_BIT(*cr1, 2);
		SET_BIT(*cr1, 3);
	}

	// Baud rate
	USART_set_baud_rate(p_USART_handle->p_USARTx,
	                    p_USART_handle->USART_Pin_Config.USART_baud);

	// Number of stop bits
	uint8_t stop_bits = p_USART_handle->USART_Pin_Config.USART_n_stop_bits;

	VALIDATE_USART_STOP_BITS(stop_bits);

	CLEAR_FIELD_2BIT(*cr2, 12);
	if (stop_bits == USART_STOPBITS_0_5) {
		SET_BIT(*cr2, 12);
	}
	else if (stop_bits == USART_STOPBITS_1) {
		// Already cleared bits for USART_STOPBITS_1
	}
	else if (stop_bits == USART_STOPBITS_1_5) {
		SET_BIT(*cr2, 12);
		SET_BIT(*cr2, 13);
	}
	else if (stop_bits == USART_STOPBITS_2) {
		SET_BIT(*cr2, 13);
	}
	// Word length
	uint8_t word_len = p_USART_handle->USART_Pin_Config.USART_word_len;

	VALIDATE_USART_WORD_LEN(word_len);

	CLEAR_BIT(*cr1, 12);
	if (word_len == USART_WORDLEN_8BITS) {
		// Already cleared bits for USART_STOPBITS_1
	}
	else if (word_len == USART_WORDLEN_9BITS) {
		SET_BIT(*cr1, 12);
	}

	// Parity control
	uint8_t parity_control =
	    p_USART_handle->USART_Pin_Config.USART_parity_control;

	VALIDATE_USART_PARITY(parity_control);

	CLEAR_BIT(*cr1, 10);
	if (parity_control == USART_PARITY_DISABLE) {
		// Already cleared bits for USART_STOPBITS_1
	}
	else if (parity_control == USART_PARITY_EN_EVEN) {
		SET_BIT(*cr1, 10);
		CLEAR_BIT(*cr1, 9);
	}
	else if (parity_control == USART_PARITY_EN_ODD) {
		SET_BIT(*cr1, 10);
		SET_BIT(*cr1, 9);
	}

	// Hardware flow control
	uint8_t hw_flow_control =
	    p_USART_handle->USART_Pin_Config.USART_hw_flow_control;

	VALIDATE_USART_HW_FLOW(hw_flow_control);

	CLEAR_FIELD_2BIT(*cr3, 8);
	if (hw_flow_control == USART_HW_FLOW_CTRL_NONE) {
		// Already cleared bits for USART_STOPBITS_1
	}
	else if (hw_flow_control == USART_HW_FLOW_CTRL_CTS) {
		SET_BIT(*cr3, 9);
	}
	else if (hw_flow_control == USART_HW_FLOW_CTRL_RTS) {
		SET_BIT(*cr3, 8);
	}
	else if (hw_flow_control == USART_HW_FLOW_CTRL_CTS_RTS) {
		SET_BIT(*cr3, 8);
		SET_BIT(*cr3, 9);
	}

	return USART_OK;
}

/********************************************************************************
 * @fn				- USART_deinit
 *
 * @brief			- Resets a USART peripheral
 *
 * @param[*p_USART_x]		- Base address of the USART peripheral
 *
 * @return			- Success / Failure status of the function
 *
 * @Note			- None
 *******************************************************************************/

USART_status_t USART_de_init(USART_TypeDef* p_USART_x)
{
	USART_status_t status = USART_ERROR_INVALID_PORT;

	if (p_USART_x == USART1) {
		USART1_REG_RESET();
		status = USART_OK;
	}
	else if (p_USART_x == USART2) {
		USART2_REG_RESET();
		status = USART_OK;
	}
	else if (p_USART_x == USART6) {
		USART6_REG_RESET();
		status = USART_OK;
	}

	return status;
}

/********************************************************************************
 * @fn				- USART_send_data
 *
 * @brief			- Sends data over USART using a polling method
 *
 * @param[*p_USART_x]		- Base address of the USART peripheral
 * @param[*p_tx_buffer]		- Transmission buffer ( pointer )
 * @param[len]			- Number of bytes to send
 *
 * @return			- Success / Failure status of the function
 *
 * @Note			- None
 *******************************************************************************/

USART_status_t USART_send_data(USART_Handle_t* p_USART_handle,
                               uint8_t* p_tx_buffer, uint32_t len)
{
	uint16_t* pdata;

	VALIDATE_PTR(p_USART_handle, USART_ERROR_NULL_PTR);
	VALIDATE_PTR(p_tx_buffer, USART_ERROR_NULL_PTR);

	USART_TypeDef* port = p_USART_handle->p_USARTx;
	VALIDATE_USART_PORT(port);

	if (len == 0)
		return USART_OK;

	VALIDATE_USART_ENABLED(port);
	VALIDATE_USART_TX_ENABLED(port);

	uint32_t word_len = p_USART_handle->USART_Pin_Config.USART_word_len;
	uint32_t parity_control =
	    p_USART_handle->USART_Pin_Config.USART_parity_control;
	volatile uint32_t* dr = &p_USART_handle->p_USARTx->DR;

	for (uint32_t i = 0; i < len; i++) {

		uint32_t timeout = POLLING_TIMEOUT;

		while (!USART_get_flag_status(p_USART_handle->p_USARTx,
		                              USART_FLAG_TXE)) {
			if (--timeout == 0U)
				return USART_ERROR_TIMEOUT;
		}

		if (word_len == USART_WORDLEN_9BITS) {
			// if 9BIT, load the DR with 2bytes masking the bits
			// other than first 9 bits
			pdata = (uint16_t*)p_tx_buffer;
			*dr = (*pdata & (uint16_t)0x01FF);

			if (parity_control == USART_PARITY_DISABLE)
				p_tx_buffer += 2;
			else
				p_tx_buffer++;
		}
		else {
			// This is 8bit data transfer
			SET_BYTE(*dr, 0, *p_tx_buffer);

			// Implement the code to increment the buffer address
			p_tx_buffer++;
		}
	}

	uint32_t timeout = POLLING_TIMEOUT;
	while (
	    !USART_get_flag_status(p_USART_handle->p_USARTx, USART_FLAG_TC)) {
		if (--timeout == 0)
			return USART_ERROR_TIMEOUT;
	}

	return USART_OK;
}

/********************************************************************************
 * @fn				- USART_receive_data
 *
 * @brief			- Receives data over USART
 *
 * @param[*p_USART_x]		- Base address of the USART peripheral
 * @param[*p_rx_buffer]		- Receive buffer ( pointer )
 * @param[len]			- Number of bytes to receive
 *
 * @return			- Success / Failure status of the function
 *
 * @Note			- Only waits for RXNE and does not check
 * FE/NF/ORE status bits. Frames received with errors will be silently discarded
 * by hardware. Error detection is implemented in the interrupt based function
 * USART_irq_handling(), prefer to use USART_receive_data_it
 *******************************************************************************/

USART_status_t USART_receive_data(USART_Handle_t* p_USART_handle,
                                  uint8_t* p_rx_buffer, uint32_t len)
{
	VALIDATE_PTR(p_USART_handle, USART_ERROR_NULL_PTR);
	VALIDATE_PTR(p_rx_buffer, USART_ERROR_NULL_PTR);

	USART_TypeDef* port = p_USART_handle->p_USARTx;
	VALIDATE_USART_PORT(port);

	if (len == 0)
		return USART_OK;

	VALIDATE_USART_ENABLED(port);
	VALIDATE_USART_RX_ENABLED(port);

	uint32_t word_len = p_USART_handle->USART_Pin_Config.USART_word_len;
	uint32_t parity_control =
	    p_USART_handle->USART_Pin_Config.USART_parity_control;
	volatile uint32_t* dr = &p_USART_handle->p_USARTx->DR;

	for (uint32_t i = 0; i < len; i++) {
		uint32_t timeout = POLLING_TIMEOUT;

		// WARNING: Error handling done in interrupt
		while (!(USART_get_flag_status(p_USART_handle->p_USARTx,
		                               USART_FLAG_RXNE))) {
			if (--timeout == 0U)
				return USART_ERROR_TIMEOUT;
		}

		if (word_len == USART_WORDLEN_9BITS) {
			if (parity_control == USART_PARITY_DISABLE) {
				*((uint16_t*)p_rx_buffer) =
				    (*dr & (uint16_t)0x01FF);

				p_rx_buffer += 2;
			}
			else {
				*p_rx_buffer = (*dr & (uint8_t)0xFF);
				p_rx_buffer++;
			}
		}
		else {

			if (parity_control == USART_PARITY_DISABLE) {
				*p_rx_buffer = (*dr & (uint8_t)0xFF);
			}

			else {
				*p_rx_buffer = (*dr & (uint8_t)0x7F);
			}
			p_rx_buffer++;
		}
	}

	return USART_OK;
}

/********************************************************************************
 * @fn				- USART_send_data_it
 *
 * @brief			- Sends data over USART in interrupt mode
 *
 * @param[*p_USART_handle]	- Handle structure of a USART peripheral
 * @param[*p_tx_buffer]		- Transmission buffer ( pointer )
 * @param[len]			- Number of bytes to send
 *
 * @return			- Success / Failure status of the function
 *
 * @Note			- None
 *******************************************************************************/

USART_status_t USART_send_data_it(USART_Handle_t* p_USART_handle,
                                  uint8_t* p_tx_buffer, uint32_t len)
{
	VALIDATE_PTR(p_USART_handle, USART_ERROR_NULL_PTR);
	VALIDATE_PTR(p_tx_buffer, USART_ERROR_NULL_PTR);

	VALIDATE_USART_PORT(p_USART_handle->p_USARTx);
	VALIDATE_USART_ENABLED(p_USART_handle->p_USARTx);
	VALIDATE_USART_TX_ENABLED(p_USART_handle->p_USARTx);

	if (len == 0U)
		return USART_OK;

	uint8_t tx_state = p_USART_handle->tx_busy_state;

	if (tx_state != USART_BUSY_IN_TX) {
		p_USART_handle->tx_len = len;
		p_USART_handle->p_tx_buffer = p_tx_buffer;
		p_USART_handle->tx_busy_state = USART_BUSY_IN_TX;

		// Enabling interrupt for TXE
		SET_BIT(p_USART_handle->p_USARTx->CR1, USART_CR1_TXEIE);

		// Enabling interrupt for TC
		SET_BIT(p_USART_handle->p_USARTx->CR1, USART_CR1_TCIE);
	}
	else
		return USART_BUSY;

	return USART_OK;
}

/********************************************************************************
 * @fn				- USART_receive_data_it
 *
 * @brief			- Receive data over USART in interrupt mode
 *
 * @param[*p_USART_handle]	- Handle structure of a USART peripheral
 * @param[*p_rx_buffer]		- Receive buffer ( pointer )
 * @param[len]			- Number of bytes to receive
 *
 * @return			- Success / Failure status of the function
 *
 * @Note			- None
 *******************************************************************************/

USART_status_t USART_receive_data_it(USART_Handle_t* p_USART_handle,
                                     uint8_t* p_rx_buffer, uint32_t len)
{
	VALIDATE_PTR(p_USART_handle, USART_ERROR_NULL_PTR);
	VALIDATE_PTR(p_rx_buffer, USART_ERROR_NULL_PTR);

	VALIDATE_USART_PORT(p_USART_handle->p_USARTx);
	VALIDATE_USART_ENABLED(p_USART_handle->p_USARTx);
	VALIDATE_USART_RX_ENABLED(p_USART_handle->p_USARTx);

	if (len == 0U)
		return USART_OK;

	uint8_t rx_state = p_USART_handle->rx_busy_state;

	if (rx_state != USART_BUSY_IN_RX) {
		p_USART_handle->rx_len = len;
		p_USART_handle->p_rx_buffer = p_rx_buffer;
		p_USART_handle->rx_busy_state = USART_BUSY_IN_RX;

		/* Enabling RXNE interrupt */
		SET_BIT(p_USART_handle->p_USARTx->CR1, USART_CR1_RXNEIE);
		SET_BIT(p_USART_handle->p_USARTx->CR3, USART_CR3_EIE);
	}

	return USART_OK;
}

/********************************************************************************
 * @fn				- USART_peri_control
 *
 * @brief			- Sets USART peripheral control
 *
 * @param[*p_USART_x]		- Base address of the USART peripheral
 * @param[EN_or_DI]		- ENABLE or DISABLE macros
 *
 * @return			- Success / Failure status of the function
 *
 * @Note			- None
 *******************************************************************************/

USART_status_t USART_peri_control(USART_TypeDef* p_USART_x, uint8_t EN_or_DI)
{
	VALIDATE_EN_DI(EN_or_DI, USART_ERROR_INVALID_STATE);

	VALIDATE_USART_PORT(p_USART_x);

	if (EN_or_DI == ENABLE) {
		SET_BIT(p_USART_x->CR1, USART_CR1_UE);
	}
	else {
		CLEAR_BIT(p_USART_x->CR1, USART_CR1_UE);
	}

	return USART_OK;
}

/********************************************************************************
 * @fn				- USART_get_flag_status
 *
 * @brief			- Returns flag status of a USART flag
 *
 * @param[*p_USART_x]		- Base address of the USART peripheral
 * @param[status_flag_name]	- Flag name
 *
 * @return			- 1 if flag set, 0 if not set
 *
 * @Note			- None
 *******************************************************************************/

uint8_t USART_get_flag_status(USART_TypeDef* p_USART_x,
                              uint8_t status_flag_name)
{
	return IS_BIT_SET(p_USART_x->SR, status_flag_name);
}

/********************************************************************************
 * @fn				- USART_clear_flag
 *
 * @brief			- Clears flag status of a USART flag
 *
 * @param[*p_USART_x]		- Base address of the USART peripheral
 * @param[status_flag_name]	- Flag name
 *
 * @return			- None
 *
 * @Note			- None
 *******************************************************************************/

void USART_clear_flag(USART_TypeDef* p_USART_x, uint16_t status_flag_name)
{
	CLEAR_BIT(p_USART_x->SR, status_flag_name);
}

/********************************************************************************
 * @fn				- USART_clear_error_flags
 *
 * @brief			- Clears error flags
 *
 * @param[*p_USART_x]		- Base address of the USART peripheral
 *
 * @return			- None
 *
 * @Note			- None
 *******************************************************************************/

static void USART_clear_error_flags(USART_TypeDef* p_USART_x)
{
	volatile uint32_t tmp;
	tmp = p_USART_x->SR;
	tmp = p_USART_x->DR;
	(void)tmp;
}

/********************************************************************************
 * @fn				- USART_irq_interrupt_config
 *
 * @brief			- Configures USART IRQ interrupt
 *
 * @param[irq_n]		- IRQ number
 * @param[EN_or_DI]		- ENABLE or DISABLE macros
 *
 * @return			- Success / Failure status of the function
 *
 * @Note			- None
 *******************************************************************************/

USART_status_t USART_irq_interrupt_config(uint8_t irq_n, uint8_t EN_or_DI)
{
	VALIDATE_EN_DI(EN_or_DI, USART_ERROR_INVALID_STATE);

	VALIDATE_IRQ_NUMBER(irq_n, USART_ERROR_INVALID_IRQ);

	if (EN_or_DI == ENABLE) {
		if (irq_n < 32)
			SET_BIT(NVIC->ISER[0], irq_n);
		else if (irq_n >= 32 && irq_n < 64)
			SET_BIT(NVIC->ISER[1], (irq_n % 32));
		else if (irq_n >= 64 && irq_n < 96)
			SET_BIT(NVIC->ISER[2], (irq_n % 64));
	}
	else {
		if (irq_n < 32)
			SET_BIT(NVIC->ICER[0], irq_n);
		else if (irq_n >= 32 && irq_n < 64)
			SET_BIT(NVIC->ICER[1], (irq_n % 32));
		else if (irq_n >= 64 && irq_n < 96)
			SET_BIT(NVIC->ICER[2], (irq_n % 64));
	}

	return USART_OK;
}

/********************************************************************************
 * @fn				- USART_irq_priority_config
 *
 * @brief			- Configures USART IRQ priority
 *
 * @param[irq_n]		- IRQ number
 * @param[irq_prio]		- IRQ priority
 *
 * @return			- Success / Failure status of the function
 *
 * @Note			- None
 *******************************************************************************/

USART_status_t USART_irq_priority_config(uint8_t irq_n, uint32_t irq_prio)
{
	VALIDATE_IRQ_NUMBER(irq_n, USART_ERROR_INVALID_IRQ);

	uint8_t ipr_x = irq_n / 4;
	uint8_t ipr_x_section = (irq_n % 4) * 8;
	uint32_t prio_field = (irq_prio & ((1U << __NVIC_PRIO_BITS) - 1U))
	                      << (8U - __NVIC_PRIO_BITS);
	uint32_t ipr = NVIC->IPR[ipr_x];
	CLEAR_BYTE(ipr, ipr_x_section);
	ipr |= (prio_field << ipr_x_section);
	NVIC->IPR[ipr_x] = ipr;

	return USART_OK;
}

/********************************************************************************
 * @fn				- USART_irq_handling
 *
 * @brief			- Handles USART interrupts
 *
 * @param[*p_USART_handle]	- Handle structure of a USART peripheral
 *
 * @return			- None
 *
 * @Note			- None
 *******************************************************************************/

void USART_irq_handling(USART_Handle_t* p_USART_handle)
{
	volatile uint32_t* cr1 = &p_USART_handle->p_USARTx->CR1;
	volatile uint32_t* cr3 = &p_USART_handle->p_USARTx->CR3;
	volatile uint32_t* sr = &p_USART_handle->p_USARTx->SR;
	volatile uint32_t* dr = &p_USART_handle->p_USARTx->DR;

	uint8_t word_len = p_USART_handle->USART_Pin_Config.USART_word_len;
	uint8_t parity_control =
	    p_USART_handle->USART_Pin_Config.USART_parity_control;

	uint16_t* p_data;
	// Check state of TC bit in SR
	uint32_t tc_state = IS_BIT_SET(*sr, USART_SR_TC);
	// Check state of TCEIE bit
	uint32_t tceie_state = IS_BIT_SET(*cr1, USART_CR1_TCIE);

	if (tc_state && tceie_state) {
		// TC caused interrupt

		// Closing transmission and calling application callback if
		// tx_len is 0
		if (p_USART_handle->tx_busy_state == USART_BUSY_IN_TX) {
			if (!p_USART_handle->tx_len) {
				CLEAR_BIT(*sr, USART_SR_TC);
				CLEAR_BIT(*cr1, USART_CR1_TCIE);
				p_USART_handle->tx_busy_state = USART_READY;
				p_USART_handle->p_tx_buffer = NULL;
				p_USART_handle->tx_len = 0;
				USART_application_event_callback(
				    p_USART_handle, USART_EVENT_TX_CMPLT);
			}
		}
	}

	// Check for TXE flag
	uint32_t txe_state = IS_BIT_SET(*sr, USART_SR_TXE);
	// Check for TXEIE flag
	uint32_t txeie_state = IS_BIT_SET(*cr1, USART_CR1_TXEIE);

	if (txe_state && txeie_state) {
		// TXE caused interrupt

		if (p_USART_handle->tx_busy_state == USART_BUSY_IN_TX) {
			if (p_USART_handle->tx_len > 0) {
				if (word_len == USART_WORDLEN_9BITS) {
					p_data =
					    (uint16_t*)
					        p_USART_handle->p_tx_buffer;
					*dr = (*p_data & (uint16_t)0x01FF);
					if (parity_control ==
					    USART_PARITY_DISABLE) {
						p_USART_handle->p_tx_buffer +=
						    2;
						p_USART_handle->tx_len -= 2;
					}
					else {
						p_USART_handle->p_tx_buffer++;
						p_USART_handle->tx_len--;
					}
				}
				else {
					*dr = (*p_USART_handle->p_tx_buffer &
					       (uint8_t)0xFF);
					p_USART_handle->p_tx_buffer++;
					p_USART_handle->tx_len--;
				}
			}
			if (p_USART_handle->tx_len == 0) {
				CLEAR_BIT(*cr1, USART_CR1_TXEIE);
			}
		}
	}

	// Check for RXNE flag
	uint32_t rxne_state = IS_BIT_SET(*sr, USART_SR_RXNE);
	// Check for RXNEIE flag
	uint32_t rxneie_state = IS_BIT_SET(*cr1, USART_CR1_RXNEIE);

	if (rxne_state && rxneie_state) {
		// RXNE

		if (p_USART_handle->rx_busy_state == USART_BUSY_IN_RX) {
			if (p_USART_handle->rx_len > 0) {
				if (word_len == USART_WORDLEN_9BITS) {
					if (parity_control ==
					    USART_PARITY_DISABLE) {
						*((uint16_t*)p_USART_handle
						      ->p_rx_buffer) =
						    (*dr & (uint16_t)0x01FF);

						p_USART_handle->p_rx_buffer +=
						    2;
						p_USART_handle->rx_len -= 2;
					}
					else {
						*p_USART_handle->p_rx_buffer =
						    (*dr & (uint8_t)0xFF);
						p_USART_handle->p_rx_buffer++;
						p_USART_handle->rx_len--;
					}
				}
				else {
					if (parity_control ==
					    USART_PARITY_DISABLE) {
						*p_USART_handle->p_rx_buffer =
						    (uint8_t)(*dr &
						              (uint8_t)0xFF);
					}
					else {
						*p_USART_handle->p_rx_buffer =
						    (uint8_t)(*dr &
						              (uint8_t)0x7F);
					}

					/* Incrementing the pRxBuffer */
					p_USART_handle->p_rx_buffer++;
					p_USART_handle->rx_len--;
				}
			}

			if (!p_USART_handle->rx_len) {
				/* Disabling the RXNE */
				CLEAR_BIT(*cr1, USART_CR1_RXNEIE);
				p_USART_handle->rx_busy_state = USART_READY;
				USART_application_event_callback(
				    p_USART_handle, USART_EVENT_RX_CMPLT);
			}
		}
	}

	// Check for CTS flag
	uint32_t cts_state = IS_BIT_SET(*sr, USART_SR_CTS);
	// Check for CTSE flag
	uint32_t ctse_state = IS_BIT_SET(*cr3, USART_CR3_CTSE);
	// Check for CTSIE flag
	uint32_t ctsie_state = IS_BIT_SET(*cr3, USART_CR3_CTSIE);

	if (cts_state && ctse_state && ctsie_state) {
		// Clearing CTS flag
		CLEAR_BIT(*sr, USART_SR_CTS);
		// CTS caused interrupt
		USART_application_event_callback(p_USART_handle,
		                                 USART_EVENT_CTS);
	}

	// Check for IDLE flag
	uint32_t idle_state = IS_BIT_SET(*sr, USART_SR_IDLE);
	// Check for IDLEIE flag
	uint32_t idleie_state = IS_BIT_SET(*cr1, USART_CR1_IDLEIE);

	if (idle_state && idleie_state) {
		CLEAR_BIT(*sr, USART_SR_IDLE);
		// IDLE caused interrupt
		USART_application_event_callback(p_USART_handle,
		                                 USART_EVENT_IDLE);
	}

	// Check for ORE flag
	// uint32_t ore_state = IS_BIT_SET(*sr, USART_SR_ORE);
	// Check for RXNEIE
	// rxneie_state = IS_BIT_SET(*cr1, USART_CR1_RXNEIE);

	// if (ore_state && rxneie_state) {
	// 	// Need not to clear the ORE flag here, instead give an api for
	// 	// the application to clear the ORE flag .
	// 	USART_application_event_callback(p_USART_handle, USART_ERR_ORE);
	// }

	// Check for error flag
	uint32_t eie_state = IS_BIT_SET(*cr3, USART_CR3_EIE);

	if (eie_state) {
		uint32_t sr_val = *sr;

		if (sr_val & USART_SR_FE) {
			/*
			                    This bit is set by hardware when a
			   de-synchronization, excessive noise or a break
			   character is detected. It is cleared by a software
			   sequence (an read to the USART_SR register followed
			   by a read to the USART_DR register).
			 */
			USART_application_event_callback(p_USART_handle,
			                                 USART_ERR_FE);
		}

		if (sr_val & USART_SR_NF) {
			/*
			                    This bit is set by hardware when
			   noise is detected on a received frame. It is cleared
			   by a software sequence (an read to the USART_SR
			   register followed by a read to the USART_DR
			   register).
			 */
			USART_application_event_callback(p_USART_handle,
			                                 USART_ERR_NE);
		}

		if (sr_val & USART_SR_ORE) {
			USART_application_event_callback(p_USART_handle,
			                                 USART_ERR_ORE);
		}

		// WARNING: This clears DR as well, which means any data that
		// was in DR at the time of the error is lost. This isn't an
		// issue as any data in DR is likely to be corrupted in case
		// this part of the code is called
		USART_clear_error_flags(p_USART_handle->p_USARTx);
	}
}

/********************************************************************************
 * @fn				- USART_set_baud_rate
 *
 * @brief			- Sets baud rate on USART
 *
 * @param[*p_USART_x]		- Base address of the USART peripheral
 * @param[baud_rate]		- Baud rate to set
 *
 * @return			- None
 *
 * @Note			- None
 *******************************************************************************/

void USART_set_baud_rate(USART_TypeDef* p_USART_x, uint32_t baud_rate)
{
	uint32_t pclk_x;

	uint32_t usart_div;

	uint32_t m_part, f_part;

	uint32_t temp_reg = 0;

	if (p_USART_x == USART1 || p_USART_x == USART6) {
		pclk_x = RCC_get_pclk_2_val();
	}
	else {
		pclk_x = RCC_get_pclk_1_val();
	}

	// Check for OVER8 configuration bit
	if (IS_BIT_SET(p_USART_x->CR1, USART_CR1_OVER8)) {
		usart_div = ((25 * pclk_x) / (2 * baud_rate));
	}
	else {
		/* OVER8 = 0. Over sampling by 16 */
		usart_div = ((25 * pclk_x) / (4 * baud_rate));
	}

	// Calculate the Mantissa part
	m_part = usart_div / 100;

	// Place the Mantissa part in appropriate bit position
	temp_reg |= m_part << 4;

	// Extract the fraction part
	f_part = (usart_div - (m_part * 100));

	// Calculate the final fractional
	if (IS_BIT_SET(p_USART_x->CR1, USART_CR1_OVER8)) {
		f_part = (((f_part * 8) + 50) / 100) & ((uint8_t)0x07);
	}
	else {
		f_part = (((f_part * 16) + 50) / 100) & ((uint8_t)0x0F);
	}

	// Place the fractional part in appropriate bit position
	temp_reg |= f_part;

	// Copy the value of tempreg in to BRR register
	p_USART_x->BRR = temp_reg;
}

/********************************************************************************
 * @fn				- USART_application_event_callback
 *
 * @brief			- Application event callback function
 *
 * @param[*p_USART_handle]	- Handle structure of a USART peripheral
 * @param[app_ev]		- Application event
 *
 * @return			- None
 *
 * @Note			- None
 *******************************************************************************/

__attribute__((weak)) void
USART_application_event_callback(USART_Handle_t* p_USART_handle,
                                 USART_AppEvent_t app_ev)
{
	/* This is a week implementation. The application may override
	 * this function. */
}
