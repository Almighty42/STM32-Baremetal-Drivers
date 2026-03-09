#include "stm32f401xx_SPI_driver.h"
#include "stm32f401xx.h"

/********************************************************************************
 *
 * TODO: Future plans for this driver:
 * 1. ...
 *
 *******************************************************************************/

/********************************************************************************
 *
 * INFO: Driver map
 * @PERIPHERAL_CLOCK_SETUP
 * @INIT_DE-INIT
 * @DATA_SEND_RECEIVE_POLLING_INTERRUPT
 * @PERIPHERAL_CONTROL_API
 * @IRQ_CONFIGURATION_AND_ISR_HANDLING
 *
 *******************************************************************************/

// NOTE: @PERIPHERAL_CLOCK_SETUP

/********************************************************************************
 * @fn				- SPI_peri_clk_control
 *
 * @brief			- Enables or disables peripheral clock for SPI
 *
 * @param[*p_USART_x]		- Base address of the SPI peripheral
 * @param[EN_or_DI]		- ENABLE or DISABLE macros
 *
 * @return			- Success / Failure status of the function
 *
 * @Note			- None
 *******************************************************************************/

SPI_status_t SPI_peri_clk_control(SPI_TypeDef* p_SPI_x, uint8_t EN_or_DI)
{
	VALIDATE_EN_DI(EN_or_DI, SPI_ERROR_INVALID_STATE);

	SPI_status_t status = SPI_ERROR_INVALID_PORT;

	if (p_SPI_x == SPI1) {
		if (EN_or_DI == ENABLE) {
			SPI1_PCLK_EN();
		}
		else
			SPI1_PCLK_DI();
		status = SPI_OK;
	}
	else if (p_SPI_x == SPI2) {
		if (EN_or_DI == ENABLE) {
			SPI2_PCLK_EN();
		}
		else
			SPI2_PCLK_DI();
		status = SPI_OK;
	}
	else if (p_SPI_x == SPI3) {
		if (EN_or_DI == ENABLE) {
			SPI3_PCLK_EN();
		}
		else
			SPI3_PCLK_DI();
		status = SPI_OK;
	}
	else if (p_SPI_x == SPI4) {
		if (EN_or_DI == ENABLE) {
			SPI4_PCLK_EN();
		}
		else
			SPI4_PCLK_DI();
		status = SPI_OK;
	}

	return status;
}

// NOTE: @INIT_DE-INIT

/********************************************************************************
 * @fn				- SPI_init
 *
 * @brief			- Initializes a SPI peripheral
 *
 * @param[*p_USART_handle]	- Handle structure of a SPI peripheral
 *
 * @return			- Success / Failure status of the function
 *
 * @Note			- None
 *******************************************************************************/

SPI_status_t SPI_init(SPI_Handle_t* p_SPI_handle)
{
	VALIDATE_PTR(p_SPI_handle, SPI_ERROR_NULL_PTR);

	SPI_TypeDef* port = p_SPI_handle->p_SPIx;
	VALIDATE_SPI_PORT(port);

	volatile uint32_t* cr1 = &p_SPI_handle->p_SPIx->CR1;
	volatile uint32_t* cr2 = &p_SPI_handle->p_SPIx->CR2;

	// Clock
	SPI_peri_clk_control(port, ENABLE);

	// Mode
	uint8_t mode = p_SPI_handle->SPI_Config.SPI_Device_Mode;

	VALIDATE_SPI_DEVICE_MODE(mode);

	CLEAR_BIT(*cr1, SPI_CR1_MSTR);
	if (mode == SPI_DEVICE_MODE_MASTER)
		SET_BIT(*cr1, SPI_CR1_MSTR);
	else if (mode == SPI_DEVICE_MODE_SLAVE)
		CLEAR_BIT(*cr1, SPI_CR1_MSTR);

	uint8_t bus_config = p_SPI_handle->SPI_Config.SPI_Bus_Config;

	VALIDATE_SPI_BUS_CONFIG(bus_config);

	if (bus_config == SPI_BUS_CONFIG_FD)
		CLEAR_BIT(*cr1, SPI_CR1_BIDIMODE);
	else if (bus_config == SPI_BUS_CONFIG_HD) {
		SET_BIT(*cr1, SPI_CR1_BIDIMODE);
	}
	else if (bus_config == SPI_BUS_CONFIG_SIMPLEX_RX_ONLY) {
		CLEAR_BIT(*cr1, SPI_CR1_BIDIMODE);
		SET_BIT(*cr1, SPI_CR1_RXONLY);
	}
	else if (bus_config == SPI_BUS_CONFIG_SIMPLEX_TX_ONLY) {
		CLEAR_BIT(*cr1, SPI_CR1_BIDIMODE);
		CLEAR_BIT(*cr1, SPI_CR1_RXONLY);
	}

	// Speed
	uint8_t speed = p_SPI_handle->SPI_Config.SPI_SCLK_Speed;

	VALIDATE_SPI_SCLK_SPEED(speed);

	if (speed == SPI_SCLK_SPEED_DIV_2) {
		CLEAR_BIT(*cr1, SPI_CR1_BR);
		CLEAR_BIT(*cr1, (SPI_CR1_BR + 1));
		CLEAR_BIT(*cr1, (SPI_CR1_BR + 2));
	}
	else if (speed == SPI_SCLK_SPEED_DIV_4) {
		SET_BIT(*cr1, SPI_CR1_BR);
		CLEAR_BIT(*cr1, (SPI_CR1_BR + 1));
		CLEAR_BIT(*cr1, (SPI_CR1_BR + 2));
	}
	else if (speed == SPI_SCLK_SPEED_DIV_8) {
		CLEAR_BIT(*cr1, SPI_CR1_BR);
		SET_BIT(*cr1, (SPI_CR1_BR + 1));
		CLEAR_BIT(*cr1, (SPI_CR1_BR + 2));
	}
	else if (speed == SPI_SCLK_SPEED_DIV_16) {
		SET_BIT(*cr1, SPI_CR1_BR);
		SET_BIT(*cr1, (SPI_CR1_BR + 1));
		CLEAR_BIT(*cr1, (SPI_CR1_BR + 2));
	}
	else if (speed == SPI_SCLK_SPEED_DIV_32) {
		CLEAR_BIT(*cr1, SPI_CR1_BR);
		CLEAR_BIT(*cr1, (SPI_CR1_BR + 1));
		SET_BIT(*cr1, (SPI_CR1_BR + 2));
	}
	else if (speed == SPI_SCLK_SPEED_DIV_64) {
		SET_BIT(*cr1, SPI_CR1_BR);
		CLEAR_BIT(*cr1, (SPI_CR1_BR + 1));
		SET_BIT(*cr1, (SPI_CR1_BR + 2));
	}
	else if (speed == SPI_SCLK_SPEED_DIV_128) {
		CLEAR_BIT(*cr1, SPI_CR1_BR);
		SET_BIT(*cr1, (SPI_CR1_BR + 1));
		SET_BIT(*cr1, (SPI_CR1_BR + 2));
	}
	else if (speed == SPI_SCLK_SPEED_DIV_256) {
		SET_BIT(*cr1, SPI_CR1_BR);
		SET_BIT(*cr1, (SPI_CR1_BR + 1));
		SET_BIT(*cr1, (SPI_CR1_BR + 2));
	}

	// DFF
	uint8_t dff = p_SPI_handle->SPI_Config.SPI_DFF;

	VALIDATE_SPI_DFF(dff);

	CLEAR_BIT(*cr1, SPI_CR1_DFF);
	if (dff == SPI_DFF_16BIT)
		SET_BIT(*cr1, SPI_CR1_DFF);
	else if (dff == SPI_DFF_8BIT)
		CLEAR_BIT(*cr1, SPI_CR1_DFF);

	// CPOL
	uint8_t cpol = p_SPI_handle->SPI_Config.SPI_CPOL;

	VALIDATE_SPI_CPOL(cpol);

	CLEAR_BIT(*cr1, SPI_CR1_CPOL);
	if (cpol == SPI_CPOL_HIGH)
		SET_BIT(*cr1, SPI_CR1_CPOL);
	else if (cpol == SPI_CPOL_LOW)
		CLEAR_BIT(*cr1, SPI_CR1_CPOL);

	// CPHA
	uint8_t cpha = p_SPI_handle->SPI_Config.SPI_CPHA;

	VALIDATE_SPI_CPHA(cpha);

	CLEAR_BIT(*cr1, SPI_CR1_CPHA);
	if (cpha == SPI_CPHA_HIGH)
		SET_BIT(*cr1, SPI_CR1_CPHA);
	else if (cpha == SPI_CPHA_LOW)
		CLEAR_BIT(*cr1, SPI_CR1_CPHA);

	// SSM
	uint8_t ssm = p_SPI_handle->SPI_Config.SPI_SSM;

	VALIDATE_SPI_SSM(ssm);

	CLEAR_BIT(*cr1, SPI_CR1_SSM);
	if (ssm == SPI_SSM_SW) {
		SET_BIT(*cr1, SPI_CR1_SSM);
		SET_BIT(*cr1, SPI_CR1_SSI);
	}
	else if (ssm == SPI_SSM_HW)
		CLEAR_BIT(*cr1, SPI_CR1_SSM);

	return SPI_OK;
}

/********************************************************************************
 * @fn				- SPI_de_init
 *
 * @brief			- Resets a SPI peripheral
 *
 * @param[*p_SPI_x]		- Base address of the SPI peripheral
 *
 * @return			- Success / Failure status of the function
 *
 * @Note			- None
 *******************************************************************************/

SPI_status_t SPI_de_init(SPI_TypeDef* p_SPI_x)
{
	SPI_status_t status = SPI_ERROR_INVALID_PORT;

	if (p_SPI_x == SPI1) {
		SPI1_PER_RESET();
		SPI1_CLK_DISABLE();
		status = SPI_OK;
	}
	else if (p_SPI_x == SPI2) {
		SPI2_PER_RESET();
		SPI2_CLK_DISABLE();
		status = SPI_OK;
	}
	else if (p_SPI_x == SPI3) {
		SPI3_PER_RESET();
		SPI3_CLK_DISABLE();
		status = SPI_OK;
	}
	else if (p_SPI_x == SPI4) {
		SPI4_PER_RESET();
		SPI4_CLK_DISABLE();
		status = SPI_OK;
	}

	return status;
}

// NOTE: @DATA_SEND_RECEIVE_POLLING_INTERRUPT

/********************************************************************************
 * @fn				- SPI_write_data_pl
 *
 * @brief			- Write data over USART using a polling method
 *
 * @param[*p_SPI_x]		- Base address of the SPI peripheral
 * @param[*p_tx_buffer]		- Transmission buffer ( pointer )
 * @param[len]			- Number of bytes to send
 *
 * @return			- Success / Failure status of the function
 *
 * @Note			- None
 *******************************************************************************/

SPI_status_t SPI_write_data_pl(SPI_Handle_t* p_SPI_handle,
                               const uint8_t* p_tx_buffer, uint32_t len)
{
	while (len > 0) {
		// wait until TXE == 1
		while (!(p_SPI_handle->p_SPIx->SR & SPI_SR_TXE)) {
			// spin
		}

		if (p_SPI_handle->p_SPIx->CR1 & SPI_CR1_DFF) {
			p_SPI_handle->p_SPIx->DR = *((uint16_t*)p_tx_buffer);
			len -= 2;
			p_tx_buffer += 2;
		}
		else {
			p_SPI_handle->p_SPIx->DR = *p_tx_buffer;
			len--;
			p_tx_buffer++;
		}
	}

	// wait until BSY == 0 (last bit shifted out)
	while (p_SPI_handle->p_SPIx->SR & SPI_SR_BSY) {
		// spin
	}

	return SPI_OK;
}

/********************************************************************************
 * @fn				- SPI_read_data_pl
 *
 * @brief			- Reads data over SPI using a polling method
 *
 * @param[*p_SPI_x]		- Base address of the SPI peripheral
 * @param[*p_rx_buffer]		- Receive buffer ( pointer )
 * @param[len]			- Number of bytes to receive
 *
 * @return			- Success / Failure status of the function
 *
 * @Note			- None
 *******************************************************************************/

SPI_status_t SPI_read_data_pl(SPI_Handle_t* p_SPI_handle, uint8_t* p_rx_buffer,
                              uint32_t len);

/********************************************************************************
 * @fn				- SPI_write_data_it
 *
 * @brief			- Enables or disables peripheral clock for USART
 *
 * @param[*p_SPI_handle]	- Handle structure of a SPI peripheral
 * @param[*p_tx_buffer]		- Transmission buffer ( pointer )
 * @param[len]			- Number of bytes to send
 *
 * @return			- Success / Failure status of the function
 *
 * @Note			- None
 *******************************************************************************/

SPI_status_t SPI_write_data_it(SPI_Handle_t* p_SPI_handle, uint8_t* p_tx_buffer,
                               uint32_t len);

/********************************************************************************
 * @fn				- SPI_read_data_it
 *
 * @brief			- Reads data over SPI via interrupts
 *
 * @param[*p_SPI_handle]	- Handle structure of a SPI peripheral
 * @param[*p_rx_buffer]		- Receive buffer ( pointer )
 * @param[len]			- Number of bytes to receive
 *
 * @return			- Success / Failure status of the function
 *
 * @Note			- None
 *******************************************************************************/

SPI_status_t SPI_read_data_it(SPI_Handle_t* p_SPI_handle, uint8_t* p_rx_buffer,
                              uint32_t len);

// NOTE: @PERIPHERAL_CONTROL_API

/********************************************************************************
 * @fn				- SPI_peri_control
 *
 * @brief			- Sets SPI peripheral control
 *
 * @param[*p_SPI_x]		- Base address of the SPI peripheral
 * @param[EN_or_DI]		- ENABLE or DISABLE macros
 *
 * @return			- Success / Failure status of the function
 *
 * @Note			- None
 *******************************************************************************/

SPI_status_t SPI_peri_control(SPI_TypeDef* p_SPI_x, uint8_t EN_or_DI)
{
	VALIDATE_EN_DI(EN_or_DI, SPI_ERROR_INVALID_STATE);

	VALIDATE_SPI_PORT(p_SPI_x);

	if (EN_or_DI == ENABLE) {
		SET_BIT(p_SPI_x->CR1, SPI_CR1_SPE);
	}
	else {
		CLEAR_BIT(p_SPI_x->CR1, SPI_CR1_SPE);
	}

	return SPI_OK;
}

// NOTE: IRQ_CONFIGURATION_AND_ISR_HANDLING

/********************************************************************************
 * @fn				- SPI_irq_interrupt_config
 *
 * @brief			- Configures SPI IRQ interrupt
 *
 * @param[irq_n]		- IRQ number
 * @param[EN_or_DI]		- ENABLE or DISABLE macros
 *
 * @return			- Success / Failure status of the function
 *
 * @Note			- None
 *******************************************************************************/

SPI_status_t SPI_irq_interrupt_config(uint8_t irq_n, uint8_t EN_or_DI)
{
	VALIDATE_EN_DI(EN_or_DI, SPI_ERROR_INVALID_STATE);

	VALIDATE_IRQ_NUMBER(irq_n, SPI_ERROR_INVALID_IRQ);

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

	return SPI_OK;
}

/********************************************************************************
 * @fn				- SPI_irq_priority_config
 *
 * @brief			- Configures SPI IRQ priority
 *
 * @param[irq_n]		- IRQ number
 * @param[irq_prio]		- IRQ priority
 *
 * @return			- Success / Failure status of the function
 *
 * @Note			- None
 *******************************************************************************/

SPI_status_t SPI_irq_priority_config(uint8_t irq_n, uint32_t irq_prio)
{
	VALIDATE_IRQ_NUMBER(irq_n, SPI_ERROR_INVALID_IRQ);

	uint8_t ipr_x = irq_n / 4;
	uint8_t ipr_x_section = (irq_n % 4) * 8;
	uint32_t prio_field = (irq_prio & ((1U << __NVIC_PRIO_BITS) - 1U))
	                      << (8U - __NVIC_PRIO_BITS);
	uint32_t ipr = NVIC->IPR[ipr_x];
	CLEAR_BYTE(ipr, ipr_x_section);
	ipr |= (prio_field << ipr_x_section);
	NVIC->IPR[ipr_x] = ipr;

	return SPI_OK;
}

/********************************************************************************
 * @fn				- SPI_irq_handling
 *
 * @brief			- Handles SPI interrupts
 *
 * @param[*p_SPI_handle]	- Handle structure of a SPI peripheral
 *
 * @return			- None
 *
 * @Note			- None
 *******************************************************************************/

void SPI_irq_handling(SPI_Handle_t* p_SPI_handle);
