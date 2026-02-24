#include "stm32f401xx_I2C_driver.h"
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

/********************************************************************************
 * @fn				- SPI_de_init
 *
 * @brief			- Resets a SPI peripheral
 *
 * @param[*p_USART_x]		- Base address of the SPI peripheral
 *
 * @return			- Success / Failure status of the function
 *
 * @Note			- None
 *******************************************************************************/

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

// NOTE: @PERIPHERAL_CONTROL_API

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
