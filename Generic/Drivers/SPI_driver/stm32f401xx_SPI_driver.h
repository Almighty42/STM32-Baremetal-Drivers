#ifndef STM32F401RE_SPI_DRIVER_H
#define STM32F401RE_SPI_DRIVER_H

#include "stm32f401xx.h"

// NOTE: --- Structures for SPI ---

// Configuration structure for SPI

typedef struct
{
	uint8_t SPI_Device_Mode;					// Possible values from @SPI_DEVICE_MODE
	uint8_t SPI_Bus_Config;						// Possible values from @SPI_BUS_CONFIG
	uint8_t SPI_SCLK_Speed;						// Possible values from @SPI_SCLK_SPEED
	uint8_t SPI_DFF;						// Possible values from @SPI_DFF
	uint8_t SPI_CPOL;						// Possible values from @SPI_CPOL
	uint8_t SPI_CPHA;						// Possible values from @SPI_CPHA
	uint8_t SPI_SSM;						// Possible values from @SPI_SSM
} SPI_Config_t;

// Handle structure for SPI

typedef struct
{
	SPI_TypeDef* p_SPIx;						// Holds the base address of the SPI peripheral 
	SPI_Config_t SPI_Config;					// Holds SPI peripheral configuration settings
} SPI_Handle_t;

// Function return status

typedef enum {
	SPI_OK = 0,							// Success
	SPI_ERROR_INVALID_STATE,					// Invalid state of a argument
	SPI_ERROR_NULL_PTR,						// NULL pointer passed
	SPI_ERROR_INVALID_PORT,						// Invalid SPI port address
	SPI_ERROR_INVALID_IRQ,						// Invalid IRQ number
	SPI_ERROR_INVALID_MODE,						// Mode value out of range
	SPI_ERROR_INVALID_BUS_CONFIG,					// Bus config value out of range
	SPI_ERROR_INVALID_SCLK_SPEED,					// SCLK speed value out of range
	SPI_ERROR_INVALID_DFF,						// DFF value out of range
	SPI_ERROR_INVALID_CPOL,						// CPOL value out of range
	SPI_ERROR_INVALID_CPHA,						// CPHA value out of range
	SPI_ERROR_INVALID_SSM,						// SSM value out of range
	SPI_ERROR_TIMEOUT,						// SPI polling timeout
	SPI_BUSY							// SPI Tx / Rx busy
} SPI_status_t;

// USAGE: --- @SPI_DEVICE_MODE ---

#define SPI_DEVICE_MODE_MASTER			1
#define SPI_DEVICE_MODE_SLAVE			0

// USAGE: --- @SPI_BUS_CONFIG ---

#define SPI_BUS_CONFIG_FD			3
#define SPI_BUS_CONFIG_HD			2
#define SPI_BUS_CONFIG_SIMPLEX_TX_ONLY		1
#define SPI_BUS_CONFIG_SIMPLEX_RX_ONLY		0

// USAGE: --- @SPI_SCLK_SPEED ---

#define SPI_SCLK_SPEED_DIV_2			7
#define SPI_SCLK_SPEED_DIV_4			6
#define SPI_SCLK_SPEED_DIV_8			5
#define SPI_SCLK_SPEED_DIV_16			4
#define SPI_SCLK_SPEED_DIV_32			3
#define SPI_SCLK_SPEED_DIV_64			2
#define SPI_SCLK_SPEED_DIV_128			1
#define SPI_SCLK_SPEED_DIV_256			0

// USAGE: --- @SPI_DFF ---

#define SPI_DFF_8BIT				1
#define SPI_DFF_16BIT				0

// USAGE: --- @SPI_CPOL ---

#define SPI_CPOL_HIGH				1
#define SPI_CPOL_LOW				0

// USAGE: --- @SPI_CPHA ---

#define SPI_CPHA_HIGH				1
#define SPI_CPHA_LOW				0

// USAGE: --- @SPI_SSM ---

#define SPI_SSM_HW				1
#define SPI_SSM_SW				0

// NOTE: --- SPI Validation macros ---

#define VALIDATE_SPI_PORT(port)		do { \
							if ((port) == NULL || \
							!((port) == SPI1 || (port) == SPI2 || (port) == SPI3 || (port) == SPI4)) { \
								return SPI_ERROR_INVALID_PORT; \
							} \
						} while(0)

#define VALIDATE_SPI_DEVICE_MODE(mode)		VALIDATE_ENUM((mode), SPI_DEVICE_MODE_MASTER, SPI_ERROR_INVALID_MODE)
#define VALIDATE_SPI_BUS_CONFIG(stop)		VALIDATE_ENUM((stop), SPI_BUS_CONFIG_FD, SPI_ERROR_INVALID_BUS_CONFIG)
#define VALIDATE_SPI_SCLK_SPEED(wlen)		VALIDATE_ENUM((wlen), SPI_SCLK_SPEED_DIV_2, SPI_ERROR_INVALID_SCLK_SPEED)
#define VALIDATE_SPI_DFF(parity)		VALIDATE_ENUM((parity), SPI_DFF_8BIT, SPI_ERROR_INVALID_DFF)
#define VALIDATE_SPI_CPOL(flow)			VALIDATE_ENUM((flow), SPI_CPOL_HIGH, SPI_ERROR_INVALID_CPOL)
#define VALIDATE_SPI_CPHA(flow)			VALIDATE_ENUM((flow), SPI_CPHA_HIGH, SPI_ERROR_INVALID_CPHA)
#define VALIDATE_SPI_SSM(flow)			VALIDATE_ENUM((flow), SPI_SSM_HW, SPI_ERROR_INVALID_SSM)

// NOTE: --- Bit position definitions SPI_SR ---

#define SPI_SR_FRE				(1U << 8)
#define SPI_SR_BSY    				(1U << 7)
#define SPI_SR_OVR    				(1U << 6)
#define SPI_SR_MODF   				(1U << 5)
#define SPI_SR_CRCERR 				(1U << 4)
#define SPI_SR_UDR    				(1U << 3)
#define SPI_SR_CHSIDE 				(1U << 2)
#define SPI_SR_TXE    				(1U << 1)
#define SPI_SR_RXNE   				(1U << 0)

// NOTE: --- Bit position definitions SPI_CR1 ---

#define SPI_CR1_BIDIMODE			15
#define SPI_CR1_BIDIOE				14
#define SPI_CR1_CRCEN				13
#define SPI_CR1_CRCNEXT				12
#define SPI_CR1_DFF				11
#define SPI_CR1_RXONLY				10
#define SPI_CR1_SSM				9
#define SPI_CR1_SSI				8
#define SPI_CR1_LSB_FIRST			7
#define SPI_CR1_SPE				6
#define SPI_CR1_BR				3
#define SPI_CR1_MSTR				2
#define SPI_CR1_CPOL				1
#define SPI_CR1_CPHA				0

// NOTE: --- Bit position definitions SPI_CR2 ---

#define SPI_CR2_TXEIE				7
#define SPI_CR2_RXNEIE				6
#define SPI_CR2_ERRIE				5
#define SPI_CR2_FRF				4
#define SPI_CR2_SSOE				2
#define SPI_CR2_TXDMAEN				1
#define SPI_CR2_RXDMAEN				0

// Peripheral clock setup
SPI_status_t SPI_peri_clk_control(SPI_TypeDef* p_SPI_x, uint8_t EN_or_DI);

// Init / de-init
SPI_status_t SPI_init(SPI_Handle_t *p_SPI_handle);
SPI_status_t SPI_de_init(SPI_TypeDef* p_SPI_x);

// Data send / receive ( polling / interrupt ) 
SPI_status_t SPI_write_data_pl(SPI_Handle_t* p_SPI_handle, const uint8_t* p_tx_buffer, uint32_t len);
SPI_status_t SPI_read_data_pl(SPI_Handle_t* p_SPI_handle, uint8_t* p_rx_buffer, uint32_t len);
SPI_status_t SPI_write_data_it(SPI_Handle_t *p_SPI_handle,uint8_t *p_tx_buffer, uint32_t len);
SPI_status_t SPI_read_data_it(SPI_Handle_t *p_SPI_handle, uint8_t *p_rx_buffer, uint32_t len);

// Peripheral control API
SPI_status_t SPI_peri_control(SPI_TypeDef* p_SPI_x, uint8_t EN_or_DI);

// IRQ configuration and ISR handling
SPI_status_t SPI_irq_interrupt_config(uint8_t irq_n, uint8_t EN_or_DI);
SPI_status_t SPI_irq_priority_config(uint8_t irq_n, uint32_t irq_prio);
void SPI_irq_handling(SPI_Handle_t *p_SPI_handle);

#endif
