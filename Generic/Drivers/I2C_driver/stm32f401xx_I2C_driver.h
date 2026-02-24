#ifndef STM32F401RE_I2C_DRIVER_H
#define STM32F401RE_I2C_DRIVER_H

#include "stm32f401xx.h"

// NOTE: --- Structures for I2C ---

// Configuration structure for a SPI

// typedef struct
// {
// 	uint8_t SPI_Device_Mode;					// Possible values from @SPI_DEVICE_MODE
// 	uint8_t SPI_Bus_Config;						// Possible values from @SPI_BUS_CONFIG
// 	uint8_t SPI_SCLK_Speed;						// Possible values from @SPI_SCLK_SPEED
// 	uint8_t SPI_DFF;						// Possible values from @SPI_DFF
// 	uint8_t SPI_CPOL;						// Possible values from @SPI_CPOL
// 	uint8_t SPI_CPHA;						// Possible values from @SPI_CPHA
// 	uint8_t SPI_SSM;						// Possible values from @SPI_SSM
// } SPI_Config_t;

// Handle structure for a GPIO pin

// typedef struct
// {
// 	SPI_TypeDef* p_SPIx;						// Holds the base address of the SPI peripheral 
// 	SPI_Config_t SPI_Config;					// Holds SPI peripheral configuration settings
// } SPI_Handle_t;

// Function return status

// typedef enum {
// 	SPI_OK = 0,							// Success
// 	SPI_ERROR_INVALID_STATE,					// Invalid state of a argument
// 	SPI_ERROR_NULL_PTR,						// NULL pointer passed
// 	SPI_ERROR_INVALID_PORT,						// Invalid SPI port address
// 	SPI_ERROR_INVALID_IRQ,						// Invalid IRQ number
// 	SPI_ERROR_INVALID_MODE,						// Mode value out of range
// 	SPI_ERROR_INVALID_BUS_CONFIG,					// Bus config value out of range
// 	SPI_ERROR_INVALID_SCLK_SPEED,					// SCLK speed value out of range
// 	SPI_ERROR_INVALID_DFF,						// DFF value out of range
// 	SPI_ERROR_INVALID_CPOL,						// CPOL value out of range
// 	SPI_ERROR_INVALID_CPHA,						// CPHA value out of range
// 	SPI_ERROR_INVALID_SSM,						// SSM value out of range
// 	SPI_ERROR_TIMEOUT,						// SPI polling timeout
// 	SPI_BUSY							// SPI Tx / Rx busy
// } SPI_status_t;

// NOTE: --- SPI Validation macros ---

// #define VALIDATE_SPI_PORT(port)		do { \
// 							if ((port) == NULL || \
// 							!((port) == SPI1 || (port) == SPI2 || (port) == SPI3 || (port) == SPI4)) { \
// 								return SPI_ERROR_INVALID_PORT; \
// 							} \
// 						} while(0)

// #define VALIDATE_SPI_DEVICE_MODE(mode)		VALIDATE_ENUM((mode), SPI_DEVICE_MODE_MASTER, SPI_ERROR_INVALID_MODE)
// #define VALIDATE_SPI_BUS_CONFIG(stop)		VALIDATE_ENUM((stop), SPI_BUS_CONFIG_FD, SPI_ERROR_INVALID_BUS_CONFIG)
// #define VALIDATE_SPI_SCLK_SPEED(wlen)		VALIDATE_ENUM((wlen), SPI_SCLK_SPEED_DIV_2, SPI_ERROR_INVALID_SCLK_SPEED)
// #define VALIDATE_SPI_DFF(parity)		VALIDATE_ENUM((parity), SPI_DFF_8BIT, SPI_ERROR_INVALID_DFF)
// #define VALIDATE_SPI_CPOL(flow)			VALIDATE_ENUM((flow), SPI_CPOL_HIGH, SPI_ERROR_INVALID_CPOL)
// #define VALIDATE_SPI_CPHA(flow)			VALIDATE_ENUM((flow), SPI_CPHA_HIGH, SPI_ERROR_INVALID_CPHA)
// #define VALIDATE_SPI_SSM(flow)			VALIDATE_ENUM((flow), SPI_SSM_HW, SPI_ERROR_INVALID_SSM)

// NOTE: --- Bit position definitions I2C_SR1 ---

#define I2C_SR1_SMB_ALERT			15
#define I2C_SR1_TIMEOUT				14
#define I2C_SR1_PEC_ERR				12
#define I2C_SR1_OVR				11
#define I2C_SR1_AF				10
#define I2C_SR1_ARLO				9
#define I2C_SR1_BERR				8
#define I2C_SR1_TxE				7
#define I2C_SR1_RxNE				6
#define I2C_SR1_STOPF				4
#define I2C_SR1_ADD10				3
#define I2C_SR1_BTF				2
#define I2C_SR1_ADDR				1
#define I2C_SR1_SB				0

// NOTE: --- Bit position definitions I2C_SR2 ---

#define I2C_SR2_PEC				8
#define I2C_SR2_DUALF				7
#define I2C_SR2_SMB_HOST			6
#define I2C_SR2_SMBDE_FAULT			5
#define I2C_SR2_GENCALL				4
#define I2C_SR2_TRA				2
#define I2C_SR2_BUSY				1
#define I2C_SR2_MSL				0

// NOTE: --- Bit position definitions I2C_CR1 ---

#define I2C_CR1_SWRST				15
#define I2C_CR1_ALERT				13
#define I2C_CR1_PEC				12
#define I2C_CR1_POS				11
#define I2C_CR1_ACK				10
#define I2C_CR1_STOP				9
#define I2C_CR1_START				8
#define I2C_CR1_NO_STRETCH			7
#define I2C_CR1_ENGC				6
#define I2C_CR1_ENPEC				5
#define I2C_CR1_ENARP				4
#define I2C_CR1_SMB_TYPE			3
#define I2C_CR1_SMBUS				1
#define I2C_CR1_PE				0

// NOTE: --- Bit position definitions I2C_CR2 ---

#define I2C_CR2_LAST				12
#define I2C_CR2_DMAEN				11
#define I2C_CR2_ITBUFEN				10
#define I2C_CR2_ITEVTEN				9
#define I2C_CR2_ITERREN				8
#define I2C_CR2_FREQ				0

// Peripheral clock setup
// SPI_status_t SPI_peri_clk_control(SPI_TypeDef* p_SPI_x, uint8_t EN_or_DI);

// Init / de-init
// SPI_status_t SPI_init(SPI_Handle_t *p_SPI_handle);
// SPI_status_t SPI_de_init(SPI_TypeDef* p_SPI_x);

// Data send / receive ( polling / interrupt ) 
// SPI_status_t SPI_write_data_pl(SPI_Handle_t* p_SPI_handle, const uint8_t* p_tx_buffer, uint32_t len);
// SPI_status_t SPI_read_data_pl(SPI_Handle_t* p_SPI_handle, uint8_t* p_rx_buffer, uint32_t len);
// SPI_status_t SPI_write_data_it(SPI_Handle_t *p_SPI_handle,uint8_t *p_tx_buffer, uint32_t len);
// SPI_status_t SPI_read_data_it(SPI_Handle_t *p_SPI_handle, uint8_t *p_rx_buffer, uint32_t len);

// Peripheral control API

// IRQ configuration and ISR handling
// SPI_status_t SPI_irq_interrupt_config(uint8_t irq_n, uint8_t EN_or_DI);
// SPI_status_t SPI_irq_priority_config(uint8_t irq_n, uint32_t irq_prio);
// void SPI_irq_handling(SPI_Handle_t *p_SPI_handle);

#endif
