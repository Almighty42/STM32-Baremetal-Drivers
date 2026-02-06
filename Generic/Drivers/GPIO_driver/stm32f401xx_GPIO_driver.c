#include "stm32f401xx_GPIO_driver.h"
#include <stdint.h>

/********************************************************************************
 * @fn			- GPIO_peri_clk_control
 *
 * @brief			- Enables or disables peripheral clock for a
 * given GPIO port
 *
 * @param[*p_GPIOx]		- Base address of the GPIO peripheral
 * @param[EN_or_DI]		- ENABLE or DISABLE macros
 *
 * @return			- None
 *
 * @Note			- None
 */

void GPIO_peri_clk_control(GPIO_TypeDef* p_GPIOx, uint8_t EN_or_DI)
{
	if (EN_or_DI == ENABLE) {
		if (p_GPIOx == GPIOA) {
			GPIOA_PCLK_EN();
		}
		else if (p_GPIOx == GPIOB) {
			GPIOB_PCLK_EN();
		}
		else if (p_GPIOx == GPIOC) {
			GPIOC_PCLK_EN();
		}
		else if (p_GPIOx == GPIOD) {
			GPIOD_PCLK_EN();
		}
		else if (p_GPIOx == GPIOE) {
			GPIOE_PCLK_EN();
		}
		else if (p_GPIOx == GPIOH) {
			GPIOH_PCLK_EN();
		}
	}
	else {
		if (p_GPIOx == GPIOA) {
			GPIOA_PCLK_DI();
		}
		else if (p_GPIOx == GPIOB) {
			GPIOB_PCLK_DI();
		}
		else if (p_GPIOx == GPIOC) {
			GPIOC_PCLK_DI();
		}
		else if (p_GPIOx == GPIOD) {
			GPIOD_PCLK_DI();
		}
		else if (p_GPIOx == GPIOE) {
			GPIOE_PCLK_DI();
		}
		else if (p_GPIOx == GPIOH) {
			GPIOH_PCLK_DI();
		}
	}
}

/********************************************************************************
 * @fn			- GPIO_init
 *
 * @brief			- Initializes a GPIO pin
 *
 * @param[*p_GPIO_handle]	- Handle structure of a GPIO pin
 *
 * @return			- None
 *
 * @Note			- None
 */
void GPIO_init(GPIO_Handle_t* p_GPIO_handle)
{
	uint32_t temp = 0;
	uint32_t GPIO_pin_offset_2 =
	    (2 * p_GPIO_handle->GPIO_Pin_Config.GPIO_Pin_Number);
	uint32_t GPIO_pin_offset_1 =
	    p_GPIO_handle->GPIO_Pin_Config.GPIO_Pin_Number;
	// Mode
	if (p_GPIO_handle->GPIO_Pin_Config.GPIO_Pin_Mode <= GPIO_MODE_ANALOG) {
		temp = (p_GPIO_handle->GPIO_Pin_Config.GPIO_Pin_Mode
		        << GPIO_pin_offset_2);
		CLEAR_FIELD_2BIT(p_GPIO_handle->p_GPIOx->MODER,
		                 GPIO_pin_offset_2);
		SET_BITS_BY_VAR(p_GPIO_handle->p_GPIOx->MODER, temp);
	}
	else {
		if (p_GPIO_handle->GPIO_Pin_Config.GPIO_Pin_Mode ==
		    GPIO_MODE_IT_FT) {
			SET_BIT(EXTI->FTSR, GPIO_pin_offset_1);
			CLEAR_BIT(EXTI->RTSR, GPIO_pin_offset_1);
		}
		else if (p_GPIO_handle->GPIO_Pin_Config.GPIO_Pin_Mode ==
		         GPIO_MODE_IT_RT) {
			SET_BIT(EXTI->RTSR, GPIO_pin_offset_1);
			CLEAR_BIT(EXTI->FTSR, GPIO_pin_offset_1);
		}
		else if (p_GPIO_handle->GPIO_Pin_Config.GPIO_Pin_Mode ==
		         GPIO_MODE_IT_RFT) {
			SET_BIT(EXTI->FTSR, GPIO_pin_offset_1);
			SET_BIT(EXTI->RTSR, GPIO_pin_offset_1);
		}

		uint8_t temp_1 =
		    p_GPIO_handle->GPIO_Pin_Config.GPIO_Pin_Number / 4;
		uint8_t temp_2 =
		    p_GPIO_handle->GPIO_Pin_Config.GPIO_Pin_Number % 4;
		uint8_t portcode =
		    GPIO_BASE_ADDR_TO_CODE(p_GPIO_handle->p_GPIOx);

		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp_1] = portcode << (temp_2 * 4);

		SET_BIT(EXTI->IMR, GPIO_pin_offset_1);
	}
	temp = 0;

	// Speed
	temp = p_GPIO_handle->GPIO_Pin_Config.GPIO_Pin_Speed
	       << GPIO_pin_offset_2;
	CLEAR_FIELD_2BIT(p_GPIO_handle->p_GPIOx->OSPEEDR, GPIO_pin_offset_2);
	SET_BITS_BY_VAR(p_GPIO_handle->p_GPIOx->OSPEEDR, temp);
	temp = 0;

	// PuPd
	temp = p_GPIO_handle->GPIO_Pin_Config.GPIO_Pin_PuPd_Control
	       << GPIO_pin_offset_2;
	CLEAR_FIELD_2BIT(p_GPIO_handle->p_GPIOx->PUPDR, GPIO_pin_offset_2);
	SET_BITS_BY_VAR(p_GPIO_handle->p_GPIOx->PUPDR, temp);
	temp = 0;

	// OTP type
	temp = p_GPIO_handle->GPIO_Pin_Config.GPIO_Pin_OTP_Type
	       << GPIO_pin_offset_1;
	CLEAR_BIT(p_GPIO_handle->p_GPIOx->OTYPER, GPIO_pin_offset_1);
	SET_BITS_BY_VAR(p_GPIO_handle->p_GPIOx->OTYPER, temp);
	temp = 0;

	// Alt func
	if (p_GPIO_handle->GPIO_Pin_Config.GPIO_Pin_Alt_Fun_Mode ==
	    GPIO_MODE_ALT) {
		uint8_t temp_offset =
		    p_GPIO_handle->GPIO_Pin_Config.GPIO_Pin_Number % 8;
		temp = p_GPIO_handle->GPIO_Pin_Config.GPIO_Pin_Alt_Fun_Mode
		       << (4 * temp_offset);
		if (p_GPIO_handle->GPIO_Pin_Config.GPIO_Pin_Number / 8 == 0) {
			CLEAR_FIELD_4BIT(p_GPIO_handle->p_GPIOx->AFRL,
			                 temp_offset);
			SET_BITS_BY_VAR(p_GPIO_handle->p_GPIOx->AFRL, temp);
		}
		else {
			CLEAR_FIELD_4BIT(p_GPIO_handle->p_GPIOx->AFRH,
			                 temp_offset);
			SET_BITS_BY_VAR(p_GPIO_handle->p_GPIOx->AFRH, temp);
		}
		temp = 0;
	}
}

/********************************************************************************
 * @fn			- GPIO_deinit
 *
 * @brief			- Resets a GPIO pin
 *
 * @param[*p_GPIO_handle]	- Handle structure of a GPIO pin
 *
 * @return			- None
 *
 * @Note			- None
 */
void GPIO_de_init(GPIO_TypeDef* p_GPIOx)
{
	if (p_GPIOx == GPIOA) {
		GPIOA_REG_RESET();
	}
	else if (p_GPIOx == GPIOB) {
		GPIOB_REG_RESET();
	}
	else if (p_GPIOx == GPIOC) {
		GPIOC_REG_RESET();
	}
	else if (p_GPIOx == GPIOD) {
		GPIOD_REG_RESET();
	}
	else if (p_GPIOx == GPIOE) {
		GPIOE_REG_RESET();
	}
	else if (p_GPIOx == GPIOH) {
		GPIOH_REG_RESET();
	}
}

/********************************************************************************
 * @fn			- GPIO_read_input_pin
 *
 * @brief			- Returns value of a GPIO input pin
 *
 * @param[*p_GPIOx]		- GPIO pin register structure
 * @param[pin_n]		- Pin number
 *
 * @return			- Value of pin
 *
 * @Note			- None
 */
uint8_t GPIO_read_input_pin(GPIO_TypeDef* p_GPIOx, uint8_t pin_n)
{
	return (uint8_t)((p_GPIOx->IDR >> pin_n) & 0x00000001);
}

/********************************************************************************
 * @fn			- GPIO_read_input_port
 *
 * @brief			- Returns value of a GPIO input port
 *
 * @param[*p_GPIOx]		- GPIO pin register structure
 *
 * @return			- Value of port
 *
 * @Note			- None
 */
uint16_t GPIO_read_input_port(GPIO_TypeDef* p_GPIOx)
{
	return (uint16_t)p_GPIOx->IDR;
}

/********************************************************************************
 * @fn			- GPIO_write_output_pin
 *
 * @brief			- Writes to a GPIO output pin
 *
 * @param[*p_GPIOx]		- GPIO pin register structure
 * @param[pin_n]		- Pin number
 * @param[val]		- Value to write
 *
 * @return			- None
 *
 * @Note			- None
 */
void GPIO_write_output_pin(GPIO_TypeDef* p_GPIOx, uint8_t pin_n, uint8_t val)
{
	if (val == GPIO_PIN_SET)
		SET_BIT(p_GPIOx->ODR, pin_n);
	else
		CLEAR_BIT(p_GPIOx->ODR, pin_n);
}

/********************************************************************************
 * @fn			- GPIO_write_output_port
 *
 * @brief			- Writes to a GPIO output port
 *
 * @param[*p_GPIOx]		- GPIO pin register structure
 * @param[val]		- Value to write
 *
 * @return			- None
 *
 * @Note			- None
 */
void GPIO_write_output_port(GPIO_TypeDef* p_GPIOx, uint8_t val)
{
	p_GPIOx->ODR = val;
}

/********************************************************************************
 * @fn			- GPIO_toggle_output_pin
 *
 * @brief			- Toggles GPIO output pin
 *
 * @param[*p_GPIOx]		- GPIO pin register structure
 * @param[pin_n]		- Pin number
 *
 * @return			- None
 *
 * @Note			- None
 */
void GPIO_toggle_output_pin(GPIO_TypeDef* p_GPIOx, uint8_t pin_n)
{
	TOGGLE_BIT(p_GPIOx->ODR, pin_n);
}

/********************************************************************************
 * @fn			- GPIO_irq_interrupt_config
 *
 * @brief			- Configures GPIO IRQ interrupt
 *
 * @param[irq_n]		- IRQ number
 * @param[EN_or_DI]		- ENABLE or DISABLE macros
 *
 * @return			- None
 *
 * @Note			- None
 */
void GPIO_irq_interrupt_config(uint8_t irq_n, uint8_t EN_or_DI)
{
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
}

/********************************************************************************
 * @fn			- GPIO_irq_priority_config
 *
 * @brief			- Configures GPIO IRQ priority
 *
 * @param[irq_n]		- IRQ number
 * @param[irq_prio]		- IRQ priority
 *
 * @return			- None
 *
 * @Note			- None
 */
void GPIO_irq_priority_config(uint8_t irq_n, uint8_t irq_prio)
{
	uint8_t ipr_x = irq_n / 4;
	uint8_t ipr_x_section = (irq_n % 4) * 8;
	uint32_t prio_field = (irq_prio & ((1U << __NVIC_PRIO_BITS) - 1U))
	                      << (8U - __NVIC_PRIO_BITS);
	uint32_t ipr = NVIC->IPR[ipr_x];
	CLEAR_BYTE(ipr, ipr_x_section);
	ipr |= (prio_field << ipr_x_section);
	NVIC->IPR[ipr_x] = ipr;

	// SET_BIT(NVIC->IPR[irq_n], )
}

/********************************************************************************
 * @fn			- GPIO_irq_handling
 *
 * @brief			- Handles GPIO EXTI interrupts
 *
 * @param[pin_n]		- Pin number
 *
 * @return			- None
 *
 * @Note			- None
 */
void GPIO_irq_handling(uint8_t pin_n)
{
	if (IS_BIT_SET(EXTI->PR, pin_n))
		SET_BIT(EXTI->PR, pin_n);
}
