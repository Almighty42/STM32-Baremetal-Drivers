# USART Driver

USART driver for the STM32F401xx series of microcontrollers, tested on the
STM32F401RE. This driver acts as a simple HAL-style API over the USART
peripheral, allowing easier configuration and control.

TX / RX operations are implemented with multiple options:
polling-based, interrupt-based, and a ring-buffer-based implementation.

### Ring buffer based TX / RX
Provides a convenient way to asynchronously interface with the USART.
The application enqueues and dequeues data in software, while the ISR
shifts data between the peripheral and a fixed-size ring buffer at its
own pace, reducing the risk of data loss during bursts and improving
reliability for logging, CLI shells, or higher-throughput protocols.

### Driver APIs
- USART peripheral initialize / de-initialize functions
- Data TX / RX logic with support for
  - Polling based TX / RX
  - Interrupt based TX / RX
  - Ring-buffer-based TX / RX (using interrupts at the low level)
- IRQ configuration / ISR handling
- Application event callback

### Also included:
- Configuration structures
- Handle structures
- Application states / events
- Return status codes
- Validation macros
- Driver map (for easier navigation of the driver)

## TODOs:
- Add helper functions to longer functions to make the code more readable
- Implement synchronous communication
- Implement DMA USART in future

