#include "stm32f401RE_header.h"
#include <stdint.h>

#define USART_SR_TC 6
#define USART_SR_RXNE 5

#define USART_CR1_OVER8 15
#define USART_CR1_UE 13
#define USART_CR1_M 12 
#define USART_CR1_PCE 10
#define USART_CR1_TE 3
#define USART_CR1_RE 2

#define USART_CR2_STOP_1 12 
#define USART_CR2_STOP_2 13 

void USART_Init(USART_Type *USARTx);

int main(void) {
    // INFO: TESTING USART FIRST

    // Enabling GPIO clock and configuring Tx pin and the Rx pin as:
    // Alternate function, high speed, push-pull , pull-up
    // -- GPIO init for USART 6 -- 
    // Enable GPIO port B clock
    // Alternative function 7 for USART 6
    // GPIO Speed
    // GPIO Push pull
    // GPIO output type
}

void USART_Init(USART_Type *USARTx) {
    // Disable USART
    CLEAR_BIT(USARTx -> CR1, USART_CR1_UE);
    // Set data length to 8 bits
    CLEAR_BIT(USARTx -> CR1, USART_CR1_M);
    // Select 1 stop bit
    CLEAR_BIT(USARTx -> CR2, USART_CR2_STOP_1);
    CLEAR_BIT(USARTx -> CR2, USART_CR2_STOP_2);
    // Set parity control as no parity
    CLEAR_BIT(USARTx -> CR1, USART_CR1_PCE);
    // Oversampling by 16
    CLEAR_BIT(USARTx -> CR1, USART_CR1_OVER8);
    // TODO:
    // Set baud rate to 9600 using APB frequency (84MHz)
    //
    // Enable transmission and reception
    SET_BIT(USARTx -> CR1, USART_CR1_RE);
    SET_BIT(USARTx -> CR1, USART_CR1_TE);
    // Enable USART
    SET_BIT(USARTx -> CR1, USART_CR1_UE);
    // Verify that USART is ready for transmission
    while(!(IS_BIT_SET(USARTx -> SR, USART_SR_TC)));
    // Verify that USART is ready for reception
    while(!(IS_BIT_SET(USARTx -> SR, USART_SR_RXNE)));
}
