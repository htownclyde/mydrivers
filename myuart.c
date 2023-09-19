// myuart.c
// Bidirectional busy-wait UART driver for STM32
// Clyde Johnson, 2023

#include "stdint.h"
#include "stm32f4xx_hal.h"
//#include "stm32f4xx.h"
//#include "stm32f4xx_hal.h"

#define sr_offset 0x00
#define dr_offset 0x04
#define brr_offset 0x08
#define cr1_offset 0x0C
#define cr2_offset 0x10
#define cr3_offset 0x14
#define base_address 0x40011000

volatile uint32_t *usart_address = (uint32_t *) base_address;
volatile uint32_t *sr = (uint32_t *) (base_address + sr_offset);
volatile uint32_t *dr = (uint32_t *) (base_address + dr_offset);
volatile uint32_t *brr = (uint32_t *) (base_address + brr_offset);
volatile uint32_t *cr1 = (uint32_t *) (base_address + cr1_offset);
volatile uint32_t *cr2 = (uint32_t *) (base_address + cr2_offset);
volatile uint32_t *cr3 = (uint32_t *) (base_address + cr3_offset);
uint64_t baud = 9600;
uint8_t stop_bits = 1;
uint8_t parity = 0;
uint64_t clock_speed = 16000000;
//float usart_div = 16000000/(16*9600);

// TODO: Create defs for bitmasking each register
void uart_init_stm32(uint32_t *usart_address, uint32_t baud, uint8_t stop_bits, uint8_t parity, uint32_t clock_speed){
	sr = (uint32_t)usart_address + sr_offset;
	dr = (uint32_t)usart_address + dr_offset;
	brr = (uint32_t)usart_address + brr_offset;
	cr1 = (uint32_t)usart_address + cr1_offset;
	cr2 = (uint32_t)usart_address + cr2_offset;
	cr3 = (uint32_t)usart_address + cr3_offset;
	*cr1 = 0x00;	  			// Clear CR1
	*cr1 |= (1U<<13); 			// UE: USART enable
	*cr1 |= (1U<<7);  			// TXEIE: TXE interrupt enable
	*cr1 |= (1U<<6);  			// TCIE: Transmission complete interrupt enable
	*cr2 &= ~(1U<<12|1<<13); 	// STOP: STOP bits
	*cr3 &= ~(1U<<3); 			// HDSEL: Half-duplex selection
	*cr1 |= (1U<<3);			// TE: Transmitter enable
	*cr1 |= (1U<<2);			// RE: Receiver enable
	//usart_div = (int)(clock_speed/(16*baud));
	//brr = ((uint8_t) usart_div<<0)|((uint8_t) usart_div&0x07FFFFF<<4);
	*brr = (int)(clock_speed/baud);
}

void uart_setbaud(uint32_t baud){

}

void uart_sendchar(char data_char){
	*dr = data_char;
	//*cr1 |= (1U<<0); // SBK: Send Break
	while(!(*sr & (1<<6))){ // TC: Wait for Transmission Complete
		osDelay(1);
	}
}

uint8_t uart_receivechar(){
	uint8_t c;
	while(!(*sr & (1<<5))){ // RXNE: Wait for Read data register not empty
		osDelay(1);
	}
	c = *dr;
	return c;
}
