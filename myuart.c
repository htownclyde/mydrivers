// myuart.c
// Bidirectional busy-wait UART driver for STM32
// Clyde Johnson, 2023

#include "stdint.h"
//#include "stm32f4xx.h"
//#include "stm32f4xx_hal.h"

#define sr_offset 0x00
#define dr_offset 0x04
#define brr_offset 0x08
#define cr1_offset 0x0C
#define cr2_offset 0x10
#define cr3_offset 0x18
#define base_address 0x40011000

volatile uint32_t *usart_address = (volatile uint32_t *) base_address;
volatile uint32_t *sr = (volatile uint32_t *) base_address + sr_offset;
volatile uint32_t *dr = (volatile uint32_t *) base_address + dr_offset;
volatile uint32_t *brr = (volatile uint32_t *) base_address + brr_offset;
volatile uint32_t *cr1 = (volatile uint32_t *) base_address + cr1_offset;
volatile uint32_t *cr2 = (volatile uint32_t *) base_address + cr2_offset;
volatile uint32_t *cr3 = (volatile uint32_t *) base_address + cr3_offset;
uint64_t baud = 115200;
uint8_t stop_bits = 1;
uint8_t parity = 0;
uint64_t clock_speed = 16000000;
//float usart_div = 16000000/(1*115200);

// TODO: Create defs for bitmasking each register
void uart_init_stm32(uint32_t *usart_address, uint64_t baud, uint8_t stop_bits, uint8_t parity, uint64_t clock_speed){
	sr = usart_address + sr_offset;
	dr = usart_address + dr_offset;
	brr = usart_address + brr_offset;
	cr1 = usart_address + cr1_offset;
	cr2 = usart_address + cr2_offset;
	cr3 = usart_address + cr3_offset;
	*cr1 = 0x00;	  			// Clear CR1
	*cr1 = (1<<13); 			// UE: USART enable
	*cr1 |= (1<<7);  			// TXEIE: TXE interrupt enable
	*cr1 |= (1<<6);  			// TCIE: Transmission complete interrupt enable
	*cr2 &= ~(1<<12|1<<13); 	// STOP: STOP bits
	*cr3 &= ~(1<<3); 			// HDSEL: Half-duplex selection
	*cr1 |= (1<<3);				// TE: Transmitter enable
	*cr1 |= (1<<2);				// RE: Receiver enable
	//usart_div = (int)(clock_speed/(16*baud));
	//brr = ((uint8_t) usart_div<<0)|((uint8_t) usart_div&0x07FFFFF<<4);
	*brr = (int)(clock_speed/(16*baud));
}

void uart_setbaud(uint32_t baud){

}

void uart_sendchar(char data_char){
	*dr = data_char;
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
