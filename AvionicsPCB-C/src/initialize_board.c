/**
 * initialize_board.c
 *
 * Created: Mar 26, 2017 5:03:53 PM
 *  Author: Fadi
 */ 
#include <asf.h>
#include "main.h"

// GPIO Module init
void init_gpio() {
	/*
		--PORT A--
		GPER: -32- 0011 0000 0111 0010
			       0000 0000 0000 0000 -0- == 0x30720000
			-Enables GPIO function for corresponding pin.
				   
		PMR0: -32- 0000 1100 0000 0000
				   0010 0000 0000 0000 -0- == 0x0C002000
				   
		PMR1: -32- 0000 0000 0000 0000
				   0000 0001 1110 0000 -0- == 0x1E0
			- [PMR1,PMR0] selects pin function if not GPIO. (00 = Function A, 01 = Function B, etc...)
				   
		--PORT B--
		GPER: -32- 0000 0000 0000 0000
				   0000 0011 1100 1111 -0- == 0x3CF
				   
		PMR0: -32- 0000 0000 0000 0000
				   0000 0000 0000 0000 -0- == 0x0
				   		
		PMR1: -32- 0000 0000 0000 0000
				   0000 1100 0000 0000 -0- == 0xC00
	*/
	volatile avr32_gpio_port_t *gpio_portA = &AVR32_GPIO.port[0];
	volatile avr32_gpio_port_t *gpio_portB = &AVR32_GPIO.port[1];
	
	gpio_portA->gper = 0x30720000UL;
	gpio_portA->pmr0 = 0x0C002000UL;
	gpio_portA->pmr1 = 0x1E0;
	
	gpio_portB->gper = 0x3CF;
	gpio_portB->pmr0 = 0;
	gpio_portB->pmr1 = 0xC00;
}

// USART init
void init_usarts() {

	//RFD900
	static const usart_options_t USART0_OPTIONS =
	{
		.baudrate     = 19200,
		.charlength   = 8,
		.paritytype   = USART_NO_PARITY,
		.stopbits     = USART_1_STOPBIT,
		.channelmode  = USART_NORMAL_CHMODE
	};

	//Iridium
	static const usart_options_t USART1_OPTIONS =
	{
		.baudrate     = 19200,//38400,
		.charlength   = 8,
		.paritytype   = USART_NO_PARITY,
		.stopbits     = USART_1_STOPBIT,
		.channelmode  = USART_NORMAL_CHMODE
	};

	//GPS
	static const usart_options_t USART2_OPTIONS =
	{
		.baudrate     = 9600,
		.charlength   = 8,
		.paritytype   = USART_NO_PARITY,
		.stopbits     = USART_1_STOPBIT,
		.channelmode  = USART_NORMAL_CHMODE
	};
	
	usart_init_rs232(&AVR32_USART0, &USART0_OPTIONS, 24000000);
	usart_init_rs232(&AVR32_USART1, &USART1_OPTIONS, 24000000);
	usart_init_rs232(&AVR32_USART2, &USART2_OPTIONS, 24000000);
}

// I2C init
void init_i2c(void)
{
	twi_master_options_t opt = {
		.speed = I2C_SPEED,
		.chip  = ALTIMETER_I2C_ADDR
	};
	twi_master_setup(&AVR32_TWI, &opt);
}

// Board init
void initialize_board() {
	init_gpio();
	init_usarts();
	init_i2c();
	
	// Initialize and enable interrupts
	irq_initialize_vectors();
	cpu_irq_enable();
	
	// Start USB stack to authorize VBus monitoring
	udc_start();
}
