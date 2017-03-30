/**
 * \file main.c
 *
 * \brief Code initializes execution here.
 *
 */
#include <asf.h>
#include <stdio.h>
#include "main.h"

int main (void) {

	// System Init
	sysclk_init();
	initialize_board();
	
	usart_write_line(&AVR32_USART1, "Hello, this is the AVR UC3 MCU saying hello!\r\n");
	println_usb_debug("Initialized Board.");
	
	//this
	char system_clock[10];
	sprintf(system_clock, "%d", sysclk_get_main_hz());
	print_usb_debug("The system clock is: ");
	println_usb_debug(system_clock);
	
	gpio_clr_gpio_pin(BLUE_LED_PIN);
	gpio_set_gpio_pin(RED_LED_PIN);
	
int i = 0;
	// Main loop
	while(1) {
		gpio_tgl_gpio_pin(BLUE_LED_PIN);
		gpio_tgl_gpio_pin(RED_LED_PIN);
		print_usb_debug(i);
		i++
		delay_ms(500);
	}
}
