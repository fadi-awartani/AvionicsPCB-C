/**
 * \file main.c
 *
 * \brief Code initializes execution here.
 *
 */
#include <asf.h>
#include <stdio.h>
#include "main.h"

int main (void)
{
	/* System Initialization */
	sysclk_init();
	initialize_board();
	
	usart_write_line(&AVR32_USART1, "Hello, this is the AVR UC3 MCU saying hello!\r\n");
	println_usb_debug("Initialized Board.");
	
	char system_clock[10];
	sprintf(system_clock, "%d", sysclk_get_main_hz());
	print_usb_debug("The system clock is: ");
	println_usb_debug(system_clock);
	
	gpio_clr_gpio_pin(BLUE_LED_PIN);
	gpio_set_gpio_pin(RED_LED_PIN);
	
	/* Application Code */
	while(1) {
		gpio_tgl_gpio_pin(BLUE_LED_PIN);
		gpio_tgl_gpio_pin(RED_LED_PIN);
		delay_ms(500);
	}
}
