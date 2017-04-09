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
	
	//this is another change
	char system_clock[10];
	sprintf(system_clock, "%d", sysclk_get_main_hz());
	print_usb_debug("The system clock is: ");
	println_usb_debug(system_clock);
	
	gpio_clr_gpio_pin(BLUE_LED_PIN);
	gpio_set_gpio_pin(RED_LED_PIN);
	
	uint8_t data_received;
	twi_package_t packet_read = {
		.addr         = BMP_DEVICE_ID_REG,      // TWI slave memory address data
		.addr_length  = 1,    // TWI slave memory address data size
		.chip         = ALTIMETER_I2C_ADDR,      // TWI slave bus address
		.buffer       = data_received,        // transfer data destination buffer
		.length       = 1                    // transfer data size (bytes)
	};
	// Perform a multi-byte read access then check the result.
	if(twi_master_read(&AVR32_TWI, &packet_read) == TWI_SUCCESS){
		gpio_tgl_gpio_pin(BLUE_LED_PIN);
		delay_ms(50);
		gpio_tgl_gpio_pin(BLUE_LED_PIN);
		delay_ms(50);
		gpio_tgl_gpio_pin(BLUE_LED_PIN);
		delay_ms(50);
	}
	
int i = 0;
	// Main loop
	while(1) {
		gpio_tgl_gpio_pin(BLUE_LED_PIN);
		gpio_tgl_gpio_pin(RED_LED_PIN);
		print_usb_debug(i);
		i++;
		delay_ms(500);
	}
}
