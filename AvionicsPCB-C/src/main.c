/**
 * \file main.c
 *
 * \brief Code initializes execution here.
 *
 */
#include <asf.h>
#include <stdio.h>
#include "main.h"
int result;
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
	
	//gpio_clr_gpio_pin(BLUE_LED_PIN);
	gpio_set_gpio_pin(RED_LED_PIN);
	
	/*uint8_t data_received;
	twi_package_t packet_read = {
		.addr         = WHO_AM_I_ADDR,      // TWI slave memory address data
		.addr_length  = 1,    // TWI slave memory address data size
		.chip         = IMU_I2C_ADDR,      // TWI slave bus address
		.buffer       = ((void *) (&data_received)),       // transfer data destination buffer
		.length		  = 1           // transfer data size (bytes)
	};
	
	// Perform a multi-byte read access then check the result.
	twi_master_read(&AVR32_TWI, &packet_read);*/
	
	//Testing I2C for altimeter -- Read Microcontroller datasheet, TWI chapter (User interface section), to find out what this means.
	(AVR32_TWI).iadr = WHO_AM_I_ADDR;
	(AVR32_TWI).cwgr = (246 << 8) | 246; //period of (1/96000) for low clock, same for high, should be 48000 Hz clock for I2C
	(AVR32_TWI).mmr = (IMU_I2C_ADDR << 16) | 0x1100; //0001 0001 0000 0000
	(AVR32_TWI).cr = 0x27;//0000 0000 0000 0000 0000 0000 0010 0111
	
	result = (AVR32_TWI).rhr;
	
	(AVR32_TWI).iadr = 0xF8;
	(AVR32_TWI).cr = 0x27;
	result = (AVR32_TWI).rhr;
	
	(AVR32_TWI).iadr = 0xF7;
	(AVR32_TWI).cr = 0x27;
	result = (AVR32_TWI).rhr;
	
int i = 0;
	// Main loop
	while(1) {
		//gpio_tgl_gpio_pin(BLUE_LED_PIN);
		gpio_tgl_gpio_pin(RED_LED_PIN);
		print_usb_debug(i);
		
		if(i % 2 == 0) {
			//pwm_start_channels(1 << BUZZER_PWM);
			pwm_start_channels(1 << BLUE_LED_PWM);
		} else {
			//pwm_stop_channels(1 << BUZZER_PWM);
			pwm_stop_channels(1 << BLUE_LED_PWM);
		}
			
		i++;
		delay_ms(500);
	}
}
