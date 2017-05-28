/**
 * \file main.c
 *
 * \brief Code initializes execution here.
 *
 */
#include "main.h"

int main (void) {
	// System Init
	sysclk_init();
	initialize_board();
	
	usart_write_line(&AVR32_USART1, "Hello, this is the AVR UC3 MCU saying hello!\r\n");
	usart_write_line(&AVR32_USART0, "Hello, this is the AVR UC3 MCU saying hello!\r\n");
	usart_write_line(&AVR32_USART2, "Hello, this is the AVR UC3 MCU saying hello!\r\n");
	println_usb_debug("Initialized Board.");
	
	//gpio_clr_gpio_pin(BLUE_LED_PIN);
	gpio_set_gpio_pin(RED_LED_PIN);
	
int i = 0;
	// Main loop
	while(1) {
		prepare_gps_data();
		
		if(isDataReady()) {
			sprintf(gen_string, "GPS is at %f,%f at time %ld\r\n", gpsLat(), gpsLong(), gpsTime());
			usart_write_line(&AVR32_USART1, gen_string);
		}
		
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
		
		usart_write_line(&AVR32_USART0, "hihihii!^%^#*\r\n");
		
		/*uint8_t data1, data2;
		if(!read_i2c_bytes(ALTIMETER_I2C_ADDR, BMP_DEVICE_ID_REG, &data1, 1) &&
			!read_i2c_bytes(IMU_I2C_ADDR, WHO_AM_I_ADDR, &data2, 1)) {
			sprintf(gen_string, "BMP device id is: 0x%x, IMU \"who am i\" is: 0x%x.\r\n", data1, data2);
			usart_write_line(&AVR32_USART1, gen_string);
		} else
			usart_write_line(&AVR32_USART1, "There was an I2C error.\n");*/
		
		i++;
		update_watchdog();
		delay_ms(500);
	}
}
