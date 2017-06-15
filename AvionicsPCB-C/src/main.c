/**
 * \file main.c
 *
 * \brief Code initializes execution here.
 *
 */
#include "main.h"
//#define DISABLE_MILLIS
static char is_sd_initialized = 0;

unsigned long tt = 0;
int main (void) {
	// System Init
	sysclk_init();
	sysclk_enable_peripheral_clock(&AVR32_TC);
	initialize_board();
	
	usart_write_line(&AVR32_USART0, "\n\nHello, this is the AVR UC3 MCU saying hello!\r\n");
	usart_write_line(&AVR32_USART1, "\n\nHello, this is the AVR UC3 MCU saying hello!\r\n");
	usart_write_line(&AVR32_USART2, "Hello, this is the AVR UC3 MCU saying hello!\r\n");
	
#ifdef EN_USB
	println_usb_debug("Initialized Board.");
#endif

	//gpio_clr_gpio_pin(BLUE_LED_PIN);
	gpio_set_gpio_pin(RED_LED_PIN);
	
int i = 0;
	// Main loop
	while(1) {
		
		#ifndef DISABLE_MILLIS
		if(millis() > tt + 5) {
			tt = millis();
		} else
		{
			continue;
		}
		#endif
		
		prepare_gps_data();
		
		if(isDataReady()) {
			gps_coordinates_t coords = getGPSCoordinates();
			sprintf(gen_string, "GPS is at %f,%f, %d m high, at time %ld\r\ngoing %f kph, %f°, at time %lf",
				coords.lat, 
				coords.lon,
				coords.alt,
				coords.time,
				coords.speed_kph,
				coords.track_degrees,
				coords.speed_time);
			usart_write_line(&RFD_USART, gen_string);
			
			#ifdef EN_USB
			println_usb_debug(gen_string);
			#endif
		}
		
		//gpio_tgl_gpio_pin(BLUE_LED_PIN);
		if(i % 100 == 0)
		gpio_tgl_gpio_pin(RED_LED_PIN);
		
		#ifdef EN_USB
		print_usb_debug(i);
		#endif
		
		if(i % 200 == 0) {
			//pwm_start_channels(1 << BUZZER_PWM);
			pwm_start_channels(1 << BLUE_LED_PWM);
		} else if(i % 100 == 0){
			//pwm_stop_channels(1 << BUZZER_PWM);
			pwm_stop_channels(1 << BLUE_LED_PWM);
		}
		
		//usart_write_line(&AVR32_USART0, "hihihii!^%^#*\r\n");
		
		/*uint8_t data1, data2;
		if(!read_i2c_bytes(ALTIMETER_I2C_ADDR, BMP_DEVICE_ID_REG, &data1, 1) &&
			!read_i2c_bytes(IMU_I2C_ADDR, WHO_AM_I_ADDR, &data2, 1)) {
			sprintf(gen_string, "BMP device id is: 0x%x, IMU \"who am i\" is: 0x%x.\r\n", data1, data2);
			//usart_write_line(&AVR32_USART0, gen_string);
			
		} else
			;//usart_write_line(&AVR32_USART0, "There was an I2C error.\n");*/
		
		if(gpio_pin_is_high(SD_DETECT_PIN)) {
			if(!is_sd_initialized) {
				sd_pdca_init();
			}
		}
		
		if(is_sd_initialized) {
			//Log data
		}
		
		#ifndef DISABLE_BMP
		altimeter_data_t alt_data = readAltimeter();
		sprintf(gen_string, "Pressure: %lf Pa, Temp: %lf C, Time: %d\r\n", alt_data.pres, alt_data.temp, alt_data.time);
		usart_write_line(&AVR32_USART0, gen_string);
		#endif
		
		#ifdef EN_USB
		println_usb_debug("Testing USB!!");
		#endif
		
		i++;
		update_watchdog();
		#ifdef DISABLE_MILLIS
		delay_ms(500);
		#endif
	}
}
