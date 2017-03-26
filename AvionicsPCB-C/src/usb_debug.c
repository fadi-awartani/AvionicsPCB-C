/*
 * usb_debug.c
 *
 * Created: Mar 26, 2017 5:15:54 PM
 *  Author: Fadi
 */ 
#include <asf.h>
#include "main.h"

//Prints message through USB (as a virtual COM Port)
void print_usb_debug(char* message) {
	while (*message != '\0') {
		// Transfer char to CDC TX
		if (!udi_cdc_is_tx_ready()) {
			// FIFO full
			udi_cdc_signal_overrun();
			} else {
			udi_cdc_putc(*message);
		}
		message++;
	}
}

void println_usb_debug(char* message) {
	print_usb_debug(message);
	print_usb_debug(ENDCHARS);
}