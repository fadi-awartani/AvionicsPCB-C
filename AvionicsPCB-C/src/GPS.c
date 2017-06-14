/*
 * GPS.c
 *
 * Created: May 21, 2017 5:17:05 PM
 *  Author: Fadi
 */ 
#include "main.h"
#include "nmea/minmea.h"

char gps_line[MINMEA_MAX_LENGTH];
int gps_index;
int dataReady; //booleans

gps_coordinates_t gps_coordinates;
int gps_nsatts = 0;

#if __GNUC__
__attribute__((__interrupt__))
#elif __ICCAVR32__
__interrupt
#endif
static void read_gps_byte(void) {
	Disable_global_interrupt();
	int val;
	char c;
	
	usart_read_char(&GPS_USART, &val);
	
	c = (char) val;
	gps_processChar(c);
	
	Enable_global_interrupt();
	
	GPS_USART.idr = 1;
	GPS_USART.ier = 1;
}

void init_gps() {
	// Disable all interrupts.
	Disable_global_interrupt();
	
	INTC_register_interrupt(&read_gps_byte, GPS_USART_IRQ, AVR32_INTC_INT0);
	
	GPS_USART.ier = AVR32_USART_IER_RXRDY_MASK;
	
	Enable_global_interrupt();
	
	turnOnGPS();
	
	//delay_ms(4);
	
	//Turn on GLL and RMC messages on every fix.
	usart_write_line(&GPS_USART, "$PMTK314,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n");
	
	//Update message rate (and possibly baud rate) to be higher
	//TODO
}

void gps_processChar(char c) {
	#ifdef ECHOGPS
	RFD_USART.thr = c;
	#endif
	
	if(c == '$' && gps_index != 0) {
		gps_index = 0;
	}
	
	gps_line[gps_index++ % (MINMEA_MAX_LENGTH-1)] = c;
	
	if(c == '\n') {
		gps_line[gps_index++ % (MINMEA_MAX_LENGTH-1)] = '\0';
		gps_index = 0;
		prepare_gps_data();
	}
}

int prepare_gps_data() {
		//Check if line is a valid NMEA sentence first. If not, just return.
		if(!minmea_check(gps_line,true)) {
			gps_index = 0;
			return;
		}
		
		switch (minmea_sentence_id(gps_line,true)) {
			case MINMEA_SENTENCE_GGA: {
				struct minmea_sentence_gga frame;
				if (minmea_parse_gga(&frame, gps_line)) {
					gps_coordinates.lat = minmea_tocoord(&(frame.latitude));
					gps_coordinates.lon = minmea_tocoord(&(frame.longitude));
					gps_coordinates.alt = (int) ((frame.altitude.scale/(float)frame.altitude.value) + 0.5);
					gps_coordinates.time = realTime();
					
					gps_nsatts = frame.satellites_tracked;
				}
			} break;
			
			case MINMEA_SENTENCE_RMC: {
				struct minmea_sentence_rmc frame;
				if (minmea_parse_rmc(&frame, gps_line)) {
					gps_coordinates.lat = minmea_tocoord(&(frame.latitude));
					gps_coordinates.lon = minmea_tocoord(&(frame.longitude));
					gps_coordinates.time = minmea_gettime(&(frame.date),&(frame.time));
					//gps_coordinates.alt = 0;
					
					millis_time_linked = millis();
					real_time_linked = gps_coordinates.time;
				}
			} break;
		}
		
		#ifndef DISABLE_VERBOSE
			usart_write_line(&AVR32_USART0, gps_line);
			#ifdef EN_USB
				println_usb_debug(gen_string);
			#endif
		#endif
		
		dataReady = 1;
}

int isDataReady() {
	if(dataReady) {
		dataReady = 0;
		return 1;
	} else
		return 0;
}

gps_coordinates_t getGPSCoordinates() {
	dataReady = 0;
	return gps_coordinates;
}

void turnOnGPS() {
	AVR32_GPIO.port[1].gpers = 1 << 8;
	AVR32_GPIO.port[1].oders = 1 << 8;
	AVR32_GPIO.port[1].ovrs = 1 << 8;
}

void resetGPS() {
	AVR32_GPIO.port[1].ovrc = 1 << 8;
	delay_ms(4);
	AVR32_GPIO.port[1].ovrs = 1 << 8;
}