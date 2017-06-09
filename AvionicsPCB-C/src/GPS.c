/*
 * GPS.c
 *
 * Created: May 21, 2017 5:17:05 PM
 *  Author: Fadi
 */ 
#include "main.h"
#include "nmea/minmea.h"

char gps_line[MINMEA_MAX_LENGTH];
int gps_index = 0;
int sentenceDone = 0, dataReady; //booleans

double gps_lon = 0;
double gps_lat = 0;
long gps_time = 0;
int gps_alt = 0;
int gps_nsatts = 0;

#if defined (__GNUC__)
__attribute__((__interrupt__))
#elif defined(__ICCAVR32__)
__interrupt
#endif
static void read_gps_byte() {
	//irqflags_t flags;
	//flags = cpu_irq_save();
	
	int val;
	char c;
	
	while(usart_test_hit(&GPS_USART))
	usart_read_char(&GPS_USART, &val);
	
	c = (char) val;
	gps_processChar(c);
	
	GPS_USART.idr = 1;
	GPS_USART.ier = 1;
	val = GPS_USART.rhr;
	
	//cpu_irq_restore(flags);
}

void init_gps() {
	
	// Disable all interrupts.
	Disable_global_interrupt();

	// Initialize interrupt vectors.
	INTC_init_interrupts();
	
	INTC_register_interrupt(&read_gps_byte, GPS_USART_IRQ, AVR32_INTC_INT0);
	
	GPS_USART.ier = AVR32_USART_IER_RXRDY_MASK;
	
	Enable_global_interrupt();
}

void gps_processChar(char c) {
	if(sentenceDone)
		return;
	
	if(c == '$' && gps_index != 0) {
		gps_index = 0;
	}
	
	gps_line[gps_index++ % MINMEA_MAX_LENGTH] = c;
	
	if(c == '\n') {
		gps_line[gps_index++ % MINMEA_MAX_LENGTH] = '\0';
		gps_index = 0;
		sentenceDone = 1;
	}
}

int prepare_gps_data() {
	if(sentenceDone) {
		switch (minmea_sentence_id(gps_line,true)) {
			case MINMEA_SENTENCE_GGA: {
				struct minmea_sentence_gga frame;
				if (minmea_parse_gga(&frame, gps_line)) {
					gps_lon = minmea_tocoord(&(frame.longitude));
					gps_lat = minmea_tocoord(&(frame.latitude));
					gps_nsatts = frame.satellites_tracked;
					//gps_time =  minmea_gettime(frame.date,frame.time);
					gps_alt = (int) ((frame.altitude.scale/(float)frame.altitude.value) + 0.5);
				}
			} break;
			
			case MINMEA_SENTENCE_RMC: {
				struct minmea_sentence_rmc frame;
				if (minmea_parse_rmc(&frame, gps_line)) {
					gps_lon = minmea_tocoord(&(frame.longitude));
					gps_lat = minmea_tocoord(&(frame.latitude));
					gps_time =  minmea_gettime(&(frame.date),&(frame.time));
				}
			} break;
		}
		
		sentenceDone = 0;
		dataReady = 1;
	}
}

int isDataReady() {
	if(dataReady) {
		dataReady = 0;
		return 1;
	} else
		return 0;
}

double gpsLong() {
	dataReady = 0;
	return gps_lon;
}

double gpsLat() {
	dataReady = 0;
	return gps_lat;
}

int gpsAlt() {
	dataReady = 0;
	return gps_alt;
}

long gpsTime() {
	dataReady = 0;
	return gps_time;
}

void turnOnGPS() {
	AVR32_GPIO.port[1].gpers = 1 << 8;
	AVR32_GPIO.port[0].oders = 0x80000000;
	AVR32_GPIO.port[1].oders = 1 << 8;
	AVR32_GPIO.port[0].ovrs = 0x80000000;
	AVR32_GPIO.port[1].ovrs = 1 << 8;
}

void resetGPS() {
	return;
}