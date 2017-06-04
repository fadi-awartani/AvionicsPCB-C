/*
 * main.h
 *
 * Created: Mar 26, 2017 5:18:29 PM
 *  Author: Fadi
 */ 

#ifndef MAIN_H_
#define MAIN_H_
#include <asf.h>
#include <stdio.h>
#include <stdlib.h>

extern void initialize_board();

char gen_string[128];


//#define EN_USB

//End characters for println.
#define ENDCHARS "\r\n"

//#define GPSLINE_NUMCHARS 128

extern void update_watchdog();

#ifdef EN_USB
extern void print_usb_debug(char* message);
extern void println_usb_debug(char* message);
#endif

extern int send_i2c_bytes(uint8_t device_address, uint8_t internal_address, uint8_t *data_bytes, int len);
extern int read_i2c_bytes(uint8_t device_address, uint8_t internal_address, uint8_t *data_bytes, int len);
//extern void read_gps_byte();

/*#if defined (__GNUC__)
__attribute__((__interrupt__))
#elif defined(__ICCAVR32__)
__interrupt
#endif
static void read_gps_byte();*/
extern void gps_processChar(char dat);

extern int prepare_gps_data();
extern double gpsLong();
extern double gpsLat();
extern int gpsAlt();
extern long gpsTime();
extern int isDataReady();

//BMP280
extern void bmp_setup ();
extern void bmp_read_temperature();
extern void bmp_read_pressure();
extern uint32_t bmp280_compensate_T_int32(int32_t adc_T);
extern uint32_t bmp280_compensate_P_int64(int32_t adc_P);
extern uint32_t getPressure();
extern uint32_t getTemperature();

#endif /* MAIN_H_ */