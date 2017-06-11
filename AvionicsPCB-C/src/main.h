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

// ---- DEFINES ----
//#define EN_USB
#define ENDCHARS "\r\n" //End characters for println.
//#define GPSLINE_NUMCHARS 128

// ---- GLOBAL VARIABLES ----
char gen_string[128];
long millis_time_linked, real_time_linked; //Link millis() time and real time 

// ---- GENERAL FUNCTIONS ----
extern void initialize_board();
extern void update_watchdog();
extern uint64_t millis();
extern uint64_t realTime();

#ifdef EN_USB
extern void print_usb_debug(char* message);
extern void println_usb_debug(char* message);
#endif

// ---- I2C ----
extern int send_i2c_bytes(uint8_t device_address, uint8_t internal_address, uint8_t *data_bytes, int len);
extern int read_i2c_bytes(uint8_t device_address, uint8_t internal_address, uint8_t *data_bytes, int len);

// ---- GPS ----
typedef struct gps_coordinates_struct {
	double lat;
	double lon;
	long time;
} gps_coordinates_t;

typedef struct gps_altitude_struct {
	int alt;
	long time;
} gps_altitude_t;

extern void init_gps();
extern void gps_processChar(char dat);
extern int prepare_gps_data();
extern gps_coordinates_t getGPSCoordinates();
extern gps_altitude_t getGPSAltitude();
extern int isDataReady();
extern void turnOnGPS();
extern void resetGPS();

// ---- BMP280 ALTIMETER ----
typedef struct altimeter_data_struct {
	int32_t pres;
	int32_t temp;
	long time;
} altimeter_data_t;
extern void init_bmp();
extern altimeter_data_t readAltimeter();

// ---- EXTERNAL INTERRUPT CONTROLLER ----
extern void init_eic();

#endif /* MAIN_H_ */