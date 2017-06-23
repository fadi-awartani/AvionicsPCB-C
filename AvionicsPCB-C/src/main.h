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
#include <string.h>
//#include "conf_sd_mmc_spi.h"

// ---- DEFINES ----
#define AVR32_PDCA_CHANNEL_SPI_RX 0 
#define AVR32_PDCA_CHANNEL_SPI_TX 1
//#define EN_USB
#define ENDCHARS "\r\n" //End characters for println.
//#define GPSLINE_NUMCHARS 128
#define DISABLE_BMP
#define DISABLE_VERBOSE
//#define SD_ENABLE
//#define ECHOGPS
//#define IS_SECOND_BOARD
//#define PAYLOAD_ENABLE

// ---- GLOBAL VARIABLES ----
char gen_string[128];
#ifdef PAYLOAD_ENABLE
int16_t payload_nums[7][2];
int payload_count;
#endif
long millis_time_linked, real_time_linked; //Link millis() time and real time 
volatile char sd_transmit_buf[256];
unsigned char txbytes[40];


// ---- GENERAL FUNCTIONS ----
extern void initialize_board();
extern void update_watchdog();
extern uint64_t millis();
extern uint64_t realTime();
uint64_t hash(unsigned char *str);

#ifdef EN_USB
extern void print_usb_debug(char* message);
extern void println_usb_debug(char* message);
#endif

// ---- FUSE BITS ----
extern void writeFusebit(int n);
extern void eraseFusebit(int n);
extern int readFusebit(int n);
#define GOFAST_BIT 31

// ---- I2C ----
extern int send_i2c_bytes(uint8_t device_address, uint8_t internal_address, uint8_t *data_bytes, int len);
extern int read_i2c_bytes(uint8_t device_address, uint8_t internal_address, uint8_t *data_bytes, int len);
extern int read_i2c_bytes_no_addr(uint8_t device_address, uint8_t *data_bytes, int len);

// ---- GPS ----
typedef struct gps_coordinates_struct {
	double lat;
	double lon;
	int32_t alt;
	uint64_t time;
	float speed_kph;
	float track_degrees;
	uint64_t speed_time;
} gps_coordinates_t;

extern void init_gps();
extern void gps_processChar(char dat);
extern int prepare_gps_data();
extern gps_coordinates_t getGPSCoordinates();
extern int isDataReady();
extern void turnOnGPS();
extern void resetGPS();

// ---- SD CARD ----
extern void sd_pdca_init();
extern void sd_mmc_resources_init(void);

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

void writeDataSen(unsigned char * buf, gps_coordinates_t data);

#endif /* MAIN_H_ */