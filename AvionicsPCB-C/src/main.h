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

extern void initialize_board();

char gen_string[128];
//End characters for println.
#define ENDCHARS "\r\n"
extern void print_usb_debug(char* message);
extern void println_usb_debug(char* message);
extern int send_i2c_bytes(uint8_t device_address, uint8_t internal_address, uint8_t *data_bytes, int len);
extern int read_i2c_bytes(uint8_t device_address, uint8_t internal_address, uint8_t *data_bytes, int len);

#endif /* MAIN_H_ */