/*
 * main.h
 *
 * Created: Mar 26, 2017 5:18:29 PM
 *  Author: Fadi
 */ 

#ifndef MAIN_H_
#define MAIN_H_

extern void initialize_board();

//End characters for println.
#define ENDCHARS "\r\n"
extern void print_usb_debug(char* message);
extern void println_usb_debug(char* message);

#endif /* MAIN_H_ */