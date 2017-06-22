/*
 * fuse_bits.c
 *
 * Created: Jun 21, 2017 8:14:30 PM
 *  Author: Fadi
 */ 

#include "main.h"
volatile int gofast = 0;
void writeFusebit(int n) {
	if(n > 31)
		return;
	AVR32_FLASHC.fcmd = 7 | (n << 8) | (0xA5 << 24);
	//gofast = 1;
}

void eraseFusebit(int n) {
	if(n > 31)
		return;
	AVR32_FLASHC.fcmd = 8 | (n << 8) | (0xA5 << 24);
	//gofast = 0;
}


int readFusebit(int n) {
	if(n > 31) return 1;
	return (AVR32_FLASHC.fgpfrlo >> (n)) & 1;
	//return gofast;
}