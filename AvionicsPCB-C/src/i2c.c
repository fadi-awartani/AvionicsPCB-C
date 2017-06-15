#include "main.h"

//48kHz
void setup_i2c_masterclock() {
	AVR32_TWI.cwgr = (246 << 8) | 246; //period of (1/96000) for low clock, same for high, should be 48000 Hz clock for I2C
}

//Based on flowchart from microcontroller datasheet.
int send_i2c_bytes(uint8_t device_address, uint8_t internal_address, uint8_t *data_bytes, int len) {
	setup_i2c_masterclock();
	AVR32_TWI.cr = 0x24;
	AVR32_TWI.mmr = (device_address << 16) | 0x100;
	AVR32_TWI.iadr = internal_address;
	//AVR32_TWI.cr = 0x1;//Start transfer
	
	int count = 0;
	for(int i = 0; i < len; i++) {
		volatile int status_reg;
		
		AVR32_TWI.thr = *(data_bytes + i);
		while(!((status_reg = AVR32_TWI.sr) & 0x4)) { //While not TX ready - busy wait loop
			delay_us(10);
			if(status_reg & 0x301) //if bits of ARBLST, NACK, or TXCOMP are set.
				return -1; //error
			if(count++ > 300)
				return -2; //timeout
		} 
	}
	
	count = 0;
	while(!(AVR32_TWI.sr & 0x1)) { //While not TXCOMP (TX complete) - busy wait loop
		delay_us(200);
		if(count++ > 24)
			return -2; //timeout
	} 
	
	return 0;//success
}

//Based on flowchart from microcontroller datasheet.
int read_i2c_bytes(uint8_t device_address, uint8_t internal_address, uint8_t *data_bytes, int len) {
	int count = 0;
	setup_i2c_masterclock();
	AVR32_TWI.cr = 0x24;
	AVR32_TWI.mmr = (device_address << 16) | 0x1100;
	AVR32_TWI.iadr = internal_address;
	if(len > 1) {
		AVR32_TWI.cr = 0x1; //Start transfer
	
		for(int i = 0; i < len-1; i++) {
			int status_reg;
		
			while(!((status_reg = AVR32_TWI.sr) & 0x2)) { //While not RX ready - busy wait loop
				if(status_reg & 0x301) //if bits of ARBLST, NACK, or TXCOMP are set.
					return -1; //error
				if(count++ > 200) 
					return -2; //timeout
				delay_us(10);
			}
		
			*(data_bytes+i) = (uint8_t) AVR32_TWI.rhr;
		}
	
		AVR32_TWI.cr = 0x2; //Stop transfer
	} else if(len == 1) {
		AVR32_TWI.cr = 0x3;
	} else
		return -3; //invalid argument
	
	volatile int status_reg;
		
	count = 0;
	while(((status_reg = AVR32_TWI.sr) & 0x2) != 0x2) { //While not RX ready - busy wait loop
		if(status_reg & 0x301) //if bits of ARBLST, NACK, or TXCOMP are set.
			return -1; //error
		if(count++ > 200) 
			return -2; //timeout
		delay_us(10);
	}
	
	*(data_bytes+len-1) = (uint8_t) AVR32_TWI.rhr;
	
	count = 0;
	while(!(AVR32_TWI.sr & 0x1)) { //While not TXCOMP (TX complete) - busy wait loop
		delay_us(200);
		if(count++ > 24)
		return -2; //timeout
	}
	
	return 0;//success
}
