/*
 * BMP280.c
 *
 * Created: 2017-05-22 2:34:49 PM
 *  Author: agsof
 */ 

#include "main.h"
//Values to measure
uint32_t pressure = 0;
uint32_t temperature = 0;

//Set up the BMP280 pressure sensor
void bmp_setup (){
	//Reset the registers
	uint8_t dat = 0xB6;
	send_i2c_bytes(ALTIMETER_I2C_ADDR, BMP_RESET_REG, &dat,1);
	
	
	//Set the resolution and measuring mode
	//pressure x16, temperature x2 and normal mode
	dat = 0x54;
	send_i2c_bytes(ALTIMETER_I2C_ADDR,BMP_CTRL_MEAS_REG,&dat,1);
	
	//Set the standby time IIR coefficient and disable SPI
	dat = 0x04;
	send_i2c_bytes(ALTIMETER_I2C_ADDR,BMP_CONFIG_REG,&dat,1);
	
}

//Read pressure from BMP280 sensor
void bmp_read_pressure(){
	
	//Get pressure value from register
	uint8_t *pressure_mes;
	read_i2c_bytes(ALTIMETER_I2C_ADDR,BMP_PRESS_MSB_REG,pressure_mes,3);
	
	pressure = (pressure_mes[0]<<12) ^ (pressure_mes[1]<<4) ^ pressure_mes[2];
	
}

//Read temperature from BMP280 sensor
void bmp_read_temperature(){
	//Get temperature value from register
	uint8_t *temperature_mes;
	read_i2c_bytes(ALTIMETER_I2C_ADDR,BMP_PRESS_MSB_REG,temperature_mes,3);
	
	temperature = (temperature_mes[0]<<9) ^ (temperature_mes[1]<<1) ^ temperature_mes[2];
	
	
}

uint32_t getPressure(){
	return pressure; 	
}

uint32_t getTemperature(){
	
	return temperature;
}
