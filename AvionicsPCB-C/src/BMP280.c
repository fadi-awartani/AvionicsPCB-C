/*
 * BMP280.c
 *
 * Created: 2017-05-22 2:34:49 PM
 *  Author: agsof
 */ 

#include "main.h"

//Calibration variables
uint32_t dig_T1 = 0; int32_t dig_T2 = 0, dig_T3 = 0;
uint32_t dig_P1 = 0; int32_t dig_P2 = 0, dig_P3 = 0, dig_P4 = 0, dig_P5 = 0, dig_P6 = 0, dig_P7 = 0, dig_P8 = 0, dig_P9 = 0;

//Variable used to measure pressure, derived from the temperature measurement
//PLEASE READ TEMPERATURE BEFORE PRESSURE AS PRESSURE IS DEPENDENT ON TEMPERATURE
int32_t t_fine = 0;


//Set up the BMP280 pressure sensor
void init_bmp (){

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

	//Set up calibration values
	uint8_t calib[6];
	read_i2c_bytes(ALTIMETER_I2C_ADDR,BMP_DIG_T1_LSB_REG,calib,6);
	dig_T1 = (calib[1]<<8) | calib[0];
	dig_T2 = (calib[3]<<8) | calib[2];
	dig_T3 = (calib[5]<<8) | calib[4];

	int32_t calib_p[18]; 
	read_i2c_bytes(ALTIMETER_I2C_ADDR,BMP_DIG_P1_LSB_REG,calib_p,18);
	dig_P1 = (calib_p[1]<<8) | calib_p[0];
	dig_P2 = (calib_p[3]<<8) | calib_p[2];
	dig_P3 = (calib_p[5]<<8) | calib_p[4];
	dig_P4 = (calib_p[7]<<8) | calib_p[6];
	dig_P5 = (calib_p[9]<<8) | calib_p[8];
	dig_P6 = (calib_p[11]<<8) | calib_p[10];
	dig_P7 = (calib_p[13]<<8) | calib_p[12];
	dig_P8 = (calib_p[15]<<8) | calib_p[14];
	dig_P9 = (calib_p[17]<<8) | calib_p[16];
}

//Read temperature from BMP280 sensor
int32_t bmp_read_uncomp_temp(){
	//Get temperature value from registers
	uint8_t temperature_mes[3];
	read_i2c_bytes(ALTIMETER_I2C_ADDR,BMP_PRESS_MSB_REG,temperature_mes,3);
	
	return (temperature_mes[0]<<12) | (temperature_mes[1]<<4) | (temperature_mes[2] >> 4);
}


//Read pressure from BMP280 sensor
//PLEASE READ TEMPERATURE BEFORE PRESSURE AS PRESSURE IS DEPENDENT ON TEMPERATURE
int32_t bmp_read_uncomp_pressure(){
	//Get pressure value from registers
	uint8_t pressure_mes[3];
	read_i2c_bytes(ALTIMETER_I2C_ADDR,BMP_PRESS_MSB_REG,pressure_mes,3);
	
	return (pressure_mes[0]<<12) | (pressure_mes[1]<<4) | (pressure_mes[2]>>4);
}

double bmp280_compensate_T_double(int32_t adc_T)
{
	double var1, var2, T;
	var1 = (((double)adc_T)/16384.0 - ((double)dig_T1)/1024.0) * ((double)dig_T2);
	var2 = ((((double)adc_T)/131072.0 - ((double)dig_T1)/8192.0) *
	(((double)adc_T)/131072.0 - ((double) dig_T1)/8192.0)) * ((double)dig_T3);
	t_fine = (int32_t)(var1 + var2);
	T = (var1 + var2) / 5120.0;
	return T;
}
// Returns pressure in Pa as double. Output value of “96386.2” equals 96386.2 Pa = 963.862 hPa
double bmp280_compensate_P_double(int32_t adc_P)
{
	double var1, var2, p;
	var1 = ((double)t_fine/2.0) - 64000.0;
	var2 = var1 * var1 * ((double)dig_P6) / 32768.0;
	var2 = var2 + var1 * ((double)dig_P5) * 2.0;
	var2 = (var2/4.0)+(((double)dig_P4) * 65536.0);
	var1 = (((double)dig_P3) * var1 * var1 / 524288.0 + ((double)dig_P2) * var1) / 524288.0;
	var1 = (1.0 + var1 / 32768.0)*((double)dig_P1);
	if (var1 == 0.0)
	{
		return 0; // avoid exception caused by division by zero
	}
	p = 1048576.0 - (double)adc_P;
	p = (p - (var2 / 4096.0)) * 6250.0 / var1;
	var1 = ((double)dig_P9) * p * p / 2147483648.0;
	var2 = p * ((double)dig_P8) / 32768.0;
	p = p + (var1 + var2 + ((double)dig_P7)) / 16.0;
	return p;
}altimeter_data_t readAltimeter() {	altimeter_data_t alt_data;	alt_data.temp = bmp280_compensate_T_double(bmp_read_uncomp_temp());	alt_data.pres = bmp280_compensate_P_double(bmp_read_uncomp_pressure());	alt_data.time = realTime();		return alt_data;}

//Compensation code for the temperature
//adc_T: Uncompensated temperature
//return: compensated temperature using calibration register values
/*uint32_t bmp280_compensate_T_int32(int32_t adc_T) {  
	int32_t var1, var2, T;  
	var1  = ((((adc_T>>3) - ((int32_t)dig_T1<<1))) * ((int32_t)dig_T2)) >> 11;
	var2 = (((((adc_T>>4) - ((int32_t)dig_T1))*((adc_T>>4)-((int32_t)dig_T1)))>>12)*((int32_t)dig_T3))>>14;
	t_fine = var1 + var2;
	T  = (t_fine * 5 + 128) >> 8;
	return T; 
}*/

//Compensation code for the pressure
//adc_P: Uncompensated pressure
//return: compensated pressure using calibration register values
/*uint32_t bmp280_compensate_P_int64(int32_t adc_P) {
	int64_t var1, var2, p;  
	var1 = ((int64_t)t_fine) - 128000;  
	var2 = var1 * var1 * (int64_t)dig_P6;  
	var2 = var2 + ((var1*(int64_t)dig_P5)<<17);  
	var2 = var2 + (((int64_t)dig_P4)<<35);  
	var1 = ((var1 * var1 * (int64_t)dig_P3)>>8) + ((var1 * (int64_t)dig_P2)<<12);  
	var1 = (((((int64_t)1)<<47)+var1))*((int64_t)dig_P1)>>33;  
	if (var1 == 0)  {   
		return 0; // avoid exception caused by division by zero  
	}  
	p = 1048576-adc_P;  
	p = (((p<<31)-var2)*3125)/var1;  
	var1 = (((int64_t)dig_P9) * (p>>13) * (p>>13)) >> 25;  
	var2 = (((int64_t)dig_P8) * p) >> 19;  
	p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7)<<4);  
	return (uint32_t)p;

}*/



//****************************************************Getters*************************************************

/*uint32_t getPressure(){
	
	return pressure; 	
}

uint32_t getTemperature(){
	
	return temperature;
}*/
