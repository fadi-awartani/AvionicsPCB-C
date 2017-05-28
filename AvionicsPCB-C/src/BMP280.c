/*
 * BMP280.c
 *
 * Created: 2017-05-22 2:34:49 PM
 *  Author: agsof
 */ 

#include "main.h" /*
//Values to measure
uint32_t pressure = 0;
uint32_t temperature = 0;
//Calibration variables
uint32_t dig_T1 = 0;
int32_t dig_T2 = 0;
int32_t dig_T3 = 0;
uint32_t dig_P1 = 0;
int32_t dig_P2 = 0;
int32_t dig_P3 = 0;
int32_t dig_P4 = 0;
int32_t dig_P5 = 0;
int32_t dig_P6 = 0;
int32_t dig_P7 = 0;
int32_t dig_P8 = 0;
int32_t dig_P9 = 0;

//Variable used to measure pressure, derived from the temperature measurement
//PLEASE READ TEMPERATURE BEFORE PRESSURE AS PRESSURE IS DEPENDENT ON TEMPERATURE
int32_t t_fine = 0;


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




	//Set up calibration values
	uint32_t u_cal[2];
	read_i2c_bytes(ALTIMETER_I2C_ADDR,BMP_DIG_T1_LSB_REG,u_cal,2);
	dig_T1 = (u_cal[0]<<8) ^ u_cal[1];

	int32_t cal[4];
	read_i2c_bytes(ALTIMETER_I2C_ADDR,BMP_DIG_T2_LSB_REG,cal,4);
	dig_T2 = (cal[0]<<8) ^ cal[1];
	dig_T3 = (cal[2]<<8) ^ cal[3];

	read_i2c_bytes(ALTIMETER_I2C_ADDR,BMP_DIG_P1_LSB_REG,u_cal,2);
	dig_P1 = (u_cal[0]<<8) ^ u_cal[1];

	int32_t calP[16]; 
	read_i2c_bytes(ALTIMETER_I2C_ADDR,BMP_DIG_P2_LSB_REG,cal,4);
	dig_P2 = (cal[0]<<8) ^ cal[1];
	dig_P3 = (cal[2]<<8) ^ cal[3];
	dig_P4 = (cal[4]<<8) ^ cal[5];
	dig_P5 = (cal[6]<<8) ^ cal[7];
	dig_P6 = (cal[8]<<8) ^ cal[9];
	dig_P7 = (cal[10]<<8) ^ cal[11];
	dig_P8 = (cal[12]<<8) ^ cal[13];
	dig_P9 = (cal[14]<<8) ^ cal[15];




	
}

//Read temperature from BMP280 sensor
void bmp_read_temperature(){
	//Get temperature value from register
	uint8_t *temperature_mes;
	read_i2c_bytes(ALTIMETER_I2C_ADDR,BMP_PRESS_MSB_REG,temperature_mes,3);
	
	temperature = (temperature_mes[0]<<9) ^ (temperature_mes[1]<<1) ^ temperature_mes[2];

	temperature = bmp280_compensate_T_int32((int32_t)temperature);
	
	
}


//Read pressure from BMP280 sensor
//PLEASE READ TEMPERATURE BEFORE PRESSURE AS PRESSURE IS DEPENDENT ON TEMPERATURE
void bmp_read_pressure(){
	
	//Get pressure value from register
	uint8_t pressure_mes[3];
	read_i2c_bytes(ALTIMETER_I2C_ADDR,BMP_PRESS_MSB_REG,pressure_mes,3);
	
	pressure = (pressure_mes[0]<<12) ^ (pressure_mes[1]<<4) ^ pressure_mes[2];

	pressure = bmp280_compensate_P_int64((int32_t)pressure);
	
}




//Compensation code for the temperature
//adc_T: Uncompensated temperature
//return: compensated temperature using calibration register values
uint32_t bmp280_compensate_T_int32(int32_t adc_T) {  
	int32_t var1, var2, T;  
	var1  = ((((adc_T>>3) - ((int32_t)dig_T1<<1))) * ((int32_t)dig_T2)) >> 11;  var2  = (((((adc_T>>4)  ((int32_t)dig_T1)) * ((adc_T>>4) - ((int32_t)dig_T1))) >> 12)*((int32_t)dig_T3)) >> 14;  
	t_fine = var1 + var2;
	T  = (t_fine * 5 + 128) >> 8;
	return (uint32_t)T; 
}

//Compensation code for the pressure
//adc_P: Uncompensated pressure
//return: compensated pressure using calibration register values
uint32_t bmp280_compensate_P_int64(int32_t adc_P) {
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

}



//****************************************************Getters*************************************************

uint32_t getPressure(){
	
	return pressure; 	
}

uint32_t getTemperature(){
	
	return temperature;
}*/
