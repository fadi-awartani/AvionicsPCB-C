/*
 * BMP280.c
 *
 * Created: 2017-05-22 2:34:49 PM
 *  Author: agsof
 */ 

#include "main.h"

//Calibration variables
uint16_t dig_T1 = 0; int16_t dig_T2 = 0, dig_T3 = 0;
uint16_t dig_P1 = 0; int16_t dig_P2 = 0, dig_P3 = 0, dig_P4 = 0, dig_P5 = 0, dig_P6 = 0, dig_P7 = 0, dig_P8 = 0, dig_P9 = 0;

//Variable used to measure pressure, derived from the temperature measurement
//PLEASE READ TEMPERATURE BEFORE PRESSURE AS PRESSURE IS DEPENDENT ON TEMPERATURE
int32_t t_fine = 0;

int32_t bmp_read_uncomp_pressure();
int32_t bmp_read_uncomp_temp();
int32_t bmp280_compensate_temperature_int32(int32_t v_uncomp_temperature_s32);

//Set up the BMP280 pressure sensor
void init_bmp (){

	//Reset the registers
	uint8_t dat = 0xB6;
	send_i2c_bytes(ALTIMETER_I2C_ADDR, BMP_RESET_REG, &dat,1);
	
	//Set the resolution and measuring mode
	//pressure x16, temperature x2 and sleep mode
	dat = 0x54;
	send_i2c_bytes(ALTIMETER_I2C_ADDR,BMP_CTRL_MEAS_REG,&dat,1);
	
	//Set the standby time IIR coefficient and disable SPI
	//dat = 0x04;
	dat = 0;//turn off IIR filter
	send_i2c_bytes(ALTIMETER_I2C_ADDR,BMP_CONFIG_REG,&dat,1);

	//Normal mode
	//dat = 0x57;
	//send_i2c_bytes(ALTIMETER_I2C_ADDR,BMP_CTRL_MEAS_REG,&dat,1);
	
	//Set up calibration values
	uint8_t calib[6];
	read_i2c_bytes(ALTIMETER_I2C_ADDR,BMP_DIG_T1_LSB_REG,calib,6);
	dig_T1 = (calib[1]<<8) | calib[0];
	dig_T2 = (calib[3]<<8) | calib[2];
	dig_T3 = (calib[5]<<8) | calib[4];

	uint8_t calib_p[18]; 
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
	
	sprintf(gen_string, "\n\nT: %d, %d, %d,\nP: %d, %d, %d, %d, %d, %d, %d, %d, %d\nuncomp:%d, %d\r\n", dig_T1, dig_T2, dig_T3, 
		dig_P1, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9, bmp_read_uncomp_temp(), bmp_read_uncomp_pressure());
	usart_write_line(&RFD_USART, gen_string);
}

//Read temperature from BMP280 sensor
int32_t bmp_read_uncomp_temp(){
	//Get temperature value from registers
	uint8_t temperature_mes[3];
	read_i2c_bytes(ALTIMETER_I2C_ADDR,BMP_PRESS_MSB_REG,temperature_mes,3);
	
	return (((int32_t)temperature_mes[0])<<12) | (((int32_t)temperature_mes[1])<<4) | (((int32_t)temperature_mes[2])>>4);
}


//Read pressure from BMP280 sensor
//PLEASE READ TEMPERATURE BEFORE PRESSURE AS PRESSURE IS DEPENDENT ON TEMPERATURE
int32_t bmp_read_uncomp_pressure(){
	//Get pressure value from registers
	uint8_t pressure_mes[3];
	read_i2c_bytes(ALTIMETER_I2C_ADDR,BMP_PRESS_MSB_REG,pressure_mes,3);
	
	return (((int32_t)pressure_mes[0])<<12) | (((int32_t)pressure_mes[1])<<4) | (((int32_t)pressure_mes[2])>>4);
}

/*!
 *	@brief Reads actual temperature
 *	from uncompensated temperature
 *	@note Returns the value in 0.01 degree Centigrade
 *	@note Output value of "5123" equals 51.23 DegC.
 *
 *
 *
 *  @param v_uncomp_temperature_s32 : value of uncompensated temperature
 *
 *
 *
 *  @return Actual temperature output as s32
 *
 */
int32_t bmp280_compensate_temperature_int32(int32_t v_uncomp_temperature_s32)
{
	int32_t v_x1_u32r = 0;
	int32_t v_x2_u32r = 0;
	int32_t temperature = 0;
	/* calculate true temperature*/
	/*calculate x1*/
	v_x1_u32r = ((((v_uncomp_temperature_s32
			>> 3)
			- ((int32_t)dig_T1
			<< 1)))
			* ((int32_t)dig_T2))
			>> 11;
	/*calculate x2*/
	v_x2_u32r = (((((v_uncomp_temperature_s32
			>> 4)
			- ((int32_t)dig_T1))
			* ((v_uncomp_temperature_s32
			>> 4)
			- ((int32_t)dig_T1)))
			>> 12)
			* ((int32_t)dig_T3))
			>> 14;
	/*calculate t_fine*/
	t_fine = v_x1_u32r + v_x2_u32r;
	/*calculate temperature*/
	temperature = (t_fine * 5 + 128)
			>> 8;

	return temperature;
}/*!
 *	@brief Reads actual pressure from uncompensated pressure
 *	and returns the value in Pascal(Pa)
 *	@note Output value of "96386" equals 96386 Pa =
 *	963.86 hPa = 963.86 millibar
 *
 *
 *
 *
 *  @param  v_uncomp_pressure_s32: value of uncompensated pressure
 *
 *
 *
 *  @return Returns the Actual pressure out put as s32
 *
 */
uint32_t bmp280_compensate_pressure_int32(int32_t v_uncomp_pressure_s32)
{
	int32_t v_x1_u32r = 0;
	int32_t v_x2_u32r = 0;
	uint32_t v_pressure_u32 = 0;
	/* calculate x1*/
	v_x1_u32r = (((int32_t)t_fine)
			>> 1) - (int32_t)64000;
	/* calculate x2*/
	v_x2_u32r = (((v_x1_u32r >> 2)
			* (v_x1_u32r >> 2))
			>> 11)
			* ((int32_t)dig_P6);
	v_x2_u32r = v_x2_u32r + ((v_x1_u32r *
			((int32_t)dig_P5))
			<< 1);
	v_x2_u32r = (v_x2_u32r >> 2)
			+ (((int32_t)dig_P4)
			<< 16);
	/* calculate x1*/
	v_x1_u32r = (((dig_P3
			* (((v_x1_u32r
			>> 2) * (v_x1_u32r
			>> 2))
			>> 13))
			>> 3)
			+ ((((int32_t)dig_P2)
			* v_x1_u32r)
			>> 1))
			>> 18;
	v_x1_u32r = ((((32768 + v_x1_u32r))
			* ((int32_t)dig_P1))
			>> 15);
	/* calculate pressure*/
	v_pressure_u32 = (((uint32_t)(((int32_t)1048576) - v_uncomp_pressure_s32)
			- (v_x2_u32r >> 12)))
			* 3125;
	/* check overflow*/
	if (v_pressure_u32 < 0x80000000)
		/* Avoid exception caused by division by zero */
		if (v_x1_u32r != 0)
			v_pressure_u32 = (v_pressure_u32
					<< 1)
					/ ((uint32_t)v_x1_u32r);
		else
			return 0;
	else
	/* Avoid exception caused by division by zero */
	if (v_x1_u32r != 0)
		v_pressure_u32 = (v_pressure_u32 / (uint32_t)v_x1_u32r) * 2;
	else
		return 0;
	/* calculate x1*/
	v_x1_u32r = (((int32_t)dig_P9) * ((int32_t)(
			((v_pressure_u32
			>> 3)
			* (v_pressure_u32
			>> 3))
			>> 13)))
			>> 12;
	/* calculate x2*/
	v_x2_u32r = (((int32_t)(v_pressure_u32 >>
			2))
			* ((int32_t)dig_P8))
			>> 13;
	/* calculate true pressure*/
	v_pressure_u32 = (uint32_t)((int32_t)v_pressure_u32 + ((v_x1_u32r + v_x2_u32r
			+ dig_P7)
			>> 4));

	return v_pressure_u32;
}altimeter_data_t readAltimeter() {	altimeter_data_t alt_data;	//alt_data.temp = bmp280_compensate_T_double(bmp_read_uncomp_temp());	alt_data.temp = bmp280_compensate_temperature_int32(bmp_read_uncomp_temp())/100.0;	//alt_data.pres = bmp280_compensate_P_double(bmp_read_uncomp_pressure());	alt_data.pres = bmp280_compensate_pressure_int32(bmp_read_uncomp_pressure());	alt_data.time = realTime();		return alt_data;}
/*// Returns temperature in DegC, double precision. Output value of “51.23” equals 51.23 DegC.
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
}*/

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
