/**
 * \file
 *
 * \brief User board configuration template
 *
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */

#ifndef CONF_BOARD_H
#define CONF_BOARD_H

// External oscillator settings.
// Uncomment and set correct values if external oscillator is used.

// External oscillator frequency
//#define BOARD_XOSC_HZ			12000000UL
//#define FOSC0					12000000UL
#define BOARD_FREQ_HZ			24000000UL

// External oscillator type.
//!< External clock signal
//#define BOARD_XOSC_TYPE        XOSC_TYPE_EXTERNAL
//!< 32.768 kHz resonator on TOSC
//#define BOARD_XOSC_TYPE        XOSC_TYPE_32KHZ
//!< 0.4 to 16 MHz resonator on XTALS
//#define BOARD_XOSC_TYPE        XOSC_TYPE_XTAL

// GPIO Pin Definitions
#define RED_LED_PIN				AVR32_PIN_PA21
#define BLUE_LED_PIN			AVR32_PIN_PA22
#define IRID_ONOFF_PIN			AVR32_PIN_PB00
#define IRID_NET_AVAIL_PIN		AVR32_PIN_PB01
#define IRID_SUPP_OUT_PIN		AVR32_PIN_PA20
#define GPS_RESET_PIN			AVR32_PIN_PB08
#define GPS_VBACKUP_PIN			AVR32_PIN_PA31
#define NANOPI_ONOFF_PIN		AVR32_PIN_PA17
#define IMU_INTERRUPT_PIN		AVR32_PIN_PB07
#define SD_DETECT_PIN			AVR32_PIN_PB06 //Connected to GND when SD not inserted
#define GPIO_28_PIN				AVR32_PIN_PA28
#define GPIO_29_PIN				AVR32_PIN_PA29
#define BATTERY_ADC_PIN			AVR32_PIN_PA03
#define NANOPI_BATTERY_ADC_PIN	AVR32_PIN_PA04
#define ADC_6_PIN				AVR32_PIN_PA30

// ADC Channels
#define BATTERY_ADC				0
#define NANOPI_BATTERY_ADC		1
#define ADC_6					6
#define GPS_VBACKUP_ADC			7

// PWM Channels of LEDs
#define RED_LED_PWM				2
#define BLUE_LED_PWM			6
#define BUZZER_PWM				2

// USARTs
#define RFD_USART				AVR32_USART0
#define IRIDIUM_USART			AVR32_USART1
#define GPS_USART				AVR32_USART2
//#define GPS_USART				AVR32_USART1 //CHANGE BACK!!! Todo
#define GPS_USART_IRQ			AVR32_USART2_IRQ //match to GPS_USART

// I2C Configuration
#define I2C_SPEED				50000
#define ALTIMETER_I2C_ADDR		0x77
#define IMU_I2C_ADDR			0b1101011
#define NANOPI_I2C_ADDR			0x0 //PUT IN CORRECT ADDRESS
#define PAYLOAD_I2C_ADDR		0x0 //PUT IN CORRECT ADDRESS

// BMP280 (Altimeter) Register Addresses
#define BMP_DEVICE_ID_REG		0xD0
#define BMP_RESET_REG			0xE0
#define BMP_CTRL_MEAS_REG		0xF4
#define BMP_CONFIG_REG			0xF5
#define BMP_PRESS_MSB_REG		0xF7
#define BMP_PRESS_LSB_REG		0xF8
#define BMP_PRESS_XLSB_REG		0xF9
#define BMP_TEMP_MSB_REG		0xFA
#define BMP_TEMP_LSB_REG		0xFB
#define BMP_TEMP_XLSB_REG		0xFC
#define BMP_DIG_T1_LSB_REG		0x88
#define BMP_DIG_T1_MSB_REG		0x89
#define BMP_DIG_T2_LSB_REG		0x8A
#define BMP_DIG_T2_MSB_REG		0x8B
#define BMP_DIG_T3_LSB_REG		0x8C
#define BMP_DIG_T3_MSB_REG		0x8D
#define BMP_DIG_P1_LSB_REG		0x8E
#define BMP_DIG_P1_MSB_REG		0x8F
#define BMP_DIG_P2_LSB_REG		0x90
#define BMP_DIG_P2_MSB_REG		0x91
#define BMP_DIG_P3_LSB_REG		0x92
#define BMP_DIG_P3_MSB_REG		0x93
#define BMP_DIG_P4_LSB_REG		0x94
#define BMP_DIG_P4_MSB_REG		0x95
#define BMP_DIG_P5_LSB_REG		0x96
#define BMP_DIG_P5_MSB_REG		0x97
#define BMP_DIG_P6_LSB_REG		0x98
#define BMP_DIG_P6_MSB_REG		0x99
#define BMP_DIG_P7_LSB_REG		0x9A
#define BMP_DIG_P7_MSB_REG		0x9B
#define BMP_DIG_P8_LSB_REG		0x9C
#define BMP_DIG_P8_MSB_REG		0x9D
#define BMP_DIG_P9_LSB_REG		0x9E
#define BMP_DIG_P9_MSB_REG		0x9F

// Imu
#define WHO_AM_I_ADDR			0x0F

#endif // CONF_BOARD_H
