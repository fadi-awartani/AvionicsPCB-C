/**
 * initialize_board.c
 *
 * Created: Mar 26, 2017 5:03:53 PM
 *  Author: Fadi
 */ 
#include <asf.h>
#include "main.h"

// GPIO Module init
void init_gpio() {
	/*
		--PORT A--
		GPER: -32- 0011 0000 0011 0010
			       0000 0000 0000 0000 -0- == 0x30320000
			-Enables GPIO function for corresponding pin.
				   
		PMR0: -32- 0000 1100 0000 0000
				   0010 0000 0000 0000 -0- == 0x0C002000
				   
		PMR1: -32- 0000 0000 0000 0000
				   0000 0001 1110 0000 -0- == 0x1E0
			- [PMR1,PMR0] selects pin function if not GPIO. (00 = Function A, 01 = Function B, etc...)
				   
		--PORT B--
		GPER: -32- 0000 0000 0000 0000
				   0000 0011 1100 0011 -0- == 0x3C3
				   
		PMR0: -32- 0000 0000 0000 0000
				   0000 0000 0000 0000 -0- == 0x0
				   		
		PMR1: -32- 0000 0000 0000 0000
				   0000 1100 0000 0000 -0- == 0xC00
	*/
	volatile avr32_gpio_port_t *gpio_portA = &AVR32_GPIO.port[0];
	volatile avr32_gpio_port_t *gpio_portB = &AVR32_GPIO.port[1];
	
	gpio_portA->gper = 0x30320000UL;
	gpio_portA->pmr0 = 0x0C002000UL;
	gpio_portA->pmr1 = 0x1E0;
	gpio_portA->puers = 0x80000600UL;//Enable pullup on I2C lines and GPS battery backup
	
	gpio_portB->gper = 0x3C3;
	gpio_portB->pmr0 = 0;
	gpio_portB->pmr1 = 0xC00;
	gpio_portB->puers = 0xC; //Enable pullup on buttons
}

// USART init
void init_usarts() {

	//RFD900
	static const usart_options_t USART0_OPTIONS =
	{
		.baudrate     = 19200,
		.charlength   = 8,
		.paritytype   = USART_NO_PARITY,
		.stopbits     = USART_1_STOPBIT,
		.channelmode  = USART_NORMAL_CHMODE
	};

	//Iridium
	static const usart_options_t USART1_OPTIONS =
	{
		.baudrate     = 19200,//38400,
		.charlength   = 8,
		.paritytype   = USART_NO_PARITY,
		.stopbits     = USART_1_STOPBIT,
		.channelmode  = USART_NORMAL_CHMODE
	};

	//GPS
	static const usart_options_t USART2_OPTIONS =
	{
		.baudrate     = 9600,
		.charlength   = 8,
		.paritytype   = USART_NO_PARITY,
		.stopbits     = USART_1_STOPBIT,
		.channelmode  = USART_NORMAL_CHMODE
	};
	
	//usart_init_hw_handshaking(&AVR32_USART0, &USART0_OPTIONS, 24000000);
	usart_init_rs232(&AVR32_USART0, &USART0_OPTIONS, 24000000);
	//usart_init_modem(&AVR32_USART1, &USART1_OPTIONS, 24000000); //For Iridium. make sure to swap tx/rx pins on board first :(
	usart_init_rs232(&AVR32_USART1, &USART1_OPTIONS, 24000000);
	usart_init_rs232(&AVR32_USART2, &USART2_OPTIONS, 24000000);
}

// I2C init
void init_i2c(void)
{
	twi_master_options_t opt = {
		.speed = I2C_SPEED,
		.pba_hz = BOARD_FREQ_HZ,
		.chip  = ALTIMETER_I2C_ADDR
	};
	twi_master_init(&AVR32_TWI, &opt);
}

static bool my_flag_autorize_cdc_transfert = false;
bool my_callback_cdc_enable(void)
{
	my_flag_autorize_cdc_transfert = true;
	return true;
}
void my_callback_cdc_disable(void)
{
	my_flag_autorize_cdc_transfert = false;
}

void init_pwm() {
	/* PWM controller configuration. */
	pwm_opt_t pwm_opt =
	{
		.diva = AVR32_PWM_DIVA_CLK_OFF,
		.divb = AVR32_PWM_DIVB_CLK_OFF,
		.prea = AVR32_PWM_PREA_MCK_DIV_2,
		.preb = AVR32_PWM_PREA_MCK_DIV_2
	};

	/* PWM channel configuration structure. */
	avr32_pwm_channel_t pwm_channel = { .ccnt = 0 };
		
		/* With these settings, the output waveform period will be:
	 * (115200/256)/20 == 22.5Hz == (MCK/prescaler)/period, with
	 * MCK == 115200Hz, prescaler == 256, period == 20. */
	pwm_channel.cdty = 5; /* Channel duty cycle, should be < CPRD. */
	pwm_channel.cprd = 20; /* Channel period. */
	pwm_channel.cupd = 0; /* Channel update is not used here. */
	pwm_channel.CMR.calg = PWM_MODE_LEFT_ALIGNED; /* Channel mode. */
	pwm_channel.CMR.cpol = PWM_POLARITY_LOW;      /* Channel polarity. */
	pwm_channel.CMR.cpd = PWM_UPDATE_DUTY;        /* Not used the first time. */
	pwm_channel.CMR.cpre = AVR32_PWM_CPRE_MCK_DIV_256; /* Channel prescaler. */	
	
	avr32_pwm_channel_t pwm_channel_blue_led = { .ccnt = 0 };
		
		/* With these settings, the output waveform period will be:
	 * (115200/256)/20 == 22.5Hz == (MCK/prescaler)/period, with
	 * MCK == 115200Hz, prescaler == 256, period == 20. */
	pwm_channel_blue_led.cdty = 14; /* Channel duty cycle, should be < CPRD. */
	pwm_channel_blue_led.cprd = 15; /* Channel period. */
	pwm_channel_blue_led.cupd = 0; /* Channel update is not used here. */
	pwm_channel_blue_led.CMR.calg = PWM_MODE_LEFT_ALIGNED; /* Channel mode. */
	pwm_channel_blue_led.CMR.cpol = PWM_POLARITY_LOW;      /* Channel polarity. */
	pwm_channel_blue_led.CMR.cpd = PWM_UPDATE_DUTY;        /* Not used the first time. */
	pwm_channel_blue_led.CMR.cpre = AVR32_PWM_CPRE_MCK_DIV_256; /* Channel prescaler. */	
	
	/* Initialize the PWM module. */
	pwm_init(&pwm_opt);
	
	/* Set channel configuration to channel 0. */
	pwm_channel_init(BUZZER_PWM, &pwm_channel);
	pwm_channel_init(BLUE_LED_PWM, &pwm_channel_blue_led);
	
	/* Start channel 0. */
	//pwm_start_channels(1 << BUZZER_PWM);
}

void init_watchdog() {
	AVR32_WDT.ctrl = 0x55001501;
	AVR32_WDT.ctrl = 0xAA001501;
}

void update_watchdog() {
	AVR32_WDT.clr = 0xFFFFFFFF;
}


//32kHz clock prescaled by 2^(4+1): Counts at 1000 Hz.
void init_rtc() {
	//AVR32_PM.oscctrl32 = 0x00060101; //enable 32kHz oscillator: Warning: gets connected to USART0 CTS and RTS
	AVR32_PM.oscctrl32 = 0; //disable 32kHz oscillator
	AVR32_RTC.val = 0;
	//AVR32_RTC.ctrl = 0x0001040D; //use 32kHz osc
	AVR32_RTC.ctrl = 0x00010605; //use internal RC oscillator. Not very accurate but whatever.
	AVR32_RTC.top = 0xFFFFFFFF;
}

//returns value of counter counting at ~1000 Hz (not very accurate)
unsigned long millis() {
	return AVR32_RTC.val*9070/8000;
}

// Board init
void initialize_board() {
	// Initialize and enable interrupts
	irq_initialize_vectors();
	cpu_irq_enable();
	
	init_rtc();
	init_gpio();
	init_usarts();
	//init_i2c();//old, not needed anymore
	init_pwm();
	init_watchdog();
	
	//Interrupt things
	INTC_init_interrupts();
	init_eic();
	init_gps();
	
	bmp_setup();
	
	// Start USB stack to authorize VBus monitoring
	#ifdef EN_USB
	udc_start();
	my_callback_cdc_enable();
	#endif
}
