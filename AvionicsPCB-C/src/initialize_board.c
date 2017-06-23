/**
 * initialize_board.c
 *
 * Created: Mar 26, 2017 5:03:53 PM
 *  Author: Fadi
 */ 
#include <asf.h>
#include "main.h"

uint64_t millis_value = 0;

// GPIO Module init
void init_gpio() {
	/*
		--PORT A--
		GPER: -32- 0011 0000 0011 0010
			       0001 1000 0000 0000 -0- == 0x30321800
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
	
	gpio_portA->gper = 0x30321800UL;
	gpio_portA->pmr0 = 0x0C002000UL;
	gpio_portA->pmr1 = 0x1E0;
	gpio_portA->puers = 0x600;//Enable pullup on I2C lines XXand GPS battery backupXX
	
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
		#ifdef IS_SECOND_BOARD
		.baudrate     = 19200,
		#else
		.baudrate     = 38400,
		#endif
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
	
	//usart_init_hw_handshaking(&RFD_USART, &USART0_OPTIONS, 24000000);
	usart_init_rs232(&RFD_USART, &USART0_OPTIONS, 24000000);
	//usart_init_modem(&IRIDIUM_USART, &USART1_OPTIONS, 24000000); //For Iridium. make sure to swap tx/rx pins on board first :(
	#ifndef IS_SECOND_BOARD
		//usart_init_rs232(&IRIDIUM_USART, &USART1_OPTIONS, 24000000);
		usart_init_rs232(&GPS_USART, &USART2_OPTIONS, 24000000);
	#endif
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
		
	pwm_channel.cdty = 5; /* Channel duty cycle, should be < CPRD. */
	pwm_channel.cprd = 20; /* Channel period. */
	pwm_channel.cupd = 0; /* Channel update is not used here. */
	pwm_channel.CMR.calg = PWM_MODE_LEFT_ALIGNED; /* Channel mode. */
	pwm_channel.CMR.cpol = PWM_POLARITY_LOW;      /* Channel polarity. */
	pwm_channel.CMR.cpd = PWM_UPDATE_DUTY;        /* Not used the first time. */
	pwm_channel.CMR.cpre = AVR32_PWM_CPRE_MCK_DIV_256; /* Channel prescaler. */	
	
	avr32_pwm_channel_t pwm_channel_blue_led = { .ccnt = 0 };
		
	pwm_channel_blue_led.cdty = 13; /* Channel duty cycle, should be < CPRD. I think this controls brightness. */
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
	AVR32_WDT.ctrl = 0x55001401;
	AVR32_WDT.ctrl = 0xAA001401;
}

void update_watchdog() {
	AVR32_WDT.clr = 0xFFFFFFFF;
}


//RC oscillator clock prescaled by 2^(6+1): Counts at ~898 Hz.
//Not used. TC module used for better reliability and accuracy.
void init_rtc() {
	//AVR32_PM.oscctrl32 = 0x00060101; //enable 32kHz oscillator: Warning: gets connected to USART0 CTS and RTS
	AVR32_PM.oscctrl32 = 0; //disable 32kHz oscillator
	AVR32_RTC.val = 0;
	//AVR32_RTC.ctrl = 0x0001040D; //use 32kHz osc
	AVR32_RTC.ctrl = 0x00010605; //use internal RC oscillator. Not very accurate but whatever.
	AVR32_RTC.top = 0xFFFFFFFF;
}

/**
 * \brief TC interrupt.
 *
 * The ISR handles RC compare interrupt and sets the update_timer flag to
 * update the timer value.
 */
#if defined (__GNUC__)
__attribute__((__interrupt__))
#elif defined (__ICCAVR32__)
#pragma handler = EXAMPLE_TC_IRQ_GROUP, 1
__interrupt
#endif
static void tc_irq(void)
{
	// Increment the ms seconds counter
	millis_value++;

	// Clear the interrupt flag. This is a side effect of reading the TC SR.
	tc_read_sr(&AVR32_TC, 0);
}

//Start the timer/counter module to generate a 1kHz counter variable for millis();
void init_tc() {
	INTC_register_interrupt(&tc_irq, AVR32_TC_IRQ0, AVR32_INTC_INT0);
	
	volatile avr32_tc_t *tc = &AVR32_TC;
	
	// Options for waveform generation.
	static const tc_waveform_opt_t waveform_opt = {
		// Channel selection.
		.channel  = 0,
		// Software trigger effect on TIOB.
		.bswtrg   = TC_EVT_EFFECT_NOOP,
		// External event effect on TIOB.
		.beevt    = TC_EVT_EFFECT_NOOP,
		// RC compare effect on TIOB.
		.bcpc     = TC_EVT_EFFECT_NOOP,
		// RB compare effect on TIOB.
		.bcpb     = TC_EVT_EFFECT_NOOP,
		// Software trigger effect on TIOA.
		.aswtrg   = TC_EVT_EFFECT_NOOP,
		// External event effect on TIOA.
		.aeevt    = TC_EVT_EFFECT_NOOP,
		// RC compare effect on TIOA.
		.acpc     = TC_EVT_EFFECT_NOOP,
		/*
		 * RA compare effect on TIOA.
		 * (other possibilities are none, set and clear).
		 */
		.acpa     = TC_EVT_EFFECT_NOOP,
		/*
		 * Waveform selection: Up mode with automatic trigger(reset)
		 * on RC compare.
		 */
		.wavsel   = TC_WAVEFORM_SEL_UP_MODE_RC_TRIGGER,
		// External event trigger enable.
		.enetrg   = false,
		// External event selection.
		.eevt     = 0,
		// External event edge selection.
		.eevtedg  = TC_SEL_NO_EDGE,
		// Counter disable when RC compare.
		.cpcdis   = false,
		// Counter clock stopped with RC compare.
		.cpcstop  = false,
		// Burst signal selection.
		.burst    = false,
		// Clock inversion.
		.clki     = false,
		// Internal source clock 3, connected to fPBA / 8.
		.tcclks   = TC_CLOCK_SOURCE_TC3
	};

	// Options for enabling TC interrupts
	static const tc_interrupt_t tc_interrupt = {
		.etrgs = 0,
		.ldrbs = 0,
		.ldras = 0,
		.cpcs  = 1, // Enable interrupt on RC compare alone
		.cpbs  = 0,
		.cpas  = 0,
		.lovrs = 0,
		.covfs = 0
	};
	// Initialize the timer/counter.
	tc_init_waveform(tc, &waveform_opt);

	/*
	 * Set the compare triggers.
	 * We configure it to count every 1 milliseconds.
	 * We want: (1 / (fPBA / 8)) * RC = 1 ms, hence RC = (fPBA / 8) / 1000
	 * to get an interrupt every 10 ms.
	 */
	tc_write_rc(tc, 0, (sysclk_get_pba_hz() / 8 / 1000));
	// configure the timer interrupt
	tc_configure_interrupts(tc, 0, &tc_interrupt);
	// Start the timer/counter.
	tc_start(tc, 0);
}

#if __GNUC__
__attribute__((__interrupt__))
#elif __ICCAVR32__
__interrupt
#endif
static void got_rfd_byte(void) {
	Disable_global_interrupt();
	int val;
	char c;
	
	usart_read_char(&RFD_USART, &val);
	
	c = (char) val;
	//do
	pwm_start_channels(1 << BUZZER_PWM);
	
	if(c == 'S') {
		writeFusebit(GOFAST_BIT);	
		//pwm_start_channels(1 << BUZZER_PWM);
	}
	
	if(c == 'p') {
		eraseFusebit(GOFAST_BIT);
		//pwm_start_channels(1 << BUZZER_PWM);
	}
	
	Enable_global_interrupt();
	
	RFD_USART.idr = 1;
	RFD_USART.ier = 1;
}

void init_rfd_receive() {
	Disable_global_interrupt();
		
	INTC_register_interrupt(&got_rfd_byte, RFD_USART_IRQ, AVR32_INTC_INT0);
	
	RFD_USART.idr = AVR32_USART_IER_RXRDY_MASK;
	RFD_USART.ier = AVR32_USART_IER_RXRDY_MASK;
		
	Enable_global_interrupt();
}

void init_adc() {
	AVR32_ADC.mr |= 0x1 << AVR32_ADC_MR_PRESCAL_OFFSET;
	adc_configure(&AVR32_ADC);
	
	adc_enable(&AVR32_ADC, GPS_VBACKUP_ADC);
	adc_enable(&AVR32_ADC, BATTERY_ADC);
	
}

uint64_t millis() {
	return millis_value;
}

uint64_t realTime() {
	return millis_value - millis_time_linked + real_time_linked;
}

// Board init
void initialize_board() {
	// Initialize and enable interrupts
	irq_initialize_vectors();
	cpu_irq_enable();
	
	init_gpio();
	init_usarts();
	init_pwm();
	#ifndef DISABLE_BMP
		init_bmp();
	#endif
	init_adc();
	init_watchdog();
	
	//Initialize interrupt-using modules.
	INTC_init_interrupts();
	init_eic();
	#ifndef IS_SECOND_BOARD
	init_gps();
	#endif
	init_rfd_receive();
	init_tc();
	
	#ifdef SD_ENABLE
	sd_mmc_resources_init();
	#endif
	
	//cpu_irq_enable();
	
	// Start USB stack to authorize VBus monitoring
	#ifdef EN_USB
	udc_start();
	my_callback_cdc_enable();
	#endif
}
