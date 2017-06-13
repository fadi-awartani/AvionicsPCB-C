/*
 * external_interrupts_buttons.c
 *
 * Created: Jun 8, 2017 9:54:57 PM
 *  Author: Fadi
 */ 
#include "main.h"

//Don't do anything if button press is within BUTTON_HOLDTIME ms of last press. For de-bouncing purposes.
#define BUTTON1_HOLDTIME 250
#define BUTTON2_HOLDTIME 250

long lastPressed[2];

#if __GNUC__
__attribute__((__interrupt__))
#elif __ICCAVR32__
__interrupt
#endif
static void button1_interrupt(void)
{
	eic_clear_interrupt_line(&AVR32_EIC, EXT_INT6);
	
	if(millis() < lastPressed[0] + BUTTON1_HOLDTIME)
		return;
		
	lastPressed[0] = millis();
	
	//turnOnGPS();
	pwm_start_channels(1 << BUZZER_PWM);
	
	usart_write_line(&GPS_USART, "$PMTK314,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n");
}

#if __GNUC__
__attribute__((__interrupt__))
#elif __ICCAVR32__
__interrupt
#endif
static void button2_interrupt(void)
{
	eic_clear_interrupt_line(&AVR32_EIC, EXT_INT7);
	
	if(millis() < lastPressed[1] + BUTTON1_HOLDTIME)
		return;
	
	lastPressed[1] = millis();
	
	pwm_stop_channels(1 << BUZZER_PWM);
	gpio_tgl_gpio_pin(RED_LED_PIN);
	
	usart_write_line(&GPS_USART, "$PMTK300,500,0,0,0,0*28\r\n");
}

void init_eic() {
	eic_options_t eic_options[2];
	
	// Enable edge-triggered interrupt.
	eic_options[0].eic_mode   = EIC_MODE_EDGE_TRIGGERED;
	// Interrupt will trigger on falling edge.
	eic_options[0].eic_edge  = EIC_EDGE_FALLING_EDGE;
	// Initialize in synchronous mode : interrupt is synchronized to the clock
	eic_options[0].eic_async  = EIC_SYNCH_MODE;
	// Set the interrupt line number.
	eic_options[0].eic_line   = EXT_INT6;
	
	// Enable edge-triggered interrupt.
	eic_options[1].eic_mode   = EIC_MODE_EDGE_TRIGGERED;
	// Interrupt will trigger on falling edge.
	eic_options[1].eic_edge  = EIC_EDGE_FALLING_EDGE;
	// Initialize in synchronous mode : interrupt is synchronized to the clock
	eic_options[1].eic_async  = EIC_SYNCH_MODE;
	// Set the interrupt line number.
	eic_options[1].eic_line   = EXT_INT7;
	
	Disable_global_interrupt();
	//INTC_init_interrupts();

	// Register the EIC interrupt handlers to the interrupt controller.
	INTC_register_interrupt(&button1_interrupt,AVR32_EIC_IRQ_6, AVR32_INTC_INT0);
	INTC_register_interrupt(&button2_interrupt,AVR32_EIC_IRQ_7, AVR32_INTC_INT0);
	
	eic_init(&AVR32_EIC, eic_options,2);
	
	eic_enable_lines(&AVR32_EIC,
		(1<<eic_options[1].eic_line)|(1<<eic_options[0].eic_line));
	eic_enable_interrupt_lines(&AVR32_EIC,
		(1<<eic_options[1].eic_line)|(1<<eic_options[0].eic_line));
		
	Enable_global_interrupt();
}