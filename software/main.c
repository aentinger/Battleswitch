/** 
 * @author Alexander Entinger, MSc / LXRobotics
 * @brief main file for the battleswitch software
 * @file main.c
 * @license
 */

/* INCLUDES */

#include <stdint.h>
#include <stdbool.h>

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include "rc_input.h"
#include "led_control.h"
#include "switch_control.h"

/* PROTOTYPES */

/** 
 * @brief initializes the application
 */
void init_application();

/* FUNCTIONS */

int main(void)
{
	typedef enum {INIT = 0, WAIT_FOR_SIGNAL = 1, ACTIVE = 2} E_BATTLESWITCH_STATE;
	E_BATTLESWITCH_STATE state = INIT;
	
	for(;;)
    {
        switch(state)
		{
		case INIT:
		{
			// call the initialize function
			init_application();
			
			// show init complete via led
			led_control_set_led(LED_OUT_1, LED_ON);
			led_control_set_led(LED_OUT_2, LED_ON);
			_delay_ms(1000);
			led_control_set_led(LED_OUT_1, LED_OFF);
			led_control_set_led(LED_OUT_2, LED_OFF);
			_delay_ms(1000);
			
			// switch to next state
			state = WAIT_FOR_SIGNAL;			
		}	
		break;
		case WAIT_FOR_SIGNAL:
		{
			// toogle leds to show we are waiting for a receiver signal
			static bool led_out_1_on = true;
			if(led_out_1_on)
			{
				led_control_set_led(LED_OUT_1, LED_ON);	
				led_control_set_led(LED_OUT_2, LED_OFF);
			}
			else
			{
				led_control_set_led(LED_OUT_1, LED_OFF);
				led_control_set_led(LED_OUT_2, LED_ON);				
			}
			led_out_1_on = !led_out_1_on;
			_delay_ms(100);
			
			// check if we have a stable signal (and if we have, transition to the active mode)
			if(rc_input_is_channel_good())
			{
				// turn leds off ... 
				led_control_set_led(LED_OUT_1, LED_OFF);
				led_control_set_led(LED_OUT_2, LED_OFF);
				
				// ... and switch to active state
				state = ACTIVE;
			}			
		}
		break;
		case ACTIVE:
		{
			if(rc_input_is_channel_good())
			{
				uint16_t const current_pulse_duration_us = rc_input_get_pulse_width_us();
				
				if(current_pulse_duration_us > 1750)
				{
					switch_control_set_switch(OUT_1, SWITCH_ON);
					
					led_control_set_led(LED_OUT_1, LED_ON);
				}
				else if(current_pulse_duration_us < 1250)
				{
					switch_control_set_switch(OUT_1, SWITCH_ON);
					switch_control_set_switch(OUT_2, SWITCH_ON);
					
					led_control_set_led(LED_OUT_1, LED_ON);
					led_control_set_led(LED_OUT_2, LED_ON);
				}
				else
				{
					switch_control_set_switch(OUT_1, SWITCH_OFF);
					switch_control_set_switch(OUT_2, SWITCH_OFF);
					
					led_control_set_led(LED_OUT_1, LED_OFF);
					led_control_set_led(LED_OUT_2, LED_OFF);
				}				
			}
			else
			{
				// turn outputs off ...
				switch_control_set_switch(OUT_1, SWITCH_OFF);
				switch_control_set_switch(OUT_2, SWITCH_OFF);
				
				// ... turn leds off ...
				led_control_set_led(LED_OUT_1, LED_OFF);
				led_control_set_led(LED_OUT_2, LED_OFF);
				
				// ... and fall back in the waiting for signal state 				
				state = WAIT_FOR_SIGNAL;
			}
		}
		break;
		default: break;
		}
    }
}


/** 
 * @brief initializes the application
 */
void init_application()
{
	// initializes led control
	led_control_init();
	
	// initialize switch control
	switch_control_init();
	
	// initialize the rc input unit
	rc_input_init();
	
	// globally enable interrupts
	sei();
}
