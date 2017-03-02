/**
* @author Alexander Entinger, MSc / LXRobotics
* @brief reads the input from the remote control receiver
* @file rc_input.c
* @license
*/

/* INCLUDES */


#include "rc_input.h"

#include <avr/io.h>
#include <avr/interrupt.h>

#include "hal.h"

/* MACROS */

#define TRIGGER_AT_RISING_EDGE()  do { EICRA &= 0x0C; EICRA |= (1<<ISC01) | (1<<ISC00); } while(0)
#define TRIGGER_AT_FALLING_EDGE() do { EICRA &= 0x0C; EICRA |= (1<<ISC01); } while(0)
	
/* GLOBAL CONSTANTS */

static uint8_t const MIN_PULSES_PER_TIMER_CYCLE = 10; // 262 ms / 20 ms = 13 (-3 to give a little room for error)
static uint16_t const MIN_PULSE_WIDTH_US = 1000;
static uint16_t const MAX_PULSE_WIDTH_US = 2000;

/* TYPEDEFS */

typedef enum {RISING, FALLING} E_PULSE_STATE;

typedef struct {
	E_PULSE_STATE pulse_state;
	uint8_t pulses_received;
	uint16_t last_good_pulse_duration_us;
	bool is_channel_good;
} s_pulse_property;

/* GLOBAL VARIABLES */
static s_pulse_property _rc_input = {RISING, 0, 0, false};

/* FUNCTIONS */

/** 
 * @brief initialize the rc input functionality
 */
void rc_input_init()
{
	// set the channel pins as input ...
	RC_DDR &= ~RC_bm;
	// ... with pullup
	RC_PORT |= RC_bm;
	
	// setup the trigger
	TRIGGER_AT_RISING_EDGE();
	
	// enable external interrupts
	EIMSK = (1<<INT0);
	
	// clear timer
	TCNT1 = 0;
	// enable timer 1 overflow interrupt
	TIMSK1 = (1<<TOIE1);
	// prescaler = 8
	// fTimer = fCPU / 64 = 16 MHz / 64 = 250 kHz
	// tTimerStep = 4 us
	// 2^16 * tTimerStep = 262.144 ms
	TCCR1B = (1<<CS11) | (1<<CS10);	
}

/** 
 * @brief returns the last good measured pulse widh in us
 */
uint16_t rc_input_get_pulse_width_us()
{
	return _rc_input.last_good_pulse_duration_us;
}

/** 
 * @brief returns true if the channel is good, false otherwise
 */
bool rc_input_is_channel_good()
{
	return _rc_input.is_channel_good;
}

/** 
 * @brief int0 (ch1) interrupt service routine
 */
ISR(INT0_vect) 
{
	static uint16_t start = 0;
	static uint16_t stop = 0;
	
	if(_rc_input.pulse_state == RISING) 
	{
		start = TCNT1;
		_rc_input.pulse_state = FALLING;
		TRIGGER_AT_FALLING_EDGE();
	} 
	else if(_rc_input.pulse_state == FALLING) 
	{
		stop = TCNT1;
		_rc_input.pulse_state = RISING;
		TRIGGER_AT_RISING_EDGE();
		
		uint16_t const pulse_duration_in_timer_steps = stop - start;
		uint16_t const timerstep_duration_in_us = 4;
		uint16_t const pulse_duration_in_us = pulse_duration_in_timer_steps * timerstep_duration_in_us;
		
		// only update when the value is within acceptable bounds
		if(pulse_duration_in_us >= MIN_PULSE_WIDTH_US && pulse_duration_in_us <= MAX_PULSE_WIDTH_US) 
		{
			_rc_input.last_good_pulse_duration_us = pulse_duration_in_us;
			
			_rc_input.pulses_received++;
		}
	}
}

/** 
 * @brief timer 1 overflow interrupt service routine
 */
ISR(TIMER1_OVF_vect) 
{
	// in case there has occured a loss of pulses ...
	bool const pulses_lost = _rc_input.pulses_received < MIN_PULSES_PER_TIMER_CYCLE;

	if(pulses_lost) 
	{
		// update the channel information
		_rc_input.pulse_state = RISING;
		TRIGGER_AT_RISING_EDGE();
		_rc_input.is_channel_good = false;
	}
	else
	{
		_rc_input.is_channel_good = true;		
	}

	// clear the pulse counters
	_rc_input.pulses_received = 0;
}
