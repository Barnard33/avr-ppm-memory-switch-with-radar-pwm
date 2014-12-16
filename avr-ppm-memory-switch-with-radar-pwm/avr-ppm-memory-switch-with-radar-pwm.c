/*
 *
 * avr-ppm-memory-switch-with-radar-pwm.c
 *
 * Copyright (C) 2014  Barnard33
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * 
 * This code is only intended to be used for educational purposes.
 * It is not production stable and must under no circumstance be used 
 * in any kind of radio controlled machinery, e.g., planes, cars, boats, etc.
 *
 * Created: 14.12.2014 10:38:53
 *
 */  

#ifndef F_CPU
#define F_CPU 9000000
#endif

#define STATUS_START 0
#define STATUS_COMPLETE 1
#define STATUS_STOP 2

#include <avr/io.h>
#include <avr/interrupt.h>

inline void init_timer0_pwm(void) {
	/* clear OC0A on compare match when up-counting
	   set OC0A on compare match when down-counting */
	TCCR0A |= (1 << COM0A1);
	
	/* phase correct PWM, 8-bit */
	TCCR0A |= (1 << WGM00);
	
	/* set clock to clkIO (no prescaler) */
	TCCR0B |= (1 << CS00);
}

inline void connect_oc0a(void) {
	DDRB |= (1 << DDB0);
}

inline void set_oc0a(uint8_t compare) {
	OCR0A = compare;
}

volatile uint8_t pulse_status;

uint16_t forward_on_pulse_length = 0;
uint16_t backwards_on_pulse_length = 0;

/**
 * A simple setup routine that uses five ppm pulses to average the pulse length in neutral position.
 * It is assumed that the joystick is in neutral position when these pulses are measured during start up.
 * A difference of 40% to the neutral position is considered to toggle one of the two channels.
 * The two threshold values are stored in the global variables <code>forward_on_pulse_length</code> and 
 * <code>backwards_on_pulse_length</code>
 */
void setup() {
	uint16_t pulse_length = 0;
	uint16_t neutral_pulse_length = 0;
	uint8_t setup_counter = 0;
	
	MCUCR |= (1 << ISC00) | (1 << ISC01); // rising edge on int0 triggers IRQ (for setup)
	pulse_status = 0;
	pulse_status |= (STATUS_STOP);
	
	sei();
	while(1) {
		if(!pulse_status) {
			pulse_length++;
		}
		else if(pulse_status & (1 << STATUS_START)) {
			// reset STATUS_STOP, which starts counting in the next cycle
			pulse_status = 0;
			pulse_length = 0;
		}
		else if(pulse_status & (1 << STATUS_COMPLETE)) {
			pulse_status &= ~(1 << STATUS_COMPLETE);
			pulse_status |= (1 << STATUS_STOP);
			
			neutral_pulse_length += pulse_length;
			setup_counter++;
			if(setup_counter == 5) {
				cli();
				neutral_pulse_length = neutral_pulse_length / (setup_counter - 1);
				uint16_t diff40 = (((((neutral_pulse_length * 20) / 15) - neutral_pulse_length) * 4) / 10);
				forward_on_pulse_length = neutral_pulse_length + diff40;
				backwards_on_pulse_length = neutral_pulse_length - diff40;
				break;
			}
			pulse_length = 0;
		}
	}
}

int main(void) {
	uint16_t pulse_length = 0;
	uint8_t was_last_on = 0;
	
	//init pwm for radar gear motor
	init_timer0_pwm();
	set_oc0a(10);
	connect_oc0a();

	DDRB &= ~(1 << DDB1); // define PB1 as input (for int0)
	
	DDRB |= (1 << DDB3) | (1 << DDB4);  // define PB3 and PB4 as out pins
	PORTB &= ~((1 << PORTB3) | (1 << PORTB4)); // set PB3 and PB4 to logical zero;
	
	GIMSK |= (1 << INT0); // enable interrupt source int0
	
	setup();
	
	MCUCR |= (1 << ISC00) | (1 << ISC01); // rising edge on int0 triggers IRQ (for normal operation)
	
	pulse_status = 0;
	pulse_status |= (1 << STATUS_STOP);
	
	sei();
	
    while(1)
    {
		if(!pulse_status) {
			pulse_length++;		
		}
		else if(pulse_status & (1 << STATUS_START)) {
			// reset STATUS_STOP, which starts counting in the next cycle
			pulse_status = 0;
			pulse_length = 0;
		}
		else if(pulse_status & (1 << STATUS_COMPLETE)) {
			pulse_status &= ~(1 << STATUS_COMPLETE);
			pulse_status |= (1 << STATUS_STOP);
			
			if((backwards_on_pulse_length < pulse_length) && (pulse_length < forward_on_pulse_length)) {
				// if the joystick returns to neutral position, 
				// moving it forwards/backwards again should toggle the switch
				was_last_on = 0; 
			}
			else if((pulse_length < backwards_on_pulse_length) && !was_last_on) {
				PORTB ^= (1 << PORTB4); // toggle PB4
				was_last_on = 1;
			}
			else if((pulse_length > forward_on_pulse_length) && !was_last_on) {
				PORTB ^= (1 << PORTB3); // toggle PB3
				was_last_on = 1;
			}
			pulse_length = 0;
		} 
    }
}

/* ISR for servo/ppm input pin on int0 */
ISR(INT0_vect) {
	if((MCUCR & (1<<ISC00)) > 0) {  // check if triggered on rising edge
		pulse_status |= (1 << STATUS_START);
		MCUCR &= ~(1 << ISC00); // falling edge on int0 triggers IRQ
	}
	else {
		MCUCR |= (1 << ISC00); // rising edge on int0 triggers IRQ
		pulse_status |= (1 << STATUS_COMPLETE);
	}
}