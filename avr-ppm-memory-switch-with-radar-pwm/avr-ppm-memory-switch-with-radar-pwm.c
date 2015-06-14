/*
 *
 * avr-ppm-memory-switch-with-radar-pwm.c
 *
 * Copyright (C) 2014 - 2015  Barnard33
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

#define AVG_PULSE_COUNT 10

#include <avr/io.h>
#include <util/delay.h>

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

uint16_t forward_on_pulse_length = 0;
uint16_t backwards_on_pulse_length = 0;
uint16_t diff40;

uint16_t pulse_length = 0;
uint16_t pulse_length_avg = 0;

uint8_t was_last_on = 0;
uint8_t pulse_count = 0;

uint8_t ppm_in = 0;
uint8_t last_ppm_in = 0;

uint8_t do_setup = 1;

int main(void) {
	//init pwm for radar gear motor
	init_timer0_pwm();
	set_oc0a(4);
	connect_oc0a();

	DDRB &= ~(1 << DDB1); // define PB1 as input for ppm
	
	DDRB |= (1 << DDB3) | (1 << DDB4);  // define PB3 and PB4 as out pins
	PORTB = (1 << PORTB3); // set PB3 to logical one; //sorry, currently no transistor applied to the breadboard, therefore the signal has to be inverted to turn the LED off
	PORTB &= ~(1 << PORTB4); // set PB4 to logical zero;
	
	//hopefully, the receiver has bound after a second and sends the ppm signal of the neutral position
	_delay_ms(1000); //here, decreased resolution is sufficient, see http://www.atmel.com/webdoc/AVRLibcReferenceManual/group__util__delay_1gad22e7a36b80e2f917324dc43a425e9d3.html
	
	//start in a defined condition, i.e., wait until a possibly active ppm pulse is over
	while(PINB & (1 << PINB1)) {
		//do nothing;
		asm ( "NOP" );
	}
	
    while(1)
    {
		//read status of pin B1
		ppm_in = PINB & (1 << PINB1);
		
		if(ppm_in && last_ppm_in) {
			//last pulse and current pulse remained high level: count pulse length
			pulse_length++;
		}
		else if(ppm_in && !last_ppm_in) {
			//rising edge: reset pulse length counter
			last_ppm_in = 1;
			pulse_length = 0;
		}
		else if(!ppm_in && last_ppm_in) {
			//falling edge: evaluate pulse length
			last_ppm_in = 0;
			
			pulse_count++;
			pulse_length_avg += pulse_length / AVG_PULSE_COUNT;
			
			if(pulse_count >= AVG_PULSE_COUNT) {
				pulse_count = 0;
					
				if(do_setup) {
					do_setup = 0;
					/* 
					 * A simple setup routine that uses five ppm pulses to average the pulse length in neutral position.
					 * It is assumed that the joystick is in neutral position when these pulses are measured during start up.
					 * A difference of 50% to the neutral position is considered to toggle one of the two channels.
					 * The two threshold values are stored in the global variables <code>forward_on_pulse_length</code> and
					 * <code>backwards_on_pulse_length</code>
					 * 
					 * (((neutral_pulse_length * 20) / 15) - neutral_pulse_length) should be expressed as neutral_pulse_length / 3
					 * 
					 * Why divided by 3? The PPM neutral position signal is usually 1.5ms long.
					 * It varies between +/- 0.5ms for the backwards/forwards signal, i.e., the full astern signal may be 1.0ms long and the full ahead signal 2.0ms.
					 * The exact value depends on the trimming capabilities of your transmitter/receiver.
					 */
					diff40 = (((((pulse_length_avg * 20) / 15) - pulse_length_avg) * 5) / 10);
					forward_on_pulse_length = pulse_length_avg + diff40;
					backwards_on_pulse_length = pulse_length_avg - diff40;
				}
				else {
					if((backwards_on_pulse_length < pulse_length_avg) && (pulse_length_avg < forward_on_pulse_length)) {
						// if the joystick returns to neutral position,
						// moving it forwards/backwards again should toggle the switch
						was_last_on = 0;
					}
					else if((pulse_length_avg < backwards_on_pulse_length) && !was_last_on) {
						PORTB ^= (1 << PORTB4); // toggle PB4
						was_last_on = 1;
					}
					else if((pulse_length_avg > forward_on_pulse_length) && !was_last_on) {
						PORTB ^= (1 << PORTB3); // toggle PB3
						was_last_on = 1;
					}	
				}
				
				pulse_length_avg = 0;
			}
		}
    }
}