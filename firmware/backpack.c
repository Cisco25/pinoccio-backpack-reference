//Pinoccio Backpack ATtiny13 Firmware
//
//Author: Cisco
//
//** Preliminary version **
//	08-14-2013
//	- Re-assigned ATtiny pinmap to match backpack pinmap
//	- Implemented reception of a byte from host (tested with Arduino Micro)
//	- Using byte value to control timer delay
//	- Code size = 460 bytes
//

// AVR IO include (must be called before delay.h)
#include <avr/io.h>

// AVR Delay include
#include <util/delay.h>

// AVR Interrupt include
#include <avr/interrupt.h>

// EEPROM include
#include "eeprom.h"

#ifndef F_CPU
	#define F_CPU 128000UL // Define software reference clock
#endif

#if defined(__AVR_ATtiny13A__) || defined(__AVR_ATtiny13__)
	// ** EXTERNAL INTERRUPTS MACROS **
	//#define EXTINT_EN 	{GIMSK|=(1<<INT0); GIFR|=(1<<INTF0);}  	// Enable External interrupts
	#define EXTINT_EN 	{GIMSK|=(1<<PCIE); GIFR|=(1<<PCIF);}  		// Enable Pin interrupts

	//#define EXTINT_DIS	GIMSK&=~(1<<INT0);  					// Disable External interrupts
	#define EXTINT_DIS	GIMSK&=~(1<<PCIE);  						// Disable Pin interrupts

	#define SET_LOWL 	MCUCR=0; 							// Set external interrupt on low level
	#define SET_ANYEDGE MCUCR=(1<<ISC00); 					// Set external interrupt on any edge
	#define SET_RISING 	MCUCR=(1<<ISC01)|(1<<ISC00);  		// Set external interrupt at rising edge
	#define SET_FALLING MCUCR=(1<<ISC01); 					// Set external interrupt at falling edge
	#define EXTINT_CHK	(GIMSK&(1<<INT0))==(1<<INT0) 		// Read interrupt state

	//#define EXTINT_VECT ISR(INT0_vect)  					// External interrupt service routine
	#define EXTINT_VECT ISR(PCINT0_vect)  					// Pin interrupt service routine
	// **

	// ** TIMER INTERRUPTS MACROS **
	#define TIMER_EN 	{TIMSK0|=(1<<TOIE0); TIFR0|=(1<<TOV0);}	// Enable timer interrupt
	#define TIMER_DIS 	TIMSK0&=~(1<<TOIE0); 					// Disable timer interrupt
	#define TIMER_VAL	TCNT0  				// Timer-counter register
	#define TIMER_VECT 	ISR(TIM0_OVF_vect) 	// Timer interrupt service routine
	// **

	// ** AVR INITIALIZATION MACRO **
	#define INIT_AVR 	CLKPR=(1<<CLKPCE);	/* To enable write of Clock Prescaler */ 		\
						CLKPR=0;			/* Clock Prescaler = 0 (Clock = 128kHz) */ 		\
						GIMSK=0;			/* Init GIMSK register (external interrupt) */ 	\
						PCMSK=(1<<PCINT0);	/* Configure PB0 as an interrupt pin source */	\
						TIMSK0=0;			/* Init TIMSK register (timer interrupt) */		\
						TCCR0A=0;			/* Timer Interrupt in Normal operation mode */	\
						TCCR0B=(1<<CS02);	//Set Timer Interrupt Prescaler to 256 (1 cycle = 256/128000 = 2ms)
	// **
#endif

#define BKPK_BUS	PB0 // Define Backpack Bus pin on PB0
#define LED			PB2	// Define Led on PB2
#define LED2		PB3	// Define Led2 on PB4

// Global Variables
uint8_t timerCount = 0x00;
uint8_t intCount = 0x00;

uint8_t data = 0x00;
uint8_t delay = 0x00;

uint8_t state = 0xFF;

// ** EXTERNAL/PIN INTERRUPT ROUTINE **
EXTINT_VECT {
	if (bit_is_clear(PINB, BKPK_BUS)) {	// If BKPK_BUS pin is low
		PORTB &= ~(1<<LED2);			// Turn off Led2

		if (state==0xFF)				// If state is in timer mode
			state = 0; 					// Switch state to read mode

		EXTINT_DIS;						// Disable external interrupts

		TCCR0B = (1<<CS00);	 			// Disable Timer Interrupt Prescaler (1 cycle = 1/128000 = 7.8125us)
		TIMER_VAL = 240;				// delay = 16 * 7.8125 = 125us
		TIMER_EN;						// Enable timer interrupts
	}
}
// **

// ** TIMER INTERRUPT ROUTINE **
TIMER_VECT {
	switch(state) {
		case 0:			// Read mode: read incoming byte on Backpack bus and assign value to delay
			TIMER_DIS;						// Disable timer interrupt: Next bit should be read after pin interrupt only

			if (bit_is_set(PINB, BKPK_BUS)) // If backpack bus is high: write 1 to the corresponding data bit
				data += (1<<intCount);		// else: do not modify data value

			intCount++;						// Increment interrupt counter (bit number)

			if(intCount > 7) {				// If we received all bits
				PORTB |= (1<<LED2);			// Data received OK: turn on Led2

				delay = data;				// Assign data value to delay
				state = 0xFF;				// Switch to Timer mode

				data = 0x00;				// Reset data
				intCount = 0x00;			// Reset interrupt counter (bit number)

				TCCR0B = (1<<CS02);			// Set Timer Interrupt Prescaler to 256 (1 cycle = 256/128000 = 2ms)
				TIMER_VAL = delay;			// Set new timer value
				TIMER_EN;					// Enable timer interrupts
			}

			EXTINT_EN;						// Enable external interrupts
			break;

		case 0xFF: 		// Timer mode: blink Led according to delay value
			PORTB ^= (1<<LED);				// Toggle Led pin
			TIMER_VAL = delay;				// Set timer value

			if (bit_is_set(PINB, LED2)) {	// If Led2 is on (data reception confirmation)
				timerCount++;				// Increment timer counter
				if(timerCount > 6) {		// If greater than 6
					PORTB &= ~(1<<LED2);	// Turn off Led2
					timerCount = 0x00;		// Reset timer counter
				}
			}
			break;

		default:		// Do nothing
			break;
	}
}
// **

// ** MAIN LOOP **
int main (void) {
	DDRB &= ~(1<<BKPK_BUS); 		// Set input direction on Backpack Bus
	DDRB |= (1<<LED)|(1<<LED2); 	// Set output direction on LEDs

	PORTB &= ~(1<<LED)|~(1<<LED2); 	// Set 0 on LED pins (turn leds off)

	INIT_AVR;						// Initializes ATtiny registers

	EEPROM_write(0x00, 0x6A);		// Write delay value in eeprom address 0x00
									// 0x6A => 300ms (256 - 106 = 150 * 2ms = 300ms)
	delay = EEPROM_read(0x00);		// Read delay value

	TIMER_VAL = delay;				// Initialize timer with delay value
	TIMER_EN;						// Enable timer interrupts

	EXTINT_EN;						// Enable external Interrupt

	sei(); 				// Enable Global interrupts

	for (;;) {			// Empty loop is required (else interrupts would not trigger)
	}

	return 0;
}
// **
