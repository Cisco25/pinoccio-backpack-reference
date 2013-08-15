//Pinoccio Backpack ATtiny13 Firmware
//
//Author: Cisco
//
//** Preliminary version **
//	08-15-2013
//	- Moved Backpack bus on PB1 (INT0) and switched to external interrupts
//  - Moved counter value management from interrupt handlers to For Loop
//  - Tried to improve variable manipulation
//	- Still some data corruption when data has many zeros in it...
//	- Code size = 462 bytes
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
	#define EXTINT_EN 	{GIMSK|=(1<<INT0); GIFR|=(1<<INTF0);}  	// Enable External interrupts
	//#define EXTINT_EN 	{GIMSK|=(1<<PCIE); GIFR|=(1<<PCIF);}  		// Enable Pin Change interrupts

	#define EXTINT_DIS	GIMSK&=~(1<<INT0);  					// Disable External interrupts
	//#define EXTINT_DIS	GIMSK&=~(1<<PCIE);  						// Disable Pin Change interrupts

	#define SET_LOWL 	MCUCR=0; 							// Set external interrupt on low level
	#define SET_ANYEDGE MCUCR=(1<<ISC00); 					// Set external interrupt on any edge
	#define SET_RISING 	MCUCR=(1<<ISC01)|(1<<ISC00);  		// Set external interrupt at rising edge
	#define SET_FALLING MCUCR=(1<<ISC01); 					// Set external interrupt at falling edge
	#define EXTINT_CHK	(GIMSK&(1<<INT0))==(1<<INT0) 		// Read interrupt state

	#define EXTINT_VECT ISR(INT0_vect)  					// External interrupt service routine
	//#define EXTINT_VECT ISR(PCINT0_vect)  					// Pin interrupt service routine
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
						TIMSK0=0;			/* Init TIMSK register (timer interrupt) */		\
						TCCR0A=0;			/* Timer Interrupt in Normal operation mode */	\
						TCCR0B=(1<<CS02);	/* Set Timer Interrupt Prescaler to 256 (1 cycle = 256/128000 = 2ms) */
	// **
#endif

#define BKPK_BUS	PB1 // Define Backpack Bus pin on PB1 (INT0)
#define LED			PB2	// Define Led on PB2
#define LED2		PB3	// Define Led2 on PB4

// Global Variables
volatile uint8_t timerCount = 0x00;
volatile uint8_t intCount = 0x00;

volatile uint8_t data = 0x00;
volatile uint8_t delay = 0x00;

volatile uint8_t state = 0xFF;

// ** EXTERNAL/PIN INTERRUPT ROUTINE **
EXTINT_VECT {
	switch(state) {
		case 0x00:		// Read mode
			break;

		case 0xFF: 		// Timer mode
			PORTB &= ~(1<<LED)|~(1<<LED2);	// Turn Leds off
			state = 0x00; 					// Switch state to read mode
			TCCR0B = (1<<CS00);	 			// Disable Timer Interrupt Prescaler (1 cycle = 1/128000 = 7.8125us)
			break;

		default:		// Do nothing
			break;
	}

	TIMER_VAL = 224;				// delay = 32 * 7.8125 = 250us
	TIMER_EN;						// Enable timer interrupts
}
// **

// ** TIMER INTERRUPT ROUTINE **
TIMER_VECT {
	switch(state) {
		case 0x00:			// Read mode: read incoming byte on Backpack bus and assign value to delay
			TIMER_DIS;						// Disable timer interrupt: Next bit is read after pin interrupt only

			if (bit_is_set(PINB, BKPK_BUS)) // If backpack bus is high
				data |= (1<<intCount);		// Write 1 to the corresponding data bit

			intCount++;						// Increment interrupt counter (bit number)
			break;

		case 0xFF: 		// Timer mode: blink Led according to delay value
			PORTB ^= (1<<LED);				// Toggle Led state
			TIMER_VAL = delay;				// Set timer value

			if (bit_is_set(PINB, LED2))		// If Led2 is on (data received recently)
				timerCount++;				// Increment timer counter
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

	SET_FALLING;					// Setting external interrupts at falling edge
	EXTINT_EN;						// Enable external interrupts

	sei(); 							// Enable Global interrupts

	for (;;) {						// For Loop: manage counters

		if (intCount > 0x07) {		// If we received all data bits
			cli();					// Disable global interrupts
			PORTB |= (1<<LED2);		// Data received OK: turn Led2 on

			delay = data;			// Assign data value to delay
			data = 0x00;			// Reset data

			intCount = 0x00;		// Reset interrupt counter (bit number)

			state = 0xFF;			// Switch to Timer mode

			TCCR0B = (1<<CS02);		// Set Timer Interrupt Prescaler to 256 (1 cycle = 256/128000 = 2ms)
			TIMER_VAL = delay;		// Set new timer value
			TIMER_EN;				// Enable timer interrupts

			sei();					// Enable global interrupts
		}

		if(timerCount > 0x06) {		// If greater than 6
			PORTB &= ~(1<<LED2);	// Turn Led2 off
			timerCount = 0x00;		// Reset timer counter
		}

	}

	return 0;
}
// **
