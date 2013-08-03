//Pinoccio Backpack ATtiny13 Firmware
//
//Author: Cisco
//
//** Preliminary version **
//	03-08-2013
//	- Added calls to eeprom functions
//	(contained in header file eeprom.h)
//	- Code size = 352 bytes
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
	#define EXTINT_CHK	(GIMSK&(1<<INT0))==(1<<INT0) 		// Read interrupt status

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
						PCMSK=(1<<PCINT4);	/* Configure PB4 as an interrupt pin source */	\
						TIMSK0=0;			/* Init TIMSK register (timer interrupt) */		\
						TCCR0A=0;			/* Timer Interrupt in Normal operation mode */	\
						TCCR0B=(1<<CS02);	/* Set Timer Interrupt Prescaler to 256
											   (1 cycle = 256/128000 = 2ms)
	**/
#endif

#define TRIG	PB4	// Define external trigger on PB4
#define LED		PB0	// Define led on PB0

// Global Variables
uint8_t TimerCount = 0x00;
uint8_t TrigCount = 0x00;
uint8_t delay = 0x00;

// ** EXTERNAL/PIN INTERRUPT ROUTINE **
EXTINT_VECT {
	if (bit_is_set(PINB, TRIG))		// If TRIG pin is high
		PORTB |= (1<<LED); 			// Set 1 on LED pin (turn led on)

	if (bit_is_clear(PINB, TRIG))	// If TRIG pin is low
		PORTB &= ~(1<<LED); 		// Set 0 on LED pin (turn led off)

	if (TrigCount < 10) {	// After 10 pin interrupt, switch to timer interrupt routine
		TrigCount++;
	}
	else {
		EXTINT_DIS;			// Disable External interrupts
		TrigCount = 0;
		TIMER_EN;			// Enable Timer
	}
}
// **

// ** TIMER INTERRUPT ROUTINE **
TIMER_VECT {
	PORTB ^= (1<<LED);		// Toggle LED pin

	if(TimerCount < 10) {	// After 10 timer interrupts, switch to pin interrupt routine
		delay = delay + 10; // Decrease delay
		TIMER_VAL = delay;	// Initialize timer with new delay value
		TimerCount++;
	}
	else {
		TIMER_DIS;			// Disable Timer
		TimerCount = 0;
		delay = EEPROM_read(0x00); // Retrieve init delay value
		EXTINT_EN;			// Enable External interrupts
	}
}
// **

// ** MAIN LOOP **
int main (void) {

	DDRB &= ~(1<<TRIG); // Set input direction on TRIG (PB4)
	DDRB |= (1<<LED); 	// Set output direction on LED (PB0)

	INIT_AVR;			// Initializes ATtiny registers

	SET_ANYEDGE;		// Set External interrupt on any edge

	PORTB |= (1<<LED); 	// Set 1 on LED pin (turn led on)


	EEPROM_write(0x00, 106);	// Setup starting delay to 300ms (256 - 106 = 150 * 2ms = 300ms)
								// And write in eeprom
	delay = EEPROM_read(0x00);	// Init delay value
	TIMER_VAL = delay;			// Initialize timer with delay value

	TIMER_EN;			// Enable Timer

	sei(); 				// Enable Global interrupts

	//No more needed (using timer interrupts)
	//for(uint8_t count=0; count<5; count++) {
	//	PORTB |= (1<<LED); 		// Set 1 on LED pin (turn led on)
	//	_delay_ms(500); 		// 500ms delay
	//	PORTB &= ~(1<<LED); 	// Set 0 on LED pin (turn led off)
	//	_delay_ms(500); 		// 500ms delay
	//}

	for (;;) {			// Empty loop is required (else interrupts would not trigger)
	}

	return 0;
}
// **
