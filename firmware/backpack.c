//Pinoccio Backpack ATtiny13 Firmware
//
//Author: Cisco
//
//** Preliminary version **
// 06-29-2013
//	- Delay accuracy test OK at 128 KHz
//	- External interrupt test OK
//	- Code size = 156 bytes
//


// AVR IO include (must be called before delay.h)
#include <avr/io.h>

// AVR Delay include
#include <util/delay.h>

// AVR Interrupt include
#include <avr/interrupt.h>

#ifndef F_CPU
	#define F_CPU 128000UL // Define software reference clock for delay duration
#endif

#if defined(__AVR_ATtiny13A__) || defined(__AVR_ATtiny13__)
	// ** EXTERNAL INTERRUPTS MACROS **
	#define EN_OWINT 	{GIMSK|=(1<<INT0);GIFR|=(1<<INTF0);}  	// Enable External interrupts
	#define DIS_OWINT	GIMSK&=~(1<<INT0);  					// Disable External interrupts
	#define SET_ANYEDGE MCUCR=(1<<ISC00); 					// Set external interrupt on any edge
	#define SET_RISING 	MCUCR=(1<<ISC01)|(1<<ISC00);  		// Set external interrupt at rising edge
	#define SET_FALLING MCUCR=(1<<ISC01); 					// Set external interrupt at falling edge
	#define CHK_INT_EN 	(GIMSK&(1<<INT0))==(1<<INT0) 		// Read interrupt status
	#define PIN_INT 	ISR(INT0_vect)  					// External interrupt service routine
	// **
#endif

#define TRIG	PB4	// Define external trigger on PB4
#define LED		PB0	// Define led on PB0

// ** EXTERNAL INTERRUPT FUNCTION **
PIN_INT {
	if (bit_is_set(PINB, TRIG))		// If TRIG pin is high
		PORTB |= (1<<LED); 			// Set 1 on LED pin (turn led on)

	if (bit_is_clear(PINB, TRIG))	// If TRIG pin is low
		PORTB &= ~(1<<LED); 		// Set 0 on LED pin (turn led off)
}
// **

// ** MAIN LOOP **
int main(void) {

	DDRB &= ~(1<<TRIG); // Set input direction on TRIG (PB4)
	DDRB |= (1<<LED); // Set output direction on LED (PB0)

	for(uint8_t count=0; count<5; count++) {
		PORTB |= (1<<LED); 		// Set 1 on LED pin (turn led on)
		_delay_ms(500); 		// 500ms delay
		PORTB &= ~(1<<LED); 	// Set 0 on LED pin (turn led off)
		_delay_ms(500); 		// 500ms delay
	}

	EN_OWINT;			// Enable External interrupts
	SET_ANYEDGE;		// Set external interrupt on any edge

	sei(); 				// Enable Global interrupts

	for (;;) {			// Empty loop is required (else interrupts would not trigger)
	}

	return 0;
}
// **
