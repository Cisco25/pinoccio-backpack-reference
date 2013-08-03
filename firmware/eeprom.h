#ifndef _EEPROM_H_
#define _EEPROM_H_

// AVR IO include
#include <avr/io.h>

void EEPROM_write(uint8_t ucAddress, uint8_t ucData)
{
  /* Wait for completion of previous write */
	while(EECR & (1<<EEPE));
	/* Set Programming mode */
	EECR = (0<<EEPM1)|(0>>EEPM0);
	/* Set up address and data registers */
	EEARL = ucAddress;
	EEDR = ucData;
	/* Write logical one to EEMPE */
	EECR |= (1<<EEMPE);
	/* Start eeprom write by setting EEPE */
	EECR |= (1<<EEPE);
}

uint8_t EEPROM_read(uint8_t ucAddress)
{
	/* Wait for completion of previous write */
	while(EECR & (1<<EEPE));
	/* Set up address register */
	EEARL = ucAddress;
	/* Start eeprom read by writing EERE */
	EECR |= (1<<EERE);
	/* Return data from data register */
	return EEDR;
}

#endif /* _EEPROM_H_ */
