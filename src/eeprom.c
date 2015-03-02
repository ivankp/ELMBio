/* ------------------------------------------------------------------------
File   : eeprom.c

Descr  : ATmega EEPROM access routines.

History: 18AUG.00; Henk B&B; Definition.
	 24JUL.01;  "    " ; Go from UINT16 to BYTE addresses, which is alright
	                     upto 256 bytes of EEPROM storage...
	 01APR.02;  "    " ; Add read/write functions with 16-bit
	                     address parameter.
	 04JUN.03;  "    " ; CAN-interrupt enable/disable not necessary if
	                     EEPROM access by CAN interrupt routine is avoided.
--------------------------------------------------------------------------- */

#include "general.h"

/* ------------------------------------------------------------------------ */

BYTE eeprom_read( BYTE addr )
{
  /* Wait until any earlier write is done */
  while( EECR & BIT(EEWE) );

  EEARH = 0x00;
  EEARL = addr;

  /* Set READ strobe (EERE) */
  SETBIT( EECR, EERE );

  /* Return data byte */
  return( EEDR );
}

/* ------------------------------------------------------------------------ */

void eeprom_write( BYTE addr, BYTE byt )
{
  /* Wait until any earlier write is done */
  while( EECR & BIT(EEWE) );

  EEARH = 0x00;
  EEARL = addr;

  EEDR = byt;

  /* Disable interrupts when used ! (EEMWE times out) */
  CLI();

  /* Set MASTER WRITE enable (EEMWE) */
  SETBIT( EECR, EEMWE );

  /* Set WRITE strobe (EEWE) */
  SETBIT( EECR, EEWE );

  /* Enable interrupts again... */
  SEI();
}

/* ------------------------------------------------------------------------ */

BYTE eepromw_read( UINT16 addr )
{
  /* Wait until any earlier write is done */
  while( EECR & BIT(EEWE) );

  EEARH = (BYTE) ((addr & 0xFF00) >> 8);
  EEARL = (BYTE)  (addr & 0x00FF);

  /* Set READ strobe (EERE) */
  SETBIT( EECR, EERE );

  /* Return data byte */
  return( EEDR );
}

/* ------------------------------------------------------------------------ */

void eepromw_write( UINT16 addr, BYTE byt )
{
  /* Wait until any earlier write is done */
  while( EECR & BIT(EEWE) );

  EEARH = (BYTE) ((addr & 0xFF00) >> 8);
  EEARL = (BYTE)  (addr & 0x00FF);

  EEDR = byt;

  /* Disable interrupts when used ! (EEMWE times out) */
  CLI();

  /* Set MASTER WRITE enable (EEMWE) */
  SETBIT( EECR, EEMWE );

  /* Set WRITE strobe (EEWE) */
  SETBIT( EECR, EEWE );

  /* Enable interrupts again... */
  SEI();
}

/* ------------------------------------------------------------------------ */
