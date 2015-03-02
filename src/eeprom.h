/* ------------------------------------------------------------------------
File   : eeprom.h

Descr  : Declarations of low-level EEPROM access functions.

History: 18AUG.00; Henk B&B; Definition.
	 24JUL.01;  "    " ; Go from UINT16 to BYTE addresses, which is alright
	                     upto 256 bytes of EEPROM storage...
	 01APR.02;  "    " ; Add read/write functions with 16-bit
	                     address parameter.
--------------------------------------------------------------------------- */

#ifndef EEPROM_H
#define EEPROM_H

/* ------------------------------------------------------------------------ */
/* Function prototypes */

/* Functions with 8-bit addresses (covers first 256 bytes of EEPROM) */
BYTE eeprom_read ( BYTE addr );
void eeprom_write( BYTE addr, BYTE byt );

/* Functions with 16-bit addresses (covers all of EEPROM) */
BYTE eepromw_read ( UINT16 addr );
void eepromw_write( UINT16 addr, BYTE byt );

#endif /* EEPROM_H */
/* ------------------------------------------------------------------------ */
