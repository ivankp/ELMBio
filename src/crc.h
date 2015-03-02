/* ------------------------------------------------------------------------
File   : crc.h

Descr  : Declarations of ELMB CRC calculation functions.

History: 18AUG.00; Henk B&B; Definition.
	 15SEP.03; Henk B&B; Added crc_get().
--------------------------------------------------------------------------- */

#ifndef CRC_H
#define CRC_H

/* ------------------------------------------------------------------------ */
/* Function prototypes */

UINT16 crc16_ram   ( BYTE *byt, UINT16 size );
UINT16 crc16_eeprom( UINT16 addr, UINT16 size );
BOOL   crc_master  ( UINT16 *pcrc );
BOOL   crc_get     ( BYTE   *pcrc_byte );
BOOL   crc_slave   ( UINT16 *pcrc );

#endif /* CRC_H */
/* ------------------------------------------------------------------------ */
