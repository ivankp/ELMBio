/* ------------------------------------------------------------------------
File   : serialno.c

Descr  : Functions for reading and writing the ELMB Serial Number.

History: 09OCT.02; Henk B&B; Start of development.
--------------------------------------------------------------------------- */

#include "general.h"
#include "can.h"
#include "crc.h"
#include "eeprom.h"
#include "objects.h"
#include "store.h"

/* ------------------------------------------------------------------------ */
/* Globals */

/* To enable a single write to an calibration constant location in EEPROM */
static BOOL SerialNoWriteEnabled = FALSE;

/* ------------------------------------------------------------------------ */

BOOL sn_set_serial_number( BYTE *sn )
{
  /* Writes the contents of 'sn' into the EEPROM location which is
     to hold the ELMB's uniquely identifying Serial Number,
     which is important when production test parameters and
     calibration constants need to be looked up offline
     (in a database which will hold all this information). */
  BYTE   result = TRUE;
  BYTE   byt;
  UINT16 ee_addr;
  UINT16 crc;

  /* Has the write operation been enabled ? */
  if( (SerialNoWriteEnabled & TRUE) == FALSE ) return FALSE;

  /* Allow 1 write operation at a time */
  SerialNoWriteEnabled = FALSE;

  /* EEPROM address of the ELMB Serial Number */
  ee_addr = STORE_ELMB_SN_ADDR;

  /* Store and check the Serial Number, byte-by-byte */
  for( byt=0; byt<STORE_ELMB_SN_SIZE; ++byt, ++ee_addr )
    {
      BYTE sn_b;
      sn_b = sn[byt];
      if( eepromw_read( ee_addr ) != sn_b ) eepromw_write( ee_addr, sn_b );
      if( eepromw_read( ee_addr ) != sn_b ) result = FALSE; /* Check */
    }

  /* (Re)calculate CRC of the Serial Number */
  crc = crc16_eeprom( STORE_ELMB_SN_ADDR, STORE_ELMB_SN_SIZE );

  /* Store CRC, MSB first */
  byt = (BYTE) ((crc & 0xFF00) >> 8);
  ee_addr = STORE_ELMB_SN_ADDR + STORE_ELMB_SN_SIZE;
  eepromw_write( ee_addr, byt );
  if( eepromw_read(ee_addr) != byt ) result = FALSE; /* Check */
  /* And now the CRC LSB */
  byt = (BYTE) (crc & 0x00FF);
  ++ee_addr;
  eepromw_write( ee_addr, byt );
  if( eepromw_read(ee_addr) != byt ) result = FALSE; /* Check */

  /* The 'valid data' byte */
  ++ee_addr;
  if( result == TRUE )
    {
      /* Now we have a valid Serial Number ! */
      if( eepromw_read( ee_addr ) != STORE_VALID_CHAR )
	eepromw_write( ee_addr, STORE_VALID_CHAR );
      if( eepromw_read( ee_addr ) != STORE_VALID_CHAR ) /* Check */
	result = FALSE;
    }
  else
    {
      /* Mark it as invalid */
      eepromw_write( ee_addr, 0xFF );
    }

  if( result == FALSE )
    {
      /* CANopen Error Code 0x5000: device hardware */
      can_write_emergency( 0x00, 0x50, EMG_EEPROM_WRITE_PARS, 0xFF,
			   STORE_ELMB_SN_SIZE, 0, ERRREG_MANUFACTURER );
    }

  return result;
}

/* ------------------------------------------------------------------------ */

BOOL sn_get_serial_number( BYTE *sn )
{
  BYTE   byt, result = TRUE;
  UINT16 ee_addr;

  SerialNoWriteEnabled = FALSE;

  /* Initialize all bytes: assume there are 4 */
  for( byt=0; byt<4; ++byt ) sn[byt] = 0;

  /* Is there a valid Serial Number present in EEPROM ? */
  if( eepromw_read( STORE_ELMB_SN_VALID_ADDR ) == STORE_VALID_CHAR ) 
    {
      /* EEPROM address of the ELMB Serial Number */
      ee_addr = STORE_ELMB_SN_ADDR;

      /* Check the CRC (run the CRC on the datablock plus
	 the stored CRC value: the result should be zero) */
      if( crc16_eeprom( ee_addr, STORE_ELMB_SN_SIZE+2 ) == 0 )
	{
	  /* Copy the Serial Number to sn[], byte-by-byte */
	  for( byt=0; byt<STORE_ELMB_SN_SIZE; ++byt, ++ee_addr )
	    {
	      sn[byt] = eepromw_read( ee_addr );
	    }
	}
      else
	{
	  /* Error in CRC: report it */
	  /* CANopen Error Code 0x5000: device hardware */
	  can_write_emergency( 0x00, 0x50, EMG_EEPROM_READ_PARS, 0xFF,
			       1, 0, ERRREG_MANUFACTURER );
	  result = FALSE;
	}
    }
  else
    {
      /* No valid constants available */
      result = FALSE;
    }
  return result;
}

/* ------------------------------------------------------------------------ */

BOOL sn_serial_number_write_enable( BYTE val )
{
  /* Set the boolean only when 'val' has a particular value */
  SerialNoWriteEnabled = FALSE;
  if( val == 0x5A )
    {
      /* Enable a single write to ELMB Serial Number location in EEPROM */
      SerialNoWriteEnabled = TRUE;
    }
  return SerialNoWriteEnabled;
}

/* ------------------------------------------------------------------------ */
