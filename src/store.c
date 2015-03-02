/* ------------------------------------------------------------------------
File   : store.c

Descr  : Contains functions to store and retrieve parameter data to/from the
	 ATmega1xx microcontroller's onchip EEPROM.

	 Storage of a data block is organized as follows:
	 - the data blocks are stored at fixed memory offsets
	   (defined in the include file)
	 - there is an infoblock for each available data block
	   (at the beginning of the EEPROM memory) with some info concerning
	   the stored data, i.e.:
	   - one byte to signify a valid data block
	   - two bytes for storage of the 16-bit CRC value
	   - ..... (anything else ?)
	 - the actual stored data block consists of:
	   - one byte with the length (in bytes) of the parameter block
	   - the parameters itself as a block of bytes
	 - the CRC is calculated over the data block only.

	 Writing the parameter block goes as follows:
	  1. length word + data block are written to EEPROM
	     (and double-checked),
	  2. CRC of data block is calculated,
	  3. CRC is written to EEPROM (and double-checked),
	  4. 'Valid parameter block present'-byte is written to EEPROM
	     (and double-checked).

	 Reading the parameter block goes as follows:
	  1. 'Valid'-byte and CRC are read from EEPROM,
	  2. block length is read from EEPROM and compared to requested value,
	  2. data block is read from EEPROM,
	  3. CRC of data block is calculated,
	  4. Calculated CRC is compared to value stored in EEPROM.
	  5. The bytes in the data block are assigned to the proper
	     parameters (= application dependent, not done here)

History: 21SEP.00; Henk B&B; Start of development; based on similar code for
                             NIKHEF SPICAN modules.
	 10SEP.01; Henk B&B; Use BYTE as EEPROM address (so limited to 255).
	 27FEB.03; Henk B&B; Added parameter block for CAN.
--------------------------------------------------------------------------- */

#include "general.h"
#include "adc.h"
#include "can.h"
#include "crc.h"
#include "dac.h"
#include "digio.h"
#include "eeprom.h"
#include "guarding.h"
#include "objects.h"
#include "pdo.h"
#include "store.h"

/* Space for storage of the results of reading the data blocks from EEPROM */
static BYTE StoreReadStatus[STORE_BLOCK_CNT];

/* ------------------------------------------------------------------------ */
/* Local function prototypes */

static BOOL store_invalidate( BYTE store_index );
static BOOL write_and_check ( BYTE addr, BYTE byt );

/* ------------------------------------------------------------------------ */

BOOL store_save_parameters( BYTE subindex )
{
  BOOL result = TRUE;

  switch( subindex )
    {
    case OD_STORE_ALL:
      if( pdo_store_config()      == FALSE ) result = FALSE;
      if( guarding_store_config() == FALSE ) result = FALSE;
      if( can_store_config()      == FALSE ) result = FALSE;
      if( adc_store_config()      == FALSE ) result = FALSE;
      if( dac_store_config()      == FALSE ) result = FALSE;
      if( digio_store_config()    == FALSE ) result = FALSE;
      break;

    case OD_STORE_COMM_PARS:
      if( pdo_store_config()      == FALSE ) result = FALSE;
      if( guarding_store_config() == FALSE ) result = FALSE;
      if( can_store_config()      == FALSE ) result = FALSE;
      break;

    case OD_STORE_APP_PARS:
      if( adc_store_config()      == FALSE ) result = FALSE;
      if( dac_store_config()      == FALSE ) result = FALSE;
      if( digio_store_config()    == FALSE ) result = FALSE;
      break;

    case OD_STORE_CUSTOM_1:
      if( adc_store_deltas()      == FALSE ) result = FALSE;
      break;

    case OD_STORE_CUSTOM_2:
      if( adc_store_upperlimits() == FALSE ) result = FALSE;
      if( adc_store_lowerlimits() == FALSE ) result = FALSE;
      break;

    default:
      result = FALSE;
      break;
    }
  return result;
}

/* ------------------------------------------------------------------------ */

BOOL store_set_defaults( BYTE subindex )
{
  /* Invalidate parameter block in EEPROM storage */

  BOOL result = TRUE;

  switch( subindex )
    {
    case OD_STORE_ALL:
      if( store_invalidate( STORE_PDO )      == FALSE ) result = FALSE;
      if( store_invalidate( STORE_GUARDING ) == FALSE ) result = FALSE;
      if( store_invalidate( STORE_CAN )      == FALSE ) result = FALSE;
      if( store_invalidate( STORE_ADC )      == FALSE ) result = FALSE;
      if( store_invalidate( STORE_DAC )      == FALSE ) result = FALSE;
      if( store_invalidate( STORE_DIGIO )    == FALSE ) result = FALSE;
      break;

    case OD_STORE_COMM_PARS:
      if( store_invalidate( STORE_PDO )      == FALSE ) result = FALSE;
      if( store_invalidate( STORE_GUARDING ) == FALSE ) result = FALSE;
      if( store_invalidate( STORE_CAN )      == FALSE ) result = FALSE;
      break;

    case OD_STORE_APP_PARS:
      if( store_invalidate( STORE_ADC )      == FALSE ) result = FALSE;
      if( store_invalidate( STORE_DAC )      == FALSE ) result = FALSE;
      if( store_invalidate( STORE_DIGIO )    == FALSE ) result = FALSE;
      break;

    case OD_STORE_CUSTOM_1:
      if( adc_invalidate_deltas()            == FALSE ) result = FALSE;
      break;

    case OD_STORE_CUSTOM_2:
      if( adc_invalidate_upperlimits()       == FALSE ) result = FALSE;
      if( adc_invalidate_lowerlimits()       == FALSE ) result = FALSE;
      break;

    default:
      result = FALSE;
      break;
    }
  return result;
}

/* ------------------------------------------------------------------------ */

void store_check_load_status( void )
{
  /* Function to check (afterwards) the status of all
     parameter read operations from EEPROM */
  BYTE block_no;
  for( block_no=0; block_no<STORE_BLOCK_CNT; ++block_no )
    {
      if( StoreReadStatus[block_no] != 0 )
	{
	  /* CANopen Error Code 0x5000: device hardware;
	     EEPROM read error: further info in status byte  */
	  can_write_emergency( 0x00, 0x50, EMG_EEPROM_READ_PARS, block_no,
			       StoreReadStatus[block_no],
			       0, ERRREG_MANUFACTURER );
	}
    }
}

/* ------------------------------------------------------------------------ */

BOOL store_write_block( BYTE store_index,
		        BYTE size,
		        BYTE *block )
{
  /* Data blocks up to 254 bytes size are stored by this function */
  UINT16 crc;
  BYTE   ee_offs;
  BYTE   i, byt;
  BOOL   result = TRUE;

  /* Make sure it's going to fit */
  if( size > STORE_BLOCK_SIZE-1 ) return FALSE;

  /* Determine address of data block */
  ee_offs = STORE_DATA_ADDR + (store_index * STORE_BLOCK_SIZE) + 1;

  /* Store length byte in EEPROM and check */
  if( !write_and_check( ee_offs - 1, size ) ) result = FALSE;

  /* Store data bytes in EEPROM and check */
  for( i=0; i<size; ++i )
    if( !write_and_check( ee_offs + i, block[i] ) ) result = FALSE;

  if( result == TRUE )
    {
      /* Determine address of info block */
      ee_offs = STORE_INFO_ADDR + (store_index * STORE_INFO_SIZE);

      /* Calculate CRC */
      crc = crc16_ram( block, size );

      /* Store CRC in EEPROM and check */
      byt = (BYTE) (crc & 0x00FF);
      if( !write_and_check( ee_offs + 1, byt ) ) result = FALSE;
      byt = (BYTE) ((crc & 0xFF00) >> 8);
      if( !write_and_check( ee_offs + 2, byt ) ) result = FALSE;

      if( result == TRUE )
	{
	  /* Store 'valid' and check it */
	  if( !write_and_check( ee_offs, STORE_VALID_CHAR ) ) result = FALSE;
	}
    }

  if( result == FALSE )
    {
      /* CANopen Error Code 0x5000: device hardware */
      can_write_emergency( 0x00, 0x50, EMG_EEPROM_WRITE_PARS,
			   store_index, size, 0, ERRREG_MANUFACTURER );

      store_invalidate( store_index );
    }

  return result;
}

/* ------------------------------------------------------------------------ */

BOOL store_read_block( BYTE store_index,
		       BYTE expected_size,
		       BYTE *block )
/* Data blocks up to 254 bytes size are retrieved
   from storage by this function */
{
  /* This function can not send CANopen EMERGENCY messages because
     at the time the data blocks are read the CAN interface
     possibly is not yet initialised or it might not yet be allowed
     to send any CAN messages...
     so any error that occurs in this routine is stored in the
     'StoreReadStatus' status byte array which should be checked
     by the application at a later stage */

  UINT16 crc;
  BYTE   ee_offs;
  BYTE   i, sz, byt;
  BOOL   result = FALSE;

  /* Determine address of info block */
  ee_offs = STORE_INFO_ADDR + (store_index * STORE_INFO_SIZE);

  /* Valid data block ? */
  if( eeprom_read( ee_offs ) == STORE_VALID_CHAR )
    {
      /* Get the CRC word */
      byt  = eeprom_read( ee_offs + 1 );
      crc  = (UINT16) byt;
      byt  = eeprom_read( ee_offs + 2 );
      crc |= (((UINT16) byt) << 8);

      /* Determine address of data block */
      ee_offs = STORE_DATA_ADDR + (store_index * STORE_BLOCK_SIZE) + 1;

      /* Read the length byte from EEPROM */
      sz = eeprom_read( ee_offs - 1 );

      /* Check if the length word makes sense */
      if( sz == expected_size )
	{
	  /* Read the data bytes from EEPROM */
	  for( i=0; i<sz; ++i ) block[i] = eeprom_read( ee_offs + i );

	  /* Check the CRC */
	  if( crc16_ram( block, sz ) == crc )
	    {
	      /* Okay! */
	      result = TRUE;
	      StoreReadStatus[store_index] = 0x00;
	    }
	  else
	    {
	      /* Error in CRC */
	      StoreReadStatus[store_index] = 0x01;
	    }
	}
      else
	{
	  /* Error in length word */
	  StoreReadStatus[store_index] = 0x02;
	}
    }
  else
    {
      /* If the data block is not valid the 'valid' byte and
	 the CRC bytes should all be equal to 0xFF */
      if( eeprom_read( ee_offs ) != 0xFF ||
	  eeprom_read( ee_offs+1 ) != 0xFF ||
	  eeprom_read( ee_offs+2 ) != 0xFF )
	/* Something is wrong with the info block */
	StoreReadStatus[store_index] = 0x04;
      else
	/* Everything in order */
	StoreReadStatus[store_index] = 0x00;
    }

  return result;
}

/* ------------------------------------------------------------------------ */

static BOOL store_invalidate( BYTE store_index )
{
  /* Write 0xFF to the 'valid-block' location and 0xFFFF to CRC location
     in EEPROM to invalidate parameters */

  BYTE i, ee_offs;
  BOOL result = TRUE;

  /* Determine address of requested infoblock */
  ee_offs = STORE_INFO_ADDR + (store_index * STORE_INFO_SIZE);

  /* Write 0xFF to all locations in the infoblock */
  for( i=0; i<STORE_INFO_SIZE; ++i )
    if( !write_and_check( ee_offs + i, 0xFF ) ) result = FALSE;

  if( result == FALSE )
    {
      /* CANopen Error Code 0x5000: device hardware */
      can_write_emergency( 0x00, 0x50, EMG_EEPROM_WRITE_PARS,
			   store_index, 0, 0, ERRREG_MANUFACTURER );
    }

  return result;
}

/* ------------------------------------------------------------------------ */

static BOOL write_and_check( BYTE addr, BYTE byt )
{
  /* Write byte to EEPROM */
  eeprom_write( addr, byt );

  /* Read it back */
  if( eeprom_read( addr ) != byt ) return FALSE;
  else return TRUE;
}

/* ------------------------------------------------------------------------ */
