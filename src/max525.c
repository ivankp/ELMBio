/* ------------------------------------------------------------------------
File   : max525.c

Descr  : Functions for operating a MAX525 4-channel 12-bit DAC.

History: 07AUG.01; Henk B&B; Start of development.
         19MAR.02; Henk B&B; Bug fix in 'max525_write_dac()'.
	 12NOV.03; Henk B&B; Use 'DacOptoDelay' in SPI operations.
--------------------------------------------------------------------------- */

#include "general.h"
#include "dac.h"
#include "max525.h"
#include "timer103.h"

extern BYTE DacOptoDelay;

/* ------------------------------------------------------------------------ */
/* Local prototypes */

static void max525_write_byte( BYTE byt );

/* ------------------------------------------------------------------------ */

void max525_write_dac( BYTE chan_no, BYTE *dac_data )
{
  /* Clear all but the data bits */
  dac_data[1] &= MAX525_DATABITS_MASK;

  /* Select channel and add control bits */
  dac_data[1] |= ((chan_no << MAX525_ADDRESSBITS_SHIFT) |
		  MAX525_CMD_LOAD_AND_UPDATE);

  /* Write 2 bytes, MSB first */
  max525_write_byte( dac_data[1] );
  max525_write_byte( dac_data[0] );
}

/* ------------------------------------------------------------------------ */

void max525_write_nop( void )
{
  /* Write 2 bytes, MSB first */
  max525_write_byte( 0x00 );
  max525_write_byte( 0x00 );
}

/* ------------------------------------------------------------------------ */

static void max525_write_byte( BYTE byt )
{
  BYTE i, b;

  b = byt;

  /* Clock the data out to the MAX DAC, MSB first */
  for( i=0; i<8; ++i )
    {
      if( b & 0x80 ) DAC_SET_SDI();
      else DAC_CLEAR_SDI();
      timer2_delay_mus( DacOptoDelay );
      DAC_SET_SCLK();
      timer2_delay_mus( DacOptoDelay );
      DAC_CLEAR_SCLK();
      b <<= 1;
    }
}

/* ------------------------------------------------------------------------ */
