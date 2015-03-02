/* ------------------------------------------------------------------------
File   : spi_gp.c

Descr  : Software SPI serial interface routines.

History: 20OCT.03; Henk B&B; Definition.
--------------------------------------------------------------------------- */

#include "general.h"
#include "spi_gp.h"
#include "timer103.h"

static BYTE SpiSelected       = 0;
static BYTE SpiSignalHoldTime = 10;
static BOOL SpiInOnRisingClk  = TRUE;

/* ------------------------------------------------------------------------ */

void spi_init( void )
{
  /* Initialize processor I/O pins involved in operation of the SPI devices.
     Note that in the ELMBio app the I/O pins are already initialized
     by the DAC initialization function */
  SPI_INIT_DDR();
  SPI_DESELECT_1();
  SPI_DESELECT_2();
  SPI_DESELECT_3();
  SPI_DESELECT_4();

  SpiSelected       = 0;
  SpiSignalHoldTime = 10;
  SpiInOnRisingClk  = TRUE;
}

/* ------------------------------------------------------------------------ */

BYTE spi_read_byte( void )
{
  BYTE i, b;

  b = 0;

  /* Clock the data in from the device, MSB first */
  if( SpiInOnRisingClk )
    {
      for( i=0; i<8; ++i )
	{
	  b <<= 1;
	  SPI_SET_SCLK();
	  timer2_delay_mus( SpiSignalHoldTime );
	  if( SPI_SDO_HIGH() ) ++b;
	  SPI_CLEAR_SCLK();
	  timer2_delay_mus( SpiSignalHoldTime );
	}
    }
  else
    {
      for( i=0; i<8; ++i )
	{
	  b <<= 1;
	  SPI_CLEAR_SCLK();
	  timer2_delay_mus( SpiSignalHoldTime );
	  if( SPI_SDO_HIGH() ) ++b;
	  SPI_SET_SCLK();
	  timer2_delay_mus( SpiSignalHoldTime );
	}
    }

  return b;
}

/* ------------------------------------------------------------------------ */

void spi_write_byte( BYTE byt )
{
  BYTE i, b;

  b = byt;

  /* Clock the data out to the device, MSB first */
  if( SpiInOnRisingClk )
    {
      for( i=0; i<8; ++i )
	{
	  if( b & 0x80 ) SPI_SET_SDI();
	  else SPI_CLEAR_SDI();
	  timer2_delay_mus( SpiSignalHoldTime );
	  SPI_SET_SCLK();
	  timer2_delay_mus( SpiSignalHoldTime );
	  SPI_CLEAR_SCLK();
	  b <<= 1;
	}
    }
  else
    {
      for( i=0; i<8; ++i )
	{
	  if( b & 0x80 ) SPI_SET_SDI();
	  else SPI_CLEAR_SDI();
	  timer2_delay_mus( SpiSignalHoldTime );
	  SPI_CLEAR_SCLK();
	  timer2_delay_mus( SpiSignalHoldTime );
	  SPI_SET_SCLK();
	  b <<= 1;
	}
    }

  /* Set SDI to one when not writing:
     switches off opto-coupler, reduces power consumption */
  SPI_SET_SDI();
}

/* ------------------------------------------------------------------------ */

BOOL spi_set_chipselect( BYTE dev_no )
{
  /* Deselect all */
  SPI_DESELECT_1();
  SPI_DESELECT_2();
  SPI_DESELECT_3();
  SPI_DESELECT_4();
  SpiSelected = 0;
  switch( dev_no )
    {
    case 0:
      /* All remain deselected */
      break;
    case 1:
      SPI_SELECT_1();
      break;
    case 2:
      SPI_SELECT_2();
      break;
    case 3:
      SPI_SELECT_3();
      break;
    case 4:
      SPI_SELECT_4();
      break;
    default:
      /* Invalid select parameter */
      return FALSE;
    }
  SpiSelected = dev_no;
  return TRUE;
}

/* ------------------------------------------------------------------------ */

BYTE spi_get_chipselect( void )
{
  return SpiSelected;
}

/* ------------------------------------------------------------------------ */

BOOL spi_set_holdtime( BYTE holdtime )
{
  if( SpiSignalHoldTime < 10 ) return FALSE;
  SpiSignalHoldTime = holdtime;
  return TRUE;
}

/* ------------------------------------------------------------------------ */

BYTE spi_get_holdtime( void )
{
  return SpiSignalHoldTime;
}

/* ------------------------------------------------------------------------ */

BOOL spi_set_rising_clk( BOOL rising )
{
  if( rising > 1 ) return FALSE;
  SpiInOnRisingClk = rising;
  return TRUE;
}

/* ------------------------------------------------------------------------ */

BOOL spi_get_rising_clk( void )
{
  return SpiInOnRisingClk;
}

/* ------------------------------------------------------------------------ */
