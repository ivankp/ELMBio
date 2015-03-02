/* ------------------------------------------------------------------------
File   : spi.c

Descr  : Software SPI serial interface routines.

History: 19JAN.00; Henk B&B; Definition.
         27SEP.00; Henk B&B; Rewrote functions to get more even bit stream.
	                     (added spi_clk()).
	 06AUG.01; Henk B&B; Replaced spi_clk() by macro.
--------------------------------------------------------------------------- */

#include "general.h"

/* Provide clock signal */
#define spi_clk()  SET_SCLK(); NOP(); CLEAR_SCLK(); NOP()

/* ------------------------------------------------------------------------ */

BYTE spi_read( void )
{
  BYTE byt;

  byt = 0;

  /* Read 8 bits */

  if( SDO_SET() ) byt += 0x80 ;  /* Bit 7 */
  spi_clk();

  if( SDO_SET() ) byt += 0x40;  /* Bit 6 */
  spi_clk();

  if( SDO_SET() ) byt += 0x20;  /* Bit 5 */
  spi_clk();

  if( SDO_SET() ) byt += 0x10;  /* Bit 4 */
  spi_clk();

  if( SDO_SET() ) byt += 0x08;  /* Bit 3 */
  spi_clk();

  if( SDO_SET() ) byt += 0x04;  /* Bit 2 */
  spi_clk();

  if( SDO_SET() ) byt += 0x02;  /* Bit 1 */
  spi_clk();

  if( SDO_SET() ) byt += 0x01;  /* Bit 0 */
  spi_clk();

  /* Return data byte */
  return byt;
}

/* ------------------------------------------------------------------------ */

void spi_write( BYTE byt )
{
  /* Write 8 bits */

  /* Bit 7 */
  if( byt & 0x80 ) SET_SDI();
  else CLEAR_SDI();
  spi_clk();

  /* Bit 6 */
  if( byt & 0x40 ) SET_SDI();
  else CLEAR_SDI();
  spi_clk();

  /* Bit 5 */
  if( byt & 0x20 ) SET_SDI();
  else CLEAR_SDI();
  spi_clk();

  /* Bit 4 */
  if( byt & 0x10 ) SET_SDI();
  else CLEAR_SDI();
  spi_clk();

  /* Bit 3 */
  if( byt & 0x08 ) SET_SDI();
  else CLEAR_SDI();
  spi_clk();

  /* Bit 2 */
  if( byt & 0x04 ) SET_SDI();
  else CLEAR_SDI();
  spi_clk();

  /* Bit 1 */
  if( byt & 0x02 ) SET_SDI();
  else CLEAR_SDI();
  spi_clk();

  /* Bit 0 */
  if( byt & 0x01 ) SET_SDI();
  else CLEAR_SDI();
  spi_clk();
}

/* ------------------------------------------------------------------------ */
