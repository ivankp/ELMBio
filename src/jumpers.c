/* ------------------------------------------------------------------------
File   : jumpers.c

Descr  : Functions for reading jumper settings.

History: 02DEC.99; Henk B&B; Definition.
         01DEC.00; Henk B&B; Compile option for 7-bit Node-ID (instead of 6).
--------------------------------------------------------------------------- */

#include "general.h"
#include "timer103.h"

/* ------------------------------------------------------------------------ */

BYTE read_nodeid( void )
{
  BYTE id;

  /* Select the jumpers to read out */
  NODEID_JUMPERS_SELECT();

  /* Let signal levels settle... (wait at least 1 ms) */
  timer2_delay_ms( 2 );

  /* Read jumpers via Port B */
  id = PINB & NODEID_JUMPERS_MASK;

  NODEID_JUMPERS_DESELECT();

#ifdef __7BIT_NODEID__
  BAUDRATE_JUMPERS_SELECT();
  timer2_delay_ms( 2 );
  /* The lower bit is used as the 7th bit of the Node-ID */
  if( PINB & 0x01 ) id |= 0x40;
  BAUDRATE_JUMPERS_DESELECT();
#endif

  return id;
}

/* ------------------------------------------------------------------------ */

BYTE read_baudrate( void )
{
  BYTE baud;

  /* Select the jumpers to read out */
  BAUDRATE_JUMPERS_SELECT();

  /* Let signal levels settle... (wait at least 1 ms) */
  timer2_delay_ms( 2 );

  /* Read jumpers via Port B */
  baud = PINB & BAUDRATE_JUMPERS_MASK;

  BAUDRATE_JUMPERS_DESELECT();

#ifdef __7BIT_NODEID__
  /* Only the upper bit is used for the baudrate selection */
  if( baud & 0x02 ) baud = BAUD125K;
  else baud = BAUD250K;
#endif

  return baud;
}

/* ------------------------------------------------------------------------ */
