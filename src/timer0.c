/* ------------------------------------------------------------------------
File   : timer0.c

Descr  : ATMEL AVR microcontroller Timer/Counter0 routines.

History: 23MAR.01; Henk B&B; Timer0 used for time-outs on operations.
--------------------------------------------------------------------------- */

#include "general.h"
#include "timer103.h"

/* Counters for time-outs */
static BYTE T0_Countdown[T0_CLIENTS];

/* ------------------------------------------------------------------------ */

void timer0_init( void )
{
  /* Initialize Timer0 for our purposes: time-outs;
     use a 'clocktick' of 10 milliseconds */

  /* Set to a 10 ms clocktick */
  SET_TIMER0_10MS();

  /* Enable Timer0 interrupt */
  TIMSK |= BIT( T0_OVERFLOW_IE );

  /* Global interrupts enabled elsewhere... */
}

/* ------------------------------------------------------------------------ */

void timer0_set_timeout_10ms( BYTE client, BYTE ticks )
{
  /* Sets a timeout for a period of ticks*10 milliseconds on Timer0
     for client 'client' on an interrupt basis;
     to be monitored by calling 'timer0_timeout(client)';
     'ticks' can be from 1 up to 255
     (so a maximum of 255*10 ms = 2.55 s can be set);
     note that if 'count' is 1 there could be an immediate time-out
     because the clock is running continuously;
     to ensure a minimum of 10 ms choose ticks>=2 */

  /* Disable Timer0 interrupt */
  TIMSK &= ~BIT( T0_OVERFLOW_IE );

  /* Initialize countdown counter */
  T0_Countdown[client] = ticks;

  /* Enable Timer0 interrupt */
  TIMSK |= BIT( T0_OVERFLOW_IE );
}

/* ------------------------------------------------------------------------ */

BOOL timer0_timeout( BYTE client )
{
  return( T0_Countdown[client] == 0 );
}

/* ------------------------------------------------------------------------ */
/* TIMER0 Overflow interrupt */

#pragma interrupt_handler timer0ovf_handler:17

void timer0ovf_handler( void )
{
  BYTE i;

  /* Stop the timer */
  TCCR0 = T0_STOP;

  /* Reinitialize the timer */
  SET_TIMER0_10MS();

  /* Decrement countdown counters, where appropriate */
  for( i=0; i<T0_CLIENTS; ++i )
    if( T0_Countdown[i] ) --T0_Countdown[i];
}

/* ------------------------------------------------------------------------ */
