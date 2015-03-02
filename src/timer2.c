/* ------------------------------------------------------------------------
File   : timer2.c

Descr  : ATMEL AVR microcontroller Timer/Counter2 routines.

History: 19SEP.00; Henk B&B; Timer2 is used for busy-wait delays.
--------------------------------------------------------------------------- */

#include "general.h"
#include "timer103.h"

/* ------------------------------------------------------------------------ */

void timer2_delay_mus( BYTE microseconds )
{
  /* This routine is suitable for a delay of an even number
     of microseconds from 10 up to 256 microseconds (an odd number
     of microseconds gets rounded to the next even number);
     NOTE: the overhead of this routine is about 7 microseconds (at 4 MHz),
           which is taken into account to achieve the desired delay. */

  /* Take overhead into account */
  if( microseconds < 9 ) microseconds = 9;
  microseconds -= 8;

  /* Stop the timer */
  TCCR2 = T2_STOP;

  /* Disable Timer2 interrupt */
  TIMSK &= ~BIT( T2_OVERFLOW_IE );

  /* Initialize and start timer (2 microseconds per tick @4MHz) */
  TCNT2 = 0 - ((microseconds+1)>>1);
  TCCR2 = T2_CK_DIV_8;

  /* Wait for the timer overflow flag */
  while( (TIFR & BIT(T2_OVERFLOW)) == 0 );

  /* Stop the timer */
  TCCR2 = T2_STOP;

  /* Clear the timer overflow flag, by writing a 1 ! */
  SETBIT( TIFR, T2_OVERFLOW );
}

/* ------------------------------------------------------------------------ */

void timer2_delay_ms( BYTE milliseconds )
{
  /* This routine is suitable for a delay of a number of milliseconds
     (actually 1.024 ms...) from 1 up to 63 milliseconds */

  /* Stop the timer */
  TCCR2 = T2_STOP;

  /* Disable Timer2 interrupt */
  TIMSK &= ~BIT( T2_OVERFLOW_IE );

  /* Initialize and start timer (256 microseconds per tick @4MHz) */
  TCNT2 = 0 - (milliseconds<<2);
  TCCR2 = T2_CK_DIV_1024;

  /* Wait for the timer overflow flag */
  while( (TIFR & BIT(T2_OVERFLOW)) == 0 );

  /* Stop the timer */
  TCCR2 = T2_STOP;

  /* Clear the timer overflow flag, by writing a 1 ! */
  SETBIT( TIFR, T2_OVERFLOW );
}

/* ------------------------------------------------------------------------ */
