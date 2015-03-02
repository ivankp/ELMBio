/* ------------------------------------------------------------------------
File   : timer1.c

Descr  : ATMEL AVR microcontroller Timer/Counter1 routines.

History: 20JUL.00; Henk B&B; Version for ELMB Master processor.
         16OCT.01; Henk B&B; Remove Master/Slave monitor mechanism from
	                     interrupt routine.
--------------------------------------------------------------------------- */

#include "general.h"
#include "guarding.h"
#include "pdo.h"
#include "timer103.h"
#include "watchdog.h"

extern BYTE CanBusOffCnt;

/* ------------------------------------------------------------------------ */

void timer1_init( void )
{
  /* Initialize Timer1 for our ELMB purposes */

  /* Set to a 1 second clock */
  SET_TIMER1_1000MS();

  /* Enable Timer1 interrupt */
  TIMSK |= BIT( T1_OVERFLOW_IE );

  /* Global interrupts enabled elsewhere... */
}

/* ------------------------------------------------------------------------ */

void timer1_stop( void )
{
  /* Stop the timer */
  TCCR1B = T1_STOP;

  /* Disable Timer1 interrupt */
  TIMSK &= ~BIT( T1_OVERFLOW_IE );
}

/* ------------------------------------------------------------------------ */
/* TIMER1 Overflow interrupt */

#pragma interrupt_handler timer1ovf_handler:15

void timer1ovf_handler( void )
{
  /* Stop the timer */
  TCCR1B = T1_STOP;

  /* Reinitialize the timer */
  SET_TIMER1_1000MS();

  /* Update some counters for various purposes:
     - periodic PDO transmissions
     - Lifeguarding
     - Heartbeat
     - Busoff retry counter */
  {
    {
      //BYTE pdo_no;
      //for( pdo_no=0; pdo_no<TPDO_CNT; ++pdo_no ) ++TPdoTimerCntr[pdo_no];
    }
    if( TPdoOnTimer[0] ) ++TPdoTimerCntr[0];
    if( TPdoOnTimer[1] ) ++TPdoTimerCntr[1];
    if( TPdoOnTimer[2] ) ++TPdoTimerCntr[2];

    ++LifeGuardCntr;

    ++HeartBeatCntr;

    if( CanBusOffCnt ) --CanBusOffCnt;
  }

  /* Time for the Master to perform the watchdog function */
  KickWatchdog = TRUE;
}

/* ------------------------------------------------------------------------ */
