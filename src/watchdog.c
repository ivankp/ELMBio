/* ------------------------------------------------------------------------
File   : watchdog.c

Descr  : ELMB Master<->Slave watchdog/monitor mechanism;
         this is the Master (ATmega103) part.

History: 20JUL.00; Henk B&B; Version for ELMB Master processor.
--------------------------------------------------------------------------- */

#include "general.h"
#ifdef __2313_SLAVE_PRESENT__
#include "can.h"
#include "objects.h"
#endif /* __2313_SLAVE_PRESENT__ */

/* ------------------------------------------------------------------------ */
/* Globals */

BOOL        KickWatchdog;
#ifdef __2313_SLAVE_PRESENT__
static BOOL SlaveHasProblem;
#endif /* __2313_SLAVE_PRESENT__ */

/* ------------------------------------------------------------------------ */

void watchdog_init( void )
{
  BOOL global_int_enabled;

  KickWatchdog = TRUE;

  /* Disable interrupts during this operation */
  global_int_enabled = FALSE;
  if( SREG & 0x80 ) global_int_enabled = TRUE;
  CLI();

  /* Watchdog timer part: set prescale to maximum (ca. 1.9 s) and enable */
  WDR();                        /* Recommended before enabling */
#ifdef __ELMB103__
  WDTCR = BIT(WDE) | BIT(WDP0) | BIT(WDP1) | BIT(WDP2); /* For ATmega103 */
#else
  WDTCR = BIT(WDCE) | BIT(WDE); /* To change prescale need timed sequence
				   on ATmega128 */
  WDTCR = BIT(WDE) | BIT(WDP0) | BIT(WDP1) | BIT(WDP2);
  WDTCR = BIT(WDE);             /* Enable watchdog */
#endif /* __ELMB103__ */

  /* Re-enable interrupts if required */
  if( global_int_enabled ) SEI();
 
#ifdef __2313_SLAVE_PRESENT__
  /* Slave processor part, if any present... */
  SlaveHasProblem = FALSE;
#endif /* __2313_SLAVE_PRESENT__ */
}

/* ------------------------------------------------------------------------ */

void watchdog_disable( void )
{
  BOOL global_int_enabled;

  /* Disable interrupts during this operation */
  global_int_enabled = FALSE;
  if( SREG & 0x80 ) global_int_enabled = TRUE;
  CLI();

  /* Watchdog timer disable */
  WDTCR = BIT(WDCE) | BIT(WDE);
  WDTCR = BIT(WDP0) | BIT(WDP1) | BIT(WDP2); /* Keep prescale bits */

  /* Re-enable interrupts if required */
  if( global_int_enabled ) SEI();
}

/* ------------------------------------------------------------------------ */

void watchdog( void )
{
  /* Watchdog timer reset */
  WDR();

  if( (KickWatchdog & TRUE) == FALSE ) return;

#ifdef __2313_SLAVE_PRESENT__
  /* Slave processor watchdog 'reset'
    (once a problem with the Slave has been detected,
     don't bother with the Slave anymore...) */
  if( (SlaveHasProblem & TRUE) == FALSE )
    {
      BYTE cnt;

      /* Disable all interrupts during this operation */
      CLI();

      /* Start aliveness sequence with Slave;
	 generate interrupt signal on Slave */
      SET_MASTER_TO_SLAVE_LOW();
      NOP(); /* Make sure the interrupt is seen by the Slave */
      NOP();
      NOP();

      /* Switch to input */
      SET_MASTER_TO_SLAVE_INPUT();

      /* Enable pull-up resistor */
      SET_MASTER_TO_SLAVE_HIGH();

      /* Slave should now see HIGH signal, and switch to output */

      /* Now await Slave taking signal LOW and then HIGH (with time-out!) */
      cnt = 0;
      do
	{
	  ++cnt;
	}
      while( MASTER_TO_SLAVE_HIGH() && cnt != 0 );

      if( cnt != 0 )
	{
	  /* Await Slave taking signal HIGH (with time-out!) */
	  cnt = 0;
	  do
	    {
	      ++cnt;
	    }
	  while( MASTER_TO_SLAVE_LOW() && cnt != 0 );

	  if( cnt != 0 )
	    {
	      /* Switch to output (will be HIGH, as set above) */
	      SET_MASTER_TO_SLAVE_OUTPUT();
	    }
	  else
	    {
	      /* There is a problem with the Slave:
		 leave Master-to-Slave signal configured as input */

	      /* Slave did not respond */
	      SlaveHasProblem = TRUE;
	    }
	}
      else
	{
	  /* Slave did not respond */
	  SlaveHasProblem = TRUE;
	}

      /* Re-enable interrupts */
      SEI();

      if( (SlaveHasProblem & TRUE) == TRUE )
	{
	  /* Report it (once) */

	  /* CANopen Error Code 0x5000: device hardware */
	  can_write_emergency( 0x00, 0x50, EMG_SLAVE_PROCESSOR,
			       0, 0, 0, ERRREG_MANUFACTURER );

	  /* Enable/start Watchdog Timer (if not enabled already),
	     now that there is no Slave to 'guard' me... */
	  /* ###........ */
	}
      else
	{
	  /* Just to make sure, set it (protection against SEU) */
	  SlaveHasProblem = FALSE;
	}
    }
  else
    {
      /* Reset the Watchdog Timer */
      //WDR();
    }
#endif /* __2313_SLAVE_PRESENT__ */

  KickWatchdog = FALSE;
}

/* ------------------------------------------------------------------------ */
