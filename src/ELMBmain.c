/* ------------------------------------------------------------------------
File   : ELMBmain.c

Descr  : Embedded Local Monitor Board (ELMB) application main function.

History: 05APR.00; Henk B&B; Start of development, based on an earlier
                             version for the first prototype LMB.
	 APR-OCT00;Henk B&B; Ongoing additions/upgrades.
	 17OCT.01; Henk B&B; Do watchdog mechanism in main loop;
	                     add refresh of 'node_state' variable.
	 17MAY.02; Henk B&B; Prepare for ELMB128 with ATmega128 processor.
	   MAY.03; Henk B&B; CAN-message reception under interrupt; here
	                     messages are read from buffers in RAM (see can.c);
			     enable/disable interrupt where necessary;
			     always do 'periodic jobs' even when a CAN-message
			     is handled, to prevent message-handling to take
			     all processor time.
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
#include "sdo.h"
#include "store.h"
#include "timer103.h"
#include "watchdog.h"

#ifdef __RS232__
#include <stdio.h>
#include "uart.h"
#endif /* __RS232__ */

/* CANopen state of this node;
   the variable could maintain a constant value for a long
   (continuous) period of time and thus is vulnerable to SEU
   (Single Event Upset by radiation) */
BYTE NodeState;

/* Readable version string in flash */
const char * const version_str = VERSION_STRING;

/* ------------------------------------------------------------------------ */

void main( void )
{
  BYTE   mcucsr;
  BOOL   hard_reset, watchdog_brownout_jtag_reset;
  BYTE   object_no;
  BYTE   dlc;
  BYTE   *can_data;
  BYTE   adc_result;
  BYTE   i;
  UINT16 crc;

  watchdog_init();    /* In case of a free-running watchdog timer,
			 need to configure it a.s.a.p. */
  watchdog_disable(); /* Disable it if possible... */

  /* Wait for at least 800 ms
     (for the ADC's 32.768 kHz crystal to become stable) */
  for( i=0; i<80; ++i ) timer2_delay_ms( 10 );

  WDR(); /* In case of a free-running watchdog timer */

 reset_node:

  /* Was it a power-on or external reset (hard reset) ? */
  mcucsr = MCUCSR;
  hard_reset = FALSE;
  if( mcucsr & (BIT(PORF) | BIT(EXTRF)) )
    hard_reset = TRUE;
  else
    hard_reset = FALSE;

  /* Was it any other 'suspicious' source of reset ? */
  if( mcucsr & (BIT(WDRF) | BIT(BORF) | BIT(JTRF)) )
    watchdog_brownout_jtag_reset = TRUE;
  else
    watchdog_brownout_jtag_reset = FALSE;

  /* Clear all reset flag bits */
  MCUCSR = 0;

  /* Initialize CANopen node state value */
  NodeState = NMT_INITIALISING;

  /* Switch off Analog Comparator (saves power) */
  ACSR |= BIT( ACD );

  /* Initialize PORTB (system-part) */
  DDRB  = PORTB_DDR_OPERATIONAL;
  PORTB = PORTB_DATA_OPERATIONAL;

  /* Initialize PORTD (system-part) */
  DDRD  = PORTD_DDR_OPERATIONAL;
  PORTD = PORTD_DATA_OPERATIONAL;

  /* Initialize PORTE (system-part) */
  DDRE  = PORTE_DDR_OPERATIONAL;
  PORTE = PORTE_DATA_OPERATIONAL;

  /* Global interrupt enable */
  SEI();

#ifdef __RS232__
  uart_init( BAUD_9600 );
  printf( "ELMB says: hello world\n\n" );
#endif /* __RS232__ */

  /* Initialize Timer0 for general-purpose time-outs */
  timer0_init();

  /* Initialize Timer1 for Master/Slave Activity Monitoring test,
     Life Guarding, Timer-triggered PDO transmissions, etc. */
  timer1_init();

  /* Initialize digital I/O stuff */
  digio_init( hard_reset );

  /* Initialize DAC stuff */
  dac_init( hard_reset );

  WDR(); /* In case of a free-running watchdog timer */

  /* Reset and initialize the ADC(s) */
  adc_result = adc_init();

  WDR(); /* In case of a free-running watchdog timer */

  /* Check the CRC added to the code in FLASH, if present */
  crc_master( &crc );

  /* (Re)enable watchdog timer and/or Slave monitoring mechanism */
  watchdog_init();

 reset_communication:

  /* Go to state NMT_PREOPERATIONAL */
  NodeState = NMT_PREOPERATIONAL;

  /* Initialize and configure the CAN-controller and message buffer */
  can_init( TRUE );

  /* Initialize PDO stuff */
  pdo_init();

  /* Initialize Node Guarding and Life Guarding stuff */
  guarding_init();

  /* Send a CANopen Bootup message */
  can_write_bootup();

  WDR(); /* In case of a free-running watchdog timer */

  /* Report it when a watchdog, brownout or JTAG reset occurred
     (include MCUCSR content) */
  if( watchdog_brownout_jtag_reset )
    {
      /* CANopen Error Code 0x5000: device hardware */
      can_write_emergency( 0x00, 0x50, EMG_IRREGULAR_RESET,
			   mcucsr, 0, 0, ERRREG_MANUFACTURER );
    }

  /* Check if EEPROM configuration retrieval went okay during
     the various device initializations and report it if not */
  store_check_load_status();

  /* Report problems that occurred during hardware initialization */
  if( adc_result == FALSE )
    {
      BYTE stat;
      adc_status( &stat );
      /* CANopen Error Code 0x5000: device hardware */
      can_write_emergency( 0x00, 0x50, EMG_ADC_INIT,
			   stat, 0, 0, ERRREG_MANUFACTURER );
    }

  /* Report CRC errors */
  if( crc != 0 )
    {
      /* CANopen Error Code 0x5000: device hardware */
      can_write_emergency( 0x00, 0x50, EMG_CRC,
			   1, 0, 0, ERRREG_MANUFACTURER );
    }

  /* The following is not according to CANopen:
     to optionally go immediately to state NMT_OPERATIONAL
     (possibly making life simpler for a host application?) */
  NodeState = canopen_init_state();

  /* Application loop */
  while(1)
    {
      /* Refresh some registers, to be more rad-tolerant... */
      CAN_INT_DISABLE();
      DDRB  = PORTB_DDR_OPERATIONAL;
      PORTB = PORTB_DATA_OPERATIONAL;
      CAN_INT_ENABLE();

      /* Do the watchdog function */
      watchdog();

      /* Check for CAN-controller errors */
      can_check_for_errors();

      if( NodeState == NMT_OPERATIONAL )
	{
	  /* Refresh some more registers, to be more rad-tolerant...
	     The 'system' I/O-pin functions change during Slave ISP
	     ==> Slave ISP only to be done when *NOT* in state OPERATIONAL!
	     The registers are also partly user-defined, so that part
	     in principle is unknown here and is kept as is;
	     refresh of other pins to be done by the respective device
	     functions (such as ADC, Digital I/O, DAC) */
	  DDRD  = (DDRD & PORTD_USERBITS_MASK) | PORTD_DDR_OPERATIONAL;
	  PORTD = (PORTD & PORTD_USERBITS_MASK) | PORTD_DATA_OPERATIONAL;
	  DDRE  = (DDRE & PORTE_USERBITS_MASK) | PORTE_DDR_OPERATIONAL;
	  PORTE = (PORTE & PORTE_USERBITS_MASK) | PORTE_DATA_OPERATIONAL;

	  /* Handle PDO transmissions for data scanning,
	     change-of-state, and timer-triggered events */
	  tpdo_scan();
	}
      else
	{
	  /* Provide some protection against SEE bitflips in 'NodeState':
	     it basically has only 3 values: STOPPED (0x04),
	     OPERATIONAL (0x05) or PRE-OPERATIONAL (0x7F),
	     of which STOPPED is probably a rare condition...;
	     we assume a single bitflip in the byte may have occurred */

	  /* Upper nibble contains 0x7? -> state should be 0x7F */
	  if( (NodeState & 0x70) == 0x70 ) NodeState = NMT_PREOPERATIONAL;

	  /* Lower nibble is all 1? -> state should be 0x7F */
	  if( (NodeState & 0x0F) == 0x0F )
	    NodeState = NMT_PREOPERATIONAL;
	  else
	    {
	      /* If not, assume NMT_OPERATIONAL is the most likely state */
	      if( NodeState & 0x01 ) NodeState = NMT_OPERATIONAL;
	    }
	}

      lifeguarding_and_heartbeat( NodeState );

      /* Poll for (a) new CAN-message(s)... */
      if( can_msg_available() == FALSE )
	{
	  /* Jump to begin of 'while'-loop */
	  continue;
	}

      /* A CAN-message is to be processed, so get the object identifier,
	 DLC and (a pointer to) the data bytes (if any) from the buffer */
      object_no = can_read( &dlc, &can_data );

      /* Reset the Life Guarding time-out counter (a message was received)
	 but should actually be done through Node Guarding only...
	 (according to CANopen standard) */
      TIMER1_DISABLE();
      LifeGuardCntr = 0;
      TIMER1_ENABLE();

      /* Process the CAN-message depending on the node's current state */
      switch( NodeState )
	{
	case NMT_OPERATIONAL:
	  /* Node responds to all known messages */

	  switch( object_no )
	    {
	    case C91_SYNC:
	      /* 'SYNC' request for data */
	      tpdo_on_sync();

	      /* Message handled: jump to start of while loop */
	      continue;

	    case C91_TPDO1_RTR:
	      /* 'RTR' request for Transmit-PDO1 */
	      tpdo1_on_rtr();

	      /* Message handled: jump to start of while loop */
	      continue;

	    case C91_TPDO2_RTR:
	      /* 'RTR' request for Transmit-PDO2 */
	      tpdo2_on_rtr();

	      /* Message handled: jump to start of while loop */
	      continue;

	    case C91_TPDO3_RTR:
	      /* 'RTR' request for Transmit-PDO3 */
	      tpdo3_on_rtr();

	      /* Message handled: jump to start of while loop */
	      continue;

	    case C91_RPDO1:
	      /* Receive-PDO1 */
	      rpdo1( dlc, can_data );

	      /* Message handled: jump to start of while loop */
	      continue;

	    case C91_RPDO2:
	      /* Receive-PDO2 */
	      rpdo2( dlc, can_data );

	      /* Message handled: jump to start of while loop */
	      continue;

	    default:
	      break;
	    }

	  /* Respond to SDO, NMT and NodeGuarding messages as well,
	     so no 'break' here... */

	case NMT_PREOPERATIONAL:
	  /* Node does not respond to PDOs */

	  switch( object_no )
	    {
	    case C91_SDORX:
	      /* Object Dictionary access */
	      if( dlc == C91_SDORX_LEN ) sdo_server( can_data );

	      /* Message handled: jump to start of while loop */
	      continue;

	    default:
	      break;
	    }

	  /* Respond to NMT and NodeGuarding messages as well,
	     so no 'break' here... */

	case NMT_STOPPED:
	  /* Node is stopped: it only responds to NMT and NodeGuarding
	     messages, not to PDOs and SDOs */

	  switch( object_no )
	    {
	    case C91_NMT:
	      {
		BYTE node_id, nmt_request;

		node_id = can_data[1];

#ifdef __VARS_IN_EEPROM__
		NodeID = eeprom_read( EE_NODEID );
#endif /* __VARS_IN_EEPROM__ */

		/* Handle NMT message if correct and
		   addressed to this node or broadcast */
		if( dlc == C91_NMT_LEN && (node_id == NodeID || node_id == 0) )
		   {
		     nmt_request = can_data[0];

		     /* Ignore requests that don't change the node state */
		     if( (nmt_request == NMT_START_REMOTE_NODE &&
			  NodeState   == NMT_OPERATIONAL) ||
			 (nmt_request == NMT_STOP_REMOTE_NODE &&
			  NodeState   == NMT_STOPPED) ||
			 (nmt_request == NMT_ENTER_PREOPERATIONAL_STATE &&
			  NodeState   == NMT_PREOPERATIONAL) )
		       break;

		     /* Properly cancel any ongoing activities if necessary */
		     pdo_on_nmt( nmt_request );

		     switch( nmt_request )
		       {
		       case NMT_START_REMOTE_NODE:
			 NodeState = NMT_OPERATIONAL;
			 break;

		       case NMT_STOP_REMOTE_NODE:
			 NodeState = NMT_STOPPED;
			 break;

		       case NMT_ENTER_PREOPERATIONAL_STATE:
			 NodeState = NMT_PREOPERATIONAL;
			 break;

		       case NMT_RESET_NODE:
			 /* Parameters in Manufacturer-specific Profile
			    and Standardised Device Profile area are set
			    to their default values */

			 goto reset_node;

			 break;

		       case NMT_RESET_COMMUNICATION:
			 /* Parameters of the Communication Profile Area
			    are set to their power-on defaults */

			 goto reset_communication;

			 break;

		       default:
			 break;
		       }
		     /* Update node state in NodeGuard message buffer
			(which may be sent automatically) */
		     CAN_INT_DISABLE();
		     can_write_reg( C91_MSGS_I + (C91_NODEGUARD*C91_MSG_SIZE),
				    NodeState | (NodeGuardToggle & 0x80) );
		     CAN_INT_ENABLE();
		   }
	      }
	      break;

	    case C91_NODEGUARD_RTR:
	      /* Remote Transmission Request for Node Guard object */
	      nodeguarding( NodeState );
	      break;

	    default:
	      /* Ignore all other messages... */
	      break;
	    }
	  break;

	default:
	  /* Unknown node state: not possible.... */
	  break;
	}
    }
}

/* ------------------------------------------------------------------------ */
