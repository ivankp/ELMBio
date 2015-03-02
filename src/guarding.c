/* ------------------------------------------------------------------------
File   : guarding.c

Descr  : Functions concerning Node- and Lifeguarding and Heartbeat.

History: 24JUL.00; Henk B&B; Definition.
	 23FEB.01; Henk B&B; Keep some variables in EEPROM (optionally).
--------------------------------------------------------------------------- */

#include "general.h"
#include "can.h"
#include "eeprom.h"
#include "guarding.h"
#include "pdo.h"
#include "store.h"
#include "timer103.h"

/* ------------------------------------------------------------------------ */
/* Globals */

/* Toggle bit for the Node Guarding CAN-message */
BYTE NodeGuardToggle;

/* This variable contains the time out value (here in seconds)
   for the CANopen Life Guarding mechanism:
   its value is determined by CANopen OD objects 'Life Time Factor'
   (U8, object 0x100D) and 'Guard Time' (U16, object 0x100C) */
static BYTE LifeTimeFactor; /* (stored in EEPROM) */

/* Keeps track of time for the Life Guarding time out
   (Timer1 is used to update this counter) */
BYTE LifeGuardCntr;

/* Producer Heartbeat cycle time */
static BYTE HeartBeatTime;  /* (stored in EEPROM) */

/* Heartbeat time counter */
BYTE HeartBeatCntr;

/* ------------------------------------------------------------------------ */
/* Local prototypes */

static void guarding_load_config( void );

/* ------------------------------------------------------------------------ */

void guarding_init( void )
{
  /* Initialize the toggle bit */
  NodeGuardToggle = 0;

  /* Initialize node/life guarding parameters */
  guarding_load_config();

  TIMER1_DISABLE();
  LifeGuardCntr = 0;
  HeartBeatCntr = 0;
  TIMER1_ENABLE();
}

/* ------------------------------------------------------------------------ */

void lifeguarding_and_heartbeat( BYTE nodestate )
{
#ifdef __VARS_IN_EEPROM__
  LifeTimeFactor = eeprom_read( EE_LIFETIMEFACTOR );
#endif

  /* If lifeguarding detects a time-out take some action ! */
  if( LifeTimeFactor > 0 && LifeGuardCntr >= LifeTimeFactor )
    {
      /* Lifeguarding time-out !:
	 reinitialize CAN-interface and try to continue... */
      can_init( FALSE );

      /* CANopen Error Code 0x8130: communication error
	 (Life Guard or Heartbeat error), see CiA DS301 pg 9-38 */
      can_write_emergency( 0x30, 0x81, 0, 0, 0, 0, ERRREG_COMMUNICATION );

      TIMER1_DISABLE();
      LifeGuardCntr = 0;
      TIMER1_ENABLE();
    }

#ifdef __VARS_IN_EEPROM__
  HeartBeatTime = eeprom_read( EE_HEARTBEATTIME );
#endif /* __VARS_IN_EEPROM__ */

  /* Time to send a Heartbeat message ? */
  if( HeartBeatTime > 0 && HeartBeatCntr >= HeartBeatTime )
    {
      /* Send a Heartbeat message.
	 Note that if the RTR bit is set for this message (for automatic
	 Nodeguard replies) it is reset and the node no longer replies to
	 Nodeguard messages, which is okay since there is now a Heartbeat;
	 the automatic replies are restored when the Heartbeat time
	 is set to zero (see below) */
      can_write( C91_NODEGUARD, C91_NODEGUARD_LEN, &nodestate );

      TIMER1_DISABLE();
      HeartBeatCntr = 0;
      TIMER1_ENABLE();
    }
}

/* ------------------------------------------------------------------------ */

void nodeguarding( BYTE nodestate )
{
  /* Request for Node Guard object: send status and toggle-bit */

  BYTE can_data;

  can_data = nodestate | (NodeGuardToggle & 0x80);

  can_write( C91_NODEGUARD, C91_NODEGUARD_LEN, &can_data );

  /* Toggle the toggle bit */
  NodeGuardToggle ^= 0x80;

  /* Reset the Life Guarding time-out counter */
  TIMER1_DISABLE();
  LifeGuardCntr = 0;
  TIMER1_ENABLE();
}

/* ------------------------------------------------------------------------ */

BYTE guarding_get_guardtime( BYTE *guardtime )
{
  /* Guardtime (16-bit) is a constant here: 1000 ms */
  guardtime[0] = 0xE8;
  guardtime[1] = 0x03;
  return 2; /* Return number of bytes */
}

/* ------------------------------------------------------------------------ */

BYTE guarding_get_lifetime( BYTE *factor )
{
#ifdef __VARS_IN_EEPROM__
  LifeTimeFactor = eeprom_read( EE_LIFETIMEFACTOR );
#endif
  *factor = LifeTimeFactor;
  return 1; /* Return number of bytes */
}

/* ------------------------------------------------------------------------ */

BOOL guarding_set_lifetime( BYTE factor )
{
  LifeTimeFactor = factor;

#ifdef __VARS_IN_EEPROM__
  if( eeprom_read( EE_LIFETIMEFACTOR ) != LifeTimeFactor )
    eeprom_write( EE_LIFETIMEFACTOR, LifeTimeFactor );
#endif

  return TRUE;
}

/* ------------------------------------------------------------------------ */

BYTE guarding_get_heartbeattime( BYTE *hbtime )
{
  //UINT16 tmp;

#ifdef __VARS_IN_EEPROM__
  HeartBeatTime = eeprom_read( EE_HEARTBEATTIME );
#endif

  /* Heartbeat time in milliseconds */
  //tmp = (UINT16) 1000 * HeartBeatTime;
  //hbtime[0] = (tmp & 0x00FF) >> 0;
  //hbtime[1] = (tmp & 0xFF00) >> 8;

  /* Heartbeat time in seconds */
  hbtime[0] = HeartBeatTime;
  hbtime[1] = 0x00;

  return 2; /* Return number of bytes */
}

/* ------------------------------------------------------------------------ */

BOOL guarding_set_heartbeattime( BYTE *hbtime )
{
  if( hbtime[1] != 0 ) return FALSE;

  /* Set it in seconds, up to 255 (CANopen: in ms, up to 65535...) */
  HeartBeatTime = hbtime[0];

#ifdef __VARS_IN_EEPROM__
  if( eeprom_read( EE_HEARTBEATTIME ) != HeartBeatTime )
    eeprom_write( EE_HEARTBEATTIME, HeartBeatTime );
#endif

  /* Restore RTR bit, if required */
  if( HeartBeatTime == 0 ) can_rtr_enable( pdo_rtr_required() );

  return TRUE;
}

/* ------------------------------------------------------------------------ */

#define GUARDING_STORE_SIZE 2

/* ------------------------------------------------------------------------ */

BOOL guarding_store_config( void )
{
  BYTE block[GUARDING_STORE_SIZE];

#ifdef __VARS_IN_EEPROM__
  LifeTimeFactor = eeprom_read( EE_LIFETIMEFACTOR );
  HeartBeatTime  = eeprom_read( EE_HEARTBEATTIME );
#endif
  block[0] = LifeTimeFactor;
  block[1] = HeartBeatTime;

  return( store_write_block( STORE_GUARDING, GUARDING_STORE_SIZE, block ) );
}

/* ------------------------------------------------------------------------ */

static void guarding_load_config( void )
{
  BYTE block[GUARDING_STORE_SIZE];

  /* Read the configuration from EEPROM, if any */
  if( store_read_block( STORE_GUARDING, GUARDING_STORE_SIZE, block ) )
    {
      LifeTimeFactor = block[0];
      HeartBeatTime  = block[1];
    }
  else
    {
      /* No valid parameters in EEPROM: use defaults */
      LifeTimeFactor = LIFETIME_FACTOR_DFLT;
      HeartBeatTime  = HEARTBEAT_TIME_DFLT;
    }

#ifdef __VARS_IN_EEPROM__
  /* Create working copies of configuration globals in EEPROM */
  if( eeprom_read( EE_LIFETIMEFACTOR ) != LifeTimeFactor )
    eeprom_write( EE_LIFETIMEFACTOR, LifeTimeFactor );
  if( eeprom_read( EE_HEARTBEATTIME ) != HeartBeatTime )
    eeprom_write( EE_HEARTBEATTIME, HeartBeatTime );
#endif /* __VARS_IN_EEPROM__ */
}

/* ------------------------------------------------------------------------ */
