/* ------------------------------------------------------------------------
File   : digio.c

Descr  : Functions for Digital-input and -output handling.

History: 21JUL.00; Henk B&B; Definition; for the time being a single 8-bit
                             in- and two 8-bit output ports are defined.
	 21AUG.00; Henk B&B; Add a polling counter for the change-of-state
	                     of inputs in order to implement 'debouncing'.
         30NOV.00; Henk B&B; Added interrupt enable: DigInIntrptEna.
	 19MAR.01; Henk B&B; Added storage of digital output setting(s)
	                     for restore after reset/power-up.
	 18JUL.01; Henk B&B; Changed output-at-reset policy:
	                     keep settings at soft-reset (= NMT-Reset).
	 23JUL.01; Henk B&B; Share 2nd Port between in- and outputs
	                     (configurable);
			     add interrupt masks for both 8-bit input ports.
	 04FEB.02; Henk B&B; Digital Outputs set to high, by default.
	 16JAN.03; Henk B&B; Ported to ATmega128.
--------------------------------------------------------------------------- */

#include "general.h"
#include "can.h"
#include "digio.h"
#include "eeprom.h"
#include "store.h"

/* After a hard reset Digital Outputs are initialized to either low or high;
   after a soft reset (= NMT Reset-Node) existing settings are kept */

/* ------------------------------------------------------------------------ */
/* Globals */

static BYTE DigInDebouncePolls; /* (stored in EEPROM) */
static BOOL DigInIntrptEna;     /* (stored in EEPROM) */
static BOOL DigOutInitHi;       /* (stored in EEPROM) */

static BYTE DigOutMask2;        /* (stored in EEPROM) */
static BYTE DigInMask2;

static BYTE DigInIntrptMask1;   /* (stored in EEPROM) */
static BYTE DigInIntrptMask2;   /* (stored in EEPROM) */

static BYTE DigInStateOld1;
static BYTE DigInStateOld2;
static BYTE DigInStatePolls;

/* ------------------------------------------------------------------------ */
/* Local prototypes */

static void digio_load_config( void );

/* ------------------------------------------------------------------------ */

void digio_init( BOOL hard_reset )
{
  /* Initialise Digital-I/O configuration parameters */
  digio_load_config();

  DigInStatePolls = 0;

  /* Digital input port 1
     ATmega103: PORTF cannot be initialized (no pull-ups present) */
#ifndef __ELMB103__
  DIGIN1_PORT = 0xFF; /* Use pull-ups */
  DIGIN1_DDR  = 0x00;
#endif /* __ELMB103__ */
  DigInStateOld1 = DIGIN1_PIN & DigInIntrptMask1;

  /* Digital input port 2:  is shared between inputs and outputs !
     set DDRA/PORTA: use pull-ups (=1) */
  DigInMask2   = ~DigOutMask2;
  DIGIN2_PORT |= DigInMask2;
  DIGIN2_DDR   = ~DigInMask2;
  DigInStateOld2 = (DIGIN2_PIN & DigInMask2) & DigInIntrptMask2;

  /* Digital output port 1: set PORTC (output only on ATmega103);
     keep old setting at soft-reset (NMT Reset-Node) */
#ifndef __ELMB103__
  DIGOUT1_DDR = 0xFF;
#endif /* __ELMB103__ */
  if( (hard_reset & TRUE) == TRUE )
    {
      /* Initialize to required setting */
      if( DigOutInitHi )
	DIGOUT1_PORT = 0xFF;
      else
	DIGOUT1_PORT = 0x00;
    }

  /* Digital output port 2: set DDRA/PORTA;
     keep old setting at soft-reset (NMT Reset-Node) */
  DIGOUT2_DDR = DigOutMask2;
  if( (hard_reset & TRUE) == TRUE )
    {
      /* Initialize to required setting */
      if( DigOutInitHi )
	DIGOUT2_PORT |= DigOutMask2;
      else
	DIGOUT2_PORT &= (~DigOutMask2);
    }
}

/* ------------------------------------------------------------------------ */

BYTE digin_port_cnt( void )
{
  return DIGIO_INPORT_CNT;
}

/* ------------------------------------------------------------------------ */

BYTE digin_get_port( BYTE port_no )
{
  if( port_no == 1 )
    {
#ifndef __ELMB103__
      /* Refresh the DDR and pull-up setting */
      DIGIN1_DDR  = 0x00;
      DIGIN1_PORT = 0xFF; /* Use pull-ups */
#endif /* __ELMB103__ */
      return DIGIN1_PIN;
    }
  if( port_no == 2 )
    {
#ifdef __VARS_IN_EEPROM__
      DigInMask2  = ~eeprom_read( EE_DIGOUTMASK2 );
#endif /* __VARS_IN_EEPROM__ */

      /* Refresh the DDR and pull-up setting */
      DIGIN2_DDR  &= (~DigInMask2);
      DIGIN2_PORT |= DigInMask2;

      return (DIGIN2_PIN & DigInMask2);
    }
  return 0x00;
}

/* ------------------------------------------------------------------------ */

void digin_pdo( void )
{
  BYTE pdo_data[DIGIO_INPORT_CNT];

  /* Read the digital-inputs and put in the message */
  pdo_data[0] = digin_get_port( 1 );
  pdo_data[1] = digin_get_port( 2 );

  /* Send as a Transmit-PDO1 CAN-message */
  can_write( C91_TPDO1, DIGIO_INPORT_CNT, pdo_data );
}

/* ------------------------------------------------------------------------ */

void digin_pdo_on_cos( void )
{
  /* Send a PDO on change-of-state of the digital inputs;
     a simple debounce-mechanism is implemented */

  BYTE current_state1, current_state2;

#ifdef __VARS_IN_EEPROM__
  DigInIntrptEna = eeprom_read( EE_DIGININTRPTENA );
#endif /* __VARS_IN_EEPROM__ */

  if( !DigInIntrptEna ) return;

#ifdef __VARS_IN_EEPROM__
  DigInIntrptMask1 = eeprom_read( EE_DIGININTRPTMASK1 );
  DigInIntrptMask2 = eeprom_read( EE_DIGININTRPTMASK2 );
  DigInMask2       = ~eeprom_read( EE_DIGOUTMASK2 );
#endif /* __VARS_IN_EEPROM__ */

  /* Read current state of (interrupt-enabled) digital inputs */
  current_state1 = DIGIN1_PIN & DigInIntrptMask1;
  current_state2 = (DIGIN2_PIN & DigInMask2) & DigInIntrptMask2;

  if( current_state1 != DigInStateOld1 || current_state2 != DigInStateOld2 )
    {
#ifdef __VARS_IN_EEPROM__
      DigInDebouncePolls = eeprom_read( EE_DIGINDEBOUNCEPOLLS );
#endif /* __VARS_IN_EEPROM__ */

      /* Assuming that a 'bouncing' switch causes the input to go
	 from a new state to the old state and back, we just wait
	 until we have seen a new state a number of times in a row */
      ++DigInStatePolls;
      if( DigInStatePolls > DigInDebouncePolls )
	{
	  /* Assume changed state of digital inputs now stable: send the PDO;
	     if this is not possible don't 'discard' this change:
	     we'll miss it if this is the last change that occurs */

	  if( can_transmitting(C91_TPDO1) ) return;

	  digin_pdo();

	  /* Remember the current state */
	  DigInStateOld1 = current_state1;
	  DigInStateOld2 = current_state2;
	}
    }
  else
    {
      DigInStatePolls = 0;
    }
}

/* ------------------------------------------------------------------------ */

BYTE digout_port_cnt( void )
{
  return DIGIO_OUTPORT_CNT;
}

/* ------------------------------------------------------------------------ */

void digout_set_port( BYTE port_no, BYTE digout_data )
{
  /* Set digital outputs according to databyte */
  if( port_no == 1 )
    {
#ifndef __ELMB103__
      /* Refresh the DDR */
      DIGOUT1_DDR = 0xFF;
#endif /* __ELMB103__ */
      DIGOUT1_PORT = digout_data;
    }
  if( port_no == 2 )
    {
      BYTE out_curr;

#ifdef __VARS_IN_EEPROM__
      DigOutMask2 = eeprom_read( EE_DIGOUTMASK2 );
#endif /* __VARS_IN_EEPROM__ */

      /* Refresh the (output-part of the) DDR */
      DIGOUT2_DDR |= DigOutMask2;

      /* Get setting of non-output pins (pull-ups) */
      out_curr = DIGOUT2_PORT & (~DigOutMask2);

      /* Combine this setting with the new output setting */
      DIGOUT2_PORT = out_curr | (digout_data & DigOutMask2);
    }
}

/* ------------------------------------------------------------------------ */

BYTE digout_get_port( BYTE port_no )
{
  BYTE portdata = 0x00;
  if( port_no == 1 )
    {
#ifndef __ELMB103__
      /* Refresh the DDR */
      DIGOUT1_DDR = 0xFF;
#endif /* __ELMB103__ */
      portdata = DIGOUT1_PORT;
    }
  if( port_no == 2 )
    {
#ifdef __VARS_IN_EEPROM__
      DigOutMask2 = eeprom_read( EE_DIGOUTMASK2 );
#endif /* __VARS_IN_EEPROM__ */

      /* Refresh the DDR */
      DIGOUT2_DDR |= DigOutMask2;
      /* Get current output setting (leaving all other bits zero) */
      portdata     = DIGOUT2_PORT & DigOutMask2;
    }
  return portdata;
}

/* ------------------------------------------------------------------------ */

void digout_pdo( BYTE dlc, BYTE *pdo_data )
{
  if( dlc >= DIGIO_OUTPORT_CNT )
    {
      /* Set digital outputs according to databyte(s) in PDO */ 
      digout_set_port( 1, pdo_data[0] );
      digout_set_port( 2, pdo_data[1] );
    }
  else
    {
      if( dlc == 1 )
	{
	  /* Set digital outputs according to databyte in PDO */ 
	  digout_set_port( 1, pdo_data[0] );
	}
      else
	{
	  /* DLC is zero ?... */

	  /* CANopen Error Code 0x8210: protocol error
	     (PDO not processed due to length error), see CiA DS301 pg 9-38 */
	  can_write_emergency( 0x10, 0x82, 1,
			       0, 0, 0, ERRREG_COMMUNICATION );
	}
    }
}

/* ------------------------------------------------------------------------ */

BOOL digout_set_init( BOOL init_hi )
{
  if( init_hi > 1 ) return FALSE;
  if( init_hi ) DigOutInitHi = TRUE;
  else DigOutInitHi = FALSE;

#ifdef __VARS_IN_EEPROM__
  if( eeprom_read( EE_DIGOUTINIT ) != DigOutInitHi )
    eeprom_write( EE_DIGOUTINIT, DigOutInitHi );
#endif /* __VARS_IN_EEPROM__ */
  return TRUE;
}

/* ------------------------------------------------------------------------ */

BOOL digout_get_init( void )
{
#ifdef __VARS_IN_EEPROM__
  DigOutInitHi = eeprom_read( EE_DIGOUTINIT );
#endif /* __VARS_IN_EEPROM__ */

  return DigOutInitHi;
}

/* ------------------------------------------------------------------------ */

void digout_set_mask( BYTE port_no, BYTE digout_mask )
{
  /* Set digital output mask according to databyte */
  if( port_no == 2 )
    {
      DigOutMask2 = digout_mask;

      /* Apply new output setting (outputs itself not initialized) */
      DIGOUT2_DDR = DigOutMask2;

      /* Sharing the port with the inputs... */
      DigInMask2  = ~DigOutMask2;

      /* Digital input port: use pull-ups (=1) */
      DIGIN2_PORT |= DigInMask2;

#ifdef __VARS_IN_EEPROM__
      if( eeprom_read( EE_DIGOUTMASK2 ) != DigOutMask2 )
	eeprom_write( EE_DIGOUTMASK2, DigOutMask2 );
#endif /* __VARS_IN_EEPROM__ */
    }
}

/* ------------------------------------------------------------------------ */

BYTE digout_get_mask( BYTE port_no )
{
#ifdef __VARS_IN_EEPROM__
  DigOutMask2 = eeprom_read( EE_DIGOUTMASK2 );
#endif /* __VARS_IN_EEPROM__ */

  if( port_no == 1 ) return 0xFF;
  if( port_no == 2 ) return DigOutMask2;

  return 0x00;
}

/* ------------------------------------------------------------------------ */

void digin_set_debounce( BYTE debounce_cntr )
{
  DigInDebouncePolls = debounce_cntr;

  /* Maximum is 0xFE */
  if( debounce_cntr == 0xFF ) DigInDebouncePolls = 0xFE;

#ifdef __VARS_IN_EEPROM__
  if( eeprom_read( EE_DIGINDEBOUNCEPOLLS ) != DigInDebouncePolls )
    eeprom_write( EE_DIGINDEBOUNCEPOLLS, DigInDebouncePolls );
#endif /* __VARS_IN_EEPROM__ */
}

/* ------------------------------------------------------------------------ */

BYTE digin_get_debounce( void )
{
#ifdef __VARS_IN_EEPROM__
  DigInDebouncePolls = eeprom_read( EE_DIGINDEBOUNCEPOLLS );
#endif /* __VARS_IN_EEPROM__ */

  return DigInDebouncePolls;
}

/* ------------------------------------------------------------------------ */

BOOL digin_set_intrpt_ena( BOOL intrpt_ena )
{
  if( intrpt_ena > 1 ) return FALSE;
  if( intrpt_ena ) DigInIntrptEna = TRUE;
  else DigInIntrptEna = FALSE;

#ifdef __VARS_IN_EEPROM__
  if( eeprom_read( EE_DIGININTRPTENA ) != DigInIntrptEna )
    eeprom_write( EE_DIGININTRPTENA, DigInIntrptEna );
#endif /* __VARS_IN_EEPROM__ */
  return TRUE;
}

/* ------------------------------------------------------------------------ */

BOOL digin_get_intrpt_ena( void )
{
#ifdef __VARS_IN_EEPROM__
  DigInIntrptEna = eeprom_read( EE_DIGININTRPTENA );
#endif /* __VARS_IN_EEPROM__ */

  return DigInIntrptEna;
}

/* ------------------------------------------------------------------------ */

void digin_set_intrpt_mask( BYTE port_no, BYTE intrpt_mask )
{
  /* Set interrupt mask according to databyte */
  if( port_no == 1 ) DigInIntrptMask1 = intrpt_mask;
  if( port_no == 2 ) DigInIntrptMask2 = intrpt_mask;

#ifdef __VARS_IN_EEPROM__
  if( eeprom_read( EE_DIGININTRPTMASK1 ) != DigInIntrptMask1 )
    eeprom_write( EE_DIGININTRPTMASK1, DigInIntrptMask1 );
  if( eeprom_read( EE_DIGININTRPTMASK2 ) != DigInIntrptMask2 )
    eeprom_write( EE_DIGININTRPTMASK2, DigInIntrptMask2 );
#endif /* __VARS_IN_EEPROM__ */
}

/* ------------------------------------------------------------------------ */

BYTE digin_get_intrpt_mask( BYTE port_no )
{
#ifdef __VARS_IN_EEPROM__
  DigInIntrptMask1 = eeprom_read( EE_DIGININTRPTMASK1 );
  DigInIntrptMask2 = eeprom_read( EE_DIGININTRPTMASK2 );
#endif /* __VARS_IN_EEPROM__ */

  if( port_no == 1 ) return DigInIntrptMask1;
  if( port_no == 2 ) return DigInIntrptMask2;

  return 0x00;
}

/* ------------------------------------------------------------------------ */

#define DIGIO_STORE_SIZE 6

/* ------------------------------------------------------------------------ */

BOOL digio_store_config( void )
{
  BYTE block[DIGIO_STORE_SIZE];

#ifdef __VARS_IN_EEPROM__
  DigInDebouncePolls = eeprom_read( EE_DIGINDEBOUNCEPOLLS );
  DigInIntrptEna     = eeprom_read( EE_DIGININTRPTENA );
  DigOutInitHi       = eeprom_read( EE_DIGOUTINIT );
  DigOutMask2        = eeprom_read( EE_DIGOUTMASK2 );
  DigInIntrptMask1   = eeprom_read( EE_DIGININTRPTMASK1 );
  DigInIntrptMask2   = eeprom_read( EE_DIGININTRPTMASK2 );
#endif /* __VARS_IN_EEPROM__ */

  block[0] = DigInDebouncePolls;
  block[1] = DigInIntrptEna;
  block[2] = DigOutInitHi;
  block[3] = DigOutMask2;
  block[4] = DigInIntrptMask1;
  block[5] = DigInIntrptMask2;

  return( store_write_block( STORE_DIGIO, DIGIO_STORE_SIZE, block ) );
}

/* ------------------------------------------------------------------------ */

static void digio_load_config( void )
{
  BYTE block[DIGIO_STORE_SIZE];

  /* Read the configuration from EEPROM, if any */
  if( store_read_block( STORE_DIGIO, DIGIO_STORE_SIZE, block ) )
    {
      DigInDebouncePolls = block[0];
      DigInIntrptEna     = block[1];
      DigOutInitHi       = block[2];
      DigOutMask2        = block[3];
      DigInIntrptMask1   = block[4];
      DigInIntrptMask2   = block[5];
    }
  else
    {
      /* No valid parameters in EEPROM: use defaults */
      DigInDebouncePolls = DIGIN_DEBOUNCE_POLLS_DFLT;
      DigInIntrptEna     = FALSE;
      DigOutInitHi       = TRUE;
      DigOutMask2        = 0xFF; /* 2nd Port is output by default */
      DigInIntrptMask1   = 0xFF;
      DigInIntrptMask2   = 0xFF;
    }
#ifdef __ADC_AVR__
  DigInIntrptEna = FALSE;
#endif

#ifdef __VARS_IN_EEPROM__
  /* Create working copies of configuration globals in EEPROM */
  if( eeprom_read( EE_DIGINDEBOUNCEPOLLS ) != DigInDebouncePolls )
    eeprom_write( EE_DIGINDEBOUNCEPOLLS, DigInDebouncePolls );
  if( eeprom_read( EE_DIGININTRPTENA ) != DigInIntrptEna )
    eeprom_write( EE_DIGININTRPTENA, DigInIntrptEna );
  if( eeprom_read( EE_DIGOUTINIT ) != DigOutInitHi )
    eeprom_write( EE_DIGOUTINIT, DigOutInitHi );
  if( eeprom_read( EE_DIGOUTMASK2 ) != DigOutMask2 )
    eeprom_write( EE_DIGOUTMASK2, DigOutMask2 );
  if( eeprom_read( EE_DIGININTRPTMASK1 ) != DigInIntrptMask1 )
    eeprom_write( EE_DIGININTRPTMASK1, DigInIntrptMask1 );
  if( eeprom_read( EE_DIGININTRPTMASK2 ) != DigInIntrptMask2 )
    eeprom_write( EE_DIGININTRPTMASK2, DigInIntrptMask2 );
#endif /* __VARS_IN_EEPROM__ */
}

/* ------------------------------------------------------------------------ */
