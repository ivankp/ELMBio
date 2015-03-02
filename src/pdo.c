/* ------------------------------------------------------------------------
File   : pdo.c

Descr  : Functions for CANopen PDO handling, which call node-specific device
         routines; the calls comply to the "CANopen Device Profile for
	 I/O Modules" (CiA DSP-401).

History: 19JUL.00; Henk B&B; Definition; for the time being only ADC
                             device routines are called.
	   AUG.00; Henk B&B; Addition of digital input and output.
	   NOV.00; Henk B&B; Addition of analogue output (DAC).
	 29NOV.00; Henk B&B; 'digin_pdo_on_cos()' not called for
	                     PDO transmission type 254.
--------------------------------------------------------------------------- */

#include "general.h"
#include "adc.h"
#include "can.h"
#include "dac.h"
#include "digio.h"
#include "eeprom.h"
#include "objects.h"
#include "pdo.h"
#include "store.h"
#include "timer103.h"

/* ------------------------------------------------------------------------ */
/* Some PDO communication parameters and the PDO mappings are constant
   in this application, and can thus be stored in program memory;
   this information is here just for reference, it is not essential
   for correct operation, although it can be read out by SDO messages;
   in the data arrays below the RPDO parameters are stored behind
   the TPDO parameters */

/* Per PDO the corresponding COB-ID (default predefined CANopen values..),
   here: TPDO1, TPDO2 and TPDO3, and RPDO1 and RPDO2 */
const UINT16 PDO_COBID[TPDO_CNT + RPDO_CNT] = { 0x180, 0x280, 0x380,
						0x200, 0x300 };

/* Per PDO the number of mapped objects */
const BYTE   PDOMAP_CNT[TPDO_CNT + RPDO_CNT] = { 2, 2, 2, 2, 2 };

/* Per PDO the mapped objects */
const UINT32 PDOMAP[TPDO_CNT + RPDO_CNT][2] =
{
  { 0x60000108L, 0x60000208L }, /* Digital Inputs: 1-8, 9-16 */
  { 0x64040008L, 0x64040118L }, /* Analog Inputs: chan number, 24-bit data */
  { 0x24040008L, 0x24040128L }, /* Analog Inputs: chan number, 40-bit data */
  { 0x62000108L, 0x62000208L }, /* Digital Outputs: 1-8, 9-16 */
  { 0x64110008L, 0x64110110L }  /* Analog Outputs: chan number, 16-bit data */
};

/* ------------------------------------------------------------------------ */
/* Globals */

/* Transmit-PDO and Receive-PDO communication parameters,
   stored in one array: first the TPDO parameters, then the RPDO parameters */
static PDO_COMM_PAR PdoCommPar[TPDO_CNT + RPDO_CNT]; /* (stored in EEPROM) */
static PDO_COMM_PAR *TPdoCommPar = &PdoCommPar[0];
static PDO_COMM_PAR *RPdoCommPar = &PdoCommPar[TPDO_CNT];

/* For timer-triggered PDO transmissions */
BOOL                TPdoOnTimer[TPDO_CNT];           /* (stored in EEPROM) */

/* Keeps track of time for the timer-triggered PDO transmissions
   (Timer1 is used to update these counters) */
UINT16              TPdoTimerCntr[TPDO_CNT];

/* ------------------------------------------------------------------------ */
/* Local prototypes */

static void pdo_load_config( void );

static BOOL pdo_get_comm_par( BYTE pdo_no,
			      BYTE od_subind,
			      BYTE *nbytes,
			      BYTE *par );

static BOOL pdo_get_mapping( BYTE pdo_no,
			     BYTE od_subind,
			     BYTE *nbytes,
			     BYTE *par );

/* ------------------------------------------------------------------------ */

void pdo_init( void )
{
  BYTE i;

  /* Initialize PDO configuration parameters */
  pdo_load_config();

#ifdef __VARS_IN_EEPROM__
  for( i=0; i<TPDO_CNT+RPDO_CNT; ++i )
    {
      BYTE byt;
      if( eeprom_read( EE_PDO_TTYPE+i ) != PdoCommPar[i].transmission_type )
	eeprom_write( EE_PDO_TTYPE+i, PdoCommPar[i].transmission_type );
      byt = (BYTE) (PdoCommPar[i].event_timer & (UINT16) 0x00FF);
      if( eeprom_read( EE_PDO_ETIMER_LO+i ) != byt )
	eeprom_write( EE_PDO_ETIMER_LO+i, byt );
      byt = (BYTE) ((PdoCommPar[i].event_timer & (UINT16) 0xFF00) >> 8);
      if( eeprom_read( EE_PDO_ETIMER_HI+i ) != byt )
	eeprom_write( EE_PDO_ETIMER_HI+i, byt );
    }
#endif /* __VARS_IN_EEPROM__ */

  /* Set timer stuff for Transmit-PDOs */
  TIMER1_DISABLE();
  for( i=0; i<TPDO_CNT; ++i )
    {
      TPdoOnTimer[i]    = ((PdoCommPar[i].transmission_type >= 254) &&
			   (PdoCommPar[i].event_timer > (UINT16)0));
      TPdoTimerCntr[i]  = (UINT16) 0;

#ifdef __VARS_IN_EEPROM__
      if( eeprom_read( EE_TPDO_ONTIMER+i ) != TPdoOnTimer[i] )
	eeprom_write( EE_TPDO_ONTIMER+i, TPdoOnTimer[i] );
#endif /* __VARS_IN_EEPROM__ */
    }
  TIMER1_ENABLE();

  /* If Remote Frames are not required adjust
     the CAN-controller's configuration */
  can_rtr_enable( pdo_rtr_required() );
}

/* ------------------------------------------------------------------------ */

void tpdo_scan( void )
{
  /* Handle ongoing multi-PDO transmissions (non-standard CANopen...),
     change-of-state PDO transmissions,
     and timer-triggered PDO transmission(s) */

  /* Ongoing multi-transmission Transmit-PDO: Analog Inputs */
  adc_pdo_scan();

  /* Some PDO(s) are to be sent on a change-of-state of the I/O:
     Transmit-PDO: Digital Inputs */
  digin_pdo_on_cos();

#ifdef __VARS_IN_EEPROM__
  TPdoOnTimer[TPDO_DIGITAL_IN] = eeprom_read( EE_TPDO_ONTIMER +
					      TPDO_DIGITAL_IN );
  TPdoOnTimer[TPDO_ANALOG_IN]  = eeprom_read( EE_TPDO_ONTIMER +
					      TPDO_ANALOG_IN );
  TPdoOnTimer[TPDO_ANALOG_IN_V]= eeprom_read( EE_TPDO_ONTIMER +
					      TPDO_ANALOG_IN_V );
#endif /* __VARS_IN_EEPROM__ */

  /* Timer-triggered Transmit-PDO: Digital Inputs */
  if( TPdoOnTimer[TPDO_DIGITAL_IN] )
    {
#ifdef __VARS_IN_EEPROM__
      PdoCommPar[TPDO_DIGITAL_IN].event_timer =
	((UINT16) eeprom_read( EE_PDO_ETIMER_LO + TPDO_DIGITAL_IN )) |
	(((UINT16) eeprom_read( EE_PDO_ETIMER_HI + TPDO_DIGITAL_IN )) << 8);
#endif /* __VARS_IN_EEPROM__ */

      /* If timer period expired... */ 
      if( TPdoTimerCntr[TPDO_DIGITAL_IN] >=
	  PdoCommPar[TPDO_DIGITAL_IN].event_timer )
	{
	  digin_pdo();

	  TIMER1_DISABLE();
	  TPdoTimerCntr[TPDO_DIGITAL_IN] = (UINT16) 0;
	  TIMER1_ENABLE();
	}
    }

  /* Timer-triggered Transmit-PDO: Analog Inputs
     (prioritize physical value read-out) */
  if( TPdoOnTimer[TPDO_ANALOG_IN_V] )
    {
#ifdef __VARS_IN_EEPROM__
      PdoCommPar[TPDO_ANALOG_IN_V].event_timer =
	((UINT16) eeprom_read( EE_PDO_ETIMER_LO + TPDO_ANALOG_IN_V )) |
	(((UINT16) eeprom_read( EE_PDO_ETIMER_HI + TPDO_ANALOG_IN_V )) << 8);
#endif /* __VARS_IN_EEPROM__ */

      /* If timer period expired start an ADC scan */ 
      if( TPdoTimerCntr[TPDO_ANALOG_IN_V] >=
	  PdoCommPar[TPDO_ANALOG_IN_V].event_timer )
	{
	  /* Readout in volts (or ADC-counts if not calibrated),
	     and forced if necessary */
	  adc_pdo_scan_start( TRUE, TRUE );

	  TIMER1_DISABLE();
	  TPdoTimerCntr[TPDO_ANALOG_IN_V] = (UINT16) 0;
	  TIMER1_ENABLE();
	}
    }
  else
    {
      if( TPdoOnTimer[TPDO_ANALOG_IN] )
	{
#ifdef __VARS_IN_EEPROM__
	  PdoCommPar[TPDO_ANALOG_IN].event_timer =
	    ((UINT16) eeprom_read( EE_PDO_ETIMER_LO + TPDO_ANALOG_IN )) |
	    (((UINT16) eeprom_read( EE_PDO_ETIMER_HI + TPDO_ANALOG_IN )) << 8);
#endif /* __VARS_IN_EEPROM__ */

	  /* If timer period expired start an ADC scan */ 
	  if( TPdoTimerCntr[TPDO_ANALOG_IN] >=
	      PdoCommPar[TPDO_ANALOG_IN].event_timer )
	    {
	      /* Readout in ADC-counts, and forced if necessary */
	      adc_pdo_scan_start( FALSE, TRUE );

	      TIMER1_DISABLE();
	      TPdoTimerCntr[TPDO_ANALOG_IN] = (UINT16) 0;
	      TIMER1_ENABLE();
	    }
	}
    }
}

/* ------------------------------------------------------------------------ */

void pdo_on_nmt( BYTE nmt_request )
{
  switch( nmt_request )
    {
    case NMT_START_REMOTE_NODE:
      {
	/* Immediately start first timer-triggered read out, if enabled... */
	BYTE pdo_no;
	TIMER1_DISABLE();
	for( pdo_no=0; pdo_no<TPDO_CNT; ++pdo_no )
	  {
#ifdef __VARS_IN_EEPROM__
	    PdoCommPar[pdo_no].event_timer =
	      ((UINT16) eeprom_read( EE_PDO_ETIMER_LO + pdo_no )) |
	      (((UINT16) eeprom_read( EE_PDO_ETIMER_HI + pdo_no )) << 8);
#endif /* __VARS_IN_EEPROM__ */

	    TPdoTimerCntr[pdo_no] = PdoCommPar[pdo_no].event_timer;
	  }
	TIMER1_ENABLE();
      }

      /* In case of ADC-readout-on-change we need to initialize
	 the ADC reference values ('last-sent' values) */
      adc_init_delta_references();

      break;

    case NMT_STOP_REMOTE_NODE:
    case NMT_ENTER_PREOPERATIONAL_STATE:
    case NMT_RESET_COMMUNICATION:
    case NMT_RESET_NODE:
      /* Do everything necessary to stop
	 ongoing channel scan operations properly */
      adc_pdo_scan_stop();

      break;

    default:
      break;
    }
}

/* ------------------------------------------------------------------------ */

void tpdo_on_sync( void )
{
  /* Send PDO(s) on the reception of a SYNC object:
     only if PDO(s) has (have) appropriate transmission type */

#ifdef __VARS_IN_EEPROM__
  PdoCommPar[TPDO_DIGITAL_IN].transmission_type =
    eeprom_read( EE_PDO_TTYPE + TPDO_DIGITAL_IN );
  PdoCommPar[TPDO_ANALOG_IN].transmission_type =
    eeprom_read( EE_PDO_TTYPE + TPDO_ANALOG_IN );
  PdoCommPar[TPDO_ANALOG_IN_V].transmission_type =
    eeprom_read( EE_PDO_TTYPE + TPDO_ANALOG_IN_V );
#endif /* __VARS_IN_EEPROM__ */

  /* Transmit-PDO: Digital Inputs */
  if( PdoCommPar[TPDO_DIGITAL_IN].transmission_type == 1 )
    {
      digin_pdo();
    }

  /* Transmit-PDO: Analog Inputs (prioritize physical value read-out) */
  if( PdoCommPar[TPDO_ANALOG_IN_V].transmission_type == 1 )
    {
      /* Readout in volts (or ADC-counts if not calibrated),
	 and forced if necessary */
      adc_pdo_scan_start( TRUE, TRUE );
    }
  else
    {
      if( PdoCommPar[TPDO_ANALOG_IN].transmission_type == 1 )
	{
	  /* Readout in ADC-counts, and forced if necessary */
	  adc_pdo_scan_start( FALSE, TRUE );
	}
    }
}

/* ------------------------------------------------------------------------ */

void tpdo1_on_rtr( void )
{
  /* Remote Transmission Request for Transmit-PDO1: Digital Inputs */

#ifdef __VARS_IN_EEPROM__
  PdoCommPar[TPDO_DIGITAL_IN].transmission_type =
    eeprom_read( EE_PDO_TTYPE + TPDO_DIGITAL_IN );
#endif /* __VARS_IN_EEPROM__ */

  /* Only if TPDO1 has appropriate transmission type */
  if( PdoCommPar[TPDO_DIGITAL_IN].transmission_type >= 253 )
    {
      digin_pdo();
    }
}

/* ------------------------------------------------------------------------ */

void tpdo2_on_rtr( void )
{
  /* Remote Transmission Request for Transmit-PDO2: Analog Inputs */

#ifdef __VARS_IN_EEPROM__
  PdoCommPar[TPDO_ANALOG_IN].transmission_type =
    eeprom_read( EE_PDO_TTYPE + TPDO_ANALOG_IN );
#endif /* __VARS_IN_EEPROM__ */

  /* Only if TPDO2 has appropriate transmission type */
  if( PdoCommPar[TPDO_ANALOG_IN].transmission_type >= 253 )
    {
      /* Readout in ADC-counts, and forced if necessary */
      adc_pdo_scan_start( FALSE, TRUE );
    }
}

/* ------------------------------------------------------------------------ */

void tpdo3_on_rtr( void )
{
  /* Remote Transmission Request for Transmit-PDO3: Analog Inputs in volts */

#ifdef __VARS_IN_EEPROM__
  PdoCommPar[TPDO_ANALOG_IN_V].transmission_type =
    eeprom_read( EE_PDO_TTYPE + TPDO_ANALOG_IN_V );
#endif /* __VARS_IN_EEPROM__ */

  /* Only if TPDO3 has appropriate transmission type */
  if( PdoCommPar[TPDO_ANALOG_IN_V].transmission_type >= 253 )
    {
      if( adc_calibrated() )
	{
	  /* Readout in volts, and forced if necessary */
	  adc_pdo_scan_start( TRUE, TRUE );
	}
      else
	{
	  /* CANopen Error Code 0x5000: device hardware */
	  can_write_emergency( 0x00, 0x50, EMG_ADC_NO_CALIB,
			       0, 0, 0, ERRREG_MANUFACTURER );
	}
    }
}

/* ------------------------------------------------------------------------ */

void rpdo1( BYTE dlc, BYTE *can_data )
{
  /* Receive-PDO1: Digital Outputs */

  digout_pdo( dlc, can_data );
}

/* ------------------------------------------------------------------------ */

void rpdo2( BYTE dlc, BYTE *can_data )
{
  /* Receive-PDO2: Analogue Outputs */

  dac_pdo( dlc, can_data );
}

/* ------------------------------------------------------------------------ */

BOOL pdo_rtr_required( void )
{
  /* Check if any of the transmission types requires CAN Remote Frames */
  BOOL pdo_no;
  BOOL required = FALSE;

  for( pdo_no=0; pdo_no<TPDO_CNT; ++pdo_no )
    {
#ifdef __VARS_IN_EEPROM__
      TPdoCommPar[pdo_no].transmission_type =
	eeprom_read( EE_PDO_TTYPE + pdo_no );
#endif /* __VARS_IN_EEPROM__ */

      /* Only if Transmit-PDO has certain transmission types */
      if( TPdoCommPar[pdo_no].transmission_type >= 253 )
	required = TRUE;
    }

  return required;
}

/* ------------------------------------------------------------------------ */

BOOL tpdo_get_comm_par( BYTE pdo_no,
			BYTE od_subind,
			BYTE *nbytes,
			BYTE *par )
{
  if( pdo_no < TPDO_CNT )
    {
      return( pdo_get_comm_par( pdo_no, od_subind, nbytes, par ) );
    }
  return FALSE;
}

/* ------------------------------------------------------------------------ */

BOOL rpdo_get_comm_par( BYTE pdo_no,
			BYTE od_subind,
			BYTE *nbytes,
			BYTE *par )
{
  if( pdo_no < RPDO_CNT )
    {
      pdo_no += TPDO_CNT;/* RPDO parameters are stored BEHIND the TPDO pars */
      return( pdo_get_comm_par( pdo_no, od_subind, nbytes, par ) );
    }
  return FALSE;
}

/* ------------------------------------------------------------------------ */

BOOL tpdo_get_mapping( BYTE pdo_no,
		       BYTE od_subind,
		       BYTE *nbytes,
		       BYTE *par )
{
  if( pdo_no < TPDO_CNT )
    {
      return( pdo_get_mapping( pdo_no, od_subind, nbytes, par ) );
    }
  return FALSE;
}

/* ------------------------------------------------------------------------ */

BOOL rpdo_get_mapping( BYTE pdo_no,
		       BYTE od_subind,
		       BYTE *nbytes,
		       BYTE *par )
{
  if( pdo_no < RPDO_CNT )
    {
      pdo_no += TPDO_CNT;/* RPDO parameters are stored BEHIND the TPDO pars */
      return( pdo_get_mapping( pdo_no, od_subind, nbytes, par ) );
    }
  return FALSE;
}

/* ------------------------------------------------------------------------ */

BOOL tpdo_set_comm_par( BYTE pdo_no,
			BYTE od_subind,
			BYTE nbytes,
			BYTE *par )
{
  /* If 'nbytes' is zero it means the data set size was
     not indicated in the SDO message */

  if( pdo_no >= TPDO_CNT ) return FALSE;

  switch( od_subind )
    {
    case OD_PDO_TRANSMTYPE:
      if( nbytes == 1 || nbytes == 0 )
	{
	  if( par[0] != 1 && par[0] != 255 ) return FALSE;

	  PdoCommPar[pdo_no].transmission_type = par[0];

#ifdef __VARS_IN_EEPROM__
	  if( eeprom_read(EE_PDO_TTYPE+pdo_no) !=
	      PdoCommPar[pdo_no].transmission_type )
	    eeprom_write( EE_PDO_TTYPE+pdo_no,
			  PdoCommPar[pdo_no].transmission_type );
#endif /* __VARS_IN_EEPROM__ */

	  /* Adjust CAN-controller configuration if necessary */
	  can_rtr_enable( pdo_rtr_required() );
	}
      else
	return FALSE;
      break;

    case OD_PDO_EVENT_TIMER:
      if( nbytes == 2 || nbytes == 0 )
	{
	  //UINT16 val;
	  //val  = (UINT16) par[0];
	  //val |= ((UINT16) par[1] << 8);
	  //val  = (val / (UINT16) 1000) & (UINT16) 0xFF;
	  /* In units of seconds, but must be <=255 ! */
	  //PdoCommPar[pdo_no].event_timer = (BYTE) val;

	  /* Timer now in units of seconds, <= 65535 */
	  PdoCommPar[pdo_no].event_timer  = (UINT16) par[0];
	  PdoCommPar[pdo_no].event_timer |= (((UINT16) par[1]) << 8);

#ifdef __VARS_IN_EEPROM__
	  if( eeprom_read( EE_PDO_ETIMER_LO+pdo_no ) != par[0] )
	    eeprom_write( EE_PDO_ETIMER_LO+pdo_no, par[0] );
	  if( eeprom_read( EE_PDO_ETIMER_HI+pdo_no ) != par[1] )
	    eeprom_write( EE_PDO_ETIMER_HI+pdo_no, par[1] );
#endif /* __VARS_IN_EEPROM__ */
	}
      else
	return FALSE;
      break;

    default:
      /* The sub-index does not exist */
      return FALSE;
    }

#ifdef __VARS_IN_EEPROM__
  PdoCommPar[pdo_no].transmission_type = eeprom_read(EE_PDO_TTYPE+pdo_no);
  PdoCommPar[pdo_no].event_timer       =
    ((UINT16) eeprom_read( EE_PDO_ETIMER_LO + pdo_no )) |
    (((UINT16) eeprom_read( EE_PDO_ETIMER_HI + pdo_no )) << 8);
#endif /* __VARS_IN_EEPROM__ */

  /* Update the PDO Event Timer stuff if necessary */
  TPdoOnTimer[pdo_no] = ((PdoCommPar[pdo_no].transmission_type >= 254)
			 && (PdoCommPar[pdo_no].event_timer > (UINT16)0));
#ifdef __VARS_IN_EEPROM__
  if( eeprom_read( EE_TPDO_ONTIMER+pdo_no ) != TPdoOnTimer[pdo_no] )
    eeprom_write( EE_TPDO_ONTIMER+pdo_no, TPdoOnTimer[pdo_no] );
#endif /* __VARS_IN_EEPROM__ */

  /* Immediately start first timer-triggered read out, if enabled... */
  TIMER1_DISABLE();
  TPdoTimerCntr[pdo_no] = PdoCommPar[pdo_no].event_timer;
  TIMER1_ENABLE();
  
  return TRUE;
}

/* ------------------------------------------------------------------------ */

static BOOL pdo_get_comm_par( BYTE pdo_no,
			      BYTE od_subind,
			      BYTE *nbytes,
			      BYTE *par )
{
  switch( od_subind )
    {
    case OD_NO_OF_ENTRIES:
      par[0]  = 5;
      *nbytes = 1;
      break;

    case OD_PDO_COBID:
      {
	UINT16 cob_id;
#ifdef __VARS_IN_EEPROM__
	NodeID = eeprom_read( EE_NODEID );
#endif /* __VARS_IN_EEPROM__ */

	/* Default values... */
	cob_id = PDO_COBID[pdo_no] | NodeID;

	par[0] = (BYTE) ((cob_id & (UINT16) 0x00ff) >> 0);
	par[1] = (BYTE) ((cob_id & (UINT16) 0xff00) >> 8);
	par[2] = 0x00;
	par[3] = 0x00;
	*nbytes = 4;
      }
      break;

    case OD_PDO_TRANSMTYPE:
#ifdef __VARS_IN_EEPROM__
      PdoCommPar[pdo_no].transmission_type = eeprom_read(EE_PDO_TTYPE+pdo_no);
#endif /* __VARS_IN_EEPROM__ */

      par[0]  = PdoCommPar[pdo_no].transmission_type;
      *nbytes = 1;
      break;

    case OD_PDO_INHIBITTIME:
      par[0]  = 0x00;
      par[1]  = 0x00;
      *nbytes = 2;
      break;

    case OD_PDO_DUMMY_ENTRY:
      par[0]  = 0x00;
      *nbytes = 1;
      break;

    case OD_PDO_EVENT_TIMER:
      {
	//UINT16 val;
#ifdef __VARS_IN_EEPROM__
	PdoCommPar[pdo_no].event_timer       =
	  ((UINT16) eeprom_read( EE_PDO_ETIMER_LO + pdo_no )) |
	  (((UINT16) eeprom_read( EE_PDO_ETIMER_HI + pdo_no )) << 8);
#endif /* __VARS_IN_EEPROM__ */

	/* In units of seconds, but must be <=255 ! */
	//val = (UINT16) 1000 * PdoCommPar[pdo_no].event_timer;
	//par[0] = (BYTE) ((val & (UINT16) 0x00FF) >> 0);
	//par[1] = (BYTE) ((val & (UINT16) 0xFF00) >> 8);

	/* Timer now in units of seconds, <= 65535 */
	par[0] = (BYTE) (PdoCommPar[pdo_no].event_timer & (UINT16) 0x00FF);
	par[1] = (BYTE) ((PdoCommPar[pdo_no].event_timer &
			  (UINT16) 0xFF00) >> 8);
	*nbytes = 2;
      }
      break;

    default:
      /* The sub-index does not exist */
      return FALSE;
    }

  return TRUE;
}

/* ------------------------------------------------------------------------ */

static BOOL pdo_get_mapping( BYTE pdo_no,
			     BYTE od_subind,
			     BYTE *nbytes,
			     BYTE *par )
{
  if( od_subind == OD_NO_OF_ENTRIES )
    {
      par[0] = PDOMAP_CNT[pdo_no];
      *nbytes = 1;
    }
  else
    {
      if( od_subind <= PDOMAP_CNT[pdo_no] )
	{
	  BYTE i;
	  const BYTE *p = (const BYTE *) &PDOMAP[pdo_no][od_subind-1];

	  for( i=0; i<4; ++i, ++p ) par[i] = *p;

	  *nbytes = 4;
	}
      else
	{
	  /* The sub-index does not exist */
	  return FALSE;
	}
    }
  return TRUE;
}

/* ------------------------------------------------------------------------ */

/* Note that not all PDO parameters fit in one storage block (16 bytes max)
   when there are more than 5 PDOs */
#define PDO_STORE_SIZE ((TPDO_CNT + RPDO_CNT) * sizeof(PDO_COMM_PAR))
// Split in 2 parts if necessary:
//#define TPDO_STORE_SIZE (TPDO_CNT * sizeof(PDO_COMM_PAR))
//#define RPDO_STORE_SIZE (RPDO_CNT * sizeof(PDO_COMM_PAR))

/* ------------------------------------------------------------------------ */

BOOL pdo_store_config( void )
{
  BYTE *p;

#ifdef __VARS_IN_EEPROM__
  BYTE i;
  for( i=0; i<TPDO_CNT+RPDO_CNT; ++i )
    {
      PdoCommPar[i].transmission_type = eeprom_read( EE_PDO_TTYPE+i );
      /* Timer now in units of seconds, <= 65535 */
      PdoCommPar[i].event_timer       =
	((UINT16) eeprom_read( EE_PDO_ETIMER_LO + i )) |
	(((UINT16) eeprom_read( EE_PDO_ETIMER_HI + i )) << 8);
    }
#endif /* __VARS_IN_EEPROM__ */

  p = (BYTE *) PdoCommPar;

  /* Store the configuration in EEPROM */
  return( store_write_block( STORE_PDO, PDO_STORE_SIZE, p ) );
}

/* ------------------------------------------------------------------------ */

static void pdo_load_config( void )
{
  BYTE *p;

  p = (BYTE *) PdoCommPar;

  /* Read the configuration from EEPROM, if any */
  if( !store_read_block( STORE_PDO, PDO_STORE_SIZE, p ) )
    {
      /* No valid parameters in EEPROM: use defaults */
      BYTE i;

      /* Set default Transmit-PDO communication parameters */
      for( i=0; i<TPDO_CNT; ++i )
	{
	  PdoCommPar[i].transmission_type = 1;	/* Respond to SYNC */
	  PdoCommPar[i].event_timer       = 0;	/* Seconds between triggers;
						   0 = not timer-triggered */
	}

      /* Set default Receive-PDO communication parameters */
      for( i=TPDO_CNT; i<RPDO_CNT+TPDO_CNT; ++i )
	{
	  PdoCommPar[i].transmission_type = 255;/* Profile specific */
	  PdoCommPar[i].event_timer       = 0;	/* not used for tPDOs... */
	}
    }
}

/* ------------------------------------------------------------------------ */
