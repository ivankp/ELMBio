/* ------------------------------------------------------------------------
File   : dac.c

Descr  : Functions for Digital-to-Analog output handling.

History: 07NOV.00; Henk B&B; Definition; multiple DACs and DAC channels are
			     supported; DAC hardware-specific code is omitted;
			     to be added in the form of a separate
			     software module.
	 23FEB.01; Henk B&B; Keep some variables in EEPROM (optionally).
	   AUG.01; Henk B&B; Added code for a 16-channel DAC module with
	                     MAX525 12-bit 4-channel DACs (under development
			     at NIKHEF for ATLAS DCS).
	 06JUN.03; Henk B&B; Version with MAX5122 DAC (for Suzanne Kersten).
	 30JUL.03; Henk B&B; Now must check for ADC-in-use because the ADC
	                     is not deselected during a conversion in a scan.
	 01AUG.03; Henk B&B; Allow use of either MAX5122 or MAX525,
	                     selected through an Object in the Dictionary.
--------------------------------------------------------------------------- */

#include "general.h"
#include "can.h"
#include "dac.h"
#include "eeprom.h"
#include "max5122.h"
#include "max525.h"
#include "store.h"

/* Need to keep track of ADC usage, because it shares the SPI */
#if !(defined(__ADC_AVR__) || defined(__ADC_NONE__))
#include "adc_elmb.h"
extern BOOL AdcConvInProgress;
extern void timer2_delay_mus( BYTE microseconds );
#endif

/* ------------------------------------------------------------------------ */
/* Globals */

/* Number of DAC modules actually connected */
static BYTE DacModuleCnt; /* (stored in EEPROM) */

/* Storage space for error bits: 1 byte per module */
static BYTE DacModuleError[DAC_MAX_MODULES];

/* Total number of DAC channels (in use) */
static BYTE DacChans;     /* (stored in EEPROM) */

/* Storage space for entered DAC values (just for reference) */
static BYTE DacData[DAC_MAX_CHANS][2];

/* Using MAX525? (instead of MAX5122) */
static BYTE DacMax525;    /* (stored in EEPROM) */

/* Signal hold time due to opto-coupler (in microseconds) */
BYTE        DacOptoDelay; /* (stored in EEPROM) */

/* ------------------------------------------------------------------------ */
/* Local prototypes */

static void dac_module_select  ( BYTE mod_no );
static void dac_module_deselect( void );
static void dac_load_config    ( void );

/* ------------------------------------------------------------------------ */

void dac_init( BOOL hard_reset )
{
  BYTE i;

  /* Initialize processor I/O pins involved in operation of the DACs */
  DAC_INIT_DDR();
  dac_module_deselect();
  DAC_CLEAR_SCLK();
  DAC_SET_SDI();

  /* Initialise DAC configuration parameters */
  dac_load_config();

  for( i=0; i<DAC_MAX_MODULES; ++i ) DacModuleError[i] = 0x00;

  for( i=0; i<DAC_MAX_CHANS; ++i )
    {
      DacData[i][0] = 0x00;
      DacData[i][1] = 0x00;
    }

  if( (hard_reset & TRUE) == TRUE )
    {
      /* DAC output initialization */
    }
}

/* ------------------------------------------------------------------------ */

BYTE dac_status( BYTE *status )
{
  BYTE i;
  /* One DAC module status per byte */
  for( i=0; i<DAC_MAX_MODULES; ++i )
    {
      status[i] = DacModuleError[i];
    }
  return DAC_MAX_MODULES; /* Return number of bytes */
}

/* ------------------------------------------------------------------------ */

BYTE dac_chan_cnt( void )
{
#ifdef __VARS_IN_EEPROM__
  DacChans = eeprom_read( EE_DACCHANS );
#endif

  return DacChans;
}

/* ------------------------------------------------------------------------ */

BOOL dac_write( BYTE chan_no, BYTE *dac_data )
{
#ifdef __VARS_IN_EEPROM__
  DacChans = eeprom_read( EE_DACCHANS );
#endif

  /* Refresh Data-Direction Register settings */
  DAC_INIT_DDR();

  if( chan_no < DacChans )
    {
      /* Set DAC channel according to 16-bit value in dac_data[] */
      BYTE module_no, module_chan_no, dac_no, dac_chan_no, i;
      BYTE chans_per_module, chans_per_dac;

#ifdef __VARS_IN_EEPROM__
      DacMax525    = eeprom_read( EE_DACMAX525 );
      DacOptoDelay = eeprom_read( EE_DACOPTODELAY );
#endif
      if( DacMax525 )
	{
	  chans_per_module = DACS_PER_MODULE*DAC_CHANS_PER_MAX525;
	  chans_per_dac    = DAC_CHANS_PER_MAX525;
	}
      else
	{
	  chans_per_module = DACS_PER_MODULE*DAC_CHANS_PER_MAX5122;
	  chans_per_dac    = DAC_CHANS_PER_MAX5122;
	}

      /* Determine DAC-module number */
      module_no = chan_no/chans_per_module;

      /* Adjust global channel number to a module channel number */
      module_chan_no = chan_no - module_no*chans_per_module;

      /* Determine index of DAC-chip within the module */
      dac_no = module_chan_no / chans_per_dac;

      /* Determine DAC-chip's channel number */
      dac_chan_no = module_chan_no - dac_no*chans_per_dac;

#if !(defined(__ADC_AVR__) || defined(__ADC_NONE__))
      if( AdcConvInProgress )
	{
	  /* Deselect ADC before selecting any DAC.. */
	  ADC_DESELECT();
	  timer2_delay_mus( 150 );
	}
#endif

      /* Select DAC module */
      dac_module_select( module_no );

      /* There are multiple DAC devices (ICs) connected in series,
	 so the highest DAC's data is shifted in first */
      for( i=DACS_PER_MODULE; i>0 ; --i )
	{
	  if( i-1==dac_no )
	    {
	      /* The DAC value to write */
	      if( DacMax525 )
		max525_write_dac( dac_chan_no, dac_data );
	      else
		max5122_write_dac( dac_chan_no, dac_data );
	    }
	  else
	    {
	      /* Write 'NOP' to other DAC devices */
	      if( DacMax525 )
		max525_write_nop();
	      else
		max5122_write_nop();
	    }
	}

      /* Set SDI to one when not writing:
	 switches off opto-coupler, reduces power consumption */
      DAC_SET_SDI();

      /* Deselect DAC module */
      dac_module_deselect();

#if !(defined(__ADC_AVR__) || defined(__ADC_NONE__))
      if( AdcConvInProgress ) ADC_SELECT();
#endif

      /* Keep a copy of the set value in RAM */
      DacData[chan_no][0] = dac_data[0];
      if( DacMax525 )
	DacData[chan_no][1] = dac_data[1] & MAX525_DATABITS_MASK;
      else
	DacData[chan_no][1] = dac_data[1] & MAX5122_DATABITS_MASK;

      return TRUE;
    }
  return FALSE;
}

/* ------------------------------------------------------------------------ */

BOOL dac_read( BYTE chan_no, BYTE *dac_data )
{
#ifdef __VARS_IN_EEPROM__
  DacChans = eeprom_read( EE_DACCHANS );
#endif

  if( chan_no < DacChans )
    {
      dac_data[0] = DacData[chan_no][0];
      dac_data[1] = DacData[chan_no][1];
      return TRUE;
    }
  else
    {
      dac_data[0] = 0x00;
      dac_data[1] = 0x00;
      return FALSE;
    }
}

/* ------------------------------------------------------------------------ */

void dac_pdo( BYTE dlc, BYTE *pdo_data )
{
  if( dlc > 2 )
    {
      /* Set DAC channel number 'pdo_data[0]'
	 according to 16-bit value in pdo_data[1]/pdo_data[2] */
      dac_write( pdo_data[0], &pdo_data[1] );
    }
  else
    {
      /* DLC too small... */

      /* CANopen Error Code 0x8210: protocol error
	 (PDO not processed due to length error), see CiA DS301 pg 9-38 */
      can_write_emergency( 0x10, 0x82, 3,
			   0, 0, 0, ERRREG_COMMUNICATION );
    }
}

/* ------------------------------------------------------------------------ */

static void dac_module_select( BYTE mod_no )
{
  /* Just to make sure: deselect */
  dac_module_deselect();

  /* Now select */
  if( mod_no == 0 ) DAC_SELECT_1();
  else if( mod_no == 1 ) DAC_SELECT_2();
  else if( mod_no == 2 ) DAC_SELECT_3();
  else if( mod_no == 3 ) DAC_SELECT_4();
}

/* ------------------------------------------------------------------------ */

static void dac_module_deselect( void )
{
  DAC_DESELECT_1();
  DAC_DESELECT_2();
  DAC_DESELECT_3();
  DAC_DESELECT_4();
}

/* ------------------------------------------------------------------------ */

#define DAC_STORE_SIZE 4

/* ------------------------------------------------------------------------ */

BOOL dac_store_config( void )
{
  BYTE block[DAC_STORE_SIZE];

#ifdef __VARS_IN_EEPROM__
  DacModuleCnt = eeprom_read( EE_DACMODULECNT );
  DacChans     = eeprom_read( EE_DACCHANS );
  DacMax525    = eeprom_read( EE_DACMAX525 );
  DacOptoDelay = eeprom_read( EE_DACOPTODELAY );
#endif

  block[0] = DacModuleCnt;
  block[1] = DacChans;
  block[2] = DacMax525;
  block[3] = DacOptoDelay;

  return( store_write_block( STORE_DAC, DAC_STORE_SIZE, block ) );
}

/* ------------------------------------------------------------------------ */

static void dac_load_config( void )
{
  BYTE block[DAC_STORE_SIZE];

  /* Read the configuration from EEPROM, if any */
  if( store_read_block( STORE_DAC, DAC_STORE_SIZE, block ) )
    {
      DacModuleCnt = block[0];
      DacChans     = block[1];
      DacMax525    = block[2];
      DacOptoDelay = block[3];
    }
  else
    {
      /* No valid parameters in EEPROM: use defaults */

      /* Set default DAC configuration settings: 1 module */
      DacModuleCnt = DAC_MAX_MODULES;
      DacMax525    = FALSE;
      DacOptoDelay = DAC_ELMB_SIGNAL_RISETIME;
      /* Total number of channels depends on the DAC type */
      if( DacMax525 )
	DacChans   = DacModuleCnt*DACS_PER_MODULE*DAC_CHANS_PER_MAX525;
      else
	DacChans   = DacModuleCnt*DACS_PER_MODULE*DAC_CHANS_PER_MAX5122;
    }
#ifdef __VARS_IN_EEPROM__
  /* Create working copies of configuration globals in EEPROM */
  if( eeprom_read( EE_DACMODULECNT ) != DacModuleCnt )
    eeprom_write( EE_DACMODULECNT, DacModuleCnt );
  if( eeprom_read( EE_DACCHANS ) != DacChans )
    eeprom_write( EE_DACCHANS, DacChans );
  if( eeprom_read( EE_DACMAX525 ) != DacMax525 )
    eeprom_write( EE_DACMAX525, DacMax525 );
  if( eeprom_read( EE_DACOPTODELAY ) != DacOptoDelay )
    eeprom_write( EE_DACOPTODELAY, DacOptoDelay );
#endif /* __VARS_IN_EEPROM__ */
}

/* ------------------------------------------------------------------------ */

BOOL dac_set_max525_select( BOOL ena )
{
  if( ena > 1 ) return FALSE;
  if( ena )
    DacMax525 = TRUE;
  else
    DacMax525 = FALSE;

  /* Total number of channels depends on the DAC type */
  if( DacMax525 )
    DacChans = DacModuleCnt*DACS_PER_MODULE*DAC_CHANS_PER_MAX525;
  else
    DacChans = DacModuleCnt*DACS_PER_MODULE*DAC_CHANS_PER_MAX5122;

#ifdef __VARS_IN_EEPROM__
  if( eeprom_read( EE_DACCHANS ) != DacChans )
    eeprom_write( EE_DACCHANS, DacChans );
  if( eeprom_read( EE_DACMAX525 ) != DacMax525 )
    eeprom_write( EE_DACMAX525, DacMax525 );
#endif /* __VARS_IN_EEPROM__ */
  return TRUE;
}

/* ------------------------------------------------------------------------ */

BOOL dac_get_max525_select( void )
{
#ifdef __VARS_IN_EEPROM__
  DacMax525 = eeprom_read( EE_DACMAX525 );
#endif /* __VARS_IN_EEPROM__ */

  return DacMax525;
}

/* ------------------------------------------------------------------------ */

BOOL dac_set_opto_delay( BYTE delay )
{
  if( delay < 10 ) return FALSE;
  DacOptoDelay = delay;

#ifdef __VARS_IN_EEPROM__
  if( eeprom_read( EE_DACOPTODELAY ) != DacOptoDelay )
    eeprom_write( EE_DACOPTODELAY, DacOptoDelay );
#endif /* __VARS_IN_EEPROM__ */
  return TRUE;
}

/* ------------------------------------------------------------------------ */

BYTE dac_get_opto_delay( void )
{
#ifdef __VARS_IN_EEPROM__
  DacOptoDelay = eeprom_read( EE_DACOPTODELAY );
#endif /* __VARS_IN_EEPROM__ */

  return DacOptoDelay;
}

/* ------------------------------------------------------------------------ */
