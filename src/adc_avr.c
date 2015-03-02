/* ------------------------------------------------------------------------
File   : adc_avr.c

Descr  : Functions for controlling the ELMB's ATmega103 onchip 8-channel
         10-bit ADC ADC.

History: 21NOV.00; Henk B&B; Start of development, all calls are based on
                             similar modules with the same API for other
			     (more complex) ADCs, so in this case several
			     functions are empty or very simple.
	 27FEB.03; Henk B&B; ATmega128: select internal 2.56V ref (in ADMUX).
--------------------------------------------------------------------------- */

#include "general.h"
#include "adc.h"
#include "can.h"
#include "eeprom.h"
#include "store.h"
#include "timer103.h"

#define ADCSR ADCSRA

/* ------------------------------------------------------------------------ */
/* Globals */

/* Total number of ADC channels */
static BYTE AdcChans;     /* (stored in EEPROM) */

/* ADC configuration to use (word rate, voltage range, unipolar/bipolar) */
static BYTE AdcConfig;    /* (stored in EEPROM) */

/* ------------------------------------------------------------------------ */
/* Global variables for ADC channel scanning operations */

/* ADC-channel index */
static BYTE AdcChanNo;

/* ADC scanning-operation-in-progress boolean */
static BOOL AdcScanInProgress;

/* ADC conversion-in-progress boolean */
static BOOL AdcConvInProgress;

/* ------------------------------------------------------------------------ */
/* Local prototypes */

static BOOL adc_scan( void );

static void adc_load_config( void );

/* ------------------------------------------------------------------------ */

BOOL adc_init( void )
{
  BYTE result=TRUE;

  /* Disable ADC */
  ADCSR = 0;

  /* Initialise ADC configuration parameters */
  adc_load_config();

  /* Initialize variables for scanning operations */
  AdcChanNo         = 0;
  AdcScanInProgress = FALSE;
  AdcConvInProgress = FALSE;

  /* Enable and configure ADC */
  ADCSR |= ADC_ENABLE | AdcConfig;

  /* Enable interrupt for noise-canceling mode */
  ADCSR |= ADC_INTERRUPT_ENABLE;

  return result;
}

/* ------------------------------------------------------------------------ */
/* ADC interrupt */

#pragma interrupt_handler adc_handler:22

void adc_handler( void )
{
  AdcConvInProgress = FALSE;
}

/* ------------------------------------------------------------------------ */

BOOL adc_read( BYTE chan_no, BYTE *conversion_data )
{
  /* Set multiplexor (on ATmega128: select internal 2.56V reference) */
  ADMUX = 0xC0 | chan_no;

  AdcConvInProgress = TRUE;

#ifdef __ELMB103__
  /* ADC conversion with ATmega103 ADC Noise Canceler Function (see doc) */

  /* Disable ADC */
  ADCSR &= ADC_ENABLE;
  /* Enable ADC + start conversion */
  ADCSR |= (ADC_ENABLE | ADC_START_CONVERSION);

  /* Now go to sleep...
     (within 14 ADC clock cycles; hopefully this is not interrupted) */
  MCUCR |= BIT( SE );
  SLEEP();

#else
  /* ADC conversion with ATmega128 ADC Noise Canceler Function (see doc) */

  /* (Re)enable ADC just to make sure... */
  ADCSR |= ADC_ENABLE;

  /* Enter ADC Noise Reduction sleep mode */
  MCUCR |= BIT(SE) | BIT(SM0);
  SLEEP();
#endif /* __ELMB103__ */

  /* Take into account that I might have been woken up
     by some other interrupt source...,
     then unfortunately (busy-)wait for conversion done */
  while( (AdcConvInProgress & TRUE) == TRUE );

  /* Read conversion data */
  conversion_data[0] = ADCL;
  conversion_data[1] = ADCH;

  return TRUE;
}

/* ------------------------------------------------------------------------ */

BYTE adc_status( BYTE *status )
{
  *status = 0;
  return 1; /* Return number of bytes */
}

/* ------------------------------------------------------------------------ */

BOOL adc_get_config( BYTE subindex, BYTE *nbytes, BYTE *par )
{
  *nbytes = 1;

  switch( subindex )
    {
    case 0:
      /* Number of entries */
      par[0] = 2;
      break;

    case 1:
#ifdef __VARS_IN_EEPROM__
      AdcChans = eeprom_read( EE_ADCCHANS );
#endif
      par[0] = AdcChans;
      break;

    case 2:
#ifdef __VARS_IN_EEPROM__
      AdcConfig = eeprom_read( EE_ADCCONFIG );
#endif
      par[0] = AdcConfig;
      break;

    default:
      return FALSE;
    }

  return TRUE;
}

/* ------------------------------------------------------------------------ */

BOOL adc_set_config( BYTE subindex, BYTE nbytes, BYTE *par )
{
  /* If 'nbytes' is zero it means the data set size was
     not indicated in the SDO message */

  /* Nothing to set... */
  return FALSE;
}

/* ------------------------------------------------------------------------ */

void adc_pdo_scan_start( BOOL volts, BOOL force_readout )
{
  /* Start scanning only if scanning not already in progress */

  if( (AdcScanInProgress & TRUE) == FALSE )
    {
      /* Start an ADC-channels-to-PDOs scan */
      AdcChanNo = 0;
      AdcScanInProgress = adc_scan();
    }
}

/* ------------------------------------------------------------------------ */

void adc_pdo_scan_stop( void )
{
  /* We have to stop a possibly ongoing ADC conversion... */

  if( (AdcScanInProgress & TRUE) == TRUE )
    {
    }

  /* Initialize variables for scanning operations */
  AdcChanNo         = 0;
  AdcScanInProgress = FALSE;
  AdcConvInProgress = FALSE;
}

/* ------------------------------------------------------------------------ */

void adc_pdo_scan( void )
{
  /* Handle an on-going ADC channel scan, otherwise do nothing */

  if( (AdcScanInProgress & TRUE) == TRUE )
    {
      /* Read out next ADC channel */
      AdcScanInProgress = adc_scan();
    }
}

/* ------------------------------------------------------------------------ */

static BOOL adc_scan( void )
{
  BYTE pdo_data[4];

  /* Should only generate a next PDO when the previous one
     has been sent */
  if( can_transmitting(C91_TPDO2) )
    {
      /* So wait... */
      return TRUE;
    }

  /* Do a conversion and read out the data */
  adc_read( AdcChanNo, &pdo_data[2] );

  /* Put ADC configuration in as well */
#ifdef __VARS_IN_EEPROM__
  AdcConfig = eeprom_read( EE_ADCCONFIG );
#endif
  pdo_data[1] = AdcConfig;

  /* Put the ADC channel number in the message */
  pdo_data[0] = AdcChanNo;

  /* Send as a Transmit-PDO2 CAN-message */
  can_write( C91_TPDO2, C91_TPDO2_LEN, pdo_data );

  /* Next time, next channel
     (SEE protection version of '++AdcChanNo;') */
  AdcChanNo = (AdcChanNo & (ADC_MAX_INPUTS-1)) + 1;

#ifdef __VARS_IN_EEPROM__
  AdcChans  = eeprom_read( EE_ADCCHANS );
#endif

  /* Are we done with the current scan ? */
  if( AdcChanNo == AdcChans )
    {
      /* Yes, we're done... */
      return FALSE;
    }

  return TRUE;
}

/* ------------------------------------------------------------------------ */

#define ADC_STORE_SIZE 2

BOOL adc_store_config( void )
{
  BYTE block[ADC_STORE_SIZE];

#ifdef __VARS_IN_EEPROM__
  AdcChans  = eeprom_read( EE_ADCCHANS );
  AdcConfig = eeprom_read( EE_ADCCONFIG );
#endif

  block[0] = AdcChans;
  block[1] = AdcConfig;

  return( store_write_block( STORE_ADC, ADC_STORE_SIZE, block ) );
}

/* ------------------------------------------------------------------------ */

static void adc_load_config( void )
{
  BYTE block[ADC_STORE_SIZE];

  /* Read the configuration from EEPROM, if any */
  if( store_read_block( STORE_ADC, ADC_STORE_SIZE, block ) )
    {
      AdcChans  = block[0];
      AdcConfig = block[1];
    }
  else
    {
      /* No valid parameters in EEPROM: use defaults */
      AdcChans  = ADC_MAX_INPUTS;
      AdcConfig = ADC_DFLT_CONV_PARS;
    }

#ifdef __VARS_IN_EEPROM__
  if( eeprom_read( EE_ADCCHANS ) != AdcChans )
    eeprom_write( EE_ADCCHANS, AdcChans );
  if( eeprom_read( EE_ADCCONFIG ) != AdcConfig )
    eeprom_write( EE_ADCCONFIG, AdcConfig );
#endif /* __VARS_IN_EEPROM__ */
}

/* ------------------------------------------------------------------------ */
