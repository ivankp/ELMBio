/* ------------------------------------------------------------------------
File   : adc_elmb.c

Descr  : Functions for controlling the ELMB onboard CRYSTAL CS5523 16-bit ADC.

History: 19JUL.00; Henk B&B; Start of development, based on a similar module
			     with routines for CS5525 ADCs for ATLAS MDT-
			     monitoring applications and an existing set of
			     basic routines for operating the CS5523.
	 18SEP.00; Henk B&B; Add routine 'adc_chan_cnt()';
	 		     add recalibration to 'adc_set_config()';
			     in the PDO replace the ADC status byte with
			     a byte containing the current ADC-configuration
			     and status-bit.
         16JAN.00; Henk B&B; Go to Motherboard V3 configuration, but
	                     allow backwards compatibility using compile
			     options __MOTHERBOARD1__ or __ALL_MOTHERBOARDS__.
	 23FEB.01; Henk B&B; Keep some variables in EEPROM (optionally).
	 02APR.01; Henk B&B; Refresh the ADC Configuration Register before
	                     every scan cycle (in adc_pdo_scan_start()).
	 18APR.01; Henk B&B; Perform a complete reset and calibration procedure
	                     before every scan cycle (in adc_pdo_scan_start()),
			     which includes a Config Register refresh.
	 25JUL.01; Henk B&B; Make the above optional/configurable...
	   AUG.02; Henk B&B; Add readout-on-change mechanism with adjustable
	                     per-channel delta parameter and optional
			     forced readout scan cycles.
	   OCT.02; Henk B&B; Add stuff for voltage range calibration
	                     and calibrated readout.
	   MAR.03; Henk B&B; Add upper- and lower-limit checking and stuff for
	                     storing and administering these limit values
			     per-channel.
	 16OCT.03; Henk B&B; Window width must at least be 1 for window check
	                     to take place.
--------------------------------------------------------------------------- */

#include "general.h"
#include "adc.h"
#include "can.h"
#include "crc.h"
#include "cs5523.h"
#include "eeprom.h"
#include "objects.h"
#include "store.h"
#include "timer103.h"

/* ------------------------------------------------------------------------ */
/* Globals */

/* Total number of ADC channels */
static BYTE    AdcChans;     /* (stored in EEPROM) */

/* ADC configuration to use (word rate, voltage range, unipolar/bipolar) */
static BYTE    AdcConfig;    /* (stored in EEPROM) */

/* ADC Configuration Register to use
   (chop frequency: depends on selected wordrate) */
static BYTE    AdcConfReg_2; /* (stored in EEPROM) */

/* Storage space for error bits concerning the ADC */
static BYTE    AdcError;

/* Signal hold time due to opto-coupler (in microseconds) */
BYTE           AdcOptoDelay; /* (stored in EEPROM) */

/* ------------------------------------------------------------------------ */
/* Global variables for ADC channel scanning operations */

/* ADC-channel index */
static BYTE    AdcChanNo;

/* ADC scanning-operation-in-progress boolean */
static BOOL    AdcScanInProgress;

/* ADC conversion-in-progress boolean */
BOOL           AdcConvInProgress;

/* ADC conversion-in-progress-to-set-latch booleans */
static BOOL    AdcLatchBeingSet;
static BOOL    AdcLatchSet;

/* ADC recalibrate before every scan cycle */
static BOOL    AdcCalibBeforeScan;

/* ADC-readout-on-change enable */
static BOOL    AdcReadoutOnChange;

/* ADC delta-scan enable */
static BOOL    AdcDeltaScanEnabled;

/* Last sent ADC channel values in ADC-readout-on-change scanning operation */
static UINT16  AdcRefCount[ADC_MAX_INPUTS];

/* When ADC-readout-on-change is enabled, setting this boolean to TRUE
   forces initalization of the reference value of all channels
   in 'AdcRefCount[]' */
static BOOL    AdcInitRefCount;

/* ADC upper/lower-limit-scan enable */
static BOOL    AdcWindowScanEnabled;

/* Inital value for ADC upper/lower-limit inside/outside window counter;
   also used for the delta-change mode */
static BYTE    AdcWindowCounterInit;

/* Inside/outside-window counter per channel */
static CHAR    AdcWindowCounter[ADC_MAX_INPUTS];

/* Delta-change counter per channel */
static BYTE    AdcDeltaCounter[ADC_MAX_INPUTS];

/* When ADC-readout-on-change is enabled, the setting of this boolean
   forces the readout values of all channels to be sent once */
static BOOL    AdcForcedReadout;

/* Readout of ADC channels during a scan is in (micro)Volts
   when this boolean is set */
static BOOL    AdcScanVolts;

/* ------------------------------------------------------------------------ */
/* Global variables for ADC calibration and volts-conversion */

/* To enable a single write to an calibration constant location in EEPROM */
static BOOL AdcCalibConstWriteEnabled;

/* ------------------------------------------------------------------------ */
/* Useful constants */

/* Map from calibration constants OD index to
   (3-bit) ADC voltage range code (and vice-versa) */
const BYTE  RANGE_ID[] = { 2, 1, 0, 3, 5, 4 };

/* ADC full-scale voltage ranges in microVolts */
const float V_FULLSCALE[] = { 100000.0,   55000.0,   25000.0,
			     1000000.0, 5000000.0, 2500000.0 };

/* Map from ADC wordrate code/id to approximate wordrate in Hz */
const BYTE  HZ_WORDRATE[] = { 15, 30, 62, 85, 101, 2, 4, 8 };

/* ------------------------------------------------------------------------ */
/* Local prototypes */

static void   adc_convert_to_volts  ( UINT16 adc_count, BYTE *microvolts );
static BOOL   adc_set_calib_regs    ( BOOL send_emergency );
static BOOL   adc_self_calibrate    ( BOOL send_emergency );
static void   adc_init_confreg      ( void );
static void   adc_init_csr          ( void );
static BOOL   adc_scan_next         ( void );
static BOOL   adc_delta_check       ( UINT16 adc_count );
static BOOL   adc_window_check      ( UINT16 adc_count );

static BOOL   adc_load_config       ( void );

static UINT16 adc_get_delta_cnt     ( BYTE chan_no );
static UINT16 adc_get_upperlimit_cnt( BYTE chan_no );
static UINT16 adc_get_lowerlimit_cnt( BYTE chan_no );
static UINT16 adc_get_limit_cnt     ( BYTE chan_no, UINT16 ee_copy_addr );

static BOOL   adc_set_limit         ( BYTE chan_no, BYTE *limit,
				      UINT16 base_addr );
static void   adc_get_limit         ( BYTE chan_no, BYTE *limit,
				      UINT16 base_addr );

static BOOL   adc_load_deltas       ( void );
static BOOL   adc_load_upperlimits  ( void );
static BOOL   adc_load_lowerlimits  ( void );
static BOOL   adc_load_limits       ( UINT16 ee_storage_addr,
				      UINT16 ee_copy_addr,
				      BYTE   err_id,
				      BYTE   dflt );

static BOOL   adc_store_limits      ( UINT16 ee_storage_addr,
				      UINT16 ee_copy_addr );

static BOOL   adc_invalidate_limits ( UINT16 ee_storage_addr );

static BOOL   adc_valid_calib_const ( BYTE od_range_id );

/* ------------------------------------------------------------------------ */

#ifdef __ALL_MOTHERBOARDS__

static void motherboard_select( BYTE version );

/* Pointers to the functions doing the actual I/O pin operations:
   the pointers get set depending on the Motherboard type used */
void (*ADC_SET_SCLK)( void );
void (*ADC_CLEAR_SCLK)( void );
void (*ADC_SET_SDI)( void );
void (*ADC_CLEAR_SDI)( void );
BOOL (*ADC_SDO_HIGH)( void );
BOOL (*ADC_SDO_LOW)( void );
void (*ADC_SET_MUX_LATCH)( void );
void (*ADC_CLEAR_MUX_LATCH)( void );
void (*ADC_SELECT)( void );
void (*ADC_DESELECT)( void );

#endif /* __ALL_MOTHERBOARDS__ */

/* ------------------------------------------------------------------------ */

BOOL adc_init( void )
{
  BYTE wordrate;
  BOOL result = TRUE;

  AdcError = 0;

  /* Initialise ADC configuration parameters */
  if( adc_load_config() == FALSE ) result = FALSE;

  /* Set wordrate-dependent setting for ADC Configuration Register:
     Chop Frequency:  256 Hz (at freq <= 30 Hz), 4096 Hz (other freq)
     		      (recommended by Crystal, CS5523 datasheet pg 14) */
  wordrate = ((AdcConfig & CS23_CSR_WORDRATE_MASK) >> CS23_CSR_WORDRATE_SHIFT);
  if( wordrate == CS23_WORDRATE_61 ||
      wordrate == CS23_WORDRATE_84 ||
      wordrate == CS23_WORDRATE_101 )
    AdcConfReg_2 = ADC_CNFREG_2_CHOP4096;
  else
    AdcConfReg_2 = ADC_CNFREG_2_CHOP256;

#ifdef __VARS_IN_EEPROM__
  /* Create working copies of configuration globals in EEPROM */
  if( eeprom_read( EE_ADCCONFREG2 ) != AdcConfReg_2 )
    eeprom_write( EE_ADCCONFREG2, AdcConfReg_2 );
#endif /* __VARS_IN_EEPROM__ */

  /* Initialize variables for scanning operations */
  AdcChanNo         = 0;
  AdcScanInProgress = FALSE;
  AdcConvInProgress = FALSE;
  AdcLatchBeingSet  = FALSE;
  AdcLatchSet       = FALSE;
  AdcForcedReadout  = FALSE;
  AdcInitRefCount   = FALSE;
  AdcScanVolts      = FALSE;

  /* Other */
  AdcCalibConstWriteEnabled = FALSE;

#ifdef __MOTHERBOARD1__
  /* Initialise I/O-pins for ADC: old Motherboard configuration */
  ADC_CLEAR_SCLK_V1();
  ADC_SET_SDI_V1();
  ADC_SET_MUX_LATCH_V1();
  ADC_DESELECT_V1();
  ADC_INIT_DDR_V1();
#else
  /* Initialise I/O-pins for ADC: assume Motherboard V3 configuration */
  ADC_CLEAR_SCLK_V3();
  ADC_SET_SDI_V3();
  ADC_SET_MUX_LATCH_V3();
  ADC_DESELECT_V3();
  ADC_INIT_DDR_V3();
#endif

#ifdef __ALL_MOTHERBOARDS__
  /* Initialise pointers to I/O-pin access functions */
  motherboard_select( 3 );
#endif /* __ALL_MOTHERBOARDS__ */

#ifndef __ALL_MOTHERBOARDS__
  /* Reset and calibrate (but don't send Emergency messages) */
  if( adc_reset_and_calibrate( FALSE ) == FALSE ) result = FALSE;
#else
  if( adc_reset_and_calibrate( FALSE ) == FALSE )
    {
      /* Problem with accessing ADC? :
	 are we dealing with Motherboard V1/2 perhaps ? */

      /* Initialise I/O-pins for ADC: Old Motherboard configuration */
      ADC_CLEAR_SCLK_V1();
      ADC_SET_SDI_V1();
      ADC_SET_MUX_LATCH_V1();
      ADC_DESELECT_V1();
      ADC_INIT_DDR_V1();

      /* Initialise pointers to I/O-pin access functions */
      motherboard_select( 1 );

      /* Try again, now assuming the ADC is connected differently:
	 reset and calibrate (but don't send Emergency messages) */
      if( adc_reset_and_calibrate( FALSE ) == FALSE ) result = FALSE;
    }
#endif /* __ALL_MOTHERBOARDS__ */

  adc_init_delta_references();

  return result;
}

/* ------------------------------------------------------------------------ */

BOOL adc_read( BYTE chan_no, BYTE *conversion_data )
{
  BYTE bank_no;
  BOOL result;

  if( (AdcConvInProgress & TRUE) == TRUE )
    {
      /* Can't do anything if the ADC is in use... */
      /*conversion_data[0] = 0xFF;
	conversion_data[1] = 0xFF;
	conversion_data[2] = 0xFE;*/
      return FALSE;
    }
  else
    {

/*#define __DO_NOT_CONVERT_WHEN_ERROR__ */

#ifdef __DO_NOT_CONVERT_WHEN_ERROR__
      if( (AdcError & ADC_ERR_IN_HARDWARE) != 0 )
	{
	  /* Something has gone wrong with this ADC previously */
	  /*conversion_data[0] = 0xFF;
	    conversion_data[1] = 0xFF;
	    conversion_data[2] = 0xFD;*/
	  return FALSE;
	}
      else
#endif /* __DO_NOT_CONVERT_WHEN_ERROR__ */
	{
	  /* Perform an ADC conversion */

#ifdef __VARS_IN_EEPROM__
	  AdcOptoDelay = eeprom_read( EE_ADCOPTODELAY );
#endif
	  ADC_SELECT();

	  /* =================================== */
	  /* Select the proper set of 16 channels */

	  bank_no = chan_no/16;

	  /* Change of 16-chan inputs bank: select bank,
	     by initiating a (dummy) conversion;
	     LC 5 to 8 contain the proper setting of A1-A0
	     (set during initialization) to accomplish this */

	  result = cs5523_read_adc( 4 + bank_no, conversion_data );

	  /* Now latch this data */
	  ADC_CLEAR_MUX_LATCH();
	  //timer2_delay_mus( CS23_ELMB_SIGNAL_RISETIME );
	  timer2_delay_mus( AdcOptoDelay );
	  ADC_SET_MUX_LATCH();

	  /* =================================== */
	  /* Select the proper channel within the chosen set of 16 */

	  if( result == TRUE )
	    {
	      BYTE a1a0, cs1cs0;
	      BYTE csr_data[1*2*2];

#ifdef __VARS_IN_EEPROM__
	      AdcConfig = eeprom_read( EE_ADCCONFIG );
#endif
	      /* Set A1-A0 and physical channel in LC 1 (+2)
		 while maintaining proper wordrate and gain */
	      a1a0        = chan_no & 3;
	      cs1cs0      = (chan_no>>2) & 3;
	      csr_data[0] = (AdcConfig |
			     ((cs1cs0 << CS23_CSR_PHYSCHAN_SEL_LO_SHIFT) &
			      CS23_CSR_PHYSCHAN_SEL_LO_MASK));
	      csr_data[1] = ((a1a0 << CS23_CSR_A1A0_SHIFT) |
			     ((cs1cs0 >> CS23_CSR_PHYSCHAN_SEL_HI_SHIFT) &
			      CS23_CSR_PHYSCHAN_SEL_HI_MASK));
	      csr_data[2] = 0x00;
	      csr_data[3] = 0x00;
	      cs5523_write_csr( 1, csr_data );

	      /* Do a conversion of LC 1 and read the result */
	      result = cs5523_read_adc( 0, conversion_data );
	    }

	  /* =================================== */

	  ADC_DESELECT();

	  if( result == FALSE )
	    {
	      /* Conversion timed out ! */
	      AdcError |= ADC_ERR_TIMEOUT;

	      /* CANopen Error Code 0x5000: device hardware */
	      can_write_emergency( 0x00, 0x50, EMG_ADC_CONVERSION, chan_no,
				   0, 0, ERRREG_MANUFACTURER );

	      adc_serial_init();

	      return FALSE;
	    }
	}
    }
  return TRUE;
}

/* ------------------------------------------------------------------------ */

BOOL adc_read_volts( BYTE chan_no, BYTE *data )
{
  BYTE   conversion_data[3];
  BYTE   microvolts[4];
  UINT16 adc_count;

  /* Readout in volts only when calibration constants are there
     (for the current ADC setting) */
  if( adc_calibrated() == FALSE ) return FALSE;

  /* Read out the ADC-count and status flags */
  if( adc_read( chan_no, conversion_data ) == FALSE ) return FALSE;

  /* The ADC status flags */
  data[0] = conversion_data[0];

  /* The ADC-count as a 16-bit (unsigned) integer */
  adc_count = (((UINT16) conversion_data[1]) |
	       (((UINT16) conversion_data[2]) << 8));

  /* Convert the 16-bit ADC-count to microVolts units (signed) */
  adc_convert_to_volts( adc_count, microvolts );

  /* The microvolts value is sent as a 3-byte integer,
     right behind the status flags byte */
  data[1] = microvolts[0];
  data[2] = microvolts[1];
  data[3] = microvolts[2];

  return TRUE;
}

/* ------------------------------------------------------------------------ */

static void adc_convert_to_volts( UINT16 adc_count, BYTE *microvolts )
{
  BOOL   unipolar;
  BYTE   adc_range_id;
  float  fullscale;

  /* Get the ADC settings */
  unipolar     = (AdcConfig & CS23_CSR_UNIPOLAR);
  adc_range_id = ((AdcConfig & CS23_CSR_GAIN_MASK) >> CS23_CSR_GAIN_SHIFT);
  fullscale    = V_FULLSCALE[adc_range_id];

  if( unipolar )
    {
      /* Unipolar measurement: use unsigned numbers ! */
      UINT32 adc_volts_ul;
      float  adc_volts_f;
      UINT32 *ptr;

      /* Do the ADC-count to microVolts calculation */
      adc_volts_f  = (((float) adc_count) / 65535.0) * fullscale;

      /* Cast to a UINT32 */
      adc_volts_ul = (UINT32) adc_volts_f;

      /* Copy the UINT32 into the BYTE array */
      ptr = (UINT32 *) microvolts;
      *ptr = adc_volts_ul;

      // The following also works(?):
      //BYTE i, *ptr = (BYTE *) &adc_volts_l;
      //for( i=0; i<4; ++i, ++ptr ) microvolts[i] = *ptr;
    }
  else
    {
      /* Bipolar measurement: use signed numbers ! */
      INT16 *padc_cnt;
      INT32 adc_volts_l;
      float adc_volts_f;
      INT32 *ptr;

      /* Interpret the UINT16 as an INT16 */
      padc_cnt    = (INT16 *) &adc_count;

      /* Do the ADC-count to microVolts calculation */
      adc_volts_f = (((float)(*padc_cnt)) / 32767.0) * fullscale;

      /* Cast to an INT32 */
      adc_volts_l = (INT32) adc_volts_f;

      /* Copy the INT32 into the BYTE array */
      ptr = (INT32 *) microvolts;
      *ptr = adc_volts_l;
    }
}

/* ------------------------------------------------------------------------ */

BYTE adc_status( BYTE *status )
{
  *status = AdcError;
  return 1; /* Return number of bytes */
}

/* ------------------------------------------------------------------------ */

BOOL adc_get_config( BYTE subindex, BYTE *nbytes, BYTE *par )
{
  *nbytes = 1;

#ifdef __VARS_IN_EEPROM__
  AdcChans     = eeprom_read( EE_ADCCHANS );
  AdcConfig    = eeprom_read( EE_ADCCONFIG );
  AdcOptoDelay = eeprom_read( EE_ADCOPTODELAY );
#endif

  switch( subindex )
    {
    case 0:
      /* Number of entries */
      par[0] = 21;
      break;

    case 1:
      par[0] = AdcChans;
      break;

    case 2:
      par[0] = ((AdcConfig & CS23_CSR_WORDRATE_MASK) >>
	        CS23_CSR_WORDRATE_SHIFT);
      break;

    case 3:
      par[0] = ((AdcConfig & CS23_CSR_GAIN_MASK) >>
		CS23_CSR_GAIN_SHIFT);
      break;

    case 4:
      par[0] = (AdcConfig & CS23_CSR_UNIPOLAR);
      break;

    case 6:
      {
	if( (AdcConvInProgress & TRUE) == TRUE ) return FALSE;

	ADC_SELECT();

	/* Read the Configuration Register */
	cs5523_read_reg( CS23_CMD_CONFIG_REG, 0, par );

	ADC_DESELECT();

	*nbytes = 4;
      }
      break;

    case 7:
    case 8:
    case 9:
    case 10:
    case 11:
    case 12:
    case 13:
    case 14:
      {
	BYTE reg, phys_chan_no;

	if( (AdcConvInProgress & TRUE) == TRUE ) return FALSE;

	if( subindex & 1 ) reg = CS23_CMD_OFFSET_REG;
	else reg = CS23_CMD_GAIN_REG;

	phys_chan_no = (subindex - 7) >> 1;

	ADC_SELECT();

	/* Read Register */
	cs5523_read_reg( reg, phys_chan_no, par );

	ADC_DESELECT();

	*nbytes = 4;
      }
      break;

    case 15:
    case 16:
    case 17:
    case 18:
      {
	BYTE csr_data[4*2*2], *csr, i;

	if( (AdcConvInProgress & TRUE) == TRUE ) return FALSE;

	ADC_SELECT();

	/* Read current CSR content */
	cs5523_read_csr( csr_data );

	ADC_DESELECT();

	subindex -= 15;

	/* Extract required CSR data */
	csr = &csr_data[subindex << 2];
	for( i=0; i<4; ++i ) par[i] = csr[i];

	*nbytes = 4;
      }
      break;

    case 19:
      {
	BYTE wordrate_id;
	wordrate_id = ((AdcConfig & CS23_CSR_WORDRATE_MASK) >>
		       CS23_CSR_WORDRATE_SHIFT);
	par[0] = HZ_WORDRATE[wordrate_id];
      }
      break;

    case 20:
      {
	BYTE   adc_range_id;
	float  v_range_f;
	UINT32 v_range_l;
	UINT32 *ptr;

	adc_range_id = ((AdcConfig & CS23_CSR_GAIN_MASK) >>
			CS23_CSR_GAIN_SHIFT);

	/* Beware: the conversion from float to UINT32 does NOT
	   work on V_FULLSCALE (which is stored flash) directly !
	   (a compiler feature?) */
	v_range_f = V_FULLSCALE[adc_range_id];
	v_range_l = (UINT32) v_range_f;

	/* Copy result to the output BYTE array */
	ptr       = (UINT32 *) par;
	*ptr      = v_range_l;

	// The following also works:
	//BYTE *ptr, i;
	//ptr = (BYTE *) &v_range_l;
	//for( i=0; i<4; ++i, ++ptr ) par[i] = *ptr;

	*nbytes = 4;
      }
      break;

    case 21:
      par[0] = AdcOptoDelay;
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

  /* Can't do anything if the ADC is in use... */
  if( (AdcConvInProgress & TRUE) == TRUE ) return FALSE;

#ifdef __VARS_IN_EEPROM__
  AdcChans     = eeprom_read( EE_ADCCHANS );
  AdcConfig    = eeprom_read( EE_ADCCONFIG );
  AdcConfReg_2 = eeprom_read( EE_ADCCONFREG2 );
  AdcOptoDelay = eeprom_read( EE_ADCOPTODELAY );
#endif

  switch( subindex )
    {
    case 1:
      if( nbytes > 1 || par[0] > ADC_MAX_INPUTS ) return FALSE;

      AdcChans = par[0];

      break;

    case 2:
      {
	if( nbytes > 1 || par[0] > CS23_WORDRATE_7 ) return FALSE;

	AdcConfig &= ~CS23_CSR_WORDRATE_MASK;
	AdcConfig |= (par[0] << CS23_CSR_WORDRATE_SHIFT);

	/* Prepare wordrate-dependent setting for ADC Configuration Register */
	if( par[0] == CS23_WORDRATE_61 ||
	    par[0] == CS23_WORDRATE_84 ||
	    par[0] == CS23_WORDRATE_101 )
	  AdcConfReg_2 = ADC_CNFREG_2_CHOP4096;
	else
	  AdcConfReg_2 = ADC_CNFREG_2_CHOP256;

	ADC_SELECT();

	/* Write the updated setting to the Config Register */
	adc_init_confreg();

	ADC_DESELECT();
      }
      break;

    case 3:
      if( nbytes > 1 || par[0] > CS23_GAIN_2V5 ) return FALSE;

      AdcConfig &= ~CS23_CSR_GAIN_MASK;
      AdcConfig |= (par[0] << CS23_CSR_GAIN_SHIFT);

      break;

    case 4:
      if( nbytes > 1 || par[0] > 1 ) return FALSE;

      if( par[0] == 1 )
	AdcConfig |= CS23_CSR_UNIPOLAR;
      else
	AdcConfig &= ~CS23_CSR_UNIPOLAR;

      break;

    case 5:
      {
	BYTE adc_conf_reg[3];

	if( nbytes > 1 || par[0] > 1 ) return FALSE;

	if( par[0] == 1 )
	  adc_conf_reg[1] = ADC_CNFREG_1 | CS23_CNF_POWER_SAVE;
	else
	  adc_conf_reg[1] = ADC_CNFREG_1 & (~CS23_CNF_POWER_SAVE);

	adc_conf_reg[0] = ADC_CNFREG_0;
	adc_conf_reg[2] = AdcConfReg_2;

	ADC_SELECT();

	/* Write the new setting to the Config Register */
	cs5523_write_reg( CS23_CMD_CONFIG_REG, 0, adc_conf_reg );

	ADC_DESELECT();
      }
      break;

    case 6:
      {
	if( !(nbytes == 4 || nbytes == 0) ) return FALSE;

	ADC_SELECT();

	/* Write the Configuration Register */
	cs5523_write_reg( CS23_CMD_CONFIG_REG, 0, par );

	ADC_DESELECT();
      }
      break;

    case 7:
    case 8:
    case 9:
    case 10:
    case 11:
    case 12:
    case 13:
    case 14:
      {
	BYTE reg, phys_chan_no;

	if( !(nbytes == 4 || nbytes == 0) ) return FALSE;

	if( subindex & 1 ) reg = CS23_CMD_OFFSET_REG;
	else reg = CS23_CMD_GAIN_REG;

	phys_chan_no = (subindex - 7) >> 1;

	ADC_SELECT();

	/* Write Register */
	cs5523_write_reg( reg, phys_chan_no, par );

	ADC_DESELECT();
      }
      break;

    case 15:
    case 16:
    case 17:
    case 18:
      {
	BYTE csr_data[4*2*2], *csr, i;

	if( !(nbytes == 4 || nbytes == 0) ) return FALSE;

	ADC_SELECT();

	/* Read current CSR content */
	cs5523_read_csr( csr_data );

	subindex -= 15;

	/* Adjust CSR content */
	csr = &csr_data[subindex << 2];
	for( i=0; i<4; ++i ) csr[i] = par[i];

	/* Write new CSR content */
	cs5523_write_csr( 4, csr_data );

	ADC_DESELECT();
      }
      break;

    case 21:
      if( nbytes > 1 || par[0] < 10 ) return FALSE;

      AdcOptoDelay = par[0];

      break;

    default:
      return FALSE;
    }

#ifdef __VARS_IN_EEPROM__
  /* Store values in EEPROM, if changed */
  if( eeprom_read( EE_ADCCHANS ) != AdcChans )
    eeprom_write( EE_ADCCHANS, AdcChans );
  if( eeprom_read( EE_ADCCONFIG ) != AdcConfig )
    eeprom_write( EE_ADCCONFIG, AdcConfig );
  if( eeprom_read( EE_ADCCONFREG2 ) != AdcConfReg_2 )
    eeprom_write( EE_ADCCONFREG2, AdcConfReg_2 );
  if( eeprom_read( EE_ADCOPTODELAY ) != AdcOptoDelay )
    eeprom_write( EE_ADCOPTODELAY, AdcOptoDelay );
#endif /* __VARS_IN_EEPROM__ */

  if( subindex == 3 || subindex == 4 )
    /* Recalibrate using the new setting of Input Range or uni/bipolar! */
    adc_reset_and_calibrate( TRUE );

  return TRUE;
}

/* ------------------------------------------------------------------------ */

BOOL adc_reset_and_calibrate( BYTE send_emergency )
{
  BOOL result;

  /* Initialise (hardware) error status */
  AdcError &= ~ADC_ERR_IN_HARDWARE;

  /* Perform calibration only if reset succeeded */
  if( (result = adc_reset( send_emergency )) == TRUE )
    result = adc_set_calib_regs( send_emergency );

  return result;
}

/* ------------------------------------------------------------------------ */

BOOL adc_reset( BOOL send_emergency )
{
  BOOL reset_result;
  BYTE err_id;

  /* Not allowed when ADC scan cycle in progress... */
  if( (AdcConvInProgress & TRUE) == TRUE ) return FALSE;

#ifdef __VARS_IN_EEPROM__
  AdcConfReg_2 = eeprom_read( EE_ADCCONFREG2 );
  AdcOptoDelay = eeprom_read( EE_ADCOPTODELAY );
#endif

  /* Select ADC */
  ADC_SELECT();

  /* Initialize the ADC's serial port interface */
  cs5523_serial_init();

  /* Reset the ADC */
  reset_result = cs5523_reset( &err_id );

  /* Initialize Channel Setup Registers */
  adc_init_csr();

  /* Initialize Configuration Register:
     Chop Frequency:  256 Hz (at freq <= 30 Hz), 4096 Hz (other freq)
     		      (recommended by Crystal, CS5523 datasheet pg 14)
     Depth Pointer : 001 (read/write 1 CSR at a time) */
  adc_init_confreg();

  /* Deselect ADC */
  ADC_DESELECT();

  if( reset_result == FALSE )
    {
      /* Reset failed */
      AdcError |= ADC_ERR_RESET;

      /* Generate emergency message ? */
      if( send_emergency )
	{
	  /* CANopen Error Code 0x5000: device hardware */
	  can_write_emergency( 0x00, 0x50, EMG_ADC_RESET, 0, err_id, 0,
			       ERRREG_MANUFACTURER );
	}

      return FALSE;
    }
  return TRUE;
}

/* ------------------------------------------------------------------------ */

static BOOL adc_set_calib_regs( BOOL send_emergency )
{
  BOOL   result;
  BYTE   adc_range_id;
  BYTE   phys_chan_no;
  BYTE   calibcnst_b[4], gainreg_b[4];
  UINT32 calibcnst_l, gainreg_l;
  float  calibcnst_f, gainreg_f;

  /* First do a 'pure' self-calibration:
     the calibration factors are applied to the resulting gains */
  result = adc_self_calibrate( send_emergency );

  /* Apply calibration parameters (when available !)
     to adjust the value of the Gain Registers:
     for each ADC physical channel (4x):
     1. Read the gain factor from EEPROM
     2. Read the ADC Gain Register
     3. Multiply the Gain Register value with the gain factor
     4. Write the resulting value to the Gain Register */

#ifdef __VARS_IN_EEPROM__
  AdcConfig    = eeprom_read( EE_ADCCONFIG );
  AdcOptoDelay = eeprom_read( EE_ADCOPTODELAY );
#endif
  adc_range_id = ((AdcConfig & CS23_CSR_GAIN_MASK) >> CS23_CSR_GAIN_SHIFT);

  for( phys_chan_no=0; phys_chan_no<4; ++phys_chan_no )
    {
      /* Read the gain factor from EEPROM and apply (only if available) */
      if( adc_get_calib_const( RANGE_ID[adc_range_id],
			       phys_chan_no, calibcnst_b, send_emergency ) )
	{
	  UINT32 *ptr;
	  BYTE   *bptr;

	  /* Read Gain Register (that's just 3 bytes) */
	  ADC_SELECT();
	  cs5523_read_reg( CS23_CMD_GAIN_REG, phys_chan_no, gainreg_b );
	  ADC_DESELECT();
	  gainreg_b[3] = 0x00;

	  /* Convert Gain Register value to float */
	  ptr       = (UINT32 *) gainreg_b;
	  gainreg_l = *ptr;
	  gainreg_f = (float) gainreg_l;

	  /* or (less efficient, but portable):
	  gainreg_l = (((UINT32) gainreg_b[0]) |
		       (((UINT32) gainreg_b[1]) << 8) |
		       (((UINT32) gainreg_b[2]) << 16));
	  gainreg_f = (float) gainreg_l;
	  */

	  /* Convert the gain factor value to float */
	  ptr         = (UINT32 *) calibcnst_b;
	  calibcnst_l = *ptr;
	  calibcnst_f = ((float) calibcnst_l) / 1000000.0;

	  /* or (less efficient, but portable):
	  calibcnst_l = (((UINT32) gain_b[0]) |
	                 (((UINT32) gain_b[1]) << 8) |
			 (((UINT32) gain_b[2]) << 16));
	  calibcnst_f = ((float) gain_l) / 1000000.0;
	  */

	  /* Make sure it has a sensible value */
	  if( calibcnst_f >= 4.0 || calibcnst_f <= 0.1 ) calibcnst_f = 1.0; 

	  /* Multiply Gain Register value with the gain factor */
	  gainreg_f = (gainreg_f * calibcnst_f) + 0.5;

	  /* Convert Gain Register float value back to an array of bytes */
	  gainreg_l = (UINT32) gainreg_f;
	  bptr      = (BYTE *) &gainreg_l;

	  /* Write the corrected Gain Register value */
	  ADC_SELECT();
	  cs5523_write_reg( CS23_CMD_GAIN_REG, phys_chan_no, bptr );
	  ADC_DESELECT();
	}
    }
  return result;
}

/* ------------------------------------------------------------------------ */

static BOOL adc_self_calibrate( BOOL send_emergency )
{
  BOOL result;

  /* Not allowed when ADC conversion in progress... */
  if( (AdcConvInProgress & TRUE) == TRUE ) return FALSE;

  /* =================================== */
  /* OFFSET calibration */

  ADC_SELECT();

  /* Do a Self Offset calibration... */
#ifdef __VARS_IN_EEPROM__
  AdcConfig    = eeprom_read( EE_ADCCONFIG );
  AdcOptoDelay = eeprom_read( EE_ADCOPTODELAY );
#endif
  result = cs5523_calibrate( CS23_CMD_OFFSET_SELF_CALIB, AdcConfig );

  ADC_DESELECT();

  if( result == FALSE )
    {
      /* Calibration error... */

      AdcError |= ADC_ERR_CALIBRATION;

      if( send_emergency )
	{
	  /* CANopen Error Code 0x5000: device hardware */
	  can_write_emergency( 0x00, 0x50, EMG_ADC_OFFSET_CALIB, 0, 0, 0,
			       ERRREG_MANUFACTURER );
	}

      adc_serial_init();

      return FALSE;
    }

  /* =================================== */
  /* GAIN calibration */

  ADC_SELECT();

  /* ...followed by a Self Gain calibration */
#ifdef __VARS_IN_EEPROM__
  AdcConfig    = eeprom_read( EE_ADCCONFIG );
  AdcOptoDelay = eeprom_read( EE_ADCOPTODELAY );
#endif
  result = cs5523_calibrate( CS23_CMD_GAIN_SELF_CALIB, AdcConfig );

  ADC_DESELECT();

  if( result == FALSE )
    {
      /* Calibration error... */

      AdcError |= ADC_ERR_CALIBRATION;

      if( send_emergency )
	{
	  /* CANopen Error Code 0x5000: device hardware */
	  can_write_emergency( 0x00, 0x50, EMG_ADC_GAIN_CALIB, 0, 0, 0,
			       ERRREG_MANUFACTURER );
	}

      adc_serial_init();

      return FALSE;
    }

  /* =================================== */

  return TRUE;
}

/* ------------------------------------------------------------------------ */

static void adc_init_confreg( void )
{
  /* Remember to select the ADC (if necessary)
     before calling this routine (and to deselect afterwards) */
  BYTE adc_conf_reg[3];

  adc_conf_reg[0] = ADC_CNFREG_0;
  adc_conf_reg[1] = ADC_CNFREG_1;
  adc_conf_reg[2] = AdcConfReg_2;

  /* Write to the Config Register */
  cs5523_write_reg( CS23_CMD_CONFIG_REG, 0, adc_conf_reg );
}

/* ------------------------------------------------------------------------ */

static void adc_init_csr( void )
{
  /* Remember to select the ADC (if necessary)
     before calling this routine (and to deselect afterwards) */

  /* Channel Setup Register data
     (4 registers, each for 2 Logical Channels,
      each consisting of 2 bytes) */
  BYTE csr_data[4*2*2];
  BYTE i;

  /* Settings for Logical Channels 1 to 4 in CSR #1 and #2
     (only LC #1 will be used) */
  for( i=0; i<4; ++i )
    {
      csr_data[2*i]   = 0; /* LC low byte */
      csr_data[2*i+1] = 0; /* LC high byte */
    }

  /* Settings for Logical Channels 5 to 8 in CSR #3 and #4: these Logical
     Channels are used only to set A1A0 pins to be latched !
     (to select one of the four 16-chan input banks;
     so do that at the maximum possible conversion rate) */
  for( i=0; i<4; ++i )
    {
      BYTE a1a0;

      a1a0 = i;

      /* LC low byte */
      //csr_data[2*(4+i)] = CS23_WORDRATE_101 << CS23_CSR_WORDRATE_SHIFT;
      /* ###Don't change the wordrate to do this, because the ADC displays
	 some erratic behaviour / noisy conversions when changing wordrates */
      csr_data[2*(4+i)]   = AdcConfig;

      /* LC high byte */
      csr_data[2*(4+i)+1] = a1a0 << CS23_CSR_A1A0_SHIFT;
    }

  /* Load initial settings in the CSRs: LC 5 to 8 are used
     to select the four 16-chan input banks (via A1A0 latch outputs) */
  cs5523_write_csr( 4, csr_data );
}

/* ------------------------------------------------------------------------ */

void adc_serial_init( void )
{
  ADC_SELECT();
  cs5523_serial_init();
  ADC_DESELECT();
}

/* ------------------------------------------------------------------------ */

void adc_pdo_scan_start( BOOL volts, BOOL force_readout )
{
  /* Start scanning only if scanning not already in progress, unless
     'force_readout' is true AND ADC-readout-on-change is enabled,
     then stop the current scan cycle and initiate a new scan cycle
     forcing read-out of the ADC channels */

#ifdef __VARS_IN_EEPROM__
  AdcChans = eeprom_read( EE_ADCCHANS );
#endif

  /* Refresh Data-Direction Register settings */
#ifdef __MOTHERBOARD1__
  /* Old Motherboard configuration */
  ADC_INIT_DDR_V1();
#endif
#ifndef __ALL_MOTHERBOARDS__
  /* Motherboard V3 configuration */
  ADC_INIT_DDR_V3();
#endif

#ifdef __VARS_IN_EEPROM__
  AdcReadoutOnChange = eeprom_read( EE_ADCREADOUTONCHANGE );
#endif /* __VARS_IN_EEPROM__ */

  if( AdcReadoutOnChange && force_readout )
    {
      /* Stop any ongoing scan cycle */
      adc_pdo_scan_stop();

      /* Force read-out of all channels at next scan cycle */
      AdcForcedReadout = TRUE;
    }

  if( (AdcScanInProgress & TRUE) == FALSE && AdcChans > 0 )
    {
#ifdef __VARS_IN_EEPROM__
      AdcCalibBeforeScan = eeprom_read( EE_ADCCALIBBEFORESCAN );
#endif /* __VARS_IN_EEPROM__ */

      if( AdcCalibBeforeScan )
	{
	  /* Perform a reset/calibration procedure before each scan... */
	  adc_reset_and_calibrate( TRUE );
	}
      else
	{
#ifdef __VARS_IN_EEPROM__
	  AdcConfReg_2 = eeprom_read( EE_ADCCONFREG2 );
	  AdcConfig    = eeprom_read( EE_ADCCONFIG );
	  AdcOptoDelay = eeprom_read( EE_ADCOPTODELAY );
#endif
	  ADC_SELECT();

	  /* Refresh the Configuration Register before each scan... */
	  adc_init_confreg();

	  /* Refresh the Channel Setup Registers before each scan... */
	  adc_init_csr();

	  ADC_DESELECT();
	}

      /* Send ADC readout in the form of ADC-counts or voltages ? */
      if( (volts & TRUE) == TRUE )
	{
	  /* Readout in volts, if possible */
	  if( adc_calibrated() )
	    {
	      AdcScanVolts = TRUE;
	    }
	  else
	    {
	      AdcScanVolts = FALSE; /* Do a scan nevertheless?.. */
	      //return;             /* ..or not ? */
	    }
	}
      else
	{
	  /* Readout in ADC counts */
	  AdcScanVolts = FALSE;
	}

      /* Start an ADC-channels-to-PDOs scan cycle */
      AdcChanNo = 0;
      AdcScanInProgress = adc_scan_next();
    }
}

/* ------------------------------------------------------------------------ */

void adc_pdo_scan_stop( void )
{
  /* Abort a channel scan cycle in-progress:
     wait for a possibly ongoing ADC conversion... */

  if( (AdcScanInProgress & TRUE) == TRUE )
    {
      if( (AdcConvInProgress & TRUE) == TRUE )
	{
#define __ADC_CLEAN_STOP__
#ifdef __ADC_CLEAN_STOP__
	  /* Wait for the conversion to finish and read out the ADC data */

	  ADC_SELECT();

	  /* Wait for SDO to go low flagging the conversion is done,
	     but use a timeout (ca. 600 ms, sufficient at 1.88 Hz wordrate) */
	  //if( cs5523_await_sdo_low( 60 ) )
	  if( cs5523_await_sdo_low( 120 ) ) // Make that 1200 ms...
	    {
	      /* Generate 8 SCLKs to clear the SDO flag */
	      cs5523_write_byte( 0 );

	      /* Read 24 bits (3 bytes) of conversion data */
	      cs5523_read_byte();
	      cs5523_read_byte();
	      cs5523_read_byte();
	    }

	  ADC_DESELECT();

#else /* !__ADC_CLEAN_STOP__ */

	  /* Is this sufficient ? --> NO!!
	     (ADC config appears to have changed afterwards) */
	  adc_serial_init();

#endif /* !__ADC_CLEAN_STOP__ */
	}
    }

  /* Initialize variables for scanning operations */
  AdcChanNo         = 0;
  AdcScanInProgress = FALSE;
  AdcConvInProgress = FALSE;
  AdcLatchBeingSet  = FALSE;
  AdcLatchSet       = FALSE;
}

/* ------------------------------------------------------------------------ */

void adc_pdo_scan( void )
{
  /* Handle an on-going ADC channel scan, or start up
     a channel scan cycle when ADC-readout-on-change is enabled */

  if( (AdcScanInProgress & TRUE) == TRUE )
    {
      /* Read out next ADC channel */
      AdcScanInProgress = adc_scan_next();
    }
  else
    {
#ifdef __VARS_IN_EEPROM__
      /* Refresh some global variables before every scan */
      AdcReadoutOnChange = eeprom_read( EE_ADCREADOUTONCHANGE );
#endif /* __VARS_IN_EEPROM__ */

      /* Do we have to scan continuously ? (if yes, then start
	 a new scan immediately when the previous one finished) */
      if( AdcReadoutOnChange )
	{
#ifdef __VARS_IN_EEPROM__
	  /* Refresh some global variables before every scan */
	  AdcDeltaScanEnabled  = eeprom_read( EE_ADCDELTASCANENABLED );
	  AdcWindowScanEnabled = eeprom_read( EE_ADCWINDOWSCANENABLED );
	  AdcWindowCounterInit = eeprom_read( EE_ADCWINDOWCOUNTERINIT );
#endif /* __VARS_IN_EEPROM__ */

	  if( AdcDeltaScanEnabled || AdcWindowScanEnabled )
	    {
	      /* Yes, but this call to 'adc_pdo_scan_start()' is not a forced
		 readout, so the parameter is set to FALSE, unless... */
	      //adc_pdo_scan_start( TRUE, FALSE );

	      /* ...this is the first scan cycle after the node
		 has been set to 'Operational' state, in which case
		 the reference ADC counts ('last-sent') need to be initialized
		 ('AdcInitRefCount' in that case has been set elsewhere to
		  'TRUE') and read-out of the ADC-channels is forced */
	      if( (AdcInitRefCount & TRUE) == TRUE )
		adc_pdo_scan_start( TRUE, TRUE );
	      else
		adc_pdo_scan_start( TRUE, FALSE );
	    }
	}
    }
}

/* ------------------------------------------------------------------------ */

static BOOL adc_scan_next( void )
{
  BYTE pdo_data[8];

  if( (AdcConvInProgress & TRUE) == TRUE )
    {
      ADC_SELECT();

      /* ! Make sure no delay is necessary here
	   (due to optocouplers in the interface) !
	   ###Not necessary because we don't deselect the ADC during a scan */
      //timer2_delay_mus( AdcOptoDelay );

      /* Conversion in progress: check if done */
      if( ADC_SDO_LOW() )
	{
	  /* SDO went low --> conversion ready ! */

	  if( (AdcLatchBeingSet & TRUE) == TRUE )
	    {
	      /* This conversion only served to set the A1A0 latches...,
		 but we *have* to read out the ADC data */

	      /* Now latch this data */
	      ADC_CLEAR_MUX_LATCH();

	      /* Need to insert a delay... */
	      //timer2_delay_mus( CS23_ELMB_SIGNAL_RISETIME );
	      timer2_delay_mus( AdcOptoDelay );

	      ADC_SET_MUX_LATCH();

	      /* Generate 8 SCLKs to clear the SDO flag */
	      cs5523_write_byte( 0 );

	      /* Read 24 bits (3 bytes) of (dummy) ADC conversion data */
	      cs5523_read_byte();
	      cs5523_read_byte();
	      cs5523_read_byte();

	      ADC_DESELECT();

	      AdcLatchBeingSet  = FALSE;
	      AdcLatchSet       = TRUE;

	      AdcConvInProgress = FALSE;
	    }
	  else
	    {
	      /* This conversion results in real data
		 that we're interested in... */
	      BYTE   adc_stat;
	      BOOL   send_pdo;
	      UINT16 adc_count;

	      ADC_DESELECT();

	      /* Should only generate a next PDO when the previous one
		 has been sent */
	      if( can_transmitting(C91_TPDO2) )
		{
		  /* A TPDO2 transmission is in progress, so wait... */
		  return TRUE;
		}

	      /* Set to FALSE only if message can be sent... */
	      AdcConvInProgress = FALSE;

	      ADC_SELECT();

	      /* Generate 8 SCLKs to clear the SDO flag */
	      cs5523_write_byte( 0 );

	      /* Read 24 bits (3 bytes) of ADC conversion data */
	      pdo_data[3] = cs5523_read_byte();
	      pdo_data[2] = cs5523_read_byte();
	      adc_stat    = cs5523_read_byte();

	      ADC_DESELECT();

#ifdef __VARS_IN_EEPROM__
	      AdcConfig = eeprom_read( EE_ADCCONFIG );
#endif
	      /* Replace the status byte with a byte containing
		 the ADC-configuration (wordrate, gain, uni/bipolar)
		 and one bit with an 'or' of the OF and OD bits
		 (here bit 7, the highest bit) */
	      if( adc_stat & CS23_DATA_ERROR )
		pdo_data[1] = AdcConfig | ADC_ERR_CONVERSION;
	      else
		pdo_data[1] = AdcConfig;

	      /* Put the ADC channel number in the message */
	      pdo_data[0] = AdcChanNo;

	      /* The ADC-count as a 16-bit (unsigned) integer */
	      adc_count = (((UINT16) pdo_data[2]) |
			   (((UINT16) pdo_data[3]) << 8));

	      if( AdcReadoutOnChange )
		{
		  /* Do the necessary checks to determine whether
		     the data is to be sent */
		  send_pdo = FALSE;

		  /* Delta-change check */
		  if( AdcDeltaScanEnabled )
		    send_pdo = adc_delta_check( adc_count );

		  /* Upper/lower limit check */
		  if( AdcWindowScanEnabled && send_pdo == FALSE )
		    send_pdo = adc_window_check( adc_count );
		}
	      else
		{
		  send_pdo = TRUE;
		}

	      /* Send data when required */
	      if( send_pdo )
		{
		  if( AdcScanVolts )
		    {
		      BYTE microvolts[4];
		      BYTE i, *ptr;

		      /* Convert the ADC-count to microVolts */
		      adc_convert_to_volts( adc_count, microvolts );

		      /* Copy the resulting value into the output array */
		      ptr = &pdo_data[2];
		      for( i=0; i<4; ++i, ++ptr ) *ptr = microvolts[i];

		      /* Send as a Transmit-PDO3 CAN-message */
		      can_write( C91_TPDO3, C91_TPDO3_LEN, pdo_data );
		    }
		  else
		    {
		      /* Send as a Transmit-PDO2 CAN-message */
		      can_write( C91_TPDO2, C91_TPDO2_LEN, pdo_data );
		    }
		}

	      /* Next time, next channel
		 (SEE-protected version of '++AdcChanNo;') */
	      AdcChanNo = (AdcChanNo & (ADC_MAX_INPUTS-1)) + 1;

#ifdef __VARS_IN_EEPROM__
	      AdcChans = eeprom_read( EE_ADCCHANS );
#endif
	      /* Are we done with the current scan cycle ? */
	      if( AdcChanNo == AdcChans )
		{
		  /* Yes, we're done... */
		  AdcForcedReadout = FALSE;
		  AdcInitRefCount  = FALSE;
		  return FALSE;
		}
	    }
	}
      else
	{
	  /* Conversion in progress...
	     check for possible timeout... */

	  /* ###Do not deselect/select during a conversion: may cause noise */
	  //ADC_DESELECT();

	  if( timer0_timeout(ADC_ELMB) )
	    {
	      /* Conversion timed out ! */
	      AdcError |= ADC_ERR_TIMEOUT;

	      /* CANopen Error Code 0x5000: device hardware */
	      can_write_emergency( 0x00, 0x50, EMG_ADC_CONVERSION, AdcChanNo,
				   0, 0, ERRREG_MANUFACTURER );

	      AdcConvInProgress = FALSE;

	      //adc_serial_init();
	      adc_reset_and_calibrate( TRUE );

	      /* Stop this scan... */
	      return FALSE;
	    }

	  /* Wait some more... */
	  return TRUE;
	}
    }

  ADC_SELECT();

  if( (AdcChanNo & 0x0F) == 0 && (AdcLatchSet & TRUE) == FALSE )
    {
      BYTE bank_no, log_chan;

      bank_no = AdcChanNo/16;

      /* Change of 16-chan inputs bank: select bank,
	 by initiating a (dummy) conversion;
	 LC 5 to 8 contain the proper setting of A1-A0
	 (set during initialization) to accomplish this */

      /* The Logical Channel to use (4, 5, 6 or 7) */
      log_chan = ((4 + bank_no) << CS23_CMD_LOGCHAN_SELECT_SHIFT);

      /* Initiate the conversion */
      cs5523_start_conversion_cmd( CS23_CMD_NORMAL_CONVERSION | log_chan );

      /* When conversion done, don't send data... */
      AdcLatchBeingSet = TRUE;
    }
  else
    {
      BYTE a1a0, cs1cs0;
      BYTE csr_data[1*2*2];

#ifdef __VARS_IN_EEPROM__
      AdcConfig    = eeprom_read( EE_ADCCONFIG );
      AdcOptoDelay = eeprom_read( EE_ADCOPTODELAY );
#endif
      /* Set A1-A0 and physical channel in LC 1 (+2)
	 while maintaining proper wordrate and gain */
      a1a0        = AdcChanNo & 3;
      cs1cs0      = (AdcChanNo>>2) & 3;
      csr_data[0] = (AdcConfig | ((cs1cs0 << CS23_CSR_PHYSCHAN_SEL_LO_SHIFT) &
				  CS23_CSR_PHYSCHAN_SEL_LO_MASK));
      csr_data[1] = ((a1a0 << CS23_CSR_A1A0_SHIFT) |
		     ((cs1cs0 >> CS23_CSR_PHYSCHAN_SEL_HI_SHIFT) &
		      CS23_CSR_PHYSCHAN_SEL_HI_MASK));
      csr_data[2] = 0x00;
      csr_data[3] = 0x00;
      cs5523_write_csr( 1, csr_data );

      /* Start a conversion of LC 1 */
      cs5523_start_conversion_cmd( CS23_CMD_NORMAL_CONVERSION | 0 );

      /* At next 16-chan inputs bank, first do a conversion to set latch */
      AdcLatchSet = FALSE;
    }

  /* ###Do not deselect/select during a conversion: may cause noise */
  //ADC_DESELECT();

  AdcConvInProgress = TRUE;

  /* Set a timeout on the conversion of about 800 ms... */
  //timer0_set_timeout_10ms( ADC_ELMB, 80 );
  timer0_set_timeout_10ms( ADC_ELMB, 120 ); // Make that 1200 ms... */

  return TRUE;
}

/* ------------------------------------------------------------------------ */

static BOOL adc_delta_check( UINT16 adc_count )
{
  UINT16 adc_delta, adc_diff;

  /* Initialize reference value if necessary ! */
  if( (AdcInitRefCount & TRUE) == TRUE ) AdcRefCount[AdcChanNo] = adc_count;

  if( (AdcForcedReadout & TRUE) == TRUE )
    {
      /* Send the data */
      return TRUE;
    }

  /* Get the delta-change value for this channel from
     the working copy in EEPROM */
  adc_delta = adc_get_delta_cnt( AdcChanNo );

  /* Don't send the data when the delta is zero */
  if( adc_delta == (UINT16) 0 ) return FALSE;

  if( AdcConfig & CS23_CSR_UNIPOLAR )
    {
      /* Unipolar measurement: use unsigned numbers ! */

      if( adc_count > AdcRefCount[AdcChanNo] )
	adc_diff = adc_count - AdcRefCount[AdcChanNo];
      else
	adc_diff = AdcRefCount[AdcChanNo] - adc_count;
    }
  else
    {
      /* Bipolar measurement: use signed numbers ! */
      INT16 *padc_count, *padc_last_sent;
      padc_count     = (INT16 *) &adc_count;
      padc_last_sent = (INT16 *) &AdcRefCount[AdcChanNo];

      if( *padc_count > *padc_last_sent )
	adc_diff = (UINT16) (*padc_count - *padc_last_sent);
      else
	adc_diff = (UINT16) (*padc_last_sent - *padc_count);
    }

  if( adc_diff >= adc_delta )
    {
      BYTE cntr;
      cntr = AdcDeltaCounter[AdcChanNo];
      ++cntr;

      if( cntr >= AdcWindowCounterInit )
	{
	  AdcDeltaCounter[AdcChanNo] = 0;

	  /* Store this value: new reference ! */
	  AdcRefCount[AdcChanNo] = adc_count;

	  /* Send the data */
	  return TRUE;
	}
      else
	{
	  AdcDeltaCounter[AdcChanNo] = cntr;

	  /* Don't send the data (yet) */
	  return FALSE;
	}
    }
  else
    {
      AdcDeltaCounter[AdcChanNo] = 0;

      /* Don't send the data */
      return FALSE;
    }
}

/* ------------------------------------------------------------------------ */

static BOOL adc_window_check( UINT16 adc_count )
{
  UINT16 adc_upper, adc_lower;
  BOOL   outside_window;
  CHAR   cntr;

  if( (AdcForcedReadout & TRUE) == TRUE )
    {
      /* Send the data */
      return TRUE;
    }

  /* Get the upper-limit value for this channel from
     the working copy in EEPROM */
  adc_upper = adc_get_upperlimit_cnt( AdcChanNo );

  /* Get the lower-limit value for this channel from
     the working copy in EEPROM */
  adc_lower = adc_get_lowerlimit_cnt( AdcChanNo );

  if( AdcConfig & CS23_CSR_UNIPOLAR )
    {
      /* Unipolar measurement: use unsigned numbers ! */

      if( adc_lower >= adc_upper ) return FALSE;

      if( adc_count > adc_upper || adc_count < adc_lower )
	outside_window = TRUE;
      else
	outside_window = FALSE;
    }
  else
    {
      /* Bipolar measurement: use signed numbers ! */
      INT16 *padc_count, *padc_upper, *padc_lower;
      padc_count = (INT16 *) &adc_count;
      padc_upper = (INT16 *) &adc_upper;
      padc_lower = (INT16 *) &adc_lower;

      if( *padc_lower >= *padc_upper ) return FALSE;

      if( *padc_count > *padc_upper || *padc_count < *padc_lower )
	outside_window = TRUE;
      else
	outside_window = FALSE;
    }

  cntr = AdcWindowCounter[AdcChanNo];

  /* Some checks to counter SEE
     (don't use counter anymore for inside-window cases) */
  //if( cntr > AdcWindowCounterInit )        cntr = AdcWindowCounterInit;
  if( cntr > 0 )                           cntr = 0;
  if( cntr < -(AdcWindowCounterInit + 1) ) cntr = -(AdcWindowCounterInit + 1);

  /* Negative counter values mean 'currently being outside window',
     positive counter values (and zero) mean 'currently being inside window'
     (which is also the initial value) */
  if( outside_window )
    {
      /*if( (AdcForcedReadout & TRUE) == TRUE )
	{
	  AdcWindowCounter[AdcChanNo] = -1;
	  return TRUE;
	}*/

      /* Already reported ? */
      if( cntr == -1 ) return FALSE;

      /* First time outside window from being inside window ?
	 --> initialise counter for this channel */
      if( cntr >= 0 ) cntr = -(AdcWindowCounterInit + 1);

      ++cntr;
      AdcWindowCounter[AdcChanNo] = cntr;

      /* Value -1 means: outside window and reported */
      if( cntr == -1 )
	{
	  /* Send the data */
	  return TRUE;
	}
    }
  else
    {
      /*if( (AdcForcedReadout & TRUE) == TRUE )
	{
	  AdcWindowCounter[AdcChanNo] = 0;
	  return TRUE;
	}*/

      /* Inside window: already reported ? */
      if( cntr == 0 ) return FALSE;

      /* We are 'inside window' */
      AdcWindowCounter[AdcChanNo] = 0;

      /* First time inside window from being outside window ?
	 (no counter used in this case!)
	 --> report it *only* if 'outside-window' has been reported,
	 --> cntr == -1 */
      if( cntr == -1 )
	{
	  /* Send the data */
	  return TRUE;
	}
    }

  return FALSE;
}

/* ------------------------------------------------------------------------ */

#define ADC_STORE_SIZE 8

/* ------------------------------------------------------------------------ */

BOOL adc_store_config( void )
{
  BYTE block[ADC_STORE_SIZE];
  BOOL result = TRUE;

  /* Store ADC delta-change parameters */
  if( adc_store_deltas() == FALSE ) result = FALSE;

  /* Store ADC upper-limit parameters */
  if( adc_store_upperlimits() == FALSE ) result = FALSE;

  /* Store ADC lower-limit parameters */
  if( adc_store_lowerlimits() == FALSE ) result = FALSE;

#ifdef __VARS_IN_EEPROM__
  AdcChans             = eeprom_read( EE_ADCCHANS );
  AdcConfig            = eeprom_read( EE_ADCCONFIG );
  AdcCalibBeforeScan   = eeprom_read( EE_ADCCALIBBEFORESCAN );
  AdcReadoutOnChange   = eeprom_read( EE_ADCREADOUTONCHANGE );
  AdcDeltaScanEnabled  = eeprom_read( EE_ADCDELTASCANENABLED );
  AdcWindowScanEnabled = eeprom_read( EE_ADCWINDOWSCANENABLED );
  AdcWindowCounterInit = eeprom_read( EE_ADCWINDOWCOUNTERINIT );
  AdcOptoDelay         = eeprom_read( EE_ADCOPTODELAY );
#endif /* __VARS_IN_EEPROM__ */

  block[0] = AdcChans;
  block[1] = AdcConfig;
  block[2] = AdcCalibBeforeScan;
  block[3] = AdcReadoutOnChange;
  block[4] = AdcDeltaScanEnabled;
  block[5] = AdcWindowScanEnabled;
  block[6] = AdcWindowCounterInit;
  block[7] = AdcOptoDelay;

  if( store_write_block( STORE_ADC, ADC_STORE_SIZE, block ) == FALSE )
    result = FALSE;

  return result;
}

/* ------------------------------------------------------------------------ */

static BOOL adc_load_config( void )
{
  BYTE block[ADC_STORE_SIZE];
  BOOL result = TRUE;

  /* Read the configuration from EEPROM, if any
     (errors in reading this datablock are caught and
      reported by functions in store.c...) */
  if( store_read_block( STORE_ADC, ADC_STORE_SIZE, block ) )
    {
      AdcChans             = block[0];
      AdcConfig            = block[1];
      AdcCalibBeforeScan   = block[2];
      AdcReadoutOnChange   = block[3];
      AdcDeltaScanEnabled  = block[4];
      AdcWindowScanEnabled = block[5];
      AdcWindowCounterInit = block[6];
      AdcOptoDelay         = block[7];
    }
  else
    {
      /* No valid parameters in EEPROM: use defaults */
      AdcChans             = ADC_MAX_INPUTS;
      AdcConfig            = ADC_DFLT_CONV_PARS;
      AdcCalibBeforeScan   = FALSE;
      AdcReadoutOnChange   = FALSE;
      AdcDeltaScanEnabled  = TRUE;
      AdcWindowScanEnabled = FALSE;
      AdcWindowCounterInit = 2;
      AdcOptoDelay         = CS23_ELMB_SIGNAL_RISETIME;
    }

  /* Initialize ADC delta-change parameters from EEPROM */
  if( adc_load_deltas() == FALSE ) result = FALSE;

  /* Initialize ADC upper-limit parameters from EEPROM */
  if( adc_load_upperlimits() == FALSE ) result = FALSE;

  /* Initialize ADC lower-limit parameters from EEPROM */
  if( adc_load_lowerlimits() == FALSE ) result = FALSE;

#ifdef __VARS_IN_EEPROM__
  /* Create working copies of configuration globals in EEPROM */
  if( eeprom_read( EE_ADCCHANS ) != AdcChans )
    eeprom_write( EE_ADCCHANS, AdcChans );
  if( eeprom_read( EE_ADCCONFIG ) != AdcConfig )
    eeprom_write( EE_ADCCONFIG, AdcConfig );
  if( eeprom_read( EE_ADCCALIBBEFORESCAN ) != AdcCalibBeforeScan )
    eeprom_write( EE_ADCCALIBBEFORESCAN, AdcCalibBeforeScan );
  if( eeprom_read( EE_ADCREADOUTONCHANGE ) != AdcReadoutOnChange )
    eeprom_write( EE_ADCREADOUTONCHANGE, AdcReadoutOnChange );
  if( eeprom_read( EE_ADCDELTASCANENABLED ) != AdcDeltaScanEnabled )
    eeprom_write( EE_ADCDELTASCANENABLED, AdcDeltaScanEnabled );
  if( eeprom_read( EE_ADCWINDOWSCANENABLED ) != AdcWindowScanEnabled )
    eeprom_write( EE_ADCWINDOWSCANENABLED, AdcWindowScanEnabled );
  if( eeprom_read( EE_ADCWINDOWCOUNTERINIT ) != AdcWindowCounterInit )
    eeprom_write( EE_ADCWINDOWCOUNTERINIT, AdcWindowCounterInit );
  if( eeprom_read( EE_ADCOPTODELAY ) != AdcOptoDelay )
    eeprom_write( EE_ADCOPTODELAY, AdcOptoDelay );
#endif /* __VARS_IN_EEPROM__ */

  return result;
}

/* ------------------------------------------------------------------------ */

#ifdef __ALL_MOTHERBOARDS__

/* ------------------------------------------------------------------------ */

void adc_set_sclk_v1( void )
{
  ADC_SET_SCLK_V1();
}

/* ------------------------------------------------------------------------ */

void adc_clear_sclk_v1( void )
{
  ADC_CLEAR_SCLK_V1();
}

/* ------------------------------------------------------------------------ */

void adc_set_sdi_v1( void )
{
  ADC_SET_SDI_V1();
}

/* ------------------------------------------------------------------------ */

void adc_clear_sdi_v1( void )
{
  ADC_CLEAR_SDI_V1();
}

/* ------------------------------------------------------------------------ */

BOOL adc_sdo_high_v1( void )
{
  return( ADC_SDO_HIGH_V1() );
}

/* ------------------------------------------------------------------------ */

BOOL adc_sdo_low_v1( void )
{
  return( ADC_SDO_LOW_V1() );
}

/* ------------------------------------------------------------------------ */

void adc_set_mux_latch_v1( void )
{
  ADC_SET_MUX_LATCH_V1();
}

/* ------------------------------------------------------------------------ */

void adc_clear_mux_latch_v1( void )
{
  ADC_CLEAR_MUX_LATCH_V1();
}

/* ------------------------------------------------------------------------ */

void adc_select_v1( void )
{
  ADC_SELECT_V1();
}

/* ------------------------------------------------------------------------ */

void adc_deselect_v1( void )
{
  ADC_DESELECT_V1();
}

/* ------------------------------------------------------------------------ */

void adc_set_sclk_v3( void )
{
  ADC_SET_SCLK_V3();
}

/* ------------------------------------------------------------------------ */

void adc_clear_sclk_v3( void )
{
  ADC_CLEAR_SCLK_V3();
}

/* ------------------------------------------------------------------------ */

void adc_set_sdi_v3( void )
{
  ADC_SET_SDI_V3();
}

/* ------------------------------------------------------------------------ */

void adc_clear_sdi_v3( void )
{
  ADC_CLEAR_SDI_V3();
}

/* ------------------------------------------------------------------------ */

BOOL adc_sdo_high_v3( void )
{
  return( ADC_SDO_HIGH_V3() );
}

/* ------------------------------------------------------------------------ */

BOOL adc_sdo_low_v3( void )
{
  return( ADC_SDO_LOW_V3() );
}

/* ------------------------------------------------------------------------ */

void adc_set_mux_latch_v3( void )
{
  ADC_SET_MUX_LATCH_V3();
}

/* ------------------------------------------------------------------------ */

void adc_clear_mux_latch_v3( void )
{
  ADC_CLEAR_MUX_LATCH_V3();
}

/* ------------------------------------------------------------------------ */

void adc_select_v3( void )
{
  ADC_SELECT_V3();
}

/* ------------------------------------------------------------------------ */

void adc_deselect_v3( void )
{
  ADC_DESELECT_V3();
}

/* ------------------------------------------------------------------------ */

static void motherboard_select( BYTE version )
{
  /* Initialize the function pointers according to the selected Motherboard */

  switch( version )
    {
    case 1:
      ADC_SET_SCLK        = adc_set_sclk_v1;
      ADC_CLEAR_SCLK      = adc_clear_sclk_v1;
      ADC_SET_SDI         = adc_set_sdi_v1;
      ADC_CLEAR_SDI       = adc_clear_sdi_v1;
      ADC_SDO_HIGH        = adc_sdo_high_v1;
      ADC_SDO_LOW         = adc_sdo_low_v1;
      ADC_SET_MUX_LATCH   = adc_set_mux_latch_v1;
      ADC_CLEAR_MUX_LATCH = adc_clear_mux_latch_v1;
      ADC_SELECT          = adc_select_v1;
      ADC_DESELECT        = adc_deselect_v1;
      break;

    default:
      ADC_SET_SCLK        = adc_set_sclk_v3;
      ADC_CLEAR_SCLK      = adc_clear_sclk_v3;
      ADC_SET_SDI         = adc_set_sdi_v3;
      ADC_CLEAR_SDI       = adc_clear_sdi_v3;
      ADC_SDO_HIGH        = adc_sdo_high_v3;
      ADC_SDO_LOW         = adc_sdo_low_v3;
      ADC_SET_MUX_LATCH   = adc_set_mux_latch_v3;
      ADC_CLEAR_MUX_LATCH = adc_clear_mux_latch_v3;
      ADC_SELECT          = adc_select_v3;
      ADC_DESELECT        = adc_deselect_v3;
      break;
    }
}

/* ------------------------------------------------------------------------ */

#endif /* __ALL_MOTHERBOARDS__ */

/* ------------------------------------------------------------------------ */

BOOL adc_set_calib_before_scan( BOOL calib )
{
  if( calib > 1 ) return FALSE;
  if( calib )
    AdcCalibBeforeScan = TRUE;
  else
    AdcCalibBeforeScan = FALSE;

#ifdef __VARS_IN_EEPROM__
  if( eeprom_read( EE_ADCCALIBBEFORESCAN ) != AdcCalibBeforeScan )
    eeprom_write( EE_ADCCALIBBEFORESCAN, AdcCalibBeforeScan );
#endif /* __VARS_IN_EEPROM__ */
  return TRUE;
}

/* ------------------------------------------------------------------------ */

BOOL adc_get_calib_before_scan( void )
{
#ifdef __VARS_IN_EEPROM__
  AdcCalibBeforeScan = eeprom_read( EE_ADCCALIBBEFORESCAN );
#endif /* __VARS_IN_EEPROM__ */

  return AdcCalibBeforeScan;
}

/* ------------------------------------------------------------------------ */

BOOL adc_set_readout_on_change( BOOL readout_on_change )
{
  if( readout_on_change > 1 ) return FALSE;
  if( readout_on_change )
    {
      /* If set from FALSE to TRUE, initialize delta reference ADC-counts
	 and readout-on-change counters */
      if( AdcReadoutOnChange == FALSE )
	adc_init_delta_references();

      AdcReadoutOnChange = TRUE;
    }
  else
    AdcReadoutOnChange = FALSE;

#ifdef __VARS_IN_EEPROM__
  if( eeprom_read( EE_ADCREADOUTONCHANGE ) != AdcReadoutOnChange )
    eeprom_write( EE_ADCREADOUTONCHANGE, AdcReadoutOnChange );
#endif /* __VARS_IN_EEPROM__ */
  return TRUE;
}

/* ------------------------------------------------------------------------ */

BOOL adc_get_readout_on_change( void )
{
#ifdef __VARS_IN_EEPROM__
  AdcReadoutOnChange = eeprom_read( EE_ADCREADOUTONCHANGE );
#endif /* __VARS_IN_EEPROM__ */

  return AdcReadoutOnChange;
}

/* ------------------------------------------------------------------------ */

BOOL adc_set_delta_scan_ena( BOOL delta_scan_enable )
{
  if( delta_scan_enable > 1 ) return FALSE;
  if( delta_scan_enable )
    {
      /* Note: if set from FALSE to TRUE in the middle of readout-on-change
	 scanning the AdcRefCount[] is not properly initialised ! */

      if( AdcDeltaScanEnabled == FALSE )
	{
	  /* Init delta counters if delta-scan is set from FALSE to TRUE */
	  BYTE chan;
	  for( chan=0; chan<ADC_MAX_INPUTS; ++chan )
	    AdcDeltaCounter[chan] = 0;

	  /* Force setting of the analog input references
	     for the delta-change check */
	  AdcInitRefCount = TRUE;
	}
      AdcDeltaScanEnabled = TRUE;
    }
  else
    AdcDeltaScanEnabled = FALSE;

#ifdef __VARS_IN_EEPROM__
  if( eeprom_read( EE_ADCDELTASCANENABLED ) != AdcDeltaScanEnabled )
    eeprom_write( EE_ADCDELTASCANENABLED, AdcDeltaScanEnabled );
#endif /* __VARS_IN_EEPROM__ */
  return TRUE;
}

/* ------------------------------------------------------------------------ */

BOOL adc_get_delta_scan_ena( void )
{
#ifdef __VARS_IN_EEPROM__
  AdcDeltaScanEnabled = eeprom_read( EE_ADCDELTASCANENABLED );
#endif /* __VARS_IN_EEPROM__ */

  return AdcDeltaScanEnabled;
}

/* ------------------------------------------------------------------------ */

BOOL adc_set_window_scan_ena( BOOL window_scan_enable )
{
  if( window_scan_enable > 1 ) return FALSE;
  if( window_scan_enable )
    {
      /* Init window counters if window-scan is set from FALSE to TRUE */
      if( AdcWindowScanEnabled == FALSE )
	{
	  BYTE chan;
	  for( chan=0; chan<ADC_MAX_INPUTS; ++chan )
	    AdcWindowCounter[chan] = 0;
	}
      AdcWindowScanEnabled = TRUE;
    }
  else
    AdcWindowScanEnabled = FALSE;

#ifdef __VARS_IN_EEPROM__
  if( eeprom_read( EE_ADCWINDOWSCANENABLED ) != AdcWindowScanEnabled )
    eeprom_write( EE_ADCWINDOWSCANENABLED, AdcWindowScanEnabled );
#endif /* __VARS_IN_EEPROM__ */
  return TRUE;
}

/* ------------------------------------------------------------------------ */

BOOL adc_get_window_scan_ena( void )
{
#ifdef __VARS_IN_EEPROM__
  AdcWindowScanEnabled = eeprom_read( EE_ADCWINDOWSCANENABLED );
#endif /* __VARS_IN_EEPROM__ */

  return AdcWindowScanEnabled;
}

/* ------------------------------------------------------------------------ */

BOOL adc_set_window_scan_cntr( BYTE window_scan_counter )
{
  if( window_scan_counter == 0 || window_scan_counter == 255 ) return FALSE;

  AdcWindowCounterInit = window_scan_counter;

#ifdef __VARS_IN_EEPROM__
  if( eeprom_read( EE_ADCWINDOWCOUNTERINIT ) != AdcWindowCounterInit )
    eeprom_write( EE_ADCWINDOWCOUNTERINIT, AdcWindowCounterInit );
#endif /* __VARS_IN_EEPROM__ */
  return TRUE;
}

/* ------------------------------------------------------------------------ */

BYTE adc_get_window_scan_cntr( void )
{
#ifdef __VARS_IN_EEPROM__
  AdcWindowCounterInit = eeprom_read( EE_ADCWINDOWCOUNTERINIT );
#endif /* __VARS_IN_EEPROM__ */

  return AdcWindowCounterInit;
}

/* ------------------------------------------------------------------------ */

static UINT16 adc_get_delta_cnt( BYTE chan_no )
{
  /* Read one ADC delta-change parameter (3 bytes, unsigned)
     from the working copy in EEPROM and convert it to a 16-bit ADC-count
     depending on the current ADC configuration; this function is called
     by the delta-change scanning function 'adc_delta_check()' */
  BOOL   unipolar;
  BYTE   adc_range_id;
  float  fullscale;
  UINT16 ee_addr;
  UINT16 delta_ui;
  UINT32 delta_ul;
  float  delta_f;
  BYTE   *ptr;

  /* Read the delta-change value from EEPROM into a longword */
  ee_addr = EE_ADC_DELTA + chan_no*STORE_ADC_LIMIT_PARSIZE;
  ptr  = (BYTE *) &delta_ul;
  *ptr = eepromw_read( ee_addr ); ++ptr; ++ee_addr;
  *ptr = eepromw_read( ee_addr ); ++ptr; ++ee_addr;
  *ptr = eepromw_read( ee_addr ); ++ptr;
  *ptr = 0x00; /* This is an unsigned number */

  /* or (less efficient, but portable):
  delta_ul = (((UINT32) eepromw_read(ee_addr)) |
	      (((UINT32) eepromw_read(ee_addr+1)) << 8) |
	      (((UINT32) eepromw_read(ee_addr+2)) << 16));
  */

  /* Get the ADC configuration */
  unipolar     = (AdcConfig & CS23_CSR_UNIPOLAR);
  adc_range_id = ((AdcConfig & CS23_CSR_GAIN_MASK) >> CS23_CSR_GAIN_SHIFT);
  fullscale    = V_FULLSCALE[adc_range_id];

  /* Convert it from microVolts to a 16-bit ADC-count */
  delta_f = ((float) delta_ul) / fullscale;
  if( unipolar )
    {
      delta_f *= 65535.0; /* Map to a 16-bit unsigned ADC-count */

      /* Clip to maximum */
      if( delta_f > 65535.0 )
	delta_ui = (UINT16) 0xFFFF;
      else
	delta_ui = (UINT16) delta_f;
    }
  else
    {
      delta_f *= 32767.0; /* Map to a 16-bit signed ADC-count */

      /* Clip to maximum */
      if( delta_f > 32767.0 )
	delta_ui = (UINT16) 0x7FFF;
      else
	delta_ui = (UINT16) delta_f;
    }

  /* If the stored delta is unequal to zero,
     make sure that the delta-in-ADC-counts is at least 1
     (a zero value is as if the channel is disabled for delta-change) */
  if( delta_ui == 0 && delta_ul > 0L ) delta_ui = 1;

  return delta_ui;
}

/* ------------------------------------------------------------------------ */

static UINT16 adc_get_upperlimit_cnt( BYTE chan_no )
{
  /* Read one ADC upper-limit parameter (3 bytes, signed)
     from the working copy in EEPROM and convert it to a 16-bit ADC-count
     depending on the current ADC configuration; this function is called
     by the upper/lower-limit scanning function 'adc_window_check()' */
  return( adc_get_limit_cnt( chan_no, EE_ADC_UPPER ) );
}

/* ------------------------------------------------------------------------ */

static UINT16 adc_get_lowerlimit_cnt( BYTE chan_no )
{
  /* Read one ADC lower-limit parameter (3 bytes, signed)
     from the working copy in EEPROM and convert it to a 16-bit ADC-count
     depending on the current ADC configuration; this function is called
     by the upper/lower-limit scanning function 'adc_window_check()' */
  return( adc_get_limit_cnt( chan_no, EE_ADC_LOWER ) );
}

/* ------------------------------------------------------------------------ */

static UINT16 adc_get_limit_cnt( BYTE chan_no, UINT16 ee_copy_addr )
{
  /* Read one ADC upper- or lower-limit parameter (3 bytes, signed)
     from the working copy in EEPROM and convert it to a 16-bit ADC-count
     depending on the current ADC configuration;
     the value could be signed or unsigned but is always returned in
     the shape of an unsigned integer */
  BOOL   unipolar;
  BYTE   adc_range_id;
  float  fullscale;
  UINT16 ee_addr;
  INT16  limit_i;
  UINT16 limit_ui;
  INT32  limit_l;
  float  limit_f;
  BYTE   *ptr;
  UINT16 *plimit_ui;

  /* Read the signed limit value from EEPROM into a longword */
  ee_addr = ee_copy_addr + chan_no*STORE_ADC_LIMIT_PARSIZE;
  ptr  = (BYTE *) &limit_l;
  *ptr = eepromw_read( ee_addr ); ++ptr; ++ee_addr;
  *ptr = eepromw_read( ee_addr ); ++ptr; ++ee_addr;
  *ptr = eepromw_read( ee_addr );
  /* Fill 4th byte: is it a negative number ? */
  if( (*ptr) & 0x80 )
    {
      /* Negative */
      ++ptr;
      *ptr = 0xFF;
    }
  else
    {
      /* Positive */
      ++ptr;
      *ptr = 0x00;
    }

  /* Get the ADC configuration */
  unipolar     = (AdcConfig & CS23_CSR_UNIPOLAR);
  adc_range_id = ((AdcConfig & CS23_CSR_GAIN_MASK) >> CS23_CSR_GAIN_SHIFT);
  fullscale    = V_FULLSCALE[adc_range_id];

  /* Convert it from microVolts to a 16-bit ADC-count */
  limit_f = ((float) limit_l) / fullscale;
  if( unipolar )
    {
      /* Clip to minimum */
      if( limit_l < 0L )
	limit_f = 0.0;      /* Can't accept a negative value */
      else
	limit_f *= 65535.0; /* Map to a 16-bit unsigned ADC-count */

      /* Clip to maximum */
      if( limit_f > 65535.0 )
	limit_ui = 0xFFFF;
      else
	limit_ui = (UINT16) limit_f;

      plimit_ui = &limit_ui;
    }
  else
    {
      limit_f *= 32767.0; /* Map to a 16-bit signed ADC-count */

      /* Clip to maximum */
      if( limit_f > 32767.0 )
	limit_i = (INT16) 0x7FFF;
      else
	/* Clip to minimum */
	if( limit_f < -32768.0 )
	  limit_i = (INT16) 0x8000;
	else
	  limit_i = (INT16) limit_f;

      plimit_ui = (UINT16 *) &limit_i;
    }

  /* Return the value as an unsigned integer */
  return *plimit_ui;
}

/* ------------------------------------------------------------------------ */

BOOL adc_set_delta( BYTE chan_no, BYTE *delta )
{
  /* Sets the _working_ copy in EEPROM of one ADC channel's delta-change
     parameters; to save it permanently the standard CANopen 'save-parameters'
     operation must be executed (OD index 1010) */
  BYTE result;

  /* Delta comes as a 4-byte (unsigned) integer value in microVolts,
     but should not be larger than 5V = 5000000 microVolts = 0x004C4B40 */
  if( delta[3] != 0x00 || delta[2] > 0x4C ) return FALSE;

  result = adc_set_limit( chan_no, delta, EE_ADC_DELTA );

  if( result == FALSE )
    {
      AdcError |= ADC_ERR_DELTA;

      /* CANopen Error Code 0x5000: device hardware */
      can_write_emergency( 0x00, 0x50, EMG_EEPROM_WRITE_LIMITS, 1,
			   STORE_ADC_LIMIT_PARSIZE, 0, ERRREG_MANUFACTURER );
    }
  return result;
}

/* ------------------------------------------------------------------------ */

BOOL adc_set_upperlimit( BYTE chan_no, BYTE *upper )
{
  /* Sets the _working_ copy in EEPROM of one ADC channel's upper-limit
     parameters; to save it permanently the standard CANopen 'save-parameters'
     operation must be executed (OD index 1010) */
  BYTE result;

  /* Upper comes as a 4-byte signed integer value in microVolts,
     but should not be larger than 5V = 5000000 microVolts = 0x004C4B40
     or smaller than -5V = -5000000 microVolts = 0xFFB3B4C0 */
  if( (upper[3] == 0x00 && upper[2] > 0x4C) ||
      (upper[3] != 0x00 && upper[3] != 0xFF) ||
      (upper[3] == 0xFF && upper[2] < 0xB3) )
    return FALSE;

  result = adc_set_limit( chan_no, upper, EE_ADC_UPPER );

  if( result == FALSE )
    {
      AdcError |= ADC_ERR_UPPER;

      /* CANopen Error Code 0x5000: device hardware */
      can_write_emergency( 0x00, 0x50, EMG_EEPROM_WRITE_LIMITS, 2,
			   STORE_ADC_LIMIT_PARSIZE, 0, ERRREG_MANUFACTURER );
    }
  return result;
}

/* ------------------------------------------------------------------------ */

BOOL adc_set_lowerlimit( BYTE chan_no, BYTE *lower )
{
  /* Sets the _working_ copy in EEPROM of one ADC channel's lower-limit
     parameters; to save it permanently the standard CANopen 'save-parameters'
     operation must be executed (OD index 1010) */
  BYTE result;

  /* Upper comes as a 4-byte signed integer value in microVolts,
     but should not be larger than 5V = 5000000 microVolts = 0x004C4B40
     or smaller than -5V = -5000000 microVolts = 0xFFB3B4C0 */
  if( (lower[3] == 0x00 && lower[2] > 0x4C) ||
      (lower[3] != 0x00 && lower[3] != 0xFF) ||
      (lower[3] == 0xFF && lower[2] < 0xB3) )
    return FALSE;

  result = adc_set_limit( chan_no, lower, EE_ADC_LOWER );

  if( result == FALSE )
    {
      AdcError |= ADC_ERR_LOWER;

      /* CANopen Error Code 0x5000: device hardware */
      can_write_emergency( 0x00, 0x50, EMG_EEPROM_WRITE_LIMITS, 3,
			   STORE_ADC_LIMIT_PARSIZE, 0, ERRREG_MANUFACTURER );
    }
  return result;
}

/* ------------------------------------------------------------------------ */

static BOOL adc_set_limit( BYTE chan_no, BYTE *limit, UINT16 base_addr )
{
  UINT16 ee_addr;
  BYTE   byt;
  BOOL   result = TRUE;

  if( chan_no >= 0xFE )
    {
      /* Set limit parameter for all channels to this value */
      BYTE ch;
      ee_addr = base_addr;
      for( ch=0; ch<ADC_MAX_INPUTS; ++ch )
	{
	  /* Copy limit[] to EEPROM, byte-by-byte */
	  for( byt=0; byt<STORE_ADC_LIMIT_PARSIZE; ++byt )
	    {
	      if( eepromw_read(ee_addr) != limit[byt] )
		eepromw_write( ee_addr, limit[byt] );
	      if( eepromw_read(ee_addr) != limit[byt] ) /* Check */
		result = FALSE;
	      ++ee_addr;
	    }
	}
    }
  else
    {
      /* Set limit parameter for the designated channel to this value */
      ee_addr = base_addr + chan_no*STORE_ADC_LIMIT_PARSIZE;
      /* Copy limit[] to EEPROM, byte-by-byte */
      for( byt=0; byt<STORE_ADC_LIMIT_PARSIZE; ++byt )
	{
	  if( eepromw_read(ee_addr) != limit[byt] )
	    eepromw_write( ee_addr, limit[byt] );
	  if( eepromw_read(ee_addr) != limit[byt] ) result = FALSE; /* Check */
	  ++ee_addr;
	}
    }

  return result;
}

/* ------------------------------------------------------------------------ */

void adc_get_delta( BYTE chan_no, BYTE *delta )
{
  /* Read an ADC delta-change parameter from the 24-bit working copy in EEPROM;
     assume 'delta' points to a 4-byte array to hold the (unsigned) long */
  adc_get_limit( chan_no, delta, EE_ADC_DELTA );
}

/* ------------------------------------------------------------------------ */

void adc_get_upperlimit( BYTE chan_no, BYTE *upper )
{
  /* Read an ADC upper-limit parameter from the 24-bit working copy in EEPROM;
     assume 'upper' points to a 4-byte array to hold the (signed) long */
  adc_get_limit( chan_no, upper, EE_ADC_UPPER );
}

/* ------------------------------------------------------------------------ */

void adc_get_lowerlimit( BYTE chan_no, BYTE *lower )
{
  /* Read an ADC lower-limit parameter from the 24-bit working copy in EEPROM;
     assume 'lower' points to a 4-byte array to hold the (signed) long */
  adc_get_limit( chan_no, lower, EE_ADC_LOWER );
}

/* ------------------------------------------------------------------------ */

static void adc_get_limit( BYTE chan_no, BYTE *limit, UINT16 base_addr )
{
  /* Retrieve the current value of an ADC limit parameter, stored in EEPROM */
  UINT16 ee_addr;
  BYTE   byt;

  /* Initialize all bytes: assume there are 4 */
  for( byt=0; byt<4; ++byt ) limit[byt] = 0;

  ee_addr = base_addr + chan_no*STORE_ADC_LIMIT_PARSIZE;

  /* Copy the delta-change parameter to delta[], byte-by-byte */
  for( byt=0; byt<STORE_ADC_LIMIT_PARSIZE; ++byt )
    {
      /* Copy the limit parameter to limit[], byte-by-byte */
      limit[byt] = eepromw_read( ee_addr );
      ++ee_addr;
    }

  /* Assume we just read a 24-bit signed integer */
  if( limit[2] & 0x80 ) limit[3] = 0xFF;
}

/* ------------------------------------------------------------------------ */

static BOOL adc_load_deltas( void )
{
  return( adc_load_limits(STORE_ADC_DELTA_ADDR,
			  EE_ADC_DELTA, ADC_ERR_DELTA, 0x00) );
}

/* ------------------------------------------------------------------------ */

static BOOL adc_load_upperlimits( void )
{
  return( adc_load_limits(STORE_ADC_UPPER_ADDR,
			  EE_ADC_UPPER, ADC_ERR_UPPER, 0xFF) );
}

/* ------------------------------------------------------------------------ */

static BOOL adc_load_lowerlimits( void )
{
  return( adc_load_limits(STORE_ADC_LOWER_ADDR,
			  EE_ADC_LOWER, ADC_ERR_LOWER, 0x00) );
}

/* ------------------------------------------------------------------------ */

static BOOL adc_load_limits( UINT16 ee_storage_addr,
			     UINT16 ee_copy_addr,
			     BYTE   err_id,
			     BYTE   dflt )
{
  /* This function is called during node initialization;
     copies one of the permanent ADC limit parameter array in EEPROM
     into the working copy location, also in EEPROM */
  BYTE   result = TRUE;
  BYTE   byt, ee;
  UINT16 i_store, i_copy;
  UINT16 i_info;

  i_store = ee_storage_addr;
  i_copy  = ee_copy_addr;
  i_info  = i_store + STORE_ADC_LIMIT_INFO_ADDR;

  WDR(); /* This can be a somewhat lengthy operation... */

  /* Is there a valid parameter data block ? */
  if( eepromw_read(i_info+2) == STORE_VALID_CHAR )
    {
      /* Check the CRC (run the CRC on the delta-change parameters plus
	 the stored CRC value: the result should be zero) */
      if( crc16_eeprom( ee_storage_addr, STORE_ADC_LIMIT_SIZE+2 ) == 0 )
	{
	  /* Okay!, so copy them (if necessary), byte-by-byte */
	  for( byt=0; byt<STORE_ADC_LIMIT_SIZE; ++byt )
	    {
	      WDR(); /* This can be a somewhat lengthy operation... */
	      ee = eepromw_read( i_store );
	      if( eepromw_read(i_copy) != ee ) eepromw_write( i_copy, ee );
	      if( eepromw_read(i_copy) != ee ) result = FALSE; /* Check */
	      ++i_store;
	      ++i_copy;
	    }
	  /* Error while copying ? */
	  if( result == FALSE ) AdcError |= err_id;
	  return result;
	}
      else
	{
	  /* Error in CRC */
	  result = FALSE;
	}
    }

  /* Initialize limit parameters to the default value */
  for( byt=0; byt<STORE_ADC_LIMIT_SIZE; ++byt, ++i_copy )
    {
      WDR(); /* This can be a somewhat lengthy operation... */
      if( eepromw_read(i_copy) != dflt ) eepromw_write( i_copy, dflt );
      if( eepromw_read(i_copy) != dflt ) result = FALSE; /* Check */
    }

  if( result == FALSE )
    {
      /* Don't send an Emergency message here (in case of a 'FALSE' result),
	 because this function is only called during node initialization when
	 CANopen messages are not allowed to be sent yet */
      AdcError |= err_id;
    }

  return result;
}

/* ------------------------------------------------------------------------ */

BOOL adc_store_deltas( void )
{
  BOOL result;

  result = adc_store_limits( STORE_ADC_DELTA_ADDR, EE_ADC_DELTA );

  if( result == FALSE )
    {
      AdcError |= ADC_ERR_DELTA;

      /* CANopen Error Code 0x5000: device hardware */
      can_write_emergency( 0x00, 0x50, EMG_EEPROM_WRITE_LIMITS, 1,
			   STORE_ADC_LIMIT_SIZE, 0, ERRREG_MANUFACTURER );
    }

  return result;
}

/* ------------------------------------------------------------------------ */

BOOL adc_store_upperlimits( void )
{
  BOOL result;

  result = adc_store_limits( STORE_ADC_UPPER_ADDR, EE_ADC_UPPER );

  if( result == FALSE )
    {
      AdcError |= ADC_ERR_UPPER;

      /* CANopen Error Code 0x5000: device hardware */
      can_write_emergency( 0x00, 0x50, EMG_EEPROM_WRITE_LIMITS, 2,
			   STORE_ADC_LIMIT_SIZE, 0, ERRREG_MANUFACTURER );
    }

  return result;
}

/* ------------------------------------------------------------------------ */

BOOL adc_store_lowerlimits( void )
{
  BOOL result;

  result = adc_store_limits( STORE_ADC_LOWER_ADDR, EE_ADC_LOWER );

  if( result == FALSE )
    {
      AdcError |= ADC_ERR_LOWER;

      /* CANopen Error Code 0x5000: device hardware */
      can_write_emergency( 0x00, 0x50, EMG_EEPROM_WRITE_LIMITS, 3,
			   STORE_ADC_LIMIT_SIZE, 0, ERRREG_MANUFACTURER );
    }

  return result;
}

/* ------------------------------------------------------------------------ */

static BOOL adc_store_limits( UINT16 ee_storage_addr,
			      UINT16 ee_copy_addr )
{
  /* Copies the working-copy ADC delta/upperlimit/lowerlimit parameter array
     in EEPROM into permanent storage, also in EEPROM;
     after power-up or reset the array in permanent storage EEPROM
     will be copied to the working copy location in EEPROM */
  BOOL   result = TRUE;
  BYTE   byt, ee;
  UINT16 i_store, i_copy;
  UINT16 i_info;
  UINT16 crc;

  i_store = ee_storage_addr;
  i_copy  = ee_copy_addr;
  i_info  = i_store + STORE_ADC_LIMIT_INFO_ADDR;

  /* Store the delta-change parameter array in EEPROM and
     check, byte-by-byte */
  for( byt=0; byt<STORE_ADC_LIMIT_SIZE; ++byt )
    {
      WDR(); /* This can be a somewhat lengthy operation... */
      ee = eepromw_read( i_copy );
      if( eepromw_read(i_store) != ee ) eepromw_write( i_store, ee );
      if( eepromw_read(i_store) != ee ) result = FALSE; /* Check */
      ++i_store;
      ++i_copy;
    }

  /* Calculate CRC of the full array */
  crc = crc16_eeprom( ee_storage_addr, STORE_ADC_LIMIT_SIZE );

  WDR(); /* This can be a somewhat lengthy operation... */

  /* Store CRC (MSB first) */
  byt = (BYTE) ((crc & 0xFF00) >> 8);
  eepromw_write( i_info, byt );
  /* Check */
  if( eepromw_read(i_info) != byt ) result = FALSE;

  /* And now the CRC LSB */
  ++i_info;
  byt = (BYTE) (crc & 0x00FF);
  eepromw_write( i_info, byt );
  /* Check */
  if( eepromw_read(i_info) != byt ) result = FALSE;

  /* The 'valid data' byte */
  ++i_info;
  if( result == TRUE )
    {
      /* Now we have a valid delta-change parameter block ! */
      if( eepromw_read(i_info) != STORE_VALID_CHAR )
	eepromw_write( i_info, STORE_VALID_CHAR );
      /* Check */
      if( eepromw_read(i_info) != STORE_VALID_CHAR ) result = FALSE;
    }
  else
    {
      /* Mark it as an invalid parameter block */
      eepromw_write( i_info, 0xFF );
    }

  return result;
}

/* ------------------------------------------------------------------------ */

BOOL adc_invalidate_deltas( void )
{
  BOOL result;

  result = adc_invalidate_limits( STORE_ADC_DELTA_ADDR );

  if( result == FALSE )
    {
      AdcError |= ADC_ERR_DELTA;

      /* CANopen Error Code 0x5000: device hardware */
      can_write_emergency( 0x00, 0x50, EMG_EEPROM_WRITE_LIMITS, 1,
			   3, 0, ERRREG_MANUFACTURER );
    }

  return result;
}

/* ------------------------------------------------------------------------ */

BOOL adc_invalidate_upperlimits( void )
{
  BOOL result;

  result = adc_invalidate_limits( STORE_ADC_UPPER_ADDR );

  if( result == FALSE )
    {
      AdcError |= ADC_ERR_UPPER;

      /* CANopen Error Code 0x5000: device hardware */
      can_write_emergency( 0x00, 0x50, EMG_EEPROM_WRITE_LIMITS, 3,
			   3, 0, ERRREG_MANUFACTURER );
    }

  return result;
}

/* ------------------------------------------------------------------------ */

BOOL adc_invalidate_lowerlimits( void )
{
  BOOL result;

  result = adc_invalidate_limits( STORE_ADC_UPPER_ADDR );

  if( result == FALSE )
    {
      AdcError |= ADC_ERR_LOWER;

      /* CANopen Error Code 0x5000: device hardware */
      can_write_emergency( 0x00, 0x50, EMG_EEPROM_WRITE_LIMITS, 2,
			   3, 0, ERRREG_MANUFACTURER );
    }

  return result;
}

/* ------------------------------------------------------------------------ */

static BOOL adc_invalidate_limits( UINT16 ee_storage_addr )
{
  /* Invalidate an ADC limit parameter array so that the defaults
     will be used after the next reset or power-up */
  UINT16 i_info;
  BYTE   i;
  BOOL   result = TRUE;

  i_info  = ee_storage_addr + STORE_ADC_LIMIT_INFO_ADDR;

  /* Write 0xFF in CRC and 'valid' byte locations */
  for( i=0; i<3; ++i )
    {
      eepromw_write( i_info, 0xFF );
      /* Check */
      if( eepromw_read(i_info) != 0xFF ) result = FALSE;
      ++i_info;
    }

  return result;
}

/* ------------------------------------------------------------------------ */

void adc_init_delta_references( void )
{
  BYTE chan;

  /* Force setting of the analog input references
     for the delta-change check */
  AdcInitRefCount = TRUE;

  /* Initialize readout-on-change counters */
  for( chan=0; chan<ADC_MAX_INPUTS; ++chan )
    {
      AdcWindowCounter[chan] = 0; AdcDeltaCounter[chan] = 0;
    }
}

/* ------------------------------------------------------------------------ */

BOOL adc_calibrate_range( BYTE od_range_id )
{
  /* This function is supposed to contain the calibration procedure
     for one voltage range, executed entirely by the ELMB itself
     (triggered by a write to a certain Object Dictionary entry).
     For now it was decided to let the host system handle the
     full procedure (in order to be more flexible), with just one new
     requirement for the ELMB: to be able to provide a 'pure' self-calibration
     function, i.e. not taking calibration constants into account that
     could already be available in EEPROM ! -->
     we need to be able to do a re-calibration ! (which needs self-calibration
     to be able to subsequently calculate the calibration factors to be applied
     to the Gain Registers, after self-calibration)
     The 'normal' power-up calibration function adc_set_calib_regs()
     now takes the presence of calibration constants into account. */

  BYTE result;

#ifdef __VARS_IN_EEPROM__
  AdcConfig = eeprom_read( EE_ADCCONFIG );
#endif /* __VARS_IN_EEPROM__ */

  /* Set the requested voltage range in the ADC configuration;
     note that 'od_range_id' has to be mapped from OD-subindex (=od_range_id)
     to the ADC voltage-range identifier (see ADC datasheet) */
  AdcConfig &= ~CS23_CSR_GAIN_MASK;
  AdcConfig |= (RANGE_ID[od_range_id] << CS23_CSR_GAIN_SHIFT);

#ifdef __VARS_IN_EEPROM__
  /* Store new value in EEPROM */
  if( eeprom_read( EE_ADCCONFIG ) != AdcConfig )
    eeprom_write( EE_ADCCONFIG, AdcConfig );
#endif /* __VARS_IN_EEPROM__ */

  /* Perform calibration only if reset succeeded */
  if( (result = adc_reset( TRUE )) == TRUE )
    result = adc_self_calibrate( TRUE );

  return result;
}

/* ------------------------------------------------------------------------ */

BOOL adc_set_calib_const( BYTE od_range_id, BYTE index, BYTE *val )
{
  /* Writes one of the calibration constants of the calibration parameter set
     for one particular voltage range.
     The new value is stored directly in EEPROM as part of the set
     and also the CRC of the parameter datablock is updated.
     This function is used for permanently storing the ADC calibration
     constants (or restoring lost/corrupted constants), and thus
     should be used with care. */
  BYTE   result = TRUE;
  BYTE   byt;
  UINT16 ee_addr;
  UINT16 crc;

  /* Has the write operation been enabled ? */
  if( (AdcCalibConstWriteEnabled & TRUE) == FALSE ) return FALSE;

  /* Allow 1 write operation at a time */
  AdcCalibConstWriteEnabled = FALSE;

  /* EEPROM address for this constant */
  ee_addr = (STORE_ADC_CALIB_ADDR + od_range_id*STORE_ADC_CALIB_BLOCKSIZE +
	     index*STORE_ADC_CALIB_PARSIZE);

  /* Store the constant in EEPROM and check, byte-by-byte */
  for( byt=0; byt<STORE_ADC_CALIB_PARSIZE; ++byt, ++ee_addr )
    {
      BYTE val_b;
      val_b = val[byt];
      if( eepromw_read(ee_addr) != val_b ) eepromw_write( ee_addr, val_b );
      if( eepromw_read(ee_addr) != val_b ) result = FALSE; /* Check */
    }

  /* (Re)calculate CRC of the full set of constants */
  ee_addr = STORE_ADC_CALIB_ADDR + od_range_id*STORE_ADC_CALIB_BLOCKSIZE;
  crc = crc16_eeprom( ee_addr, STORE_ADC_CALIB_SIZE );

  /* Store CRC immediately behind the data, MSB first */
  byt = (BYTE) ((crc & 0xFF00) >> 8);
  ee_addr += STORE_ADC_CALIB_SIZE;
  eepromw_write( ee_addr, byt );
  if( eepromw_read(ee_addr) != byt ) result = FALSE; /* Check */
  /* And now the CRC LSB */
  byt = (BYTE) (crc & 0x00FF);
  ++ee_addr;
  eepromw_write( ee_addr, byt );
  if( eepromw_read(ee_addr) != byt ) result = FALSE; /* Check */

  /* The 'valid data' byte */
  ++ee_addr;
  if( result == TRUE )
    {
      /* Now we have a valid calibration constants datablock ! */
      if( eepromw_read(ee_addr) != STORE_VALID_CHAR )
	eepromw_write( ee_addr, STORE_VALID_CHAR );
      if( eepromw_read(ee_addr) != STORE_VALID_CHAR ) /* Check */
	result = FALSE;
    }
  else
    {
      /* Mark it as an invalid parameter block */
      eepromw_write( ee_addr, 0xFF );
    }

  if( result == FALSE )
    {
      AdcError |= ADC_ERR_CALIB_CNST;

      /* CANopen Error Code 0x5000: device hardware */
      can_write_emergency( 0x00, 0x50, EMG_EEPROM_WRITE_PARS, 0xFE,
			   STORE_ADC_CALIB_SIZE, 0, ERRREG_MANUFACTURER );
    }

  return result;
}

/* ------------------------------------------------------------------------ */

BOOL adc_get_calib_const( BYTE od_range_id, BYTE index, BYTE *val,
			  BOOL send_emergency )
{
  BYTE   byt, result = TRUE;
  UINT16 ee_addr;

  AdcCalibConstWriteEnabled = FALSE;

  /* Initialize all bytes: assume there are 4 */
  for( byt=0; byt<4; ++byt ) val[byt] = 0;

  /* Is there a valid calibration constants datablock
     for the designated voltage range? */
  if( adc_valid_calib_const( od_range_id ) )
    {
      /* Start address of this set of constants */
      ee_addr = STORE_ADC_CALIB_ADDR + od_range_id*STORE_ADC_CALIB_BLOCKSIZE;

      /* Check the CRC (run the CRC on the datablock plus
	 the stored CRC value: the result should be zero) */
      if( crc16_eeprom( ee_addr, STORE_ADC_CALIB_SIZE+2 ) == 0 )
	{
	  /* EEPROM address for this constant */
	  ee_addr = (STORE_ADC_CALIB_ADDR +
		     od_range_id*STORE_ADC_CALIB_BLOCKSIZE +
		     index*STORE_ADC_CALIB_PARSIZE);

	  /* Copy the constant value to val[], byte-by-byte */
	  for( byt=0; byt<STORE_ADC_CALIB_PARSIZE; ++byt, ++ee_addr )
	    {
	      val[byt] = eepromw_read( ee_addr );
	    }
	}
      else
	{
	  /* Error in CRC */
	  AdcError |= ADC_ERR_CALIB_CNST;

	  if( send_emergency )
	    {
	      /* CANopen Error Code 0x5000: device hardware */
	      can_write_emergency( 0x00, 0x50, EMG_EEPROM_READ_PARS, 0xFE,
				   1, 0, ERRREG_MANUFACTURER );
	    }
	  result = FALSE;
	}
    }
  else
    {
      /* No valid constants available */
      result = FALSE;
    }
  return result;
}

/* ------------------------------------------------------------------------ */

BOOL adc_erase_calib_const( BYTE od_range_id, BYTE val )
{
  /* Erases a complete set of calibration constants for
     one particular voltage range from EEPROM. To be used with care.... */
  BYTE   byt, result = TRUE;
  UINT16 ee_addr;

  /* Has the erase operation been enabled ? */
  if( (AdcCalibConstWriteEnabled & TRUE) == FALSE ) return FALSE;

  /* Allow 1 erase operation at a time */
  AdcCalibConstWriteEnabled = FALSE;

  /* Allow only when 'val' has a particular value */
  if( val != 0xEE ) return FALSE;

  /* EEPROM address for this set of constants */
  ee_addr = STORE_ADC_CALIB_ADDR + od_range_id*STORE_ADC_CALIB_BLOCKSIZE;

  /* Erase, byte-by-byte, by setting EEPROM locations to 0xFF */
  for( byt=0; byt<STORE_ADC_CALIB_BLOCKSIZE; ++byt, ++ee_addr )
    {
      if( eepromw_read(ee_addr) != 0xFF ) eepromw_write( ee_addr, 0xFF );
      if( eepromw_read(ee_addr) != 0xFF ) result = FALSE; /* Check */
    }

  if( result == FALSE )
    {
      AdcError |= ADC_ERR_CALIB_CNST;

      /* CANopen Error Code 0x5000: device hardware */
      can_write_emergency( 0x00, 0x50, EMG_EEPROM_WRITE_PARS, 0xFE,
			   STORE_ADC_CALIB_BLOCKSIZE, 0, ERRREG_MANUFACTURER );
    }

  return result;
}

/* ------------------------------------------------------------------------ */

BOOL adc_calib_const_write_enable( BYTE val )
{
  /* Set the boolean only when 'val' has a particular value */
  AdcCalibConstWriteEnabled = FALSE;
  if( val == 0xA5 )
    {
      /* Enable a single write to a calibration constant location in EEPROM */
      AdcCalibConstWriteEnabled = TRUE;
    }
  return AdcCalibConstWriteEnabled;
}

/* ------------------------------------------------------------------------ */

BOOL adc_calibrated( void )
{
  /* Is the ADC calibrated for its current setting ? */
  BYTE adc_range_id;

#ifdef __VARS_IN_EEPROM__
  AdcConfig = eeprom_read( EE_ADCCONFIG );
#endif

  adc_range_id = ((AdcConfig & CS23_CSR_GAIN_MASK) >> CS23_CSR_GAIN_SHIFT);

  return( adc_valid_calib_const(RANGE_ID[adc_range_id]) );
}

/* ------------------------------------------------------------------------ */

static BOOL adc_valid_calib_const( BYTE od_range_id )
{
  /* Is the calibration constants datablock valid for this voltage range ? */
  UINT16 ee_addr;

  /* Address of the 'valid' byte */
  ee_addr = (STORE_ADC_CALIB_ADDR + od_range_id*STORE_ADC_CALIB_BLOCKSIZE +
	     STORE_ADC_CALIB_PARS*STORE_ADC_CALIB_PARSIZE + 2);

  return( eepromw_read(ee_addr) == STORE_VALID_CHAR );
}

/* ------------------------------------------------------------------------ */
