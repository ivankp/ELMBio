/* ------------------------------------------------------------------------
File   : sdo.c

Descr  : The CANopen SDO server, which serves read/write requests to the
	 Object Dictionary.

History: 25JAN.00; Henk B&B; Start of development of a version for the ELMB.
--------------------------------------------------------------------------- */

#include "general.h"
#include "adc.h"
#include "can.h"
#include "crc.h"
#include "dac.h"
#include "digio.h"
#include "guarding.h"
#include "objects.h"
#include "pdo.h"
#include "serialno.h"
#include "spi_gp.h"
#include "store.h"
#include "timer103.h"

#ifndef __ELMB103__
#include "watchdog.h"
#endif

#ifdef __2313_SLAVE_PRESENT__
#include "download.h"
#endif /* __2313_SLAVE_PRESENT__ */

#ifdef __INCLUDE_TESTS__
#include "iotest.h"
#endif

/* ------------------------------------------------------------------------ */
/* Local prototypes */

static BYTE sdo_expedited_read( BYTE *msg_data );
static BYTE sdo_expedited_write( BYTE *msg_data );
static void sdo_abort( BYTE error_class,
		       BYTE error_code,
		       BYTE *msg_data );

static void jump_to_bootloader( void );

/* ------------------------------------------------------------------------ */

void sdo_server( BYTE *msg_data )
{
  BYTE sdo_mode;
  BYTE cs;
  BYTE sdo_error;

  /* SDO modifier bits are in the first data byte */
  sdo_mode = msg_data[0];

  /* Determine the command specifier (cs) from
     the SDO modifier bits in the first byte */
  cs = sdo_mode & SDO_COMMAND_SPECIFIER_MASK;

  switch( cs )
    {
    case SDO_INITIATE_UPLOAD_REQ:
      /* ==> Read from the Object Dictionary <== */

      /* Expedited transfer only (data: 4 bytes or less) */

      /* Process the SDO received */
      sdo_error = sdo_expedited_read( msg_data );

      /* Send the SDO reply... */
      if( !sdo_error )
	{
	  /* All went okay */
	  can_write( C91_SDOTX, C91_SDOTX_LEN, msg_data );
	}
      else
	{
	  /* Aborted... */
	  sdo_abort( SDO_ECLASS_ACCESS, sdo_error, msg_data );
	}
      break;

    case SDO_INITIATE_DOWNLOAD_REQ:
      /* ==> Write to the Object Dictionary <== */

      if( sdo_mode & SDO_EXPEDITED )
	{
	  /* Expedited transfer (data: 4 bytes or less) */

	  /* Process the SDO received */
	  sdo_error = sdo_expedited_write( msg_data );

	  /* Send the SDO reply... */
	  if( !sdo_error )
	    {
	      /* All went okay */
	      can_write( C91_SDOTX, C91_SDOTX_LEN, msg_data );
	    }
	  else
	    {
	      /* Aborted... */
	      sdo_abort( SDO_ECLASS_ACCESS, sdo_error, msg_data );
	    }
	}
      else
	{
	  /* Start of segmented transfer (OD write): no service... */
	  sdo_abort( SDO_ECLASS_SERVICE, SDO_ECODE_PAR_ILLEGAL, msg_data );
	}
      break;

    case SDO_UPLOAD_SEGMENT_REQ:
      /* ==> Read from the Object Dictionary (segmented): no service... <== */
      sdo_abort( SDO_ECLASS_SERVICE, SDO_ECODE_PAR_ILLEGAL, msg_data );
      break;

    default:
      /* Unknown command specifier !? */
      sdo_abort( SDO_ECLASS_SERVICE, SDO_ECODE_PAR_ILLEGAL, msg_data );
      break;
    }
}

/* ------------------------------------------------------------------------ */

static BYTE sdo_expedited_read( BYTE *msg_data )
{
  BYTE sdo_error;
  BYTE nbytes;
  BYTE od_index_hi, od_index_lo, od_subind;

  /* No error */
  sdo_error   = 0;

  /* Extract Object Dictionary indices */
  od_index_lo = msg_data[1];
  od_index_hi = msg_data[2];
  od_subind   = msg_data[3];

  /* Initialise data bytes to zero */
  msg_data[4] = 0;
  msg_data[5] = 0;
  msg_data[6] = 0;
  msg_data[7] = 0;

  /* Default number of significant bytes:
     set to a different value if it saves more statements,
     now default assuming 32-bit data item... */
  nbytes = 4;

  /* Get the requested object */
  switch( od_index_hi )
    {
    case OD_DEVICE_INFO_HI:
      switch( od_index_lo )
	{
	case OD_DEVICE_TYPE_LO:
	  if( od_subind == 0 )
	    {
	      msg_data[4] = DEVICE_TYPE_CHAR0;
	      msg_data[5] = DEVICE_TYPE_CHAR1;
	      msg_data[6] = DEVICE_TYPE_CHAR2;
	      msg_data[7] = DEVICE_TYPE_CHAR3;
	    }
	  else
	    {
	      /* The sub-index does not exist */
	      sdo_error = SDO_ECODE_ATTRIBUTE;
	    }
	  break;

	case OD_ERROR_REG_LO:
	  if( od_subind == 0 )
	    {
	      msg_data[4] = CANopenErrorReg;
	      nbytes = 1;  /* Significant bytes < 4 */
	    }
	  else
	    {
	      /* The sub-index does not exist */
	      sdo_error = SDO_ECODE_ATTRIBUTE;
	    }
	  break;

	case OD_STATUS_REG_LO:
	  if( od_subind == 0 )
	    {
	      adc_status( &msg_data[4] );
	    }
	  else
	    {
	      /* The sub-index does not exist */
	      sdo_error = SDO_ECODE_ATTRIBUTE;
	    }
	  break;

	case OD_DEVICE_NAME_LO:
	  if( od_subind == 0 )
	    {
	      msg_data[4] = MNFCT_DEV_NAME_CHAR0;
	      msg_data[5] = MNFCT_DEV_NAME_CHAR1;
	      msg_data[6] = MNFCT_DEV_NAME_CHAR2;
	      msg_data[7] = MNFCT_DEV_NAME_CHAR3;
	    }
	  else
	    {
	      /* The sub-index does not exist */
	      sdo_error = SDO_ECODE_ATTRIBUTE;
	    }
	  break;

	case OD_HW_VERSION_LO:
	  if( od_subind == 0 )
	    {
	      msg_data[4] = MNFCT_HARDW_VERSION_CHAR0;
	      msg_data[5] = MNFCT_HARDW_VERSION_CHAR1;
	      msg_data[6] = MNFCT_HARDW_VERSION_CHAR2;
	      msg_data[7] = MNFCT_HARDW_VERSION_CHAR3;
	    }
	  else
	    {
	      /* The sub-index does not exist */
	      sdo_error = SDO_ECODE_ATTRIBUTE;
	    }
	  break;

	case OD_SW_VERSION_LO:
	  if( od_subind == 0 )
	    {
	      msg_data[4] = MNFCT_SOFTW_VERSION_CHAR0;
	      msg_data[5] = MNFCT_SOFTW_VERSION_CHAR1;
	      msg_data[6] = MNFCT_SOFTW_VERSION_CHAR2;
	      msg_data[7] = MNFCT_SOFTW_VERSION_CHAR3;
	    }
	  else
	    {
	      if( od_subind == 1 )
		{
		  msg_data[4] = SOFTW_MINOR_VERSION_CHAR0;
		  msg_data[5] = SOFTW_MINOR_VERSION_CHAR1;
		  msg_data[6] = SOFTW_MINOR_VERSION_CHAR2;
		  msg_data[7] = SOFTW_MINOR_VERSION_CHAR3;
		}
	      else
		{
		  /* The sub-index does not exist */
		  sdo_error = SDO_ECODE_ATTRIBUTE;
		}
	    }
	  break;

	case OD_GUARDTIME_LO:
	  if( od_subind == 0 )
	    {
	      nbytes = guarding_get_guardtime( &msg_data[4] );
	    }
	  else
	    {
	      /* The sub-index does not exist */
	      sdo_error = SDO_ECODE_ATTRIBUTE;
	    }
	  break;

	case OD_LIFETIME_FACTOR_LO:
	  if( od_subind == 0 )
	    {
	      nbytes = guarding_get_lifetime( &msg_data[4] );
	    }
	  else
	    {
	      /* The sub-index does not exist */
	      sdo_error = SDO_ECODE_ATTRIBUTE;
	    }
	  break;

	case OD_STORE_PARAMETERS_LO:
	case OD_DFLT_PARAMETERS_LO:
	  if( od_subind == OD_NO_OF_ENTRIES )
	    {
	      msg_data[4] = OD_STORE_MAX_SUBID;
	      nbytes = 1;  /* Significant bytes < 4 */
	    }
	  else
	    {
	      if( od_subind <= OD_STORE_MAX_SUBID )
		{
		  /* Device saves parameters on command (OD_STORE_PARAMETERS),
		     restores parameters (OD_DFLT_PARAMETERS) */
		  msg_data[4] = 0x01;

		  /* ###??? Device saves parameters autonomously
		     (OD_STORE_PARAMETERS_LO) */
		  /*if( od_ind_lo == OD_STORE_PARAMETERS_LO )
		    msg_data[4] = 0x03; */
		}
	      else
		{
		  /* The sub-index does not exist */
		  sdo_error = SDO_ECODE_ATTRIBUTE;
		}
	    }
	  break;

	case OD_HEARTBEAT_TIME_LO:
	  if( od_subind == 0 )
	    {
	      nbytes = guarding_get_heartbeattime( &msg_data[4] );
	    }
	  else
	    {
	      /* The sub-index does not exist */
	      sdo_error = SDO_ECODE_ATTRIBUTE;
	    }
	  break;

	case OD_IDENTITY_LO:
	  switch( od_subind )
	    {
	    case OD_NO_OF_ENTRIES:
	      msg_data[4] = 1;
	      nbytes = 1;  /* Significant bytes < 4 */
	      break;
	    case 1:
	      msg_data[4] = 0x78;
	      msg_data[5] = 0x56;
	      msg_data[6] = 0x34;
	      msg_data[7] = 0x12;
	      break;
	    default:
	      /* The sub-index does not exist */
	      sdo_error = SDO_ECODE_ATTRIBUTE;
	      break;
	    }
	  break;

	default:
	  /* The index can not be accessed, does not exist */
	  sdo_error = SDO_ECODE_NONEXISTENT;
	  break;
	}
      break;

    case OD_RPDO_PAR_HI:
      if( od_index_lo < RPDO_CNT )
	{
	  if( rpdo_get_comm_par( od_index_lo, od_subind,
				 &nbytes, &msg_data[4] ) == FALSE )
	    {
	      /* The sub-index does not exist */
	      sdo_error = SDO_ECODE_ATTRIBUTE;
	    }
	}
      else
	{
	  /* The index can not be accessed, does not exist */
	  sdo_error = SDO_ECODE_NONEXISTENT;
	}
      break;

    case OD_RPDO_MAP_HI:
      if( od_index_lo < RPDO_CNT )
	{
	  if( rpdo_get_mapping( od_index_lo, od_subind,
				&nbytes, &msg_data[4] ) == FALSE )
	    {
	      /* The sub-index does not exist */
	      sdo_error = SDO_ECODE_ATTRIBUTE;
	    }
	}
      else
	{
	  /* The index can not be accessed, does not exist */
	  sdo_error = SDO_ECODE_NONEXISTENT;
	}
      break;

    case OD_TPDO_PAR_HI:
      if( od_index_lo < TPDO_CNT )
	{
	  if( tpdo_get_comm_par( od_index_lo, od_subind,
				 &nbytes, &msg_data[4] ) == FALSE )
	    {
	      /* The sub-index does not exist */
	      sdo_error = SDO_ECODE_ATTRIBUTE;
	    }
	}
      else
	{
	  /* The index can not be accessed, does not exist */
	  sdo_error = SDO_ECODE_NONEXISTENT;
	}
      break;

    case OD_TPDO_MAP_HI:
      if( od_index_lo < TPDO_CNT )
	{
	  if( tpdo_get_mapping( od_index_lo, od_subind,
				&nbytes, &msg_data[4] ) == FALSE )
	    {
	      /* The sub-index does not exist */
	      sdo_error = SDO_ECODE_ATTRIBUTE;
	    }
	}
      else
	{
	  /* The index can not be accessed, does not exist */
	  sdo_error = SDO_ECODE_NONEXISTENT;
	}
      break;

    case OD_CAN_CONFIG_HI:
      if( od_index_lo == OD_CAN_CONFIG_LO )
	{
	  switch( od_subind )
	    {
	    case OD_NO_OF_ENTRIES:
	      msg_data[4] = 3;
	      nbytes = 1;  /* Significant bytes < 4 */
	      break;
	    case 1:
	      msg_data[4] = can_get_rtr_disabled();
	      nbytes = 1;  /* Significant bytes < 4 */
	      break;
	    case 2:
	      msg_data[4] = can_get_opstate_init();
	      nbytes = 1;  /* Significant bytes < 4 */
	      break;
	    case 3:
	      msg_data[4] = can_get_busoff_maxcnt();
	      nbytes = 1;  /* Significant bytes < 4 */
	      break;
	    default:
	      /* The sub-index does not exist */
	      sdo_error = SDO_ECODE_ATTRIBUTE;
	      break;
	    }
	}
      else
	{
	  /* The index can not be accessed, does not exist */
	  sdo_error = SDO_ECODE_NONEXISTENT;
	}
      break;

    case OD_DIGITAL_IN_HI:
      switch( od_index_lo )
	{
	case OD_DIGITAL_IN_8_LO:
	  if( od_subind == OD_NO_OF_ENTRIES )
	    {
	      msg_data[4] = digin_port_cnt();
	      nbytes = 1;  /* Significant bytes < 4 */
	    }
	  else
	    {
	      if( od_subind <= digin_port_cnt() )
		{
		  msg_data[4] = digin_get_port( od_subind );
		  nbytes = 1;  /* Significant bytes < 4 */
		}
	      else
		{
		  /* The sub-index does not exist */
		  sdo_error = SDO_ECODE_ATTRIBUTE;
		}
	    }
	  break;

	case OD_DIGIN_INTRPT_ENA_LO:
	  if( od_subind == 0 )
	    {
	      msg_data[4] = digin_get_intrpt_ena();
	      nbytes = 1;  /* Significant bytes < 4 */
	    }
	  else
	    {
	      /* The sub-index does not exist */
	      sdo_error = SDO_ECODE_ATTRIBUTE;
	    }
	  break;

	case OD_DIGIN_INTRPT_MSK_LO:
	  if( od_subind == OD_NO_OF_ENTRIES )
	    {
	      msg_data[4] = digin_port_cnt();
	      nbytes = 1;  /* Significant bytes < 4 */
	    }
	  else
	    {
	      if( od_subind <= digin_port_cnt() )
		{
		  msg_data[4] = digin_get_intrpt_mask( od_subind );
		  nbytes = 1;  /* Significant bytes < 4 */
		}
	      else
		{
		  /* The sub-index does not exist */
		  sdo_error = SDO_ECODE_ATTRIBUTE;
		}
	    }
	  break;

	default:
	  /* The index can not be accessed, does not exist */
	  sdo_error = SDO_ECODE_NONEXISTENT;
	  break;
	}
      break;

    case OD_DIGITAL_OUT_HI:
      switch( od_index_lo )
	{
	case OD_DIGITAL_OUT_8_LO:
	case OD_DIGITAL_OUT_8_MSK_LO:
	  if( od_subind == OD_NO_OF_ENTRIES )
	    {
	      msg_data[4] = digout_port_cnt();
	      nbytes = 1;  /* Significant bytes < 4 */
	    }
	  else
	    {
	      if( od_subind <= digout_port_cnt() )
		{
		  if( od_index_lo == OD_DIGITAL_OUT_8_LO )
		    msg_data[4] = digout_get_port( od_subind );
		  else
		    msg_data[4] = digout_get_mask( od_subind );
		  nbytes = 1;  /* Significant bytes < 4 */
		}
	      else
		{
		  /* The sub-index does not exist */
		  sdo_error = SDO_ECODE_ATTRIBUTE;
		}
	    }
	  break;

	default:
	  /* The index can not be accessed, does not exist */
	  sdo_error = SDO_ECODE_NONEXISTENT;
	  break;
	}
      break;

    case OD_ANALOG_IN_VOLTS_HI:
      if( od_index_lo == OD_ANALOG_IN_VOLTS_LO )
	{
	  if( od_subind == OD_NO_OF_ENTRIES )
	    {
	      msg_data[4] = ADC_MAX_INPUTS;
	      nbytes = 1;  /* Significant bytes < 4 */
	    }
	  else
	    {
	      if( od_subind <= ADC_MAX_INPUTS )
		{
		  if( adc_read_volts( od_subind-1, &msg_data[4] ) == FALSE )
		    {
		      /* Something went wrong... */
		      sdo_error = SDO_ECODE_HARDWARE;
		    }
		}
	      else
		{
		  /* The sub-index does not exist */
		  sdo_error = SDO_ECODE_ATTRIBUTE;
		}
	    }
	}
      else
	{
	  /* The sub-index does not exist */
	  sdo_error = SDO_ECODE_ATTRIBUTE;
	}
      break;

    case OD_ANALOG_HI:
      switch( od_index_lo )
	{
	case OD_ANALOG_IN_LO:
	  if( od_subind == OD_NO_OF_ENTRIES )
	    {
	      msg_data[4] = ADC_MAX_INPUTS;
	      nbytes = 1;  /* Significant bytes < 4 */
	    }
	  else
	    {
	      if( od_subind <= ADC_MAX_INPUTS )
		{
		  if( adc_read( od_subind-1, &msg_data[4] ) == FALSE )
		    {
		      /* Something went wrong... */
		      sdo_error = SDO_ECODE_HARDWARE;
		    }
		  else
		    {
		      /* Performed a conversion */
		      nbytes = 3;  /* Significant bytes < 4 */
		    }
		}
	      else
		{
		  /* The sub-index does not exist */
		  sdo_error = SDO_ECODE_ATTRIBUTE;
		}
	    }
	  break;

	case OD_ANALOG_OUT_16_LO:
	  if( od_subind == OD_NO_OF_ENTRIES )
	    {
	      msg_data[4] = dac_chan_cnt();
	      nbytes = 1;  /* Significant bytes < 4 */
	    }
	  else
	    {
	      if( od_subind <= dac_chan_cnt() )
		{
		  if( dac_read( od_subind-1, &msg_data[4] ) == FALSE )
		    {
		      /* Something went wrong... */
		      sdo_error = SDO_ECODE_HARDWARE;
		    }
		  else
		    {
		      /* Read current DAC setting */
		      nbytes = 2;  /* Significant bytes < 4 */
		    }
		}
	      else
		{
		  /* The sub-index does not exist */
		  sdo_error = SDO_ECODE_ATTRIBUTE;
		}
	    }
	  break;

	case OD_ANALOG_IN_INT_ENA_LO:
	  if( od_subind == 0 )
	    {
	      msg_data[4] = adc_get_readout_on_change();
	      nbytes = 1;  /* Significant bytes < 4 */
	    }
	  else
	    {
	      /* The sub-index does not exist */
	      sdo_error = SDO_ECODE_ATTRIBUTE;
	    }
	  break;

	case OD_ANALOG_IN_DELTA_LO:
	case OD_ANALOG_IN_UPPER_LO:
	case OD_ANALOG_IN_LOWER_LO:
	  if( od_subind == OD_NO_OF_ENTRIES )
	    {
	      msg_data[4] = ADC_MAX_INPUTS;
	      nbytes = 1;  /* Significant bytes < 4 */
	    }
	  else
	    {
	      if( od_subind <= ADC_MAX_INPUTS )
		{
		  switch( od_index_lo )
		    {
		    case OD_ANALOG_IN_DELTA_LO:
		      adc_get_delta( od_subind-1, &msg_data[4] );
		      break;
		    case OD_ANALOG_IN_UPPER_LO:
		      adc_get_upperlimit( od_subind-1, &msg_data[4] );
		      break;
		    case OD_ANALOG_IN_LOWER_LO:
		      adc_get_lowerlimit( od_subind-1, &msg_data[4] );
		      break;
		    default:
		      /* The sub-index does not exist */
		      sdo_error = SDO_ECODE_ATTRIBUTE;
		      break;
		    }
		}
	      else
		{
		  /* The sub-index does not exist */
		  sdo_error = SDO_ECODE_ATTRIBUTE;
		}
	    }
	  break;

	default:
	  /* The index can not be accessed, does not exist */
	  sdo_error = SDO_ECODE_NONEXISTENT;
	  break;
	}
      break;

    case OD_ADC_CONFIG_HI:
      switch( od_index_lo )
	{
	case OD_ADC_CONFIG_LO:
	  if( adc_get_config( od_subind,
			      &nbytes, &msg_data[4] ) == FALSE )
	    {
	      /* The sub-index does not exist or
		 cannot be accessed at the moment (ADC in use) */
	      sdo_error = SDO_ECODE_ATTRIBUTE;
	    }
	  break;

	case OD_ADC_RECALIB_SCAN_LO:
	  if( od_subind == 0 )
	    {
	      msg_data[4] = adc_get_calib_before_scan();
	      nbytes = 1;  /* Significant bytes < 4 */
	    }
	  else
	    {
	      /* The sub-index does not exist */
	      sdo_error = SDO_ECODE_ATTRIBUTE;
	    }
	  break;

	case OD_ADC_DELTA_ENA_LO:
	  if( od_subind == 0 )
	    {
	      msg_data[4] = adc_get_delta_scan_ena();
	      nbytes = 1;  /* Significant bytes < 4 */
	    }
	  else
	    {
	      /* The sub-index does not exist */
	      sdo_error = SDO_ECODE_ATTRIBUTE;
	    }
	  break;

	case OD_ADC_WINDOW_ENA_LO:
	  if( od_subind == 0 )
	    {
	      msg_data[4] = adc_get_window_scan_ena();
	      nbytes = 1;  /* Significant bytes < 4 */
	    }
	  else
	    {
	      /* The sub-index does not exist */
	      sdo_error = SDO_ECODE_ATTRIBUTE;
	    }
	  break;

	case OD_ADC_WINDOW_CNTR_LO:
	  if( od_subind == 0 )
	    {
	      msg_data[4] = adc_get_window_scan_cntr();
	      nbytes = 1;  /* Significant bytes < 4 */
	    }
	  else
	    {
	      /* The sub-index does not exist */
	      sdo_error = SDO_ECODE_ATTRIBUTE;
	    }
	  break;


	default:
	  /* The index can not be accessed, does not exist */
	  sdo_error = SDO_ECODE_NONEXISTENT;
	  break;
	}
      break;

    case OD_ADC_CALIB_RANGE_HI:
      if( od_index_lo == OD_ADC_CALIB_RANGE_LO )
	{
	  if( od_subind == 0 )
	    {
	      msg_data[4] = STORE_ADC_CALIB_BLOCKS;
	      nbytes = 1;  /* Significant bytes < 4 */
	    }
	  else
	    {
	      /* The sub-index does not exist */
	      sdo_error = SDO_ECODE_ATTRIBUTE;
	    }
	}
      else
	{
	  /* The index can not be accessed, does not exist */
	  sdo_error = SDO_ECODE_NONEXISTENT;
	}
      break;

    case OD_ADC_CALIB_PARS_HI:
      if( od_index_lo < STORE_ADC_CALIB_BLOCKS )
	{
	  if( od_subind == OD_NO_OF_ENTRIES )
	    {
	      msg_data[4] = 4;
	      nbytes = 1;  /* Significant bytes < 4 */
	    }
	  else
	    {
	      if( od_subind <= STORE_ADC_CALIB_PARS )
		{
		  if( adc_get_calib_const( od_index_lo, od_subind-1,
					   &msg_data[4], TRUE ) == FALSE )
		    {
		      /* EEPROM read operation failed
			 or calibration constant simply not present */
		      sdo_error = SDO_ECODE_HARDWARE;
		    }
		}
	      else
		{
		  /* The sub-index does not exist */
		  sdo_error = SDO_ECODE_ATTRIBUTE;
		}
	    }
	}
      else
	{
	  /* The index can not be accessed, does not exist */
	  sdo_error = SDO_ECODE_NONEXISTENT;
	}
      break;

    case OD_DIGIN_PAR_HI:
      if( od_index_lo == OD_DIGIN_DEBOUNCE_LO )
	{
	  if( od_subind == 0 )
	    {
	      msg_data[4] = digin_get_debounce();
	      nbytes = 1;  /* Significant bytes < 4 */
	    }
	  else
	    {
	      /* The sub-index does not exist */
	      sdo_error = SDO_ECODE_ATTRIBUTE;
	    }
	}
      else
	{
	  /* The index can not be accessed, does not exist */
	  sdo_error = SDO_ECODE_NONEXISTENT;
	}
      break;

    case OD_DIGOUT_PAR_HI:
      if( od_index_lo == OD_DIGOUT_INIT_LO )
	{
	  if( od_subind == 0 )
	    {
	      msg_data[4] = digout_get_init();
	      nbytes = 1;  /* Significant bytes < 4 */
	    }
	  else
	    {
	      /* The sub-index does not exist */
	      sdo_error = SDO_ECODE_ATTRIBUTE;
	    }
	}
      else
	{
	  /* The index can not be accessed, does not exist */
	  sdo_error = SDO_ECODE_NONEXISTENT;
	}
      break;

    case OD_DAC_CONFIG_HI:
      if( od_index_lo == OD_DAC_CONFIG_LO )
	{
	  switch( od_subind )
	    {
	    case OD_NO_OF_ENTRIES:
	      msg_data[4] = 3;
	      nbytes = 1;  /* Significant bytes < 4 */
	      break;
	    case 1:
	      msg_data[4] = dac_chan_cnt();
	      nbytes = 1;  /* Significant bytes < 4 */
	      break;
	    case 2:
	      msg_data[4] = dac_get_max525_select();
	      nbytes = 1;  /* Significant bytes < 4 */
	      break;
	    case 3:
	      msg_data[4] = dac_get_opto_delay();
	      nbytes = 1;  /* Significant bytes < 4 */
	      break;
	    default:
	      /* The sub-index does not exist */
	      sdo_error = SDO_ECODE_ATTRIBUTE;
	      break;
	    }
	}
      else
	{
	  /* The index can not be accessed, does not exist */
	  sdo_error = SDO_ECODE_NONEXISTENT;
	}
      break;

    case OD_SPI_HI:
      switch( od_index_lo )
	{
	case OD_SPI_ACCESS_LO:
	  switch( od_subind )
	    {
	    case OD_NO_OF_ENTRIES:
	      msg_data[4] = 4;
	      nbytes = 1;  /* Significant bytes < 4 */
	      break;
	    case 4:
	      msg_data[7] = spi_read_byte();
	    case 3:
	      msg_data[6] = spi_read_byte();
	    case 2:
	      msg_data[5] = spi_read_byte();
	    case 1:
	      msg_data[4] = spi_read_byte();
	      break;
	    default:
	      /* The sub-index does not exist */
	      sdo_error = SDO_ECODE_ATTRIBUTE;
	      break;
	    }
	  break;

	case OD_SPI_SELECT_LO:
	  msg_data[4] = spi_get_chipselect();
	  nbytes = 1;  /* Significant bytes < 4 */
	  break;

	case OD_SPI_CONFIG_LO:
	  nbytes = 1;  /* Significant bytes < 4 */
	  switch( od_subind )
	    {
	    case OD_NO_OF_ENTRIES:
	      msg_data[4] = 2;
	      break;
	    case 1:
	      msg_data[4] = spi_get_holdtime();
	      break;
	    case 2:
	      msg_data[4] = spi_get_rising_clk();
	      break;
	    default:
	      /* The sub-index does not exist */
	      sdo_error = SDO_ECODE_ATTRIBUTE;
	      break;
	    }
	  break;

	default:
	  /* The index can not be accessed, does not exist */
	  sdo_error = SDO_ECODE_NONEXISTENT;
	  break;
	}
      break;

    case OD_CALC_CRC_HI:
      if( od_index_lo == OD_CALC_CRC_LO )
	{
	  UINT16 crc;
	  BYTE   result;

	  switch( od_subind )
	    {
	    case OD_NO_OF_ENTRIES:
	      msg_data[4] = 3;
	      nbytes = 1;  /* Significant bytes < 4 */
	      break;

	    case OD_CRC_MASTER_FLASH:
	    case OD_CRC_SLAVE_FLASH:
	      if( od_subind == OD_CRC_MASTER_FLASH )
		result = crc_master( &crc );
	      else
		result = crc_slave( &crc );

	      if( result == FALSE )
		{
		  /* Something went wrong... */
		  if( crc == (UINT16) 0 )
		    {
		      /* No CRC found... */
		      sdo_error = SDO_ECODE_ACCESS;
		    }
		  else
		    {
		      /* Access error while reading Master FLASH */
		      sdo_error = SDO_ECODE_HARDWARE;
		    }
		}
	      else
		{
		  nbytes = 2;  /* Significant bytes < 4 */
		  msg_data[4] = (BYTE) (crc & (UINT16) 0x00FF);
		  msg_data[5] = (BYTE) ((crc & (UINT16) 0xFF00) >> 8);
		}
	      break;

	    case OD_CRC_MASTER_FLASH_GET:
	      result = crc_get( &msg_data[4] );
	      if( result == FALSE )
		{
		  /* No CRC found... */
		  sdo_error = SDO_ECODE_ACCESS;
		}
	      nbytes = 2;  /* Significant bytes < 4 */
	      break;

	    default:
	      /* The sub-index does not exist */
	      sdo_error = SDO_ECODE_ATTRIBUTE;
	      break;
	    }
	}
      else
	{
	  /* The index can not be accessed, does not exist */
	  sdo_error = SDO_ECODE_NONEXISTENT;
	}
      break;

    case OD_ELMB_SERIAL_NO_HI:
      if( od_index_lo == OD_ELMB_SERIAL_NO_LO )
	{
	  if( od_subind == 0 )
	    {
	      if( sn_get_serial_number( &msg_data[4] ) == FALSE )
		{
		  /* EEPROM read operation failed
		     or Serial Number simply not present */
		  sdo_error = SDO_ECODE_HARDWARE;
		}
	    }
	  else
	    {
	      /* The sub-index does not exist */
	      sdo_error = SDO_ECODE_ATTRIBUTE;
	    }
	}
      else
	{
	  /* The index can not be accessed, does not exist */
	  sdo_error = SDO_ECODE_NONEXISTENT;
	}
      break;

    case OD_COMPILE_OPTIONS_HI:
      if( od_index_lo == OD_COMPILE_OPTIONS_LO )
	{
	  if( od_subind == 0 )
	    {
#ifdef __ALL_MOTHERBOARDS__
	      msg_data[4] |= 0x01;
#endif
#ifdef __MOTHERBOARD1__
	      msg_data[4] |= 0x02;
#endif
#ifdef __ADC_AVR__
	      msg_data[4] |= 0x08;
#endif
#ifdef __ADC_NONE__
	      msg_data[4] |= 0x10;
#endif
#ifdef __7BIT_NODEID__
	      msg_data[4] |= 0x20;
#endif
#ifdef __RS232__
	      msg_data[4] |= 0x40;
#endif
#ifdef __ELMB103__
	      msg_data[4] |= 0x80;
#endif
#ifdef __VARS_IN_EEPROM__
	      msg_data[5] |= 0x01;
#endif
#ifdef __INCLUDE_TESTS__
	      msg_data[5] |= 0x04;
#endif
#ifdef __CAN_REFRESH__
	      msg_data[5] |= 0x10;
#endif
#ifdef __2313_SLAVE_PRESENT__
	      msg_data[5] |= 0x20;
#endif
	    }
	  else
	    {
	      /* The sub-index does not exist */
	      sdo_error = SDO_ECODE_ATTRIBUTE;
	    }
	}
      else
	{
	  /* The index can not be accessed, does not exist */
	  sdo_error = SDO_ECODE_NONEXISTENT;
	}
      break;

#ifdef __INCLUDE_TESTS__
    case OD_TEST_HI:
      /* Some (self)tests can be performed on I/O and memory, etc... */ 

      if( od_index_lo == OD_TEST_LO )
	{
	  switch( od_subind )
	    {
	    case OD_NO_OF_ENTRIES:
	      /* The number of tests available */
	      msg_data[4] = 2;
	      nbytes = 1;  /* Significant bytes < 4 */
	      break;

	    case OD_IO_TEST:
	      /* Do a predefined test on all available I/O PORTs and PINs;
		 this can be one of the production acceptance tests
		 of the (ELMB +) Motherboard */
	      iotest( &msg_data[4] );
#if !(defined(__ADC_AVR__) || defined(__ADC_NONE__))
	      /* ADC serial interface has been corrupted by I/O-test */
	      adc_serial_init();
#endif
	      break;

	    case OD_WATCHDOG_RESET_TEST:
	      /* The Watchdog Timer should kick in... */
	      while(1)
		{
		  MCUCR |= BIT( SE );
		  SLEEP();
		}
	      break;

	    default:
	      /* The sub-index does not exist */
	      sdo_error = SDO_ECODE_ATTRIBUTE;
	      break;
	    }
	}
      else
	{
	  /* The index can not be accessed, does not exist */
	  sdo_error = SDO_ECODE_NONEXISTENT;
	}
      break;
#endif /* __INCLUDE_TESTS__ */

    default:
      /* The index can not be accessed, does not exist */
      sdo_error = SDO_ECODE_NONEXISTENT;
      break;
    }

  /* Set appropriate SDO command specifier for reply... */
  msg_data[0]  = SDO_INITIATE_UPLOAD_RESP | SDO_EXPEDITED;

  /* ...and segment size (count of non-significant bytes) */
  msg_data[0] |= (SDO_SEGMENT_SIZE_INDICATED |
		  ((4-nbytes) << SDO_DATA_SIZE_SHIFT));

  return sdo_error;
}

/* ------------------------------------------------------------------------ */

static BYTE sdo_expedited_write( BYTE *msg_data )
{
  BYTE sdo_error;
  BYTE sdo_mode, nbytes;
  BYTE od_index_hi, od_index_lo, od_subind;

  /* No error */
  sdo_error   = 0;

  /* Get the number of significant bytes */
  sdo_mode = msg_data[0];
  if( sdo_mode & SDO_DATA_SIZE_INDICATED )
    /* Size indicated */
    nbytes = 4-((sdo_mode & SDO_DATA_SIZE_MASK)>>SDO_DATA_SIZE_SHIFT);
  else
    /* If number of bytes is zero, size was not indicated... */
    nbytes = 0;

  /* Extract Object Dictionary indices */
  od_index_lo = msg_data[1];
  od_index_hi = msg_data[2];
  od_subind   = msg_data[3];

  /* Write the requested object */
  switch( od_index_hi )
    {
    case OD_DIGITAL_OUT_HI:
      switch( od_index_lo )
	{
	case OD_DIGITAL_OUT_8_LO:
	case OD_DIGITAL_OUT_8_MSK_LO:
	  if( od_subind != 0 && od_subind <= digout_port_cnt() )
	    {
	      if( nbytes <= 1 )
		{
		  if( od_index_lo == OD_DIGITAL_OUT_8_LO )
		    digout_set_port( od_subind, msg_data[4] );
		  else
		    {
		      if( od_subind == 2 )
			digout_set_mask( od_subind, msg_data[4] );
		      else
			/* The sub-index cannot be written to */
			sdo_error = SDO_ECODE_ACCESS;
		    }
		}
	      else
		{
		  /* Wrong number of bytes provided */
		  sdo_error = SDO_ECODE_ATTRIBUTE;
		}
	    }
	  else
	    {
	      /* The sub-index does not exist */
	      sdo_error = SDO_ECODE_ATTRIBUTE;
	    }
	  break;

	default:
	  /* The index can not be accessed, does not exist */
	  sdo_error = SDO_ECODE_NONEXISTENT;
	  break;
	}
      break;

    case OD_ANALOG_HI:
      switch( od_index_lo )
	{
	case OD_ANALOG_OUT_16_LO:
	  if( od_subind != 0 && od_subind <= dac_chan_cnt() )
	    {
	      if( nbytes == 2 || nbytes == 0 )
		{
		  if( dac_write( od_subind-1, &msg_data[4] ) == FALSE )
		    {
		      /* Something wrong with parameters */
		      sdo_error = SDO_ECODE_ATTRIBUTE;
		    }
		}
	      else
		{
		  /* Wrong number of bytes provided */
		  sdo_error = SDO_ECODE_ATTRIBUTE;
		}
	    }
	  else
	    {
	      /* The sub-index does not exist */
	      sdo_error = SDO_ECODE_ATTRIBUTE;
	    }
	  break;

	case OD_ANALOG_IN_DELTA_LO:
	case OD_ANALOG_IN_UPPER_LO:
	case OD_ANALOG_IN_LOWER_LO:
	  if( od_subind != 0 &&
	      (od_subind <= ADC_MAX_INPUTS || od_subind == 0xFF) )
	    {
	      if( nbytes == 4 || nbytes == 0 )
		{
		  switch( od_index_lo )
		    {
		    case OD_ANALOG_IN_DELTA_LO:
		      if( adc_set_delta( od_subind-1, &msg_data[4] ) == FALSE )
			{
			  /* EEPROM write operation failed */
			  sdo_error = SDO_ECODE_HARDWARE;
			}
		      break;
		    case OD_ANALOG_IN_UPPER_LO:
		      if( adc_set_upperlimit( od_subind-1, &msg_data[4] ) ==
			  FALSE )
			{
			  /* EEPROM write operation failed */
			  sdo_error = SDO_ECODE_HARDWARE;
			}
		      break;
		    case OD_ANALOG_IN_LOWER_LO:
		      if( adc_set_lowerlimit( od_subind-1, &msg_data[4] ) ==
			  FALSE )
			{
			  /* EEPROM write operation failed */
			  sdo_error = SDO_ECODE_HARDWARE;
			}
		      break;
		    default:
		      /* The sub-index does not exist */
		      sdo_error = SDO_ECODE_ATTRIBUTE;
		      break;
		    }
		}
	      else
		{
		  /* Wrong number of bytes provided */
		  sdo_error = SDO_ECODE_ATTRIBUTE;
		}
	    }
	  else
	    {
	      /* The sub-index does not exist */
	      sdo_error = SDO_ECODE_ATTRIBUTE;
	    }
	  break;

	case OD_ANALOG_IN_INT_ENA_LO:
	  if( od_subind == 0 )
	    {
	      if( nbytes <= 1 )
		{
		  if( adc_set_readout_on_change( msg_data[4] ) == FALSE )
		    sdo_error = SDO_ECODE_ATTRIBUTE;
		}
	      else
		{
		  /* Wrong number of bytes provided */
		  sdo_error = SDO_ECODE_ATTRIBUTE;
		}
	    }
	  else
	    {
	      /* The sub-index does not exist */
	      sdo_error = SDO_ECODE_ATTRIBUTE;
	    }
	  break;

	default:
	  /* The index can not be accessed, does not exist */
	  sdo_error = SDO_ECODE_NONEXISTENT;
	  break;
	}
      break;

    case OD_ADC_CONFIG_HI:
      switch( od_index_lo )
	{
	case OD_ADC_CONFIG_LO:
	  if( adc_set_config( od_subind, nbytes, &msg_data[4] ) == FALSE )
	    {
	      /* Setting the parameter did not succeed:
		 - the sub-index does not exist or
		 - the parameter is read-only or
		 - the number of bytes is incorrect or
		 - parameter value is out of range or
		 - parameter can not be set: ADC in use */
	      sdo_error = SDO_ECODE_ATTRIBUTE;
	    }
	  break;

	case OD_ADC_RESET_CALIB_LO:
	  if( od_subind == 0 )
	    {
	      if( nbytes <= 1 )
		{
		  if( adc_reset_and_calibrate( TRUE ) == FALSE )
		    {
		      /* Reset or calibrate operation failed */
		      sdo_error = SDO_ECODE_HARDWARE;
		    }
		}
	      else
		{
		  /* Wrong number of bytes provided */
		  sdo_error = SDO_ECODE_ATTRIBUTE;
		}
	    }
	  else
	    {
	      /* The sub-index does not exist */
	      sdo_error = SDO_ECODE_ATTRIBUTE;
	    }
	  break;

	case OD_ADC_RECALIB_SCAN_LO:
	  if( od_subind == 0 )
	    {
	      if( nbytes <= 1 )
		{
		  if( adc_set_calib_before_scan( msg_data[4] ) == FALSE )
		    sdo_error = SDO_ECODE_ATTRIBUTE;
		}
	      else
		{
		  /* Wrong number of bytes provided */
		  sdo_error = SDO_ECODE_ATTRIBUTE;
		}
	    }
	  else
	    {
	      /* The sub-index does not exist */
	      sdo_error = SDO_ECODE_ATTRIBUTE;
	    }
	  break;

	case OD_ADC_DELTA_ENA_LO:
	  if( od_subind == 0 )
	    {
	      if( nbytes <= 1 )
		{
		  if( adc_set_delta_scan_ena( msg_data[4] ) == FALSE )
		    sdo_error = SDO_ECODE_ATTRIBUTE;
		}
	      else
		{
		  /* Wrong number of bytes provided */
		  sdo_error = SDO_ECODE_ATTRIBUTE;
		}
	    }
	  else
	    {
	      /* The sub-index does not exist */
	      sdo_error = SDO_ECODE_ATTRIBUTE;
	    }
	  break;

	case OD_ADC_WINDOW_ENA_LO:
	  if( od_subind == 0 )
	    {
	      if( nbytes <= 1 )
		{
		  if( adc_set_window_scan_ena( msg_data[4] ) == FALSE )
		    sdo_error = SDO_ECODE_ATTRIBUTE;
		}
	      else
		{
		  /* Wrong number of bytes provided */
		  sdo_error = SDO_ECODE_ATTRIBUTE;
		}
	    }
	  else
	    {
	      /* The sub-index does not exist */
	      sdo_error = SDO_ECODE_ATTRIBUTE;
	    }
	  break;

	case OD_ADC_WINDOW_CNTR_LO:
	  if( od_subind == 0 )
	    {
	      if( nbytes <= 1 )
		{
		  if( adc_set_window_scan_cntr( msg_data[4] ) == FALSE )
		    sdo_error = SDO_ECODE_ATTRIBUTE;
		}
	      else
		{
		  /* Wrong number of bytes provided */
		  sdo_error = SDO_ECODE_ATTRIBUTE;
		}
	    }
	  else
	    {
	      /* The sub-index does not exist */
	      sdo_error = SDO_ECODE_ATTRIBUTE;
	    }
	  break;


	default:
	  /* The index can not be accessed, does not exist */
	  sdo_error = SDO_ECODE_NONEXISTENT;
	  break;
	}
      break;

    case OD_TPDO_PAR_HI:
      if( od_index_lo < TPDO_CNT )
	{
	  if( tpdo_set_comm_par( od_index_lo, od_subind,
				 nbytes, &msg_data[4] ) == FALSE )
	    {
	      /* The subindex does not exist or the number of bytes
		 is incorrect or the parameter could not be written */
	      sdo_error = SDO_ECODE_ATTRIBUTE;
	    }
	}
      else
	{
	  /* The index can not be accessed, does not exist */
	  sdo_error = SDO_ECODE_NONEXISTENT;
	}
      break;

    case OD_DIGITAL_IN_HI:
      switch( od_index_lo )
	{
	case OD_DIGIN_INTRPT_ENA_LO:
	  if( od_subind == 0 )
	    {
	      if( nbytes <= 1 )
		{
		  if( digin_set_intrpt_ena( msg_data[4] ) == FALSE )
		    sdo_error = SDO_ECODE_ATTRIBUTE;
		}
	      else
		{
		  /* Wrong number of bytes provided */
		  sdo_error = SDO_ECODE_ATTRIBUTE;
		}
	    }
	  else
	    {
	      /* The sub-index does not exist */
	      sdo_error = SDO_ECODE_ATTRIBUTE;
	    }
	  break;

	case OD_DIGIN_INTRPT_MSK_LO:
	  if( od_subind != 0 && od_subind <= digin_port_cnt() )
	    {
	      if( nbytes <= 1 )
		{
		  digin_set_intrpt_mask( od_subind, msg_data[4] );
		}
	      else
		{
		  /* Wrong number of bytes provided */
		  sdo_error = SDO_ECODE_ATTRIBUTE;
		}
	    }
	  else
	    {
	      /* The sub-index does not exist */
	      sdo_error = SDO_ECODE_ATTRIBUTE;
	    }
	  break;

	default:
	  /* The index can not be accessed, does not exist */
	  sdo_error = SDO_ECODE_NONEXISTENT;
	  break;
	}
      break;

    case OD_DIGIN_PAR_HI:
      if( od_index_lo == OD_DIGIN_DEBOUNCE_LO )
	{
	  if( od_subind == 0 )
	    {
	      if( nbytes <= 1 )
		{
		  digin_set_debounce( msg_data[4] );
		}
	      else
		{
		  /* Wrong number of bytes provided */
		  sdo_error = SDO_ECODE_ATTRIBUTE;
		}
	    }
	  else
	    {
	      /* The sub-index does not exist */
	      sdo_error = SDO_ECODE_ATTRIBUTE;
	    }
	}
      else
	{
	  /* The index can not be accessed, does not exist */
	  sdo_error = SDO_ECODE_NONEXISTENT;
	}
      break;

    case OD_DIGOUT_PAR_HI:
      if( od_index_lo == OD_DIGOUT_INIT_LO )
	{
	  if( od_subind == 0 )
	    {
	      if( nbytes <= 1 )
		{
		  if( digout_set_init( msg_data[4] ) == FALSE )
		    sdo_error = SDO_ECODE_ATTRIBUTE;
		}
	      else
		{
		  /* Wrong number of bytes provided */
		  sdo_error = SDO_ECODE_ATTRIBUTE;
		}
	    }
	  else
	    {
	      /* The sub-index does not exist */
	      sdo_error = SDO_ECODE_ATTRIBUTE;
	    }
	}
      else
	{
	  /* The index can not be accessed, does not exist */
	  sdo_error = SDO_ECODE_NONEXISTENT;
	}
      break;

    case OD_DAC_CONFIG_HI:
      if( od_index_lo == OD_DAC_CONFIG_LO )
	{
	  switch( od_subind )
	    {
	    case 2:
	      if( nbytes <= 1 )
		{
		  if( dac_set_max525_select( msg_data[4] ) == FALSE )
		    sdo_error = SDO_ECODE_ATTRIBUTE;
		}
	      else
		{
		  /* Wrong number of bytes provided */
		  sdo_error = SDO_ECODE_ATTRIBUTE;
		}
	      break;
	    case 3:
	      if( nbytes <= 1 )
		{
		  if( dac_set_opto_delay( msg_data[4] ) == FALSE )
		    sdo_error = SDO_ECODE_ATTRIBUTE;
		}
	      else
		{
		  /* Wrong number of bytes provided */
		  sdo_error = SDO_ECODE_ATTRIBUTE;
		}
	      break;
	    default:
	      /* The sub-index does not exist */
	      sdo_error = SDO_ECODE_ATTRIBUTE;
	      break;
	    }
	}
      else
	{
	  /* The index can not be accessed, does not exist */
	  sdo_error = SDO_ECODE_NONEXISTENT;
	}
      break;

    case OD_SPI_HI:
      switch( od_index_lo )
	{
	case OD_SPI_ACCESS_LO:
	  if( od_subind > OD_NO_OF_ENTRIES && od_subind < 5 )
	    {
	      if( nbytes == 4 || nbytes == 0 )
		{
		  switch( od_subind )
		    {
		    case 4:
		      spi_write_byte( msg_data[7] );
		    case 3:
		      spi_write_byte( msg_data[6] );
		    case 2:
		      spi_write_byte( msg_data[5] );
		    case 1:
		      spi_write_byte( msg_data[4] );
		      break;
		    }
		}
	      else
		{
		  /* Wrong number of bytes provided */
		  sdo_error = SDO_ECODE_ATTRIBUTE;
		}
	    }
	  else
	    {
	      /* The sub-index does not exist */
	      sdo_error = SDO_ECODE_ATTRIBUTE;
	    }
	  break;

	case OD_SPI_SELECT_LO:
	  if( od_subind == 0 )
	    {
	      if( nbytes <= 1 )
		{
		  if( spi_set_chipselect( msg_data[4] ) == FALSE )
		    sdo_error = SDO_ECODE_ATTRIBUTE;
		}
	      else
		{
		  /* Wrong number of bytes provided */
		  sdo_error = SDO_ECODE_ATTRIBUTE;
		}
	    }
	  else
	    {
	      /* The sub-index does not exist */
	      sdo_error = SDO_ECODE_ATTRIBUTE;
	    }
	  break;

	case OD_SPI_CONFIG_LO:
	  switch( od_subind )
	    {
	    case 1:
	      if( nbytes <= 1 )
		{
		  if( spi_set_holdtime( msg_data[4] ) == FALSE )
		    sdo_error = SDO_ECODE_ATTRIBUTE;
		}
	      else
		{
		  /* Wrong number of bytes provided */
		  sdo_error = SDO_ECODE_ATTRIBUTE;
		}
	      break;
	    case 2:
	      if( nbytes <= 1 )
		{
		  if( spi_set_rising_clk( msg_data[4] ) == FALSE )
		    sdo_error = SDO_ECODE_ATTRIBUTE;
		}
	      else
		{
		  /* Wrong number of bytes provided */
		  sdo_error = SDO_ECODE_ATTRIBUTE;
		}
	      break;
	    default:
	      /* The sub-index does not exist */
	      sdo_error = SDO_ECODE_ATTRIBUTE;
	      break;
	    }
	  break;

	default:
	  /* The index can not be accessed, does not exist */
	  sdo_error = SDO_ECODE_NONEXISTENT;
	  break;
	}
      break;

#ifdef __2313_SLAVE_PRESENT__
    case OD_PROGRAM_CODE_HI:
      if( od_index_lo == OD_PROGRAM_CODE_LO )
	{
	  if( od_subind == 1 )
	    {
	      if( nbytes == 4 || nbytes == 0 )
		{
		  if( do_serial_instruction( &msg_data[4] ) == FALSE )
		    {
		      /* Something wrong with parameters */
		      sdo_error = SDO_ECODE_ATTRIBUTE;
		    }
		}
	      else
		{
		  /* Wrong number of bytes provided */
		  sdo_error = SDO_ECODE_ATTRIBUTE;
		}
	    }
	  else
	    {
	      /* The sub-index does not exist */
	      sdo_error = SDO_ECODE_ATTRIBUTE;
	    }
	}
      else
	{
	  /* The index can not be accessed, does not exist */
	  sdo_error = SDO_ECODE_NONEXISTENT;
	}
      break;
#endif /* __2313_SLAVE_PRESENT__ */

    case OD_SWITCH_TO_LOADER_HI:
      if( od_index_lo == OD_SWITCH_TO_LOADER_LO )
	{
	  if( od_subind == 0 )
	    {
	      if( nbytes <= 1 )
		{
#ifdef __2313_SLAVE_PRESENT__
		  /* Disable Timer1 interrupt to stop
		     the Slave aliveness-check mechanism:
		     Slave should take control of the node,
		     after some time, unless.... */
		  timer1_stop();
#endif /* __2313_SLAVE_PRESENT__ */

		  /* Send a reply before making the jump... */
		  msg_data[0] = SDO_INITIATE_DOWNLOAD_RESP;
		  msg_data[4] = 0;
		  can_write( C91_SDOTX, C91_SDOTX_LEN, msg_data );
		  timer2_delay_ms( 5 );

		  /* ...there is a Bootloader: it will take control
		     (and also keep the Slave happy, if present) */
		  jump_to_bootloader();
		}
	      else
		{
		  /* Wrong number of bytes provided */
		  sdo_error = SDO_ECODE_ATTRIBUTE;
		}
	    }
	  else
	    {
	      /* The sub-index does not exist */
	      sdo_error = SDO_ECODE_ATTRIBUTE;
	    }
	}
      else
	{
	  /* The index can not be accessed, does not exist */
	  sdo_error = SDO_ECODE_NONEXISTENT;
	}
      break;

    case OD_DEVICE_INFO_HI:
      switch( od_index_lo )
	{
	case OD_LIFETIME_FACTOR_LO:
	  if( od_subind == 0 )
	    {
	      if( nbytes <= 1 )
		{
		  /* Set new Life Time Factor */
		  if( guarding_set_lifetime( msg_data[4] ) == FALSE )
		    sdo_error = SDO_ECODE_ATTRIBUTE;
		}
	      else
		{
		  /* Wrong number of bytes provided */
		  sdo_error = SDO_ECODE_ATTRIBUTE;
		}
	    }
	  else
	    {
	      /* The sub-index does not exist */
	      sdo_error = SDO_ECODE_ATTRIBUTE;
	    }
	  break;

	case OD_STORE_PARAMETERS_LO:
	  if( od_subind >= 1 && od_subind <= OD_STORE_MAX_SUBID )
	    {
	      if( nbytes == 4 || nbytes == 0 )
		{
		  /* Check for correct signature */
		  if( msg_data[4] == 's' && msg_data[5] == 'a' &&
		      msg_data[6] == 'v' && msg_data[7] == 'e' )
		    {
		      if( store_save_parameters( od_subind ) == FALSE )
			{
			  /* Something went wrong */
			  sdo_error = SDO_ECODE_HARDWARE;
			}
		    }
		}
	      else
		{
		  /* Wrong number of bytes provided */
		  sdo_error = SDO_ECODE_ATTRIBUTE;
		}
	    }
	  else
	    {
	      /* The sub-index does not exist */
	      sdo_error = SDO_ECODE_ATTRIBUTE;
	    }
	  break;

	case OD_DFLT_PARAMETERS_LO:
	  if( od_subind >= 1 && od_subind <= OD_STORE_MAX_SUBID )
	    {
	      if( nbytes == 4 || nbytes == 0 )
		{
		  /* Check for correct signature */
		  if( msg_data[4] == 'l' && msg_data[5] == 'o' &&
		      msg_data[6] == 'a' && msg_data[7] == 'd' )
		    {
		      if( store_set_defaults( od_subind ) == FALSE )
			{
			  /* Something went wrong */
			  sdo_error = SDO_ECODE_HARDWARE;
			}
		    }
		}
	      else
		{
		  /* Wrong number of bytes provided */
		  sdo_error = SDO_ECODE_ATTRIBUTE;
		}
	    }
	  else
	    {
	      /* The sub-index does not exist */
	      sdo_error = SDO_ECODE_ATTRIBUTE;
	    }
	  break;

	case OD_HEARTBEAT_TIME_LO:
	  if( od_subind == 0 )
	    {
	      if( nbytes == 2 || nbytes == 0 )
		{
		  /* Set new Heartbeat Time */
		  if( guarding_set_heartbeattime( &msg_data[4] ) == FALSE )
		    sdo_error = SDO_ECODE_ATTRIBUTE;
		}
	      else
		{
		  /* Wrong number of bytes provided */
		  sdo_error = SDO_ECODE_ATTRIBUTE;
		}
	    }
	  else
	    {
	      /* The sub-index does not exist */
	      sdo_error = SDO_ECODE_ATTRIBUTE;
	    }
	  break;

	default:
	  /* The index can not be accessed, does not exist */
	  sdo_error = SDO_ECODE_NONEXISTENT;
	  break;
	}
      break;

    case OD_CAN_CONFIG_HI:
      if( od_index_lo == OD_CAN_CONFIG_LO )
	{
	  switch( od_subind )
	    {
	    case 1:
	      if( nbytes <= 1 )
		{
		  if( can_set_rtr_disabled( msg_data[4] ) == FALSE )
		    sdo_error = SDO_ECODE_ATTRIBUTE;
		}
	      else
		/* Wrong number of bytes provided */
		sdo_error = SDO_ECODE_ATTRIBUTE;
	      break;
	    case 2:
	      if( nbytes <= 1 )
		{
		  if( can_set_opstate_init( msg_data[4] ) == FALSE )
		    sdo_error = SDO_ECODE_ATTRIBUTE;
		}
	      else
		/* Wrong number of bytes provided */
		sdo_error = SDO_ECODE_ATTRIBUTE;
	      break;
	    case 3:
	      if( nbytes <= 1 )
		{
		  if( can_set_busoff_maxcnt( msg_data[4] ) == FALSE )
		    sdo_error = SDO_ECODE_ATTRIBUTE;
		}
	      else
		/* Wrong number of bytes provided */
		sdo_error = SDO_ECODE_ATTRIBUTE;
	      break;
	    default:
	      /* The sub-index does not exist */
	      sdo_error = SDO_ECODE_ATTRIBUTE;
	      break;
	    }
	}
      else
	{
	  /* The index can not be accessed, does not exist */
	  sdo_error = SDO_ECODE_NONEXISTENT;
	  break;
	}
      break;

    case OD_ADC_CALIB_RANGE_HI:
      if( od_index_lo == OD_ADC_CALIB_RANGE_LO )
	{
	  /* There are six voltage ranges on this ADC */
	  if( od_subind != 0 && od_subind <= STORE_ADC_CALIB_BLOCKS )
	    {
	      if( nbytes == 4 || nbytes == 0 )
		{
		  if( adc_calibrate_range( od_subind-1 ) == FALSE )
		    {
		      /* Reset or self-calibrate operation failed */
		      sdo_error = SDO_ECODE_HARDWARE;
		    }
		}
	      else
		{
		  /* Wrong number of bytes provided */
		  sdo_error = SDO_ECODE_ATTRIBUTE;
		}
	    }
	  else
	    {
	      /* The sub-index does not exist */
	      sdo_error = SDO_ECODE_ATTRIBUTE;
	    }
	}
      else
	{
	  /* The index can not be accessed, does not exist */
	  sdo_error = SDO_ECODE_NONEXISTENT;
	}
      break;

    case OD_ADC_CALIB_PARS_HI:
      if( od_index_lo < STORE_ADC_CALIB_BLOCKS )
	{
	  /* Check for valid parameters */
	  if( od_subind != 0 && od_subind <= STORE_ADC_CALIB_PARS )
	    {
	      if( nbytes == 4 || nbytes == 0 )
		{
		  if( adc_set_calib_const( od_index_lo, od_subind-1,
					   &msg_data[4] ) == FALSE )
		    {
		      /* Something went wrong while writing to EEPROM */
		      sdo_error = SDO_ECODE_HARDWARE;
		    }
		}
	      else
		{
		  /* Wrong number of bytes provided */
		  sdo_error = SDO_ECODE_ATTRIBUTE;
		}
	    }
	  else
	    {
	      /* The sub-index does not exist */
	      sdo_error = SDO_ECODE_ATTRIBUTE;
	    }
	}
      else
	{
	  /* The index can not be accessed, does not exist */
	  sdo_error = SDO_ECODE_NONEXISTENT;
	}
      break;

    case OD_ADC_CALIB_ERASE_HI:
      if( od_index_lo < STORE_ADC_CALIB_BLOCKS )
	{
	  /* Check for valid parameters */
	  if( od_subind == 0 )
	    {
	      if( nbytes <= 1 )
		{
		  if( adc_erase_calib_const( od_index_lo,
					     msg_data[4] ) == FALSE )
		    {
		      /* Something went wrong while writing to EEPROM */
		      sdo_error = SDO_ECODE_HARDWARE;
		    }
		}
	      else
		{
		  /* Wrong number of bytes provided */
		  sdo_error = SDO_ECODE_ATTRIBUTE;
		}
	    }
	  else
	    {
	      /* The sub-index does not exist */
	      sdo_error = SDO_ECODE_ATTRIBUTE;
	    }
	}
      else
	{
	  /* The index can not be accessed, does not exist */
	  sdo_error = SDO_ECODE_NONEXISTENT;
	}
      break;

    case OD_ADC_CALIB_WR_ENA_HI:
      if( od_index_lo == OD_ADC_CALIB_WR_ENA_LO )
	{
	  if( od_subind == 0 )
	    {
	      if( nbytes <= 1 )
		{
		  if( adc_calib_const_write_enable( msg_data[4] ) == FALSE )
		    {
		      /* Something wrong with parameters */
		      sdo_error = SDO_ECODE_ATTRIBUTE;
		    }
		}
	      else
		{
		  /* Wrong number of bytes provided */
		  sdo_error = SDO_ECODE_ATTRIBUTE;
		}
	    }
	  else
	    {
	      /* The sub-index does not exist */
	      sdo_error = SDO_ECODE_ATTRIBUTE;
	    }
	}
      else
	{
	  /* The index can not be accessed, does not exist */
	  sdo_error = SDO_ECODE_NONEXISTENT;
	}
      break;

    case OD_ELMB_SERIAL_NO_HI:
      switch( od_index_lo )
	{
	case OD_ELMB_SERIAL_NO_LO:
	  if( od_subind == 0 )
	    {
	      if( nbytes == 4 || nbytes == 0 )
		{
		  /* Set the ELMB Serial Number */
		  if( sn_set_serial_number( &msg_data[4] ) == FALSE )
		    {
		      /* Something went wrong */
		      sdo_error = SDO_ECODE_HARDWARE;
		    }
		}
	      else
		{
		  /* Wrong number of bytes provided */
		  sdo_error = SDO_ECODE_ATTRIBUTE;
		}
	    }
	  else
	    {
	      /* The sub-index does not exist */
	      sdo_error = SDO_ECODE_ATTRIBUTE;
	    }
	  break;

	case OD_ELMB_SN_WRITE_ENA_LO:
	  if( od_subind == 0 )
	    {
	      if( nbytes <= 1 )
		{
		  /* Enable a write-operation to the ELMB Serial Number */
		  if( sn_serial_number_write_enable( msg_data[4] ) == FALSE )
		    {
		      /* Something wrong with parameters */
		      sdo_error = SDO_ECODE_ATTRIBUTE;
		    }
		}
	      else
		{
		  /* Wrong number of bytes provided */
		  sdo_error = SDO_ECODE_ATTRIBUTE;
		}
	    }
	  else
	    {
	      /* The sub-index does not exist */
	      sdo_error = SDO_ECODE_ATTRIBUTE;
	    }
	  break;

	default:
	  /* The index can not be accessed, does not exist */
	  sdo_error = SDO_ECODE_NONEXISTENT;
	  break;
	}
      break;

    default:
      /* The index can not be accessed, does not exist */
      sdo_error = SDO_ECODE_NONEXISTENT;
      break;
    }

  /* Set appropriate SDO command specifier for reply */
  msg_data[0] = SDO_INITIATE_DOWNLOAD_RESP;

  /* CANopen: bytes 4 to 7 reserved, so set to zero, except when programming
     the Slave: the SDO reply possibly contains a read memory byte... */
  if( od_index_hi != OD_PROGRAM_CODE_HI )
    {
      msg_data[4] = 0;
      msg_data[5] = 0;
      msg_data[6] = 0;
      msg_data[7] = 0;
    }

  return sdo_error;
}

/* ------------------------------------------------------------------------ */

static void sdo_abort( BYTE error_class,
		       BYTE error_code,
		       BYTE *msg_data )
{
  msg_data[0] = SDO_ABORT_TRANSFER;

  /* msg_data[1], msg_data[2], msg_data[3] should contain
     index and sub-index: leave intact */

  /* Error class */
  msg_data[7] = error_class;

  /* Error code */
  msg_data[6] = error_code;

  /* Additional code, not filled in for the time being */
  msg_data[5] = 0;
  msg_data[4] = 0;

  can_write( C91_SDOTX, C91_SDOTX_LEN, msg_data );
}

/* ------------------------------------------------------------------------ */
/* Function call which results in a jump to address 0xF000
   which starts the Bootloader program
   (provided the Bootloader size is set (by the fuses) to 4 kWords!) */

static void jump_to_bootloader( void )
{
#ifndef __ELMB103__
  BYTE flashbyte;

  /* Set (byte) address in the proper registers (for ELPM access) */
  asm( "ldi R30,0x00" );
  asm( "ldi R31,0xE0" );

  /* Set RAMPZ register to access the upper 64k page of program memory */
  RAMPZ = 1;

  /* Read the program memory byte and store it in 'flashbyte' */
  asm( "elpm" );
  asm( "mov %flashbyte, R0" );

  /* Reset RAMPZ register */
  RAMPZ = 0;

  /* If there is no Bootloader, return to the user application ! */
  if( flashbyte == 0xFF )
    {
      /* CANopen Error Code 0x6000: device software */
      can_write_emergency( 0x00, 0x50, EMG_NO_BOOTLOADER,
			   0, 0, 0, ERRREG_MANUFACTURER );
      return;
    }
  
  /* Disable watchdog timer (if possible) */
  watchdog_disable();

  /* Disable all interrupts */
  CLI();

  /* Z-pointer: 0xF000 (word address) */
  asm( "ldi R30,0x00" );
  asm( "ldi R31,0xF0" );
  
  /* Jump to the Bootloader at (word) address 0xF000 */
  asm( "ijmp" );
#endif /* __ELMB103__ */
}

/* ------------------------------------------------------------------------ */
