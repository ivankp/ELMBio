/* ------------------------------------------------------------------------
File   : cs5523.c

Descr  : Functions for operating a CRYSTAL CS5523 (CS5524) 4-chan 16-bit
         (24-bit) ADC with SPI serial interface.

History: 01JUN.00; Henk B&B; Start of development, based on routines for
                             a CS5525 ADC.
	 22SEP.00; Henk B&B; In cs5523_write_csr() assume Depth Pointer
	                     is by default set to 1 (which is the case
			     if a first call is made to it for a write to
			     a depth of 8 (to be done by the ADC init routine).
	 29NOV.00; Henk B&B; BUG FIX: in cs5523_reset() used Timer2 twice
	                     at the same time, in a timeout and for reading
			     out the CS5523 registers, resulting in a faulty
			     timeout causing a hang situation (e.g. when
			     CS5523 not present).
	 06AUG.01; Henk B&B; Replaced 2 functions by macros.
--------------------------------------------------------------------------- */

#include "general.h"
#include "adc.h"
#include "cs5523.h"
#include "timer103.h"

extern BYTE AdcOptoDelay;

/* ------------------------------------------------------------------------ */

void cs5523_read_reg( BYTE reg, BYTE ain, BYTE *regdata )
{
  /* Make sure to address a valid register */
  reg  &= CS23_CMD_REG_SELECT_MASK;
  ain <<= CS23_CMD_PHYSCHAN_SELECT_SHIFT;
  ain  &= CS23_CMD_PHYSCHAN_SELECT_MASK;

  /* Write the 'read register' command */
  cs5523_select_register_cmd( CS23_CMD_READ | reg | ain );

  /* Read 24 bits (3 bytes) of data, MSB first */

  /* Store Most Significant Byte in highest byte */
  regdata += 2;
  *regdata = cs5523_read_byte();
  --regdata;
  *regdata = cs5523_read_byte();
  --regdata;
  *regdata = cs5523_read_byte();
}

/* ------------------------------------------------------------------------ */

void cs5523_write_reg( BYTE reg, BYTE ain, BYTE *regdata )
{
  /* Make sure to address a valid register */
  reg  &= CS23_CMD_REG_SELECT_MASK;
  ain <<= CS23_CMD_PHYSCHAN_SELECT_SHIFT;
  ain  &= CS23_CMD_PHYSCHAN_SELECT_MASK;

  /* Write the 'write register' command */
  cs5523_select_register_cmd( CS23_CMD_WRITE | reg | ain );

  /* Write 24 bits (3 bytes) of data, MSB first */

  /* Get Most Significant Byte in highest byte */
  regdata += 2;
  cs5523_write_byte( *regdata );
  --regdata;
  cs5523_write_byte( *regdata );
  --regdata;
  cs5523_write_byte( *regdata );
}

/* ------------------------------------------------------------------------ */

void cs5523_read_csr( BYTE *csrdata )
{
  /* Read all 4 Channel-Setup Registers (data for 8 Logical Channels)
     without changing other Configuration Register settings;
     use 2 bytes per Logical Channel (so need 16 for 8 LCs) */
  BYTE config[3];
  BYTE lc;

  /* Read the current Configuration Register setting */
  cs5523_read_reg( CS23_CMD_CONFIG_REG, 0, config );

  /* Set the Depth Pointer to 7 (8 Logical Channel settings to read) */
  config[1] |= (7 << CS23_CNF_CSR_DEPTH_SHIFT);

  /* Write the Configuration Register setting */
  cs5523_write_reg( CS23_CMD_CONFIG_REG, 0, config );

  /* Now write the 'read CSR register' command */
  cs5523_select_register_cmd( CS23_CMD_READ | CS23_CMD_CHAN_SETUP_REG );

  /* Read 4*24=96 bits of data, MSB first, in chunks of 4 and 8 bits,
     each chunk stored in 1 byte, stored LSB first */
  for( lc=0; lc<8; ++lc )
    {
      csrdata[lc*2+1] = cs5523_read_nibble(); /* A1,A0,NU,CS1 */
      csrdata[lc*2]   = cs5523_read_byte();   /* the lower 8 bits */
    }

  /* Reset the Depth Pointer to 1 */
  config[1] &= ~CS23_CNF_CSR_DEPTH_MASK;
  config[1] |= (1 << CS23_CNF_CSR_DEPTH_SHIFT);

  /* Write the Configuration Register setting */
  cs5523_write_reg( CS23_CMD_CONFIG_REG, 0, config );
}

/* ------------------------------------------------------------------------ */

void cs5523_write_csr( BYTE no_of_csr, BYTE *csrdata )
{
  /* Write 1 to 4 Channel-Setup Registers (data for up to 8 Logical Channels)
     without changing other Configuration Register settings;
     use 2 bytes per Logical Channel (so need 16 for 8 LCs) */
  BYTE config[3];
  BYTE no_of_lc, lc;

  if( no_of_csr > 4 )  no_of_csr = 4;
  if( no_of_csr == 0 ) no_of_csr = 1;

  no_of_lc = no_of_csr * 2;

  /* Assume everything is already properly set for writing 1 CSR... */
  if( no_of_csr > 1 )
    {
      /* Read the current Configuration Register setting */
      cs5523_read_reg( CS23_CMD_CONFIG_REG, 0, config );

      /* Set the Depth Pointer */
      config[1] &= ~CS23_CNF_CSR_DEPTH_MASK;
      config[1] |= ((no_of_lc-1) << CS23_CNF_CSR_DEPTH_SHIFT);

      /* Write the Configuration Register setting */
      cs5523_write_reg( CS23_CMD_CONFIG_REG, 0, config );
    }

  /* Now write the 'write CSR register' command */
  cs5523_select_register_cmd( CS23_CMD_WRITE | CS23_CMD_CHAN_SETUP_REG );

  /* Write up to 4*24=96 bits of data, MSB first, in chunks of 4 and 8 bits,
     each chunk stored in 1 byte, stored LSB first */
  for( lc=0; lc<no_of_lc; ++lc )
    {
      cs5523_write_nibble( csrdata[lc*2+1] ); /* A1,A0,NU,CS1 */
      cs5523_write_byte( csrdata[lc*2] );     /* the lower 8 bits */
    }

  /* Assume everything was properly set for writing 1 CSR... */
  if( no_of_csr > 1 )
    {
      /* Reset the Depth Pointer to 1 */
      config[1] &= ~CS23_CNF_CSR_DEPTH_MASK;
      config[1] |= (1 << CS23_CNF_CSR_DEPTH_SHIFT);

      /* Write the Configuration Register setting */
      cs5523_write_reg( CS23_CMD_CONFIG_REG, 0, config );
    }
}

/* ------------------------------------------------------------------------ */

BOOL cs5523_read_adc( BYTE log_chan, BYTE *conversion_data )
{
  /* Make sure to use a valid Logical Channel */
  log_chan = (log_chan << CS23_CMD_LOGCHAN_SELECT_SHIFT) &
             CS23_CMD_LOGCHAN_SELECT_MASK;

  /* Initiate the conversion */
  cs5523_start_conversion_cmd( CS23_CMD_NORMAL_CONVERSION | log_chan );

  /* Wait for SDO to go low flagging the conversion is done,
     but use a timeout (ca. 600 ms, sufficient at 1.88 Hz wordrate) */
  //if( cs5523_await_sdo_low( 60 ) )
  if( cs5523_await_sdo_low( 120 ) ) // Make that 1200 ms...
    {
      /* Generate 8 SCLKs to clear the SDO flag */
      cs5523_write_byte( 0 );

      /* Read 24 bits (3 bytes) of conversion data */
      conversion_data[2] = cs5523_read_byte();
      conversion_data[1] = cs5523_read_byte();
      conversion_data[0] = cs5523_read_byte();
      return TRUE;
    }
  else
    {
      /* Time out ! Meaning there was a time-out on conversion-done... */
      conversion_data[2] = 0xFF;
      conversion_data[1] = 0xFF;
      conversion_data[0] = 0xFE;
      return FALSE;
    }
}

/* ------------------------------------------------------------------------ */

BOOL cs5523_calibrate( BYTE calib_type, BYTE adc_config )
{
  /* Perform the requested calibration on all 4 physical channels */

  BYTE ain;
  BYTE csr_data[4];

  /* Dummy data for LC 2... */
  csr_data[2] = 0;
  csr_data[3] = 0;

  /* Always use 15 Hz calibration wordrate */
  adc_config &= ~CS23_CSR_WORDRATE_MASK;
  adc_config |= (CS23_WORDRATE_15 << CS23_CSR_WORDRATE_SHIFT);

  for( ain=0; ain<4; ++ain )
    {
      /* Set physical channel, wordrate and gain in LC 1:
         wordrate and gain (and measurement mode) are in 'adc_config' ! */
      csr_data[0]  = adc_config |
	             ((ain << CS23_CSR_PHYSCHAN_SEL_LO_SHIFT) &
		      CS23_CSR_PHYSCHAN_SEL_LO_MASK);
      csr_data[1]  = ((ain >> CS23_CSR_PHYSCHAN_SEL_HI_SHIFT) &
		      CS23_CSR_PHYSCHAN_SEL_HI_MASK);

      /* Write to one CSR (LCs 1 and 2) */
      cs5523_write_csr( 1, csr_data );

      /* Perform the requested calibration using LC 1 */
      cs5523_start_conversion_cmd( calib_type |
				   (0 << CS23_CMD_LOGCHAN_SELECT_SHIFT) );

      /* Busy-wait for calibration-ready or time-out */
      //if( !cs5523_await_sdo_low( 60 ) )
      if( !cs5523_await_sdo_low( 120 ) ) // Make that 1200 ms...
	{
	  /* Time out ! */
	  return FALSE;
	}

      /* Calibration done ! */

      /* Generate 8 SCLKs to clear the SDO flag:
	 ! not needed after calibration (says CRYSTAL) ! */
      /*cs5523_write_byte( 0 );*/
    }

  return TRUE;
}

/* ------------------------------------------------------------------------ */

BOOL cs5523_reset( BYTE *err_id )
{
  /* Initiate a converter reset and
     and check for default contents of Configuration, Offset and Gain
     Registers, which should be:
     config  = 0x000040 (CS23_CNF_RESET_VALID),
     offset  = 0x000000,
     gain    = 0x400000,
     (setups = 0x000000)
     and Configuration Register should contain 0x000000 after the first read
  */

  BYTE config[3];
  BOOL reset_valid;
  BYTE ain;
  BYTE ms50_cnt;

  *err_id = 0;

  /* Write 'reset system' command to the Configuration Register
     by setting the RS-bit */
  config[0] = CS23_CNF_RESET_SYSTEM;
  config[1] = 0;
  config[2] = 0;
  cs5523_write_reg( CS23_CMD_CONFIG_REG, 0, config ); 

  /* Read Configuration Register until RV-bit is set,
     or time-out (ca. 400 ms) */
  ms50_cnt    = 0;
  reset_valid = FALSE;
  while( !reset_valid && ms50_cnt < 8 )
    {
      /* Pause for 50 ms */
      timer0_set_timeout_10ms( ADC_ELMB, 5 );
      while( !timer0_timeout(ADC_ELMB) );
      ++ms50_cnt;

      /* Read Configuration Register */
      cs5523_read_reg( CS23_CMD_CONFIG_REG, 0, config );

      /* RV-bit set ? */
      if( config[0] & CS23_CNF_RESET_VALID )
	{
	  reset_valid = TRUE;
	  /* RV-bit is set, reset the RS-bit */
	  config[0] = 0;
	  config[1] = 0;
	  config[2] = 0;
	  cs5523_write_reg( CS23_CMD_CONFIG_REG, 0, config );
	}
      else reset_valid = FALSE;
    }

  if( reset_valid )
    {
      /* Read Configuration Register once more to reset the RV-bit */
      cs5523_read_reg( CS23_CMD_CONFIG_REG, 0, config );

      /* RV-bit reset ? */
      cs5523_read_reg( CS23_CMD_CONFIG_REG, 0, config );
      if( config[0] != 0x00 )
	{
	  *err_id |= CS23_ERR_RV_NOT_RESET;
	  return FALSE;
	}
    }
  else
    {
      /* Time out ! */
      *err_id |= CS23_ERR_RV_TIMEOUT;
      return FALSE;
    }

  /* Check Offset Registers, should contain 0x000000 */
  for( ain=0; ain<4; ++ain )
    {
      cs5523_read_reg( CS23_CMD_OFFSET_REG, ain, config );
      if( config[0] != 0x00 || config[1] != 0x00 || config[2] != 0x00 )
	{
	  *err_id |= CS23_ERR_OFFSET;
	  return FALSE;
	}
    }

  /* Check Gain Registers, should contain 0x400000 */
  for( ain=0; ain<4; ++ain )
    {
      cs5523_read_reg( CS23_CMD_GAIN_REG, ain, config );
      if( config[0] != 0x00 || config[1] != 0x00 || config[2] != 0x40 )
	{
	  *err_id |= CS23_ERR_GAIN;
	  return FALSE;
	}
    }

  return TRUE;
}

/* ------------------------------------------------------------------------ */

void cs5523_serial_init( void )
{
  /* Perform the serial port initialization sequence:
     at least 15 command bytes 0xFF, then one byte 0xFE */
  BYTE i;
  for( i=0; i<15; ++i ) cs5523_write_byte( 0xFF );
  cs5523_write_byte( 0xFE );
}

/* ------------------------------------------------------------------------ */

BOOL cs5523_await_sdo_low( BYTE ms10_cnt )
{
  /* Wait for about ms10_cnt * 10 ms
     (e.g. for about 2200 ms maximum... (ca. 4 conversions @ 1.88 Hz)) */

  timer0_set_timeout_10ms( ADC_ELMB, ms10_cnt );

  /* Busy-wait for conversion-ready or time-out */
  while( !(ADC_SDO_LOW() || timer0_timeout(ADC_ELMB)) );

  return( ADC_SDO_LOW() );
}

/* ------------------------------------------------------------------------ */

BYTE cs5523_read_nibble( void )
{
  BYTE i, b;

  b = 0;

  /* Clock the data in from the CS5523, MSB first */
  for( i=0; i<4; ++i )
    {
      b <<= 1;
      ADC_SET_SCLK();
      //timer2_delay_mus( CS23_ELMB_SIGNAL_RISETIME );
      timer2_delay_mus( AdcOptoDelay );
      if( ADC_SDO_HIGH() ) ++b;
      ADC_CLEAR_SCLK();
      //timer2_delay_mus( CS23_ELMB_SIGNAL_FALLTIME );
      timer2_delay_mus( AdcOptoDelay );
    }

  return b;
}

/* ------------------------------------------------------------------------ */

void cs5523_write_nibble( BYTE byt )
{
  BYTE i, b;

  b = byt;

  /* Clock the data out to the CS5523, MSB first */
  for( i=0; i<4; ++i )
    {
      if( b & 0x08 ) ADC_SET_SDI();
      else ADC_CLEAR_SDI();
      //timer2_delay_mus( CS23_ELMB_SIGNAL_FALLTIME );
      timer2_delay_mus( AdcOptoDelay );
      ADC_SET_SCLK();
      //timer2_delay_mus( CS23_ELMB_SIGNAL_RISETIME );
      timer2_delay_mus( AdcOptoDelay );
      ADC_CLEAR_SCLK();
      b <<= 1;
    }
}

/* ------------------------------------------------------------------------ */

BYTE cs5523_read_byte( void )
{
  BYTE i, b;

  b = 0;

  /* Clock the data in from the CS5523, MSB first */
  for( i=0; i<8; ++i )
    {
      b <<= 1;
      ADC_SET_SCLK();
      //timer2_delay_mus( CS23_ELMB_SIGNAL_RISETIME );
      timer2_delay_mus( AdcOptoDelay );
      if( ADC_SDO_HIGH() ) ++b;
      ADC_CLEAR_SCLK();
      //timer2_delay_mus( CS23_ELMB_SIGNAL_FALLTIME );
      timer2_delay_mus( AdcOptoDelay );
    }

  return b;
}

/* ------------------------------------------------------------------------ */

void cs5523_write_byte( BYTE byt )
{
  BYTE i, b;

  b = byt;

  /* Clock the data out to the CS5523, MSB first */
  for( i=0; i<8; ++i )
    {
      if( b & 0x80 ) ADC_SET_SDI();
      else ADC_CLEAR_SDI();
      //timer2_delay_mus( CS23_ELMB_SIGNAL_FALLTIME );
      timer2_delay_mus( AdcOptoDelay );
      ADC_SET_SCLK();
      //timer2_delay_mus( CS23_ELMB_SIGNAL_RISETIME );
      timer2_delay_mus( AdcOptoDelay );
      ADC_CLEAR_SCLK();
      b <<= 1;
    }

  /* Set SDI to one when not writing:
     switches off opto-coupler, reduces power consumption */
  ADC_SET_SDI();

  //timer2_delay_mus( CS23_ELMB_SIGNAL_FALLTIME );
  timer2_delay_mus( AdcOptoDelay );
}

/* ------------------------------------------------------------------------ */
