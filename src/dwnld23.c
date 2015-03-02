/* ------------------------------------------------------------------------
File   : dwnld23.c

Descr  : Functions concerning serial downloading of code into the AT90S2313
         (slave) processor (connected to ISP interface terminals).

History: 21MAR.00; Henk B&B; Definition.
--------------------------------------------------------------------------- */

#include "general.h"
#include "download.h"
#include "timer103.h"

static void download_instruction( BYTE *instr );
static void isp_write( BYTE byt );
static BYTE isp_read( void );

/* ------------------------------------------------------------------------ */

BOOL do_serial_instruction( BYTE *instr )
{
  BYTE cmd, par, echo;
  BOOL result;

  cmd = instr[0];
  par = instr[1];

  /* ###Special:
     CAN_W_ (PB5) and ISP_MOSI (PE0) are connected,
     but can't be both output at the same time ! */
  DDRB &= ~BIT(CAN_W_);  /* input */
  DDRE |= BIT(ISP_MOSI); /* output */

  result = TRUE;

  switch( cmd )
    {
    case ISP_ENABLE_OR_ERASE:
      if( par == ISP_CHIP_ERASE )
	{
	  download_instruction( instr );

	  /* Now wait for at least 18 ms (for an AVR at 3.2V)... */
	  timer2_delay_ms( 40 );

	  /* ...and re-enable serial programming */
	  par = ISP_PROGRAMMING_ENABLE;
	}

      if( par == ISP_PROGRAMMING_ENABLE )
	{
	  BYTE tmp;
	  BYTE cnt = 0;

	  /* Stop Slave monitoring */
	  timer1_stop();

	  /* PORT settings (preserve settings of user bits) */
	  tmp   = DDRD & PORTD_USERBITS_MASK;
	  DDRD  = tmp | PORTD_DDR_FOR_ISP;
	  tmp   = PORTD & PORTD_USERBITS_MASK;
	  PORTD = tmp | PORTD_DATA_FOR_ISP;
	  tmp   = DDRE & PORTE_USERBITS_MASK;
	  DDRE  = tmp | PORTE_DDR_FOR_ISP;
	  tmp   = PORTE & PORTE_USERBITS_MASK;
	  PORTE = tmp | PORTE_DATA_FOR_ISP;

	  CLEAR_ISP_SCK();

	  /* Give reset signal to AVR processor a positive pulse of
	     at least two XTAL1 periods (necessary if SCK was not
	     guaranteed to be low during power-up; also necessary
	     after the chip-erase operation done above;
	     see datasheets for details)
	     The Slave reset signal has not yet been applied */
	  SET_SLAVE_RESET();
	  NOP(); NOP();
	  CLEAR_SLAVE_RESET();
	  NOP(); NOP();
	  SET_SLAVE_RESET();

	  /* Now wait for at least 20 ms... */
	  timer2_delay_ms( 40 );

	  /* Enabling programming should succeed within 32 attempts */
	  while( cnt < 32 )
	    {
	      ++cnt;

	      CLEAR_ISP_SCK();

	      /* Write the Programming Enable instruction;
		 at the 3rd byte, the 2nd downloaded byte (=0x53)
		 should be echoed */
	      isp_write( cmd );
	      isp_write( par );
	      echo = isp_read();
	      isp_read();

	      /* 3rd byte contains echoed byte */
	      instr[2] = echo;
	      /* 4th byte contains the number of tries before success */
	      instr[3] = cnt;

	      /* Success if byte 2 is echoed in the 3rd byte;
	         if not seen within 32 attempts, there is no
		 functional device connected */
	      if( echo == ISP_PROGRAMMING_ENABLE )
		{
		  /* Programming enabled, exit while loop */
		  break;
		}

	      /* Give SCK a positive pulse */
	      SET_ISP_SCK();
	    }
	  CLEAR_ISP_SCK();
	}
      else
	{
	  result = FALSE;
	}
      break;

    case ISP_WRITE_FLASH_LO_BYTE:
    case ISP_WRITE_FLASH_HI_BYTE:
    case ISP_WRITE_EEPROM_BYTE:
      download_instruction( instr );

      /* Minimum delay after a FLASH or EEPROM write:
	 9 ms for an AVR at 3.2V */
      timer2_delay_ms( 10 );

      break;

    case ISP_READ_FLASH_LO_BYTE:
    case ISP_READ_FLASH_HI_BYTE:
    case ISP_READ_EEPROM_BYTE:
    case ISP_READ_SIGNATURE_BYTE:
      isp_write( cmd );
      isp_write( par );
      isp_write( instr[2] );
      instr[3] = isp_read();
      break;

    case ISP_END_OF_PROGRAMMING:
      /* Take Slave processor out of programming-enabled state,
         or to be done by resetting the node ! */

      {
	/* PORT settings (preserve settings of user bits) */
	BYTE tmp;
	tmp   = DDRD & PORTD_USERBITS_MASK;
	DDRD  = tmp | PORTD_DDR_OPERATIONAL;
	tmp   = PORTD & PORTD_USERBITS_MASK;
	PORTD = tmp | PORTD_DATA_OPERATIONAL;
	tmp   = DDRE & PORTE_USERBITS_MASK;
	DDRE  = tmp | PORTE_DDR_OPERATIONAL;
	tmp   = PORTE & PORTE_USERBITS_MASK;
	PORTE = tmp | PORTE_DATA_OPERATIONAL;
      }

      /* Resume Slave monitoring */
      timer1_init();

      break;

    default:
      result = FALSE;
      break;
    }

  /* ###Special:
     CAN_W_ (PB5) and ISP_MOSI (PE0) are connected,
     but can't be both output at the same time ! */
  DDRE &= ~BIT(ISP_MOSI); /* input */
  DDRB |= BIT(CAN_W_);    /* output */

  return result;
}

/* ------------------------------------------------------------------------ */

static void download_instruction( BYTE *instr )
{
  isp_write( instr[0] );
  isp_write( instr[1] );
  isp_write( instr[2] );
  isp_write( instr[3] );
}

/* ------------------------------------------------------------------------ */

static void isp_write( BYTE byt )
{
  BYTE bitcount;

  /* Write 8 bits */
  for( bitcount=0; bitcount<8; ++bitcount )
    {
      /* Set MOSI */
      if( byt & 0x80 ) SET_ISP_MOSI();
      else CLEAR_ISP_MOSI();

      NOP();

      /* Provide clock signal */
      SET_ISP_SCK();

      NOP();
      NOP();

      CLEAR_ISP_SCK();

      /* Get ready for next bit */
      byt <<= 1;
    }
}

/* ------------------------------------------------------------------------ */

static BYTE isp_read( void )
{
  BYTE bitcount, byt;

  byt = 0;

  /* Read 8 bits */
  for( bitcount=0; bitcount<8; ++bitcount )
    {
      /* Make space for next bit */
      byt <<= 1;

      /* Set the new bit to '1' if input bit is set */
      if( ISP_MISO_SET() ) ++byt;

      /* Provide clock signal */
      SET_ISP_SCK();

      NOP();
      NOP();

      CLEAR_ISP_SCK();
    }

  /* Return data byte */
  return byt;
}

/* ------------------------------------------------------------------------ */
