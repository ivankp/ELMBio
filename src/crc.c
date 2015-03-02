/* ------------------------------------------------------------------------
File   : crc.c

Descr  : Functions for CRC calculations by the ELMB ATmega103 processor.

History: 04MAY.99; Henk B&B; First version based on version of CRC
			     calculations of earlier date (May 4 1999)
	   AUG.00; Henk B&B; Special function 'crc_slave()' for CRC-ing
	                     ELMB Slave processor program memory;
			     special routine 'get_code_hi()' for accessing
			     program memory on ATmega103 over 64 kbyte
			     boundary.
	 30JAN.01; Henk B&B; Implement CRC over program code > 64 Kbytes
	                     using crc16_code_over_64k().
	 19APR.01; Henk B&B; Above addition contains bug: size word cannot
	                     be > 64k; use 3rd byte (was 0x00) to signify
			     code size > 64k (by setting 3rd byte to 0x01).
	 15SEP.03; Henk B&B; Added crc_get().
--------------------------------------------------------------------------- */

#include "general.h"
#include "crc.h"
#include "eeprom.h"
#ifdef __2313_SLAVE_PRESENT__
#include "download.h"
#endif /* __2313_SLAVE_PRESENT__ */

/* ------------------------------------------------------------------------ */
/* Local prototypes */

static UINT16 crc16_code_upto_64k( UINT16 index_until );
static UINT16 crc16_code_over_64k( UINT16 crc, UINT16 index_until );
static BYTE   get_code_hi        ( UINT16 addr );

/* ------------------------------------------------------------------------ */

UINT16 crc16_ram( BYTE *byt, UINT16 size )
{
  /* Calculate CRC-16 value; uses The CCITT-16 Polynomial,
     expressed as X^16 + X^12 + X^5 + 1
     CRC start value: 0xFFFF */

  UINT16 crc = (UINT16) 0xFFFF;
  UINT16 index;
  BYTE   b;

  for( index=0; index<size; ++index )
    {
      crc ^= (((UINT16) byt[index]) << 8);
      for( b=0; b<8; ++b )
	{
	  if( crc & (UINT16) 0x8000 )
	    crc = (crc << 1) ^ (UINT16) 0x1021;
	  else
	    crc = (crc << 1);
	}
    }
  return crc;
}

/* ------------------------------------------------------------------------ */

UINT16 crc16_eeprom( UINT16 addr, UINT16 size )
{
  /* Calculate CRC-16 value of datablock in EEPROM;
     uses The CCITT-16 Polynomial, expressed as X^16 + X^12 + X^5 + 1
     CRC start value: 0xFFFF */

  UINT16 crc = (UINT16) 0xFFFF;
  UINT16 index;
  BYTE   b;

  for( index=addr; index<addr+size; ++index )
    {
      crc ^= (((UINT16) eepromw_read(index)) << 8);
      for( b=0; b<8; ++b )
	{
	  if( crc & (UINT16) 0x8000 )
	    crc = (crc << 1) ^ (UINT16) 0x1021;
	  else
	    crc = (crc << 1);
	}
    }
  return crc;
}

/* ------------------------------------------------------------------------ */

#ifdef __ELMB103__
#define ADDR_CRC_MASTER_FLASH  0xFFFF
#else
/* Take into account that on an ATmega128 space for a bootloader
   is reserved in the high 4 kWords (8 kBytes) */
#define ADDR_CRC_MASTER_FLASH  0xDFFF
#endif /* __ELMB103__ */

BOOL crc_master( UINT16 *pcrc )
{
  UINT16 size;
  BYTE   byt;

  *pcrc = (UINT16) 0;

  /* Get the size of the program code (in bytes) */
  byt   = get_code_hi( ADDR_CRC_MASTER_FLASH-1 );
  size  = (UINT16) byt;
  byt   = get_code_hi( ADDR_CRC_MASTER_FLASH );
  size |= (((UINT16) byt) << 8);

  /* Check for presence of CRC */
  if( get_code_hi( ADDR_CRC_MASTER_FLASH-2 ) == 0x00 )
    {
      /* Parameter is the memory index to include in the CRC calculation */
      *pcrc = crc16_code_upto_64k( size-1 );

      return TRUE;
    }
  else
    {
      if( get_code_hi( ADDR_CRC_MASTER_FLASH-2 ) == 0x01 )
	{
	  /* Code size is 64k+ .....: the size calculated in the above
	     statements is the size of the code over 64k (in bytes) */

	  /* Got to calculate the CRC in 2 parts: upto 64k and above... */
	  *pcrc = crc16_code_upto_64k( (UINT16) 0xFFFF );
	  if( size > (UINT16) 0 ) *pcrc = crc16_code_over_64k( *pcrc, size-1 );

	  return TRUE;
	}
      else
	{
	  /* Returning FALSE and a CRC equal to zero means
	     no CRC was present ! */
	  return FALSE;
	}
    }
}

/* ------------------------------------------------------------------------ */

BOOL crc_get( BYTE *pcrc_byte )
{
  UINT16 size;
  BYTE   byt;
  BYTE   k64;

  /* Get the size of the program code (in bytes) */
  byt   = get_code_hi( ADDR_CRC_MASTER_FLASH-1 );
  size  = (UINT16) byt;
  byt   = get_code_hi( ADDR_CRC_MASTER_FLASH );
  size |= (((UINT16) byt) << 8);
  k64   = get_code_hi( ADDR_CRC_MASTER_FLASH-2 );

  /* Check for presence of CRC */
  if( k64 <= 0x01 )
    {
      if( k64 == 0x01 )
	{
	  pcrc_byte[0] = get_code_hi( size - 1 );
	  pcrc_byte[1] = get_code_hi( size - 2 );
	}
      else
	{
	  const BYTE *codebyte = (const BYTE *) 0x0000;
	  pcrc_byte[0] = codebyte[size - 1];
	  pcrc_byte[1] = codebyte[size - 2];
	}
      return TRUE;
    }
  else
    {
      /* No CRC present */
      return FALSE;
    }
}

/* ------------------------------------------------------------------------ */

#ifdef __2313_SLAVE_PRESENT__
BOOL crc_slave( UINT16 *pcrc )
{
  BYTE isp_instr[4];

  *pcrc = (UINT16) 1;

  isp_instr[0] = ISP_ENABLE_OR_ERASE;
  isp_instr[1] = ISP_PROGRAMMING_ENABLE;
  isp_instr[2] = 0;
  isp_instr[3] = 0;

  if( do_serial_instruction( isp_instr ) == FALSE ||
      isp_instr[2] != ISP_PROGRAMMING_ENABLE)
    {
      /* Programming enable failed */

      isp_instr[0] = ISP_END_OF_PROGRAMMING;
      do_serial_instruction( isp_instr );

      /* Returning FALSE and a CRC unequal to zero means
	 something went wrong ! */
      return FALSE;
    }

  /* Check for presence of CRC */
  isp_instr[0] = ISP_READ_FLASH_HI_BYTE;
  isp_instr[1] = 0x03;
  isp_instr[2] = 0xFE;
  isp_instr[3] = 0xFF;
  do_serial_instruction( isp_instr );
  if( isp_instr[3] == 0 )
    {
      /* CRC present, go check it... */
      UINT16 addr;
      UINT16 crc;
      BYTE byt;
      BYTE b;

      crc = (UINT16) 0xFFFF;

      for( addr=0; addr<2048; ++addr )
	{
	  if( addr & 1 )
	    isp_instr[0] = ISP_READ_FLASH_HI_BYTE;
	  else
	    isp_instr[0] = ISP_READ_FLASH_LO_BYTE;

	  /* High byte of word address */
	  isp_instr[1] = (BYTE) (((addr>>1) & 0xff00) >> 8);

	  /* Low byte of word address */
	  isp_instr[2] = (BYTE) (((addr>>1) & 0x00ff) >> 0);

	  /* Get the byte from the Slave FLASH */
	  do_serial_instruction( isp_instr );

	  byt = isp_instr[3];

	  crc ^= (((UINT16) byt) << 8);
	  for( b=0; b<8; ++b )
	    {
	      if( crc & (UINT16) 0x8000 )
		crc = (crc << 1) ^ (UINT16) 0x1021;
	      else
		crc = (crc << 1);
	    }
	}

      *pcrc = crc;

      isp_instr[0] = ISP_END_OF_PROGRAMMING;
      do_serial_instruction( isp_instr );

      /* Returning TRUE and a CRC unequal to zero
	 means there is a CRC error ! */
      return TRUE;
    }
  else
    {
      /* Returning FALSE and a CRC equal to zero means no CRC was present ! */
      *pcrc = (UINT16) 0;
      return FALSE;
    }
}
#else
BOOL crc_slave( UINT16 *pcrc )
{
  /* Returning FALSE and a CRC equal to zero means no CRC was present ! */
  *pcrc = (UINT16) 0;
  return FALSE;
}
#endif /* __2313_SLAVE_PRESENT__ */

/* ------------------------------------------------------------------------ */

static UINT16 crc16_code_upto_64k( UINT16 index_until )
{
  /* Calculate CRC-16 value; uses The CCITT-16 Polynomial,
     expressed as X^16 + X^12 + X^5 + 1 */

  /* NB: does not cover any code over the 64 kbytes limit ! */

  const BYTE *codebyte = (const BYTE *) 0x0000;
  UINT16 crc = (UINT16) 0xFFFF;
  UINT16 index;
  BYTE   b;

  for( index=0; index<=index_until; ++index, ++codebyte )
    {
      WDR(); /* In case of a free-running watchdog timer */
      crc ^= (((UINT16) *codebyte) << 8);
      for( b=0; b<8; ++b )
	{
	  if( crc & (UINT16) 0x8000 )
	    crc = (crc << 1) ^ (UINT16) 0x1021;
	  else
	    crc = (crc << 1);
	}
    }
  return crc;
}

/* ------------------------------------------------------------------------ */

static UINT16 crc16_code_over_64k( UINT16 crc, UINT16 index_until )
{
  /* Calculate CRC-16 value, initializing CRC with 'crc';
     uses The CCITT-16 Polynomial, expressed as X^16 + X^12 + X^5 + 1 */

  /* NB: covers the code part beyond the 64 kbytes limit only ! */

  UINT16 lcrc = crc;
  UINT16 index;
  BYTE   b;

  for( index=0; index<=index_until; ++index )
    {
      WDR(); /* In case of a free-running watchdog timer */
      lcrc ^= (((UINT16) get_code_hi(index)) << 8);
      for( b=0; b<8; ++b )
	{
	  if( lcrc & (UINT16) 0x8000 )
	    lcrc = (lcrc << 1) ^ (UINT16) 0x1021;
	  else
	    lcrc = (lcrc << 1);
	}
    }
  return lcrc;
}

/* ------------------------------------------------------------------------ */

static BYTE get_code_hi( UINT16 addr )
{
  /* This routine reads a byte from the upper 64k of the program memory;
     the upper 64k can _not_ be read by defining simply a 'const BYTE *' ! */

  BYTE codbyt;

  /* Move 'addr' to the proper registers (for ELPM) */
  asm( "mov R30,R16" );
  asm( "mov R31,R17" );

  /* Set RAMPZ register to access the upper 64k page of program memory */
  RAMPZ = 1;

  /* Read the program memory byte and store it in 'codbyt' */
  asm( "elpm" );
  asm( "mov %codbyt, R0" );

  /* Reset RAMPZ register */
  RAMPZ = 0;

  return codbyt;
}

/* ------------------------------------------------------------------------ */
