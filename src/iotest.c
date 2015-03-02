/* ------------------------------------------------------------------------
File   : iotest.c

Descr  : Functions for ELMB Digital-input and -output testing.
         Test of the I/O lines (on ELMB itself and the Motherboard) not used
	 in the general-purpose ELMBio application; done by connecting custom
	 test-cables to the appropriate Motherboard connectors.
	 Tested are: PORTA, PORTC, (part of) PORTD, (part of) PORTE and PORTF.

	 - to be able to use define compile option __INCLUDE_TESTS__
	 - used to test Motherboard and ELMB I/O lines (not the ADC)
	 - does tests on the available I/O lines, assuming they are
	   interconnected in the following way:
	   PORTA 0 to 3 connected to PORTA 4 to 7,
	   PORTD 3 to 7 connected to PORTE 3 to 7,
	   PORTC 0 to 7 connected to PORTF 0 to 7.
	 - the tests:
	 - all possible output values of PORTA(0-3), PORTD(3-7) and PORTC,
	   per port, using PORTA(4-7), PORTE(3-7) and PORTF as inputs.
	 - all possible output values of PORTA(4-7)and PORTE(3-7), per port,
	   using PORTA(0-3) and PORTD(3-7) as inputs.
	 - a walking-1 and a walking-0 on the combined ports
	   (the walking-1/0 'runs' from PORTA to PORTD to PORTC;
	   checks are on PORTA(4-7), PORTE(3-7) and PORTF).
	 - the test is triggered by reading CANopen Object 5FFF, subindex 1
	 - the returned 32-bit value contains 4 bytes with errorbits per port:
	   Byte 0: PORTA, Byte 1: PORTD, Byte 2: PORTE, Byte 3: PORTF
	   A bit that is set means the corresponding I/O line of the
	   corresponding Port has at least once been _read_ incorrectly
	   during the tests.
	   (PORTC has no errorbits because it is never 'read', but of course
	   the fault -if there is any- shown in the errorbits of PORTF
	   could still be located at the PORTC connector!).
         - total test time is in the order of 300 ms.
	   (using a signal settling time of 1 ms).

History: 12JAN.00; Henk B&B; Start of development.
         27JUN.02; Henk B&B; Exclude pin PD3 from test because it is connected
	                     to the ADC chip-select; it means that pin PE3 is
			     not tested at all; generalize code for PORTD/PORTE
			     interconnections using some '#define's.
--------------------------------------------------------------------------- */

#include "general.h"
#include "timer103.h"

/* Define PORTD range of bits to be included in test */
#define PORTD_TEST_BITS     4
#define PORTD_TEST_FIRSTBIT 4
#define PORTD_TEST_BITMASK  ((1<<PORTD_TEST_BITS)-1) << PORTD_TEST_FIRSTBIT

/* Delay in ms between setting output and reading input */
#define IOTEST_DELAY       1

/* Indices in array of results */
#define ERR_A              0
#define ERR_D              1
#define ERR_E              2
#define ERR_F              3

/* ------------------------------------------------------------------------ */

void iotest( BYTE *result )
{
  BYTE err, out, in;
  BYTE loop;
  BYTE porta, ddra;
  BYTE portc;
  BYTE portd, ddrd;
  BYTE porte, ddre;
#ifndef __ELMB103__
  BYTE ddrc;
  BYTE portf, ddrf;
#endif /* __ELMB103__ */

  /* Initialize array with test results */
  for( loop=0; loop<4; ++loop ) result[loop] = 0x00;

  /* Remember the current I/O port settings */
  porta = PORTA;
  ddra  = DDRA;
  portc = PORTC;
  portd = PORTD;
  ddrd  = DDRD;
  porte = PORTE;
  ddre  = DDRE;
#ifndef __ELMB103__
  ddrc  = DDRC;
  portf = PORTF;
  ddrf  = DDRF;
#endif /* __ELMB103__ */

#ifndef __ELMB103__
  DDRC  = 0xFF; /* Outputs */
  DDRF  = 0x00; /* Inputs  */
  PORTF = 0xFF; /* Pullups */
#endif /* __ELMB103__ */

  /* -------------------------------------------------------- */
  /* PORTA standalone */
  {
    err = 0x00;

    /* PORTA: PA0->PA4, PA1->PA5, PA2->PA6, PA3->PA7 */
    DDRA = 0x0F;
    for( loop=0x00; loop<=0x0F; ++loop )
      {
	out   = loop;
	PORTA = out;
	timer2_delay_ms( IOTEST_DELAY );
	in = (PINA & 0xF0);
	/* In the error byte set the input bits that differ */
	if( in != (out<<4) ) err |= (in ^ (out<<4));
      }

    /* PORTA: PA0<-PA4, PA1<-PA5, PA2<-PA6, PA3<-PA7 */
    DDRA = 0xF0;
    for( loop=0x00; loop<=0x0F; ++loop )
      {
	out   = (loop << 4);
	PORTA = out;
	timer2_delay_ms( IOTEST_DELAY );
	in = (PINA & 0x0F);
	/* In the error byte set the input bits that differ */
	if( in != (out>>4) ) err |= (in ^ (out>>4));
      }

    result[ERR_A] |= err;
  }

  /* -------------------------------------------------------- */
  /* PORTD+PORTE standalone */
  {
    err = 0x00;

    /* PD4->PE4, PD5->PE5, PD6->PE6, PD7->PE7 */
    DDRD = (0xFF & PORTD_TEST_BITMASK) | (ddrd & ~PORTD_TEST_BITMASK);
    DDRE = 0x00 | (ddre & ~PORTD_TEST_BITMASK);
    for( loop=0x00; loop<=((1<<PORTD_TEST_BITS)-1); ++loop )
      {
	out   = (loop << PORTD_TEST_FIRSTBIT);
	PORTD = out | (portd & ~PORTD_TEST_BITMASK);
	timer2_delay_ms( IOTEST_DELAY );
	in = (PINE & PORTD_TEST_BITMASK);
	/* In the error byte set the bits that differ */
	if( in != out ) err |= (in ^ out);
      }

    result[ERR_E] |= err;

    err = 0x00;

    /* PD4<-PE4, PD5<-PE5, PD6<-PE6, PD7<-PE7 */
    DDRD = 0x00 | (ddrd & ~PORTD_TEST_BITMASK);
    DDRE = (0xFF & PORTD_TEST_BITMASK) | (ddre & ~PORTD_TEST_BITMASK);
    for( loop=0x00; loop<=((1<<PORTD_TEST_BITS)-1); ++loop )
      {
	out   = (loop << PORTD_TEST_FIRSTBIT);
	PORTE = out | (porte & ~PORTD_TEST_BITMASK);
	timer2_delay_ms( IOTEST_DELAY );
	in = (PIND & PORTD_TEST_BITMASK);
	/* In the error byte set the bits that differ */
	if( in != out ) err |= (in ^ out);
      }

    result[ERR_D] |= err;
  }

  /* -------------------------------------------------------- */
  /* PORTC+PORTF standalone */
  {
    err = 0x00;

    /* All 8 pins of PORTC connected to PORTF:
       PC0->PF0, PC1->PF1, PC2->PF2, PC3->PF3,
       PC4->PF4, PC5->PF5, PC6->PF6, PC7->PF7 */

    for( out=0x00; out<0xFF; ++out )
      {
	PORTC = out;
	timer2_delay_ms( IOTEST_DELAY );
	in = PINF;
	/* In the error byte set the bits that differ */
	if( in != out ) err |= (in ^ out);
      }

    result[ERR_F] |= err;
  }

  /* -------------------------------------------------------- */
  /* Walking 1 */
  {
    DDRA  = 0x0F;
    DDRD  = (0xFF & PORTD_TEST_BITMASK) | (ddrd & ~PORTD_TEST_BITMASK);
    DDRE  = 0x00 | (ddre & ~PORTD_TEST_BITMASK);
    PORTA = 0x00;
    PORTC = 0x00;
    PORTD = 0x00 | (portd & ~PORTD_TEST_BITMASK);
    PORTE = 0x00 | (porte & ~PORTD_TEST_BITMASK);

    /* Walking-1 PORTA (4 bits) */
    for( loop=0; loop<4; ++loop )
      {
	out   = (1 << loop);
	PORTA = out;
	timer2_delay_ms( IOTEST_DELAY );
	in = (PINA & 0xF0);
	if( in != (out<<4) )        result[ERR_A] |= (in ^ (out<<4));
	if( PINF != 0x00 )          result[ERR_F] |= PINF;
	if( (PINE & PORTD_TEST_BITMASK) != 0x00 )
	  result[ERR_E] |= (PINE & PORTD_TEST_BITMASK);
      }
    PORTA = 0x00;

    /* Walking-1 PORTC (8 bits) */
    for( loop=0; loop<8; ++loop )
      {
	out   = (1 << loop);
	PORTC = out;
	timer2_delay_ms( IOTEST_DELAY );
	in = PINF;
	if( (PINA & 0xF0) != 0x00 ) result[ERR_A] |= (PINA & 0xF0);
	if( in != out )             result[ERR_F] |= (in ^ out);
	if( (PINE & PORTD_TEST_BITMASK) != 0x00 )
	  result[ERR_E] |= (PINE & PORTD_TEST_BITMASK);
      }
    PORTC = 0x00;

    /* Walking-1 PORTD (PORTD_TEST_BITS bits) */
    for( loop=PORTD_TEST_FIRSTBIT;
	 loop<PORTD_TEST_FIRSTBIT+PORTD_TEST_BITS; ++loop )
      {
	out   = (1 << loop);
	PORTD = out | (portd & ~PORTD_TEST_BITMASK);
	timer2_delay_ms( IOTEST_DELAY );
	in = (PINE & PORTD_TEST_BITMASK);
	if( (PINA & 0xF0) != 0x00 ) result[ERR_A] |= (PINA & 0xF0);
	if( PINF != 0x00 )          result[ERR_F] |= PINF;
	if( in != out )             result[ERR_E] |= (in ^ out);
      }
    PORTD = 0x00 | (portd & ~PORTD_TEST_BITMASK);
  }

  /* -------------------------------------------------------- */
  /* Walking 0 */
  {
    DDRA  = 0x0F;
    DDRD  = (0xFF & PORTD_TEST_BITMASK) | (ddrd & ~PORTD_TEST_BITMASK);
    DDRE  = 0x00 | (ddre & ~PORTD_TEST_BITMASK);
    PORTA = 0x0F;
    PORTC = 0xFF;
    PORTD = (0xFF & PORTD_TEST_BITMASK) | (portd & ~PORTD_TEST_BITMASK);
    PORTE = (0xFF & PORTD_TEST_BITMASK) | (porte & ~PORTD_TEST_BITMASK);

    /* Walking-0 PORTA */
    for( loop=0; loop<4; ++loop )
      {
	out   = ~(1 << loop) & 0x0F;
	PORTA = out;
	timer2_delay_ms( IOTEST_DELAY );
	in = (PINA & 0xF0);
	if( in != (out<<4) ) result[ERR_A] |= (in ^ (out<<4));
	if( PINF != 0xff )   result[ERR_F] |= ~PINF;
	if( (PINE & PORTD_TEST_BITMASK) != PORTD_TEST_BITMASK )
	  result[ERR_E] |= (~PINE & PORTD_TEST_BITMASK);
      }
    PORTA = 0x0F;

    /* Walking-0 PORTC */
    for( loop=0; loop<8; ++loop )
      {
	out   = ~(1 << loop);
	PORTC = out;
	timer2_delay_ms( IOTEST_DELAY );
	in = PINF;
	if( (PINA & 0xF0) != 0xF0 ) result[ERR_A] |= (~PINA & 0xF0);
	if( in != out )             result[ERR_F] |= (in ^ out);
	if( (PINE & PORTD_TEST_BITMASK) != PORTD_TEST_BITMASK )
	  result[ERR_E] |= (~PINE & PORTD_TEST_BITMASK);
      }
    PORTC = 0xFF;

    /* Walking-0 PORTD */
    for( loop=PORTD_TEST_FIRSTBIT;
	 loop<PORTD_TEST_FIRSTBIT+PORTD_TEST_BITS; ++loop )
      {
	out   = ~(1 << loop) & PORTD_TEST_BITMASK;
	PORTD = out | (portd & ~PORTD_TEST_BITMASK);
	timer2_delay_ms( IOTEST_DELAY );
	in = (PINE & PORTD_TEST_BITMASK);
	if( (PINA & 0xF0) != 0xF0 ) result[ERR_A] |= (~PINA & 0xF0);
	if( PINF != 0xFF )          result[ERR_F] |= ~PINF;
	if( in != out )             result[ERR_E] |= (in ^ out);
      }
    PORTD = PORTD_TEST_BITMASK | (portd & ~PORTD_TEST_BITMASK);
  }

  /* -------------------------------------------------------- */

  /* Restore the saved I/O port settings */
  PORTA = porta;
  DDRA  = ddra;
  PORTC = portc;
  PORTD = portd;
  DDRD  = ddrd;
  PORTE = porte;
  DDRE  = ddre;
#ifndef __ELMB103__
  DDRC  = ddrc;
  PORTF = portf;
  DDRF  = ddrf;
#endif /* __ELMB103__ */
}

/* ------------------------------------------------------------------------ */
