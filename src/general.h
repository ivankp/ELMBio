/* ------------------------------------------------------------------------
File   : general.h

Descr  : Some useful general-purpose definitions +
         includes of appropriate processor type include file and
	 file with our application-specific processor configuration.

History: 30NOV.99; Henk B&B; Version for AVR applications.
--------------------------------------------------------------------------- */

#ifndef GENERAL_H
#define GENERAL_H 

/* ------------------------------------------------------------------------ */
/* Type definitions */

typedef unsigned char     BOOL;
typedef unsigned char     BYTE;
typedef signed char       CHAR;
typedef unsigned int      UINT16;
typedef signed int        INT16;
typedef unsigned long     UINT32;
typedef signed long       INT32;

/* ------------------------------------------------------------------------ */
/* Constants */

#define TRUE              1
#define FALSE             0

/* ------------------------------------------------------------------------ */
/* Macros */

#define BIT(x)            (1 << (x))
#define SETBIT(addr,x)    (addr |= BIT(x))
#define CLEARBIT(addr,x)  (addr &= ~BIT(x))

#define CLI()             asm( "CLI" )
#define NOP()             asm( "NOP" )
#define SEI()             asm( "SEI" )
#define SLEEP()           asm( "SLEEP" )
#define WDR()             asm( "WDR" )

/* ------------------------------------------------------------------------ */

/* Choose the following include file according to the processor type used */
#ifdef __ELMB103__
#include "iom103v.h"
/* Declare some items missing or having other names
   to remain compatible with the ATmega128 */
#define BORF   2
#define WDRF   3
#define JTRF   4
#define WDCE   WDTOE
#define UBRR0L UBRR
#define UCSR0B UCR
#define RXEN0  RXEN
#define TXEN0  TXEN
#define RXCIE0 RXCIE
#define TXCIE0 TXCIE
#else
#include "iom128v.h"
#endif /* __ELMB103__ */

/* ELMB-specific processor configuration */
#include "103conf.h"

#endif /* GENERAL_H */
/* ------------------------------------------------------------------------ */
