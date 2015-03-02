/* ------------------------------------------------------------------------
File   : uart.c

Descr  : Initialisation routine for the AVR's onchip UART.

History: 11APR.00; Henk B&B; Created.
--------------------------------------------------------------------------- */

#include "general.h"

/* ------------------------------------------------------------------------ */

void uart_init( BYTE baud )
{
  /* Set the Baud Rate Register */
  UBRR0L = baud;

  /* Set the UART Control Register: enable receive and send */
  UCSR0B = BIT(RXEN0) | BIT(TXEN0);

  /* Enable interrupts */
  /*UCSR0B |= BIT(RXCIE0) | BIT(TXCIE0);*/
}

/* ------------------------------------------------------------------------ */
