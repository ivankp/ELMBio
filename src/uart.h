/* ------------------------------------------------------------------------
File   : uart.h

Descr  : Include file for the AVR's onchip UART.

History: 11APR.00; Henk B&B; Created.
--------------------------------------------------------------------------- */

/* ------------------------------------------------------------------------ */
/* The AVR baud rate generator generates a baud rate according to:
       baudrate = clockfreqency / 16*(UBRR + 1)
   with UBRR is the contents of the UART Baud Rate Register.

   With a clock of 4 MHz:
     baudrate  UBRR  %Err
     --------  ----  ----
        2400    103   0.2
        4800     51   0.2
        9600     25   0.2
       19200     12   0.2
*/

#define BAUD_9600  25
#define BAUD_4800  51

/* ------------------------------------------------------------------------ */
/* Function prototypes */

void uart_init( BYTE baud );

/* ------------------------------------------------------------------------ */
