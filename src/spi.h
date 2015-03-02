/* ------------------------------------------------------------------------
File   : spi.h

Descr  : Declarations of (software) SPI serial interface functions.

History: 19JAN.00; Henk B&B; Definition.
--------------------------------------------------------------------------- */

#ifndef SPI_H
#define SPI_H

/* ------------------------------------------------------------------------ */
/* Function prototypes */

BYTE spi_read ( void );
void spi_write( BYTE byt );

#endif /* SPI_H */
/* ------------------------------------------------------------------------ */
