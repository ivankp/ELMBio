/* ------------------------------------------------------------------------
File   : serialno.h

Descr  : Declarations of ELMB Serial Number functions.

History: 09OCT.02; Henk B&B; Definition.
--------------------------------------------------------------------------- */

#ifndef SERIALNO_H
#define SERIALNO_H

/* ------------------------------------------------------------------------ */
/* Function prototypes */

BOOL sn_set_serial_number         ( BYTE *sn );
BOOL sn_get_serial_number         ( BYTE *sn );

BOOL sn_serial_number_write_enable( BYTE val );

#endif /* SERIALNO_H */
/* ------------------------------------------------------------------------ */
