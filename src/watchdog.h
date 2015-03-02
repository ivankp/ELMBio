/* ------------------------------------------------------------------------
File   : watchdog.h

Descr  : Watchdog functions.

History: 16OCT.01; Henk B&B; Definition.
--------------------------------------------------------------------------- */

#ifndef WATCHDOG_H
#define WATCHDOG_H

/* ------------------------------------------------------------------------ */
/* Globals */

extern BOOL KickWatchdog;

/* ------------------------------------------------------------------------ */
/* Function prototypes */

void watchdog_init   ( void );
void watchdog_disable( void );
void watchdog        ( void );

#endif /* WATCHDOG_H */
/* ------------------------------------------------------------------------ */
