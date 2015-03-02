/* ------------------------------------------------------------------------
File   : guarding.h

Descr  : Include file for the Node- and Lifeguarding routines in guarding.c.

History: 23JUL.00; Henk B&B; Definition.
--------------------------------------------------------------------------- */

#ifndef GUARDING_H
#define GUARDING_H

#define LIFETIME_FACTOR_DFLT 70

#define HEARTBEAT_TIME_DFLT  0

/* ------------------------------------------------------------------------ */
/* Globals */

/* Keeps track of time for the Life Guarding time out */
extern BYTE LifeGuardCntr;

/* Keeps track of time for the Heartbeat generation */
extern BYTE HeartBeatCntr;

/* Toggle bit for the Node Guarding CAN-message */
extern BYTE NodeGuardToggle;

/* ------------------------------------------------------------------------ */
/* Function prototypes */

void guarding_init             ( void );
void lifeguarding_and_heartbeat( BYTE nodestate );
void nodeguarding              ( BYTE nodestate );
BYTE guarding_get_guardtime    ( BYTE *guardtime );
BYTE guarding_get_lifetime     ( BYTE *factor );
BOOL guarding_set_lifetime     ( BYTE factor );
BYTE guarding_get_heartbeattime( BYTE *hbtime );
BOOL guarding_set_heartbeattime( BYTE *hbtime );
BOOL guarding_store_config     ( void );

#endif /* GUARDING_H */

/* ------------------------------------------------------------------------ */
