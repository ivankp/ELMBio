/* ------------------------------------------------------------------------
File   : pdo.h

Descr  : Include file for the PDO routines in pdo.c.

History: 19JUL.00; Henk B&B; Definition.
	   AUG.00; Henk B&B; Addition of PDOs for digital input and output.
	   NOV.00; Henk B&B; Addition of PDO for analogue output (DAC).
--------------------------------------------------------------------------- */

#ifndef PDO_H
#define PDO_H

/* Number of Transmit-PDOs */
#define TPDO_CNT          3

/* Number of Receive-PDOs */
#define RPDO_CNT          2

/* Which PDO is used for what */
#define TPDO_DIGITAL_IN   (1-1)
#define TPDO_ANALOG_IN    (2-1)
#define TPDO_ANALOG_IN_V  (3-1)
#define RPDO_DIGITAL_OUT  (1-1)
#define RPDO_ANALOG_OUT   (2-1)

/* ------------------------------------------------------------------------ */
/* Globals */

/* For timer-triggered PDO transmissions */
extern BOOL   TPdoOnTimer[];

/* Keeps track of time for the timer-triggered PDO transmissions */
extern UINT16 TPdoTimerCntr[];

/* ------------------------------------------------------------------------ */
/* Function prototypes */

void pdo_init          ( void );
void tpdo_scan         ( void );
void pdo_on_nmt        ( BYTE nmt_request );
void tpdo_on_sync      ( void );
void tpdo1_on_rtr      ( void );
void tpdo2_on_rtr      ( void );
void tpdo3_on_rtr      ( void );
void rpdo1             ( BYTE dlc, BYTE *can_data );
void rpdo2             ( BYTE dlc, BYTE *can_data );
BOOL pdo_rtr_required  ( void );

BOOL tpdo_get_comm_par ( BYTE pdo_no,
			 BYTE od_subind,
			 BYTE *nbytes,
			 BYTE *par );
BOOL rpdo_get_comm_par ( BYTE pdo_no,
			 BYTE od_subind,
			 BYTE *nbytes,
			 BYTE *par );

BOOL tpdo_get_mapping  ( BYTE pdo_no,
			 BYTE od_subind,
			 BYTE *nbytes,
			 BYTE *par );
BOOL rpdo_get_mapping  ( BYTE pdo_no,
			 BYTE od_subind,
			 BYTE *nbytes,
			 BYTE *par );

BOOL tpdo_set_comm_par ( BYTE pdo_no,
			 BYTE od_subind,
			 BYTE nbytes,
			 BYTE *par );

BOOL pdo_store_config  ( void );

#endif /* PDO_H */

/* ------------------------------------------------------------------------ */
