/* ------------------------------------------------------------------------
File   : digio.h

Descr  : Include file for the Digital-I/O routines in digio.c.

History: 21JUL.00; Henk B&B; Definition.
         30NOV.00; Henk B&B; Added interrupt enable.
	 23JUL.01; Henk B&B; Share PORTA between in- and outputs;
	                     add input interrupt masks.
	 16JAN.03; Henk B&B; Ported to ATmega128.
--------------------------------------------------------------------------- */

#ifndef DIGIO_H
#define DIGIO_H

/* ------------------------------------------------------------------------ */
/* Digital I/O configuration */

/* The number of ports in our configuration */
#define DIGIO_INPORT_CNT                2
#define DIGIO_OUTPORT_CNT               2

/* How the ports are mapped:
   note that PORTA is shared between in- and outputs ! */

/* ----------------------------------------- */
/* The Inputs: */

#define DIGIN1_PIN                      PINF
#ifndef __ELMB103__
#define DIGIN1_PORT                     PORTF
#define DIGIN1_DDR                      DDRF
#endif /* __ELMB103__ */

#define DIGIN2_PIN                      PINA
#define DIGIN2_PORT                     PORTA
#define DIGIN2_DDR                      DDRA

/* According to simulator measurements each poll is ca. 0.5 ms apart
   ('ELMBio' application on ATmega103 at 4 MHz) */
#define DIGIN_DEBOUNCE_POLLS_DFLT       10

/* ----------------------------------------- */
/* The Outputs: */

#define DIGOUT1_PORT                    PORTC
#ifndef __ELMB103__
#define DIGOUT1_DDR                     DDRC
#endif /* __ELMB103__ */

#define DIGOUT2_PORT                    PORTA
#define DIGOUT2_PIN                     PINA
#define DIGOUT2_DDR                     DDRA

/* ------------------------------------------------------------------------ */
/* Function prototypes */

void    digio_init           ( BOOL hard_reset );

BYTE    digin_port_cnt       ( void );
BYTE    digin_get_port       ( BYTE port_no );
void    digin_pdo            ( void );
void    digin_pdo_on_cos     ( void );

BYTE    digout_port_cnt      ( void );
void    digout_set_port      ( BYTE port_no, BYTE digout_data );
BYTE    digout_get_port      ( BYTE port_no );
void    digout_pdo           ( BYTE dlc, BYTE *pdo_data );

void    digout_set_mask      ( BYTE port_no, BYTE digout_mask );
BYTE    digout_get_mask      ( BYTE port_no );

BOOL    digout_set_init      ( BOOL init_hi );
BOOL    digout_get_init      ( void );

void    digin_set_debounce   ( BYTE debounce_cntr );
BYTE    digin_get_debounce   ( void );

BOOL    digin_set_intrpt_ena ( BOOL intrpt_ena );
BOOL    digin_get_intrpt_ena ( void );

void    digin_set_intrpt_mask( BYTE port_no, BYTE intrpt_mask );
BYTE    digin_get_intrpt_mask( BYTE port_no );

BOOL    digio_store_config   ( void );

#endif /* DIGIO_H */

/* ------------------------------------------------------------------------ */
