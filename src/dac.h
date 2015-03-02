/* ------------------------------------------------------------------------
File   : dac.h

Descr  : Headerfile for dac.c.

History: 08NOV.00; Henk B&B; Definition.
--------------------------------------------------------------------------- */

#ifndef DAC_H
#define DAC_H

/* ------------------------------------------------------------------------ */
/* Configuration */

/* Define here the ports and pins interfacing to the DAC(s) */
#define DAC_SET_SCLK()              SETBIT( PORTD, 4 )
#define DAC_CLEAR_SCLK()            CLEARBIT( PORTD, 4 )
#define DAC_SET_SDI()               SETBIT( PORTD, 5 )
#define DAC_CLEAR_SDI()             CLEARBIT( PORTD, 5 )
#define DAC_SDO_HIGH()              (PIND & BIT( 6 ))
#define DAC_SDO_LOW()               (!DAC_SDO_HIGH())
/* The selection lines */
#define DAC_SELECT_1()              CLEARBIT( PORTE, 3 )
#define DAC_DESELECT_1()            SETBIT( PORTE, 3 )
#define DAC_SELECT_2()              CLEARBIT( PORTE, 4 )
#define DAC_DESELECT_2()            SETBIT( PORTE, 4 )
#define DAC_SELECT_3()              CLEARBIT( PORTE, 5 )
#define DAC_DESELECT_3()            SETBIT( PORTE, 5 )
#define DAC_SELECT_4()              CLEARBIT( PORTE, 6 )
#define DAC_DESELECT_4()            SETBIT( PORTE, 6 )

/* Make sure this corresponds to the stuff above ! */
#define DAC_INIT_DDR()              {SETBIT(DDRD,4);SETBIT(DDRD,5); \
                                     CLEARBIT(DDRD,6);SETBIT(PORTD,6); \
                                     SETBIT(DDRE,3);SETBIT(DDRE,4); \
				     SETBIT(DDRE,5);SETBIT(DDRE,6);}

/* Maximum number of DAC-modules */
#define DAC_MAX_MODULES             4

/* Number of DACs per module */
#define DACS_PER_MODULE             4

/* Number of channels per DAC chip: e.g. 4 for MAX525, 1 for MAX5122 */
#define DAC_CHANS_PER_MAX5122       1
#define DAC_CHANS_PER_MAX525        4

/* Number of channels per DAC module */
#define DAC_MAX_CHANS_PER_MODULE    (DACS_PER_MODULE*DAC_CHANS_PER_MAX525)

/* Maximum number of channels */
#define DAC_MAX_CHANS               (DAC_MAX_MODULES*DAC_MAX_CHANS_PER_MODULE)

/* The signal rise/fall times on the ELMB (due to the opto-couplers) */
#define DAC_ELMB_SIGNAL_RISETIME    75
#define DAC_ELMB_SIGNAL_FALLTIME    75

/* ------------------------------------------------------------------------ */
/* Function prototypes */

void dac_init             ( BOOL hard_reset );

BYTE dac_status           ( BYTE *status );
BYTE dac_chan_cnt         ( void );

BOOL dac_write            ( BYTE dac_chan_no, BYTE *dac_data );
BOOL dac_read             ( BYTE dac_chan_no, BYTE *dac_data );

void dac_pdo              ( BYTE dlc, BYTE *pdo_data );

BOOL dac_store_config     ( void );

BOOL dac_set_max525_select( BOOL ena );
BOOL dac_get_max525_select( void );

BOOL dac_set_opto_delay   ( BYTE delay );
BYTE dac_get_opto_delay   ( void );

#endif /* DAC_H */
/* ------------------------------------------------------------------------ */
