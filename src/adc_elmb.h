/* ------------------------------------------------------------------------
File   : adc_elmb.h

Descr  : Headerfile for adc_elmb.c (with CS5523 ADC).

History: 20JUL.00; Henk B&B; Definition.
         16JAN.00; Henk B&B; Go to Motherboard V3 configuration, but
	                     allow backwards compatibility using compile
			     options __MOTHERBOARD1__ or __ALL_MOTHERBOARDS__
			     (the last option tries to access the ADC
			      as on Motherboard V3 and if that fails tries
			      to access the ADC as on Motherboard V1/2).
--------------------------------------------------------------------------- */

#ifndef ADC_ELMB_H
#define ADC_ELMB_H

/* ------------------------------------------------------------------------ */
/* I/O configuration */

/*
  PORTD (Normal operational mode)
  ======
  BIT	FUNCTION	I/O	DfltData  DDRD  PORTD
  ---   --------        ---     --------  ----  -----
  d7	ADC_MUX_LATCH	out	1	    1     1    ELMB Motherboard V3
  d6	ADC_SDO		in	pullup	    0     1    ELMB Motherboard V3
  d5	ADC_SDI		out	1	    1     1    ELMB Motherboard V3
  d4	ADC_SCLK	out	0	    1     0    ELMB Motherboard V3
  d3	ADC_SELECT	out	1	    1     1    ELMB Motherboard V3
  d2	(system)	---	---	    -	  -
  d1	(system)	---	---	    -     -
  d0	(system)	---	---	    -	  -
                                           ---   ---
					   $B8	 $E8
  PORTE (Normal operational mode)
  ======
  BIT	FUNCTION	I/O	DfltData  DDRE  PORTE
  ---   --------        ---     --------  ----  -----
  e7	(user defined)	---	---	    0     0
  e6	(user defined)	---	---	    0     0
  e5	(user defined)	---	---	    0     0
  e4	ADC_MUX_LATCH	out	1	    1     1    ELMB Motherboard V1/2
  e3	ADC_SELECT	out	1	    1     1    ELMB Motherboard V1/2
  e2	(system)	---	---	    -     -
  e1	(system)	---	---	    -     -
  e0	(system)	---	---	    -     -
                                           ---   ---
					   $18	 $18
*/

/* SPI serial interface for ADC using Motherboard V1 and V2 */
#define ADC_SET_SCLK_V1()           SET_SCLK()
#define ADC_CLEAR_SCLK_V1()         CLEAR_SCLK()
#define ADC_SET_SDI_V1()            SET_SDI()
#define ADC_CLEAR_SDI_V1()          CLEAR_SDI()
#define ADC_SDO_HIGH_V1()           SDO_SET()
#define ADC_SDO_LOW_V1()            (!SDO_SET())
/* Other signals are located on PORTE */
#define ADC_SET_MUX_LATCH_V1()      SETBIT( PORTE, 4 )
#define ADC_CLEAR_MUX_LATCH_V1()    CLEARBIT( PORTE, 4 )
#define ADC_SELECT_V1()             CLEARBIT( PORTE, 3 )
#define ADC_DESELECT_V1()           SETBIT( PORTE, 3 )

/* ! Make sure the macro below matches the stuff above ! */
#define ADC_INIT_DDR_V1()           {SETBIT(DDRE,4);SETBIT(DDRE,3);}


/* SPI serial interface for ADC using Motherboard V3:
   all signals located on PORTD */
#define ADC_SET_SCLK_V3()           SETBIT( PORTD, 4 )
#define ADC_CLEAR_SCLK_V3()         CLEARBIT( PORTD, 4 )
#define ADC_SET_SDI_V3()            SETBIT( PORTD, 5 )
#define ADC_CLEAR_SDI_V3()          CLEARBIT( PORTD, 5 )
#define ADC_SDO_HIGH_V3()           (PIND & BIT( 6 ))
#define ADC_SDO_LOW_V3()            (!ADC_SDO_HIGH_V3())
#define ADC_SET_MUX_LATCH_V3()      SETBIT( PORTD, 7 )
#define ADC_CLEAR_MUX_LATCH_V3()    CLEARBIT( PORTD, 7 )
#define ADC_SELECT_V3()             CLEARBIT( PORTD, 3 )
/* No need to deselect if nothing else connected to this SPI..
   (but now we might have ELMB-DACs connected to this ELMB !) */
#define ADC_DESELECT_V3()           SETBIT( PORTD, 3 )

/* ! Make sure the macro below matches the stuff above ! */
#define ADC_INIT_DDR_V3()           {SETBIT(DDRD,3);SETBIT(PORTD,3); \
                                     SETBIT(DDRD,4); \
                                     SETBIT(DDRD,5); \
                                     CLEARBIT(DDRD,6);SETBIT(PORTD,6); \
                                     SETBIT(DDRD,7);}

#ifdef __MOTHERBOARD1__
/* We are using Motherboard V1 or V2 with the ADC connected to
   the same SPI interface as the CAN-controller */

/* SPI serial interface for ELMB on-board ADC
   (same as ELMB SPI serial interface to CAN-controller) */
#define ADC_SET_SCLK()              ADC_SET_SCLK_V1()
#define ADC_CLEAR_SCLK()            ADC_CLEAR_SCLK_V1()
#define ADC_SET_SDI()               ADC_SET_SDI_V1()
#define ADC_CLEAR_SDI()             ADC_CLEAR_SDI_V1()
#define ADC_SDO_HIGH()              ADC_SDO_HIGH_V1()
#define ADC_SDO_LOW()               ADC_SDO_LOW_V1()
/* Other signals are located elsewhere */
#define ADC_SET_MUX_LATCH()         ADC_SET_MUX_LATCH_V1()
#define ADC_CLEAR_MUX_LATCH()       ADC_CLEAR_MUX_LATCH_V1()
#define ADC_SELECT()                ADC_SELECT_V1()
#define ADC_DESELECT()              ADC_DESELECT_V1()

#else /* *NOT* __MOTHERBOARD1__ */

#ifdef __ALL_MOTHERBOARDS__
/* We can handle Motherboards V1/V2 _and_ V3:
   the ADC is connected to any of 2 SPI interfaces */

/* Pointers to the functions doing the actual bit operations */
extern void (*ADC_SET_SCLK)( void );
extern void (*ADC_CLEAR_SCLK)( void );
extern void (*ADC_SET_SDI)( void );
extern void (*ADC_CLEAR_SDI)( void );
extern BOOL (*ADC_SDO_HIGH)( void );
extern BOOL (*ADC_SDO_LOW)( void );
extern void (*ADC_SET_MUX_LATCH)( void );
extern void (*ADC_CLEAR_MUX_LATCH)( void );
extern void (*ADC_SELECT)( void );
extern void (*ADC_DESELECT)( void );

#else /* Motherboard3 */
/* We are using Motherboard V3 with the ADC connected
   to its own SPI interface; this is the default */

/* SPI serial interface for ADC with Motherboard V3:
   all signals located on PORTD */
#define ADC_SET_SCLK()              ADC_SET_SCLK_V3()
#define ADC_CLEAR_SCLK()            ADC_CLEAR_SCLK_V3()
#define ADC_SET_SDI()               ADC_SET_SDI_V3()
#define ADC_CLEAR_SDI()             ADC_CLEAR_SDI_V3()
#define ADC_SDO_HIGH()              ADC_SDO_HIGH_V3()
#define ADC_SDO_LOW()               ADC_SDO_LOW_V3()
#define ADC_SET_MUX_LATCH()         ADC_SET_MUX_LATCH_V3()
#define ADC_CLEAR_MUX_LATCH()       ADC_CLEAR_MUX_LATCH_V3()
#define ADC_SELECT()                ADC_SELECT_V3()
#define ADC_DESELECT()              ADC_DESELECT_V3()

#endif /* __ALL_MOTHERBOARDS__ */
#endif /* __MOTHERBOARD1__ */

/* Maximum number of analog inputs per ADC */
#define ADC_MAX_INPUTS              64

/* Conversion parameters */
#define ADC_DFLT_CONV_PARS          ((CS23_WORDRATE_15 << \
				      CS23_CSR_WORDRATE_SHIFT) | \
				     (CS23_GAIN_5V << \
				      CS23_CSR_GAIN_SHIFT) | \
				     CS23_CSR_UNIPOLAR)

/* Configuration parameters */
#define ADC_CNFREG_0                0
#define ADC_CNFREG_1                ((1 << CS23_CNF_CSR_DEPTH_SHIFT) | \
				     CS23_CNF_CHARGE_PUMP_DISABLE)
#define ADC_CNFREG_2_CHOP256        (CS23_CHOPFREQ_256 << \
				     CS23_CNF_CHOPFREQ_SHIFT)
#define ADC_CNFREG_2_CHOP4096       (CS23_CHOPFREQ_4096 << \
				     CS23_CNF_CHOPFREQ_SHIFT)

/* ------------------------------------------------------------------------ */
/* Error ID bits */

/* In AdcError status byte */
#define ADC_ERR_RESET               0x01
#define ADC_ERR_CALIBRATION         0x02
#define ADC_ERR_TIMEOUT             0x04
#define ADC_ERR_CALIB_CNST          0x08
#define ADC_ERR_DELTA               0x10
#define ADC_ERR_UPPER               0x20
#define ADC_ERR_LOWER               0x40
#define ADC_ERR_IN_HARDWARE         (ADC_ERR_RESET|ADC_ERR_CALIBRATION| \
                                     ADC_ERR_TIMEOUT)

/* In CAN-messages with ADC data */
#define ADC_ERR_CONVERSION          0x80

/* ------------------------------------------------------------------------ */
/* ELMB-ADC-specific function prototypes (the rest can be found in adc.h) */

BOOL   adc_read_volts              ( BYTE chan_no, BYTE *data );
BOOL   adc_reset_and_calibrate     ( BOOL send_emergency );
BOOL   adc_reset                   ( BOOL send_emergency );
void   adc_serial_init             ( void );

BOOL   adc_set_calib_before_scan   ( BOOL calib );
BOOL   adc_get_calib_before_scan   ( void );

BOOL   adc_set_readout_on_change   ( BOOL readout_on_change );
BOOL   adc_get_readout_on_change   ( void );

BOOL   adc_set_delta_scan_ena      ( BOOL delta_scan_enable );
BOOL   adc_get_delta_scan_ena      ( void );

BOOL   adc_set_window_scan_ena     ( BOOL window_scan_enable );
BOOL   adc_get_window_scan_ena     ( void );

BOOL   adc_set_window_scan_cntr    ( BYTE window_scan_counter );
BYTE   adc_get_window_scan_cntr    ( void );

BOOL   adc_set_delta               ( BYTE chan_no, BYTE *delta );
void   adc_get_delta               ( BYTE chan_no, BYTE *delta );

BOOL   adc_set_upperlimit          ( BYTE chan_no, BYTE *upper );
void   adc_get_upperlimit          ( BYTE chan_no, BYTE *upper );

BOOL   adc_set_lowerlimit          ( BYTE chan_no, BYTE *lower );
void   adc_get_lowerlimit          ( BYTE chan_no, BYTE *lower );

BOOL   adc_store_deltas            ( void );
BOOL   adc_store_upperlimits       ( void );
BOOL   adc_store_lowerlimits       ( void );

BOOL   adc_invalidate_deltas       ( void );
BOOL   adc_invalidate_upperlimits  ( void );
BOOL   adc_invalidate_lowerlimits  ( void );

void   adc_init_delta_references   ( void );

BOOL   adc_calibrate_range         ( BYTE od_range_id );

BOOL   adc_set_calib_const         ( BYTE od_range_id, BYTE index, BYTE *val );
BOOL   adc_get_calib_const         ( BYTE od_range_id, BYTE index, BYTE *val,
				     BOOL send_emergency );

BOOL   adc_erase_calib_const       ( BYTE od_range_id, BYTE val );

BOOL   adc_calib_const_write_enable( BYTE val );

BOOL   adc_calibrated              ( void );

/* ------------------------------------------------------------------------ */
#endif /* ADC_ELMB_H */
