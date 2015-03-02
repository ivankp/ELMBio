/* ------------------------------------------------------------------------
File   : adc.h

Descr  : Include file to define the use of a particular ADC(-module).

History: 19JUL.00; Henk B&B; Definition.
         22JUN.01; Henk B&B; Removed unnecessary 'adc_no' parameter from
	                     some function's parameters.
--------------------------------------------------------------------------- */

#ifndef ADC_H
#define ADC_H

#ifdef __ADC_AVR__
#include "adc_avr.h"
#else
#ifndef __ADC_NONE__
#include "adc_elmb.h"
#endif
#endif

#ifndef __ADC_NONE__
/* ------------------------------------------------------------------------ */
/* Function prototypes */

BOOL adc_init               ( void );
BOOL adc_read               ( BYTE channel_no, BYTE *conversion_data );
BYTE adc_status             ( BYTE *status );
BOOL adc_get_config         ( BYTE subindex,
			      BYTE *nbytes, BYTE *par );
BOOL adc_set_config         ( BYTE subindex,
			      BYTE nbytes,  BYTE *par );
void adc_pdo_scan_start     ( BOOL volts, BOOL force_readout );
void adc_pdo_scan_stop      ( void );
void adc_pdo_scan           ( void );

BOOL adc_store_config       ( void );

#else
/* ------------------------------------------------------------------------ */
/* Dummy functions when no ADC is present or none is used */

#define adc_init()                                                TRUE
#define adc_read( channel_no, conversion_data )                   FALSE
#define adc_status( status )
#define adc_get_config( subindex, nbytes, par )                   FALSE
#define adc_set_config( subindex, nbytes,  par )                  FALSE
#define adc_get_calib_config( subindex, nbytes, par )             FALSE
#define adc_set_calib_config( subindex, nbytes, par )             FALSE
#define adc_pdo_scan_start( volts, force_readout )
#define adc_pdo_scan_stop()
#define adc_pdo_scan()
#define adc_store_config()                                        TRUE

/* Dummies for the adc_elmb.h functions not included in adc.h */
#define ADC_MAX_INPUTS                                            64
#define adc_read_volts( chan_no, data )                           FALSE
#define adc_reset_and_calibrate( send_emergency )                 FALSE
#define adc_reset( send_emergency )                               FALSE
#define adc_serial_init()
#define adc_set_calib_before_scan( calib )                        FALSE
#define adc_get_calib_before_scan()                               FALSE
#define adc_set_readout_on_change( readout_on_change )            FALSE
#define adc_get_readout_on_change()                               FALSE
#define adc_set_delta_scan_ena( delta_scan_enable )               FALSE
#define adc_get_delta_scan_ena()                                  FALSE
#define adc_set_window_scan_ena( window_scan_enable )             FALSE
#define adc_get_window_scan_ena()                                 FALSE
#define adc_set_window_scan_cntr( window_scan_counter )           FALSE
#define adc_get_window_scan_cntr()                                0
#define adc_set_delta( chan_no, delta )                           FALSE
#define adc_get_delta( chan_no, delta )
#define adc_set_upperlimit( chan_no, upper )                      FALSE
#define adc_get_upperlimit( chan_no, upper )
#define adc_set_lowerlimit( chan_no, lower )                      FALSE
#define adc_get_lowerlimit( chan_no, lower )
#define adc_store_deltas()                                        FALSE
#define adc_store_upperlimits()                                   FALSE
#define adc_store_lowerlimits()                                   FALSE
#define adc_invalidate_deltas()                                   FALSE
#define adc_invalidate_upperlimits()                              FALSE
#define adc_invalidate_lowerlimits()                              FALSE
#define adc_init_delta_references()
#define adc_calibrate_range( range )                              FALSE
#define adc_set_calib_const( od_range_id, index, val )            FALSE
#define adc_get_calib_const( od_range_id, index, val, send_emergency ) FALSE
#define adc_erase_calib_const( od_range_id, val )                 FALSE
#define adc_calib_const_write_enable( val )                       FALSE
#define adc_calibrated()                                          FALSE

#endif /* __ADC_NONE__ */
/* ------------------------------------------------------------------------ */
#endif /* ADC_H */

