/* ------------------------------------------------------------------------
File   : adc_avr.h

Descr  : Headerfile for adc_avr.c (with ATmega103 onchip 8-channel 10-bit ADC).

History: 21NOV.00; Henk B&B; Definition.
--------------------------------------------------------------------------- */

#ifndef ADC_AVR_H
#define ADC_AVR_H

/* ------------------------------------------------------------------------ */
/* ADC Control and Status Register bits */

#define ADC_ENABLE                  BIT( ADEN )
#define ADC_START_CONVERSION        BIT( ADSC )
#define ADC_CONVERSION_DONE         BIT( ADIF )
#define ADC_INTERRUPT_ENABLE        BIT( ADIE )

/* ADC Prescaler select:
   the ADC input clock frequency should be in the range 50-200 kHz,
   so with a system clock of 4 MHz only ADC_PRESCALE_CK_DIV_32 and up
   are valid settings ! */
#define ADC_PRESCALE_INVALID        0x00
#define ADC_PRESCALE_CK_DIV_2       0x01
#define ADC_PRESCALE_CK_DIV_4       0x02
#define ADC_PRESCALE_CK_DIV_6       0x03
#define ADC_PRESCALE_CK_DIV_16      0x04
#define ADC_PRESCALE_CK_DIV_32      0x05
#define ADC_PRESCALE_CK_DIV_64      0x06
#define ADC_PRESCALE_CK_DIV_128     0x07

/* ------------------------------------------------------------------------ */
/* Configuration constants */

/* Maximum number of analog inputs per ADC */
#define ADC_MAX_INPUTS              8

/* Conversion parameters */
#define ADC_DFLT_CONV_PARS          ADC_PRESCALE_CK_DIV_128

/* ------------------------------------------------------------------------ */
/* Error ID bits */

/* ------------------------------------------------------------------------ */

#define adc_read_volts( chan_no, data )                           FALSE
#define adc_reset_and_calibrate( send_emergency )                 FALSE
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

#endif /* ADC_AVR_H */
/* ------------------------------------------------------------------------ */
