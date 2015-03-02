/* ------------------------------------------------------------------------
File   : cs5523.h

Descr  : Include file containing constants concerning the
         CRYSTAL CS5523 (CS5524) 4-channel 16-bit (24-bit) ADC.

History: 02MAY.00; Henk B&B; Start of development.
         06AUG.01; Henk B&B; Replaced 2 functions by macros.
	 04FEB.02; Henk B&B; Bug fix: Power save bit is 0x02, not 0x08.
--------------------------------------------------------------------------- */

#ifndef CS5523_H
#define CS5523_H

/* The signal rise time on the ELMB (due to the opto-couplers)
   is about 100 microseconds, signal fall time is faster
   ###After certain radiation tests the numbers have been changed
   ###to 75 and 75 (28 July 2003)
   ###After doing tests with opto-couplers from the production batches
   ###a considerable slower response was found (up to 60/70 us)
   ###so numbers changed to 150 (8 Dec 2003) */
#define CS23_ELMB_SIGNAL_RISETIME       150
#define CS23_ELMB_SIGNAL_FALLTIME       150

/* ------------------------------------------------------------------------ */
/* Command Register (1 byte) */

/* First set of commands */ 
#define CS23_CMD_REGISTER_SELECT        0x00

#define CS23_CMD_PHYSCHAN_SELECT_SHIFT  4
#define CS23_CMD_PHYSCHAN_SELECT_MASK   (0x03<<CS23_CMD_PHYSCHAN_SELECT_SHIFT)

#define CS23_CMD_READ                   0x08
#define CS23_CMD_WRITE                  0x00

#define CS23_CMD_REG_SELECT_MASK        0x07

#define CS23_CMD_OFFSET_REG             0x01
#define CS23_CMD_GAIN_REG               0x02
#define CS23_CMD_CONFIG_REG             0x03
#define CS23_CMD_CONV_DATA_REG          0x04
#define CS23_CMD_CHAN_SETUP_REG         0x05

/* Second set of commands */ 
#define CS23_CMD_START_CONVERSION       0x80

#define CS23_CMD_LOGCHAN_SELECT_SHIFT   3
#define CS23_CMD_LOGCHAN_SELECT_MASK    (0x07<<CS23_CMD_LOGCHAN_SELECT_SHIFT)

#define CS23_CMD_NORMAL_CONVERSION      0x00
#define CS23_CMD_OFFSET_SELF_CALIB      0x01
#define CS23_CMD_GAIN_SELF_CALIB        0x02
#define CS23_CMD_OFFS_GAIN_SELF_CALIB   0x03
#define CS23_CMD_OFFSET_SYSTEM_CALIB    0x05
#define CS23_CMD_GAIN_SYSTEM_CALIB      0x06

/* ------------------------------------------------------------------------ */
/* Channel-Setup Registers (2 bytes each) */

/* High byte */
/* --------- */

#define CS23_CSR_A1A0_SHIFT             2
#define CS23_CSR_A1A0_MASK              (0x03 << CS23_CSR_A1A0_SHIFT)

#define CS23_CSR_PHYSCHAN_SEL_HI_SHIFT  1
#define CS23_CSR_PHYSCHAN_SEL_HI_MASK   (0x03 >> \
					 CS23_CSR_PHYSCHAN_SEL_HI_SHIFT)

/* Low byte */
/* -------- */

#define CS23_CSR_PHYSCHAN_SEL_LO_SHIFT  7
#define CS23_CSR_PHYSCHAN_SEL_LO_MASK   (0x03 << \
					 CS23_CSR_PHYSCHAN_SEL_LO_SHIFT)
#define CS23_CSR_WORDRATE_SHIFT         4
#define CS23_CSR_WORDRATE_MASK          (0x07<<CS23_CSR_WORDRATE_SHIFT)
#define CS23_WORDRATE_15                0x0
#define CS23_WORDRATE_30                0x1
#define CS23_WORDRATE_61                0x2
#define CS23_WORDRATE_84                0x3
#define CS23_WORDRATE_101               0x4
#define CS23_WORDRATE_1                 0x5
#define CS23_WORDRATE_3                 0x6
#define CS23_WORDRATE_7                 0x7

#define CS23_CSR_GAIN_SHIFT             1
#define CS23_CSR_GAIN_MASK              (0x07 << CS23_CSR_GAIN_SHIFT)
#define CS23_GAIN_100MV                 0x0
#define CS23_GAIN_55MV                  0x1
#define CS23_GAIN_25MV                  0x2
#define CS23_GAIN_1V                    0x3
#define CS23_GAIN_5V                    0x4
#define CS23_GAIN_2V5                   0x5

#define CS23_CSR_BIPOLAR                0x00
#define CS23_CSR_UNIPOLAR               0x01

/* ------------------------------------------------------------------------ */
/* Configuration Register (3 bytes) */

/* High byte */
/* --------- */

#define CS23_CNF_CHOPFREQ_SHIFT         4
#define CS23_CNF_CHOPFREQ_MASK          (0x03 << CS23_CNF_CHOPFREQ_SHIFT)
#define CS23_CHOPFREQ_256               0x0
#define CS23_CHOPFREQ_4096              0x1
#define CS23_CHOPFREQ_16384             0x2
#define CS23_CHOPFREQ_1024              0x3

#define CS23_CNF_MULTIPLE_CONV          0x04
#define CS23_CNF_SINGLE_CONV            0x00

/* Mid byte */
/* -------- */

#define CS23_CNF_CSR_DEPTH_SHIFT        4
#define CS23_CNF_CSR_DEPTH_MASK         (0x07 << CS23_CNF_CSR_DEPTH_SHIFT)

#define CS23_CNF_POWER_SAVE             0x02

#define CS23_CNF_CHARGE_PUMP_DISABLE    0x04
#define CS23_CNF_CHARGE_PUMP_ENABLE     0x00

/* Low byte */
/* -------- */

#define CS23_CNF_RESET_SYSTEM           0x80
#define CS23_CNF_RESET_VALID            0x40
#define CS23_CNF_OSCILLATION_DETECT     0x20
#define CS23_CNF_OVERRANGE_FLAG         0x10

/* ------------------------------------------------------------------------ */
/* Conversion data (FIFO) */

#define CS23_DATA_OVERFLOW              0x01
#define CS23_DATA_OSCILLATION           0x02
#define CS23_DATA_ERROR                 (CS23_DATA_OVERFLOW | \
                                         CS23_DATA_OSCILLATION)

/* ------------------------------------------------------------------------ */
/* Error Identifiers */

#define CS23_ERR_RV_NOT_RESET           0x01
#define CS23_ERR_RV_TIMEOUT             0x02
#define CS23_ERR_OFFSET                 0x04
#define CS23_ERR_GAIN                   0x08

/* ------------------------------------------------------------------------ */
/* Function prototypes */

#define cs5523_select_register_cmd( cmd ) \
        cs5523_write_byte( CS23_CMD_REGISTER_SELECT | cmd )
#define cs5523_start_conversion_cmd( cmd ) \
        cs5523_write_byte( CS23_CMD_START_CONVERSION | cmd )

void cs5523_read_reg     ( BYTE reg, BYTE ain, BYTE *regdata );
void cs5523_write_reg    ( BYTE reg, BYTE ain, BYTE *regdata );
void cs5523_read_csr     ( BYTE *csrdata );
void cs5523_write_csr    ( BYTE no_of_csr, BYTE *csrdata );
BOOL cs5523_read_adc     ( BYTE log_chan, BYTE *conversion_data );
BOOL cs5523_calibrate    ( BYTE calib_type, BYTE adc_config );
BOOL cs5523_reset        ( BYTE *err_id );
void cs5523_serial_init  ( void );
BOOL cs5523_await_sdo_low( BYTE ms10_cnt );
BYTE cs5523_read_nibble  ( void );
void cs5523_write_nibble ( BYTE byt );
BYTE cs5523_read_byte    ( void );
void cs5523_write_byte   ( BYTE byt );

#endif

/* ------------------------------------------------------------------------ */
