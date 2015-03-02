/* ------------------------------------------------------------------------
File   : max5122.h

Descr  : Include file containing constants concerning the
         MAX5122 1-channel 12-bit DAC.

History: 06JUN.03; Henk B&B; Start of development, based on MAX525 stuff.
--------------------------------------------------------------------------- */

#ifndef MAX5122_H
#define MAX5122_H

/* ------------------------------------------------------------------------ */

/* High byte */
/* --------- */

/* Data bits */
#define MAX5122_DATABITS_SHIFT          0
#define MAX5122_DATABITS_MASK           (0x1F<<MAX5122_DATABITS_SHIFT)

/* Control bits */
#define MAX5122_CNTRLBITS_SHIFT         5
#define MAX5122_CNTRLBITS_MASK          (0x07<<MAX5122_CNTRLBITS_SHIFT)
#define MAX5122_CMD_NOP                 (0x00<<MAX5122_CNTRLBITS_SHIFT)
#define MAX5122_CMD_LOAD                (0x01<<MAX5122_CNTRLBITS_SHIFT)
#define MAX5122_CMD_LOAD_AND_UPDATE     (0x02<<MAX5122_CNTRLBITS_SHIFT)
#define MAX5122_CMD_UPDATE_FROM_INPUT   (0x03<<MAX5122_CNTRLBITS_SHIFT)
#define MAX5122_CMD_SHUTDOWN            (0x05<<MAX5122_CNTRLBITS_SHIFT)
#define MAX5122_CMD_OUTPUT_LOW          (0x04<<MAX5122_CNTRLBITS_SHIFT)
#define MAX5122_CMD_OUTPUT_HIGH         (0x06<<MAX5122_CNTRLBITS_SHIFT)
#define MAX5122_CMD_MODE1               (0x07<<MAX5122_CNTRLBITS_SHIFT)+0x10
#define MAX5122_CMD_MODE0               (0x07<<MAX5122_CNTRLBITS_SHIFT)+0x00

/* ------------------------------------------------------------------------ */
/* Function prototypes */

void max5122_write_dac( BYTE chan_no, BYTE *dac_data );
void max5122_write_nop( void );

#endif /* MAX5122_H */
/* ------------------------------------------------------------------------ */
