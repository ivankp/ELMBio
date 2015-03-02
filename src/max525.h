/* ------------------------------------------------------------------------
File   : max525.h

Descr  : Include file containing constants concerning the
         MAX525 4-channel 12-bit DAC.

History: 07AUG.01; Henk B&B; Start of development.
--------------------------------------------------------------------------- */

#ifndef MAX525_H
#define MAX525_H

/* ------------------------------------------------------------------------ */

/* High byte */
/* --------- */

/* Data bits */
#define MAX525_DATABITS_SHIFT           0
#define MAX525_DATABITS_MASK            (0x0F<<MAX525_DATABITS_SHIFT)

/* Address bits */
#define MAX525_ADDRESSBITS_SHIFT        6
#define MAX525_ADDRESSBITS_MASK         (0x03<<MAX525_ADDRESSBITS_SHIFT)

/* Control bits */
#define MAX525_CNTRLBITS_SHIFT          4
#define MAX525_CNTRLBITS_MASK           (0x03<<MAX525_CNTRLBITS_SHIFT)
#define MAX525_CMD_LOAD                 (0x01<<MAX525_CNTRLBITS_SHIFT)
#define MAX525_CMD_LOAD_AND_UPDATE      (0x03<<MAX525_CNTRLBITS_SHIFT)
#define MAX525_CMD_UPDATE_FROM_INPUT    (0x04<<MAX525_CNTRLBITS_SHIFT)
#define MAX525_CMD_LOAD_ALL_FROM_SHIFT  (0x08<<MAX525_CNTRLBITS_SHIFT)
#define MAX525_CMD_SHUTDOWN             (0x0C<<MAX525_CNTRLBITS_SHIFT)
#define MAX525_CMD_OUTPUT_LOW           (0x02<<MAX525_CNTRLBITS_SHIFT)
#define MAX525_CMD_OUTPUT_HIGH          (0x06<<MAX525_CNTRLBITS_SHIFT)
#define MAX525_CMD_NOP                  (0x00<<MAX525_CNTRLBITS_SHIFT)
#define MAX525_CMD_MODE1                (0x0E<<MAX525_CNTRLBITS_SHIFT)
#define MAX525_CMD_MODE0                (0x0A<<MAX525_CNTRLBITS_SHIFT)

/* ------------------------------------------------------------------------ */
/* Function prototypes */

void max525_write_dac( BYTE chan_no, BYTE *dac_data );
void max525_write_nop( void );

#endif /* MAX525_H */
/* ------------------------------------------------------------------------ */
