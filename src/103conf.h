/* ------------------------------------------------------------------------
File   : 103conf.h

Descr  : ELMB system configuration stuff for the ELMB Master processor
         (ATMega103/128).

History: 04APR.00; Henk B&B; Start of development.
         30NOV.00; Henk B&B; ELMBio Version 3.1.
         03APR.01; Henk B&B; ELMBio Version 3.2.
         ..APR.01; Henk B&B; ELMBio Version 3.3.
         ..AUG.01; Henk B&B; ELMBio Version 3.4.
         ..FEB.02; Henk B&B; ELMBio Version 3.5.
         ..MAY.02; Henk B&B; ELMBio Version 3.6.
         ..MAY.02; Henk B&B; ELMBio Version 4.0.
         ..OCT.02; Henk B&B; ELMBio Version 4.1.
         08DEC.03; Henk B&B; ELMBio Version 4.2.0001.
         13JAN.04; Henk B&B; ELMBio Version 4.2.0002:
	                     - Fixed problem in RTR handling in combination
			       with can_recv_descriptor_refresh().
			     - Moved final DAC_SET_SDI() from max5xx.c to
			       dac_write() in dac.c to prevent spikey SDI.
         ..MAR.04; Henk B&B; ELMBio Version 4.3.0001:
	                     - Added toggle bit to CANopen Emergency message.
			     - Added general-purpose SPI objects (see sdo.c
			       and objects.h).
			     - Added files spi_gp.c/h with SPI functions
			       to project.
			     - adc_elmb.c:
			       - Add an exceed counter for every channel
			         in delta mode (report delta-change only when
			         the change has been seen a consecutive number
				 of times); use the minimum exceed count of
				 the window mode.
			       - Bug fix in adc_window_check().
			       - Proper check when setting window scan counter
			         (1<=value<=254).
			     - sdo.c: reply to 'activate Bootloader' request,
			       before actually jumping to the Bootloader.
			     - can.c: remove can_recv_descriptor_refresh()
	                       from can_write(): message loss is possible.
         06MAY.04; Henk B&B; ELMBio Version 4.3.0002:
	                     - adc_elmb.c, cs5523.c: ADC conversion time-out
			       set to 1200 ms (from 800), because there are
			       sometimes conversions that last longer, so it
			       appears.
			     - adc_elmb.c: in PDO readout, after time-out
			       do a complete reset+calib sequence.
			     - guarding.h: enable Life Guarding by default,
			       at 70 secs.
         18MAY.04; Henk B&B; ELMBio Version 4.3.0003:
	                     - adc_elmb.c: bug fix: reset+calib after time-out
			       works if first AdcConvInProgress set to FALSE.
			     - can.c:
			       - Initialize Emergency toggle bit globally,
			         once, not in can_init(), to prevent bit
				 not toggling in repeated can_init() calls.
			       - Init error counter only when all is init'ed.
--------------------------------------------------------------------------- */

#ifndef CONF103_H
#define CONF103_H

/* ------------------------------------------------------------------------ */
/* Node identification */

/* CANopen device type = 0x000F0191,
   i.e. DSP-401 device profile supporting analog in- and outputs
   and digital in- and outputs */
#define DEVICE_TYPE_CHAR0               0x91
#define DEVICE_TYPE_CHAR1               0x01
#define DEVICE_TYPE_CHAR2               0x0F
#define DEVICE_TYPE_CHAR3               0x00

/* Manufacturer device name: ELMB */
#define MNFCT_DEV_NAME_CHAR0            'E'
#define MNFCT_DEV_NAME_CHAR1            'L'
#define MNFCT_DEV_NAME_CHAR2            'M'
#define MNFCT_DEV_NAME_CHAR3            'B'

/* Hardware version: ELMB version 4.0 (ELMB128) */
#define MNFCT_HARDW_VERSION_CHAR0       'e'
#define MNFCT_HARDW_VERSION_CHAR1       'l'
#define MNFCT_HARDW_VERSION_CHAR2       '4'
#define MNFCT_HARDW_VERSION_CHAR3       '0'

/* Firmware version: ELMBio firmware version 4.3) */
#define MNFCT_SOFTW_VERSION_CHAR0       'M'
#ifdef __ADC_AVR__
#define MNFCT_SOFTW_VERSION_CHAR1       'V'
#else
#ifdef __ADC_NONE__
#define MNFCT_SOFTW_VERSION_CHAR1       'N'
#else
#define MNFCT_SOFTW_VERSION_CHAR1       'A'
#endif
#endif
#define MNFCT_SOFTW_VERSION_CHAR2       '4'
#define MNFCT_SOFTW_VERSION_CHAR3       '3'

/* Firmware 'sub'version: ELMBio firmware version 4.3.0003) */
#define SOFTW_MINOR_VERSION_CHAR0       '0'
#define SOFTW_MINOR_VERSION_CHAR1       '0'
#define SOFTW_MINOR_VERSION_CHAR2       '0'
#define SOFTW_MINOR_VERSION_CHAR3       '3'

#define VERSION_STRING               " ELMBio v4.3.3, 18 May 2004, HB, NIKHEF "

/* ------------------------------------------------------------------------ */
/* Port pin usage */

/*
  PORTB (Normal operational mode)
  ======
  BIT	FUNCTION	I/O	DfltDATA  DDRB  PORTB
  ---   --------        ---     --------  ----  -----
  b7	---		in	pullup      0	  1
  b6	CAN_CS_		out	1	    1	  1 
  b5	CAN_W_		out	1           1     1
  b4	---		in	pullup	    0     1
  b3	SDO		in	pullup	    0     1
  b2	SDI		out	0	    1     0
  b1	SCLK		out	0	    1     0
  b0	---		in	pullup	    0     1
                                           ---   ---
				           $66   $F9
  PORTB (Read jumper settings mode)
  ======
  (note: PB0 and PB1 have double functions):
  BIT	FUNCTION	I/O	DfltData  DDRB  PORTB
  ---   --------        ---     --------  ----  -----
  b7	NODEID_SELECT	out	1	    1     1
  b6	---		in	pullup      0	  1
  b5	ID bit5		in	pullup      0	  1
  b4	ID bit4		in	pullup	    0	  1
  b3	ID bit3 	in	pullup	    0     1
  b2	ID bit2		in	pullup	    0     1
  b1	ID/Baud	bit1/1	in	pullup	    0	  1
  b0	ID/Baud	bit0/0	in	pullup	    0     1
                                           ---   ---
				           $80   $FF  
  Jumpers are configured as follows:
  BIT   JUMPERS  FUNCTION
  ---   -------  --------
  PB0	  o o    Baudrate  
  PB1     o o    Baudrate
  PB0     o o    CAN Node-ID LSB bit0 (1)
  PB1	  o o    CAN Node-ID	 bit1 (2)
  PB2	  o o    CAN Node-ID	 bit2 (4)
  PB3	  o o    CAN Node-ID	 bit3 (8)
  PB4	  o o    CAN Node-ID	 bit4 (16)
  PB5	  o o    CAN Node-ID MSB bit5 (32)

  PORTD (Normal operational mode)
  ======
  BIT	FUNCTION	I/O	DfltData  DDRD  PORTD
  ---   --------        ---     --------  ----  -----
  d7	(user defined)	---	---	    0     0
  d6	(user defined)	---	---	    0     0
  d5	(user defined)	---	---	    0     0
  d4	(user defined)	---	---	    0     0
  d3	(user defined)	---	---	    0     0
  d2	MASTERtoSLAVE	out	1	    1	  1
  d1	CAN_INT_	in	pullup	    0     1
  d0	SLAVE_RESET_	in	pullup	    0	  1
                                           ---   ---
					   $04	 $07
  PORTD (Slave programming mode)
  ======
  BIT	FUNCTION	I/O	DfltData  DDRD  PORTD
  ---   --------        ---     --------  ----  -----
  d7	(user defined)	---	---	    0     0
  d6	(user defined)	---	---	    0     0
  d5	(user defined)	---	---	    0     0
  d4	(user defined)	---	---	    0     0
  d3	(user defined)	---	---	    0     0
  d2	(MASTERtoSLAVE)	in	pullup	    0	  1
  d1	ISP_SCK		out	0	    1     0
  d0	SLAVE_RESET_	out	1	    1	  1
                                           ---   ---
					   $03	 $05

  PORTE (Normal operational mode)
  ======
  BIT	FUNCTION	I/O	DfltData  DDRE  PORTE
  ---   --------        ---     --------  ----  -----
  e7	(user defined)	---	---	    0     0
  e6	(user defined)	---	---	    0     0
  e5	(user defined)	---	---	    0     0
  e4	(user defined)	---	---	    0     0
  e3	(user defined)	---	---	    0     0
  e2	BAUDRATE_SELECT	out	1	    1     1
  e1	(ISP_MISO)	in	pullup      0     1
  e0	(ISP_MOSI)	in	pullup      0     1
                                           ---   ---
					   $04	 $07
  (NOTE: PE1 is forced to TXD output if UART-TXEN bit is set;
         PE0 is forced to RXD input  if UART-RXEN bit is set)

  PORTE (Slave programming mode)
  ======
  BIT	FUNCTION	I/O	DfltData  DDRE  PORTE
  ---   --------        ---     --------  ----  -----
  e7	(user defined)	---	---	    0     0
  e6	(user defined)	---	---	    0     0
  e5	(user defined)	---	---	    0     0
  e4	(user defined)	---	---	    0     0
  e3	(user defined)	---	---	    0     0
  e2	BAUDRATE_SELECT	out	1	    1     1
  e1	ISP_MISO	in	pullup      0     1
  e0	ISP_MOSI	out	1           1     1
                                           ---   ---
					   $05	 $07
*/

/* ------------------------------------------------------------------------ */
/* Default port settings */

/* PORTB data direction and default data bitmasks */
#define PORTB_DDR_OPERATIONAL           0x66
#define PORTB_DATA_OPERATIONAL          0xF9
#define PORTB_DDR_FOR_JUMPERS           0x80
#define PORTB_DATA_FOR_JUMPERS          0xFF

/* PORTD data direction and default data bitmasks */
#define PORTD_DDR_OPERATIONAL           0x04
#define PORTD_DATA_OPERATIONAL          0x07
#define PORTD_DDR_FOR_ISP               0x03
#define PORTD_DATA_FOR_ISP              0x05
#define PORTD_USERBITS_MASK             0xF8

/* PORTE data direction and default data bitmasks */
#define PORTE_DDR_OPERATIONAL           0x04
#define PORTE_DATA_OPERATIONAL          0x07
#define PORTE_DDR_FOR_ISP               0x05
#define PORTE_DATA_FOR_ISP              0x07
#define PORTE_USERBITS_MASK             0xF8

/* ------------------------------------------------------------------------ */
/* Pin usages */

/* Reset to the 'Slave' microcontroller */
#define SLAVE_RESET_                    0
#define SET_SLAVE_RESET()               CLEARBIT( PORTD, SLAVE_RESET_ )
#define CLEAR_SLAVE_RESET()             SETBIT( PORTD, SLAVE_RESET_ )

/* Master/Slave activity monitoring ('watchdog' function) */
#define MASTER_TO_SLAVE                 2
#define SET_MASTER_TO_SLAVE_HIGH()      SETBIT( PORTD, MASTER_TO_SLAVE )
#define SET_MASTER_TO_SLAVE_LOW()       CLEARBIT( PORTD, MASTER_TO_SLAVE )
#define SET_MASTER_TO_SLAVE_OUTPUT()    SETBIT( DDRD, MASTER_TO_SLAVE )
#define SET_MASTER_TO_SLAVE_INPUT()     CLEARBIT( DDRD, MASTER_TO_SLAVE )
#define MASTER_TO_SLAVE_HIGH()          (PIND & BIT(MASTER_TO_SLAVE))
#define MASTER_TO_SLAVE_LOW()           !(PIND & BIT(MASTER_TO_SLAVE))

/* Jumpers */
#define BAUDRATE_ENABLE                 2
#define BAUDRATE_JUMPERS_SELECT()       CLEARBIT( PORTE, BAUDRATE_ENABLE )
#define BAUDRATE_JUMPERS_DESELECT()     SETBIT( PORTE, BAUDRATE_ENABLE )
#define BAUDRATE_JUMPERS_MASK           0x03
#define NODEID_ENABLE                   7
#define NODEID_JUMPERS_SELECT()         CLEARBIT( PORTB, NODEID_ENABLE )
#define NODEID_JUMPERS_DESELECT()       SETBIT( PORTB, NODEID_ENABLE )
#define NODEID_JUMPERS_MASK             0x3f

/* ISP serial port for programming of Slave microcontroller */
#define ISP_SCK                         1
#define SET_ISP_SCK()                   SETBIT( PORTD, ISP_SCK )
#define CLEAR_ISP_SCK()                 CLEARBIT( PORTD, ISP_SCK )
#define ISP_MOSI                        0
#define SET_ISP_MOSI()                  SETBIT( PORTE, ISP_MOSI )
#define CLEAR_ISP_MOSI()                CLEARBIT( PORTE, ISP_MOSI )
#define ISP_MISO                        1
#define ISP_MISO_SET()                  (PINE & BIT(ISP_MISO))

/* SAE81C91 CAN-controller */
#define CAN_W_                          5
#define CAN_WRITE_ENABLE()              CLEARBIT( PORTB, CAN_W_ )
#define CAN_READ_ENABLE()               SETBIT( PORTB, CAN_W_ )
#define CAN_CS_                         6
#define CAN_SELECT()                    CLEARBIT( PORTB, CAN_CS_ )
#define CAN_DESELECT()                  SETBIT( PORTB, CAN_CS_ )
#define CAN_INT_                        1
#define CAN_INT_HIGH()                  (PIND & BIT(CAN_INT_))
#define CAN_INT_ENABLE()                EIMSK |= BIT(INT1)
#define CAN_INT_DISABLE()               EIMSK &= ~BIT(INT1)

/* SPI serial interface */
#define SCLK                            1
#define SET_SCLK()                      SETBIT( PORTB, SCLK )
#define CLEAR_SCLK()                    CLEARBIT( PORTB, SCLK )
#define SDI                             2
#define SET_SDI()                       SETBIT( PORTB, SDI )
#define CLEAR_SDI()                     CLEARBIT( PORTB, SDI )
#define SDO                             3
#define SDO_SET()                       (PINB & BIT(SDO))

/* ------------------------------------------------------------------------ */
/* Baudrate jumper readings */

#define BAUD50K                         0x00
#define BAUD500K                        0x01
#define BAUD250K                        0x02
#define BAUD125K                        0x03

#endif

/* ------------------------------------------------------------------------ */
