/* ------------------------------------------------------------------------
File   : 81c91.h

Descr  : Include file containing constants concerning the
         SIEMENS SAE81C91 CAN-controller.

History: 19JAN.00; Henk B&B; First version.
         20SEP.00; Henk B&B; Left application-specific stuff in 'can.h'
	                     and renamed this part '81c91.h'.
--------------------------------------------------------------------------- */

#ifndef SAE81C91_H
#define SAE81C91_H

/* ------------------------------------------------------------------------ */
/* SAE81C91 CAN-controller register addresses and register bits */

#define C91_BL1_I                       0x00
#define C91_BL2_I                       0x01
#define C91_OUTPUTCONTROL_I             0x02
#define C91_BRP_I                       0x03

#define C91_RECV_READY1_I               0x04
#define C91_RECV_READY2_I               0x05

#define C91_RECV_INTERRUPT_MASK1_I      0x06
#define C91_RECV_INTERRUPT_MASK2_I      0x07

#define C91_TRANSMIT_REQ1_I             0x08
#define C91_TRANSMIT_REQ2_I             0x09

#define C91_INTERRUPT_MASK_I            0x0A

#define C91_MODE_STATUS_I               0x10
#define C91_INTERRUPT_I                 0x11
#define C91_CONTROL_I                   0x12

#define C91_CLOCKCONTROL_I              0x14

#define C91_REMOTE_PENDING1_I           0x1A
#define C91_REMOTE_PENDING2_I           0x1B

/* Mode/Status Register bits */
#define C91_ADE                         0x80
#define C91_RS                          0x40
#define C91_TC                          0x20
#define C91_TWL                         0x10
#define C91_RWL                         0x08
#define C91_BS                          0x04
#define C91_RES                         0x02
#define C91_IM                          0x01

/* Interrupt(-Mask) Register bits */
#define C91_TRANSM_CHECK_INT            0x80
#define C91_ERROR_PASSIVE_INT           0x40
#define C91_BUS_OFF_INT                 0x20
#define C91_WAKEUP_INT                  0x10
#define C91_REMOTE_FRAME_INT            0x08
#define C91_WARNING_LEVEL_INT           0x04
#define C91_TRANSM_INT                  0x02
#define C91_RECV_INT                    0x01

/* Control Register bits */
#define C91_MONITOR_MODE                0x01
#define C91_TRANSMIT_CHECK_ENABLE       0x02

/* Number of message buffers */
#define C91_MSG_BUFFERS                 16
#define C91_MSG_BUFFERS_PER_RRR         8

/* Descriptor Registers */
#define C91_DR00_I                      0x40
#define C91_DR01_I                      0x42
#define C91_DR02_I                      0x44
#define C91_DR03_I                      0x46
#define C91_DR04_I                      0x47
#define C91_DR05_I                      0x4A
#define C91_DR06_I                      0x4C
#define C91_DR07_I                      0x4E
#define C91_DR08_I                      0x50
#define C91_DR09_I                      0x52
#define C91_DR10_I                      0x54
#define C91_DR11_I                      0x56
#define C91_DR12_I                      0x58
#define C91_DR13_I                      0x5A
#define C91_DR14_I                      0x5C
#define C91_DR15_I                      0x5E

/* Descriptor Register bit masks */
#define C91_DR_RTR_MASK                 0x10
#define C91_DR_DLC_MASK                 0x0F

/* Offset to message buffers */
#define C91_MSGS_I                      0x80

/* Message buffer size */
#define C91_MSG_SIZE                    8

/* ------------------------------------------------------------------------ */
/* SAE81C91 CAN-controller bit timings */

#define __ELMB_TIMINGS__
//#define __CIA_TIMINGS__
//#define __LMB_TIMINGS__

/* Note: TSEG1 = 1+TS1, TSEG2 = 1+TS2, TSJW = 1 + SJW */

#ifdef __ELMB_TIMINGS__

/* Settings used for ELMB (due to inaccurate clock crystal): */
/* --------------------------------------------------------- */

/*   50 kbit/s: BRP=4, TS1=8, TS2=5, SJW=2 */
#define C91_BRP_50K                     0x04
#define C91_BL1_50K                     0x58
#define C91_BL2_50K                     0x42

/*  125 kbit/s: BRP=1, TS1=8, TS2=5, SJW=2 */
#define C91_BRP_125K                    0x01
#define C91_BL1_125K                    0x58
#define C91_BL2_125K                    0x42

/*  250 kbit/s: BRP=0, TS1=8, TS2=5, SJW=2 */
#define C91_BRP_250K                    0x00
#define C91_BL1_250K                    0x58
#define C91_BL2_250K                    0x42

/*  500 kbit/s: BRP=0, TS1=3, TS2=2, SJW=1 */
#define C91_BRP_500K                    0x00
#define C91_BL1_500K                    0x23
#define C91_BL2_500K                    0x41

#endif /* __ELMB_TIMINGS__ */

#ifdef __CIA_TIMINGS__

/* CiA CANopen recommended settings: */
/* --------------------------------- */

/*   50 kbit/s: BRP=4, TS1=12, TS2=1, SJW=0 */
#define C91_BRP_50K                     0x04
#define C91_BL1_50K                     0x1C
#define C91_BL2_50K                     0x40

/*  125 kbit/s: BRP=1, TS1=12, TS2=1, SJW=0 */
#define C91_BRP_125K                    0x01
#define C91_BL1_125K                    0x1C
#define C91_BL2_125K                    0x40

/*  250 kbit/s: BRP=0, TS1=12, TS2=1, SJW=0 */
#define C91_BRP_250K                    0x00
#define C91_BL1_250K                    0x1C
#define C91_BL2_250K                    0x40

/*  500 kbit/s: BRP=0, TS1= 5, TS2=0, SJW=0 */
#define C91_BRP_500K                    0x00
#define C91_BL1_500K                    0x05
#define C91_BL2_500K                    0x40

/* 1000 kbit/s: BRP=0, TS1= 1, TS2=0, SJW=0 */
#define C91_BRP_1000K                   0x00
#define C91_BL1_1000K                   0x01
#define C91_BL2_1000K                   0x40

#endif /* __CIA_TIMINGS__ */

#endif /* SAE81C91_H */
/* ------------------------------------------------------------------------ */
