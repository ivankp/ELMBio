/* ------------------------------------------------------------------------
File   : can.h

Descr  : Definitions and declarations for CAN message-to-controller-buffer
         mappings, CANopen message sending and low-level CAN-controller access
	 functions.

History: 19JAN.00; Henk B&B; First version.
         20SEP.00; Henk B&B; Left application-specific stuff here,
	                     moved rest to include file 81c91.h.
	 09OCT.02; Henk B&B; Included "canopen.h": used where "can.h" is used.
	   MAY.03; Henk B&B; Added receive message buffering under interrupt.
--------------------------------------------------------------------------- */

#ifndef CAN_H
#define CAN_H

#include "81c91.h"
#include "canopen.h"

/* ------------------------------------------------------------------------ */
/* Globals */

extern BYTE NodeID;
extern BYTE CANopenErrorReg;

/* ------------------------------------------------------------------------ */
/* CANopen object mapping to the SAE81C91 CAN-controller buffers */

/* Message/buffer number of the CANopen objects
   (NB: Remote Transmission Requests can only be received by buffer 0!) */
#define C91_RTR                         0
#define C91_NMT                         1
#define C91_SYNC                        2
#define C91_EMERGENCY                   3
#define C91_SDOTX                       4
#define C91_SDORX                       5
#define C91_NODEGUARD                   6
#define C91_TPDO2                       7
#define C91_TPDO1                       8
#define C91_RPDO1                       9
#define C91_RPDO2                       10
#define C91_TPDO3                       11
#define C91__FREE__4                    12
#define C91__FREE__3                    13
#define C91__FREE__2                    14
#define C91__FREE__1                    15

/* Same COB-ID for 2 different CANopen objects */
#define C91_BOOTUP                      C91_NODEGUARD

/* Message numbers for Remote Transmission Requests (RTR)
   received in 81C91 buffer 0 (C91_RTR) */
#define C91_NODEGUARD_RTR               (C91_NODEGUARD + 16)
#define C91_TPDO1_RTR                   (C91_TPDO1     + 16)
#define C91_TPDO2_RTR                   (C91_TPDO2     + 16)
#define C91_TPDO3_RTR                   (C91_TPDO3     + 16)

/* If no CAN-message is available the following ID is returned */
#define NO_OBJECT                       32

/* ------------------------------------------------------------------------ */
/* CANopen object message lengths */

#define C91_NMT_LEN                     2
#define C91_SYNC_LEN                    0
#define C91_EMERGENCY_LEN               8
#define C91_SDOTX_LEN                   8
#define C91_SDORX_LEN                   8
#define C91_NODEGUARD_LEN               1
#define C91_BOOTUP_LEN                  C91_NODEGUARD_LEN
#define C91_TPDO1_LEN                   2
#define C91_TPDO2_LEN                   4
#define C91_RPDO1_LEN                   2
#define C91_RPDO2_LEN                   3
#define C91_TPDO3_LEN                   6

/* ------------------------------------------------------------------------ */
/* Function prototypes */

BYTE can_read_reg         ( BYTE regaddr );
void can_write_reg        ( BYTE regaddr, BYTE byt );
void can_init             ( BOOL init_msg_buffer );
BOOL can_msg_available    ( void );
BYTE can_read             ( BYTE *pdlc, BYTE **ppmsg_data );
void can_write            ( BYTE object_no, BYTE len, BYTE *msg_data );
void can_write_bootup     ( void );
void can_write_emergency  ( BYTE err_low,
			    BYTE err_high,
			    BYTE mfct_field_0,
			    BYTE mfct_field_1,
			    BYTE mfct_field_2,
			    BYTE mfct_field_3,
			    BYTE canopen_err_bit );
BOOL can_transmitting     ( BYTE object_no );
void can_check_for_errors ( void );
void can_rtr_enable       ( BOOL enable );
BOOL can_set_rtr_disabled ( BOOL disable );
BOOL can_get_rtr_disabled ( void );
BYTE canopen_init_state   ( void );
BOOL can_set_opstate_init ( BOOL enable );
BOOL can_get_opstate_init ( void );
BOOL can_set_busoff_maxcnt( BYTE cntr );
BYTE can_get_busoff_maxcnt( void );
BOOL can_store_config     ( void );

#endif /* CAN_H */
/* ------------------------------------------------------------------------ */
