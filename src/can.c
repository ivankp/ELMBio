/* ------------------------------------------------------------------------
File   : can.c

Descr  : Functions for serial operation of an 81C91 CAN-controller.

History: 19JAN.00; Henk B&B; Definition.
         31JAN.00; Henk B&B; Beware!: start reading/writing message
	                     with most-significant byte (7) and end
			     with data byte 0.
	 21JUL.00; Henk B&B; Enable the use of all 16 message objects
	                     (up to now just the first 8 were considered).
	 22FEB.01; Henk B&B; Keep some variables in EEPROM (optionally).
	 17JUL.01; Henk B&B; Go to sleep for a while at bus-off, before
	                     reinitialising.
	 17OCT.01; Henk B&B; Implement CAN descriptor register refresh.
	 27FEB.03; Henk B&B; Add a CAN configuration parameters datablock
	                     (in EEPROM);
			     add possibility to disable Remote Frames,
	                     through the Object Dictionary, but keep
			     Nodeguarding possible (through automatic replies).
	   MAY.03; Henk B&B; Added receive message buffering under interrupt.
	 02JUN.03; Henk B&B; Add to CAN configuration parameters datablock:
			     - add CANopenOpStateInit bool, to automatically
			       go to state Operational at startup
			     - add Bus-off max counter value
	 26SEP.03; Henk B&B; Added can_recv_descriptor_refresh();
	                     call it in can_read() as well as can_write(),
			     to refresh descriptors of all receiving buffers.
	 23OCT.03; Henk B&B; Enable the 81c91's Transmit Check feature.
	 25FEB.04; Henk B&B; Added toggle bit to Emergency message.
	 31MAR.04; Henk B&B; remove can_recv_descriptor_refresh() from
	                     can_write(): message loss is possible.
--------------------------------------------------------------------------- */

#include "general.h"
#include "can.h"
#include "guarding.h"
#include "jumpers.h"
#include "pdo.h"
#include "spi.h"
#include "store.h"
#include "timer103.h"

#ifdef __VARS_IN_EEPROM__
#include "eeprom.h"
#endif

/* CANopen state of this node (declared in ELMBmain.c) */
extern BYTE NodeState;

/* ------------------------------------------------------------------------ */
/* CAN message buffering in RAM */

/* Number of message buffers (here must be a power of 2!) */
#define CAN_BUFS        64
#define CAN_BUFS_MASK   CAN_BUFS-1
/* Size of message buffer */
#define CAN_BUF_SIZE    11

/* Indices into the individual buffers */
#define MSG_DATA_I      0
#define MSG_DLC_I       8
#define MSG_OBJECT_I    9
#define MSG_VALID_I     10

/* Buffer values:
   'message-present' byte in location MSG_VALID_I:
   0x00 means 'message present', 0xFF means 'no message present'.
   To tolerate up to 2 bitflips:
   bits-in-byte <= 2: message
   bits-in-byte >= 6: no message */
#define BUF_EMPTY       0xFF
#define BUF_NOT_EMPTY   0x00

/* Array of CAN message buffers */
static BYTE CanMsgBuf[CAN_BUFS][CAN_BUF_SIZE];

static BOOL CanBufFull;

/* Parameters which are being used according to a majority voting mechanism:
   MsgOutIndex_ : index of the first-to-be-handled CAN-message buffer,
   MsgCounter_: number of unhandled CAN-messages in buffers */
static BYTE MsgOutIndex1, MsgOutIndex2, MsgOutIndex3;
static BYTE MsgCounter1,  MsgCounter2,  MsgCounter3;

/* NB: make sure to disable the CAN INT interrupt before writing or reading
   anything to/from the CAN-controller in the main program loop !
   (and also when updating the buffer counter in the main loop) */

/* ------------------------------------------------------------------------ */
/* CAN-controller message object descriptors */

/* Description Registers settings for NMT, SDO-tx/SDO-rx, Emergency,
   Bootup/Nodeguard, SYNC, PDOs, etc.
   (according to the CANopen Predefined Connection Set);
   in the following array is space for all available message buffers;
   NB: to simplify refreshing the descriptor registers the CAN_DESCRIPTOR
       array contains the values in order of message buffer number to
       write to; make absolutely sure this list matches the message/buffer
       numbering in can.h !! */

const BYTE CAN_DESCRIPTOR[C91_MSG_BUFFERS][2] =
{
  /* Buffer 0 must be used if we want to filter out RTRs for this node
     (and to let them be handled by the microcontroller!) */
  { 0xFF,		0xE0 },

  /* NMT */
  { NMT_OBJ,		C91_NMT_LEN },

  /* SYNC */
  { SYNC_OBJ,		C91_SYNC_LEN },

  /* EMERGENCY */
  { EMERGENCY_OBJ,	C91_EMERGENCY_LEN },

  /* SDO-Transmit */
  { SDOTX_OBJ,		C91_SDOTX_LEN },

  /* SDO-Receive */
  { SDORX_OBJ,		C91_SDORX_LEN },

  /* Bootup/Nodeguard */
  { NODEGUARD_OBJ,	C91_NODEGUARD_LEN },

  /* Transmit-PDO2 */
  { TPDO2_OBJ,		C91_TPDO2_LEN },

  /* Transmit-PDO1 */
  { TPDO1_OBJ,		C91_TPDO1_LEN },

  /* Receive-PDO1 */
  { RPDO1_OBJ,		C91_RPDO1_LEN },

  /* Receive-PDO2 */
  { RPDO2_OBJ,		C91_RPDO2_LEN },

  /* Transmit-PDO3 */
  { TPDO3_OBJ,		C91_TPDO3_LEN },

  /* ---- object available ---- */
  { 0xFF,		0xE0 },

  /* ---- object available ---- */
  { 0xFF,		0xE0 },

  /* ---- object available ---- */
  { 0xFF,		0xE0 },

  /* ---- object available ---- */
  { 0xFF,		0xE0 }
};

/* This array of BOOLs indicates which CAN-controller buffers
   are receiving buffers; their descriptors are refreshed
   (if __CAN_REFRESH__ is defined) in can_read() and can_write()
   Make sure this array matches CAN_DESCRIPTOR[] !
   Do not refresh RTR buffer like this: RTR may get lost */
const BOOL CANBUF_IS_RECV[C91_MSG_BUFFERS] =
{
  FALSE, TRUE,  TRUE,  FALSE,
  FALSE, TRUE,  FALSE, FALSE,
  FALSE, TRUE,  TRUE,  FALSE,
  FALSE, FALSE, FALSE, FALSE
};

/* Index for CANBUF_IS_RECV[] */
static BYTE CanRefreshIndex;

/* CAN-controller register settings for baudrate configuration */
const BYTE CAN_BAUDRATE_CONFIGS[4][3] =
{
  { C91_BRP_50K,	C91_BL1_50K,	C91_BL2_50K },
  { C91_BRP_500K,	C91_BL1_500K,	C91_BL2_500K },
  { C91_BRP_250K,	C91_BL1_250K,	C91_BL2_250K },
  { C91_BRP_125K,	C91_BL1_125K,	C91_BL2_125K }
};

/* ------------------------------------------------------------------------ */
/* Globals */

BYTE        NodeID;      /* (stored in EEPROM) */

BYTE        CANopenErrorReg;

/* Counter for errors in the CAN communication */
static BYTE CanErrorCntr;

BOOL        RtrDisabled; /* (stored in EEPROM) */

/* Boolean to enable 'autostart' (automatically goto Operational mode) */
static BOOL CANopenOpStateInit; /* (copy in EEPROM) */

/* Max counter value for bus access retries after Bus-off events */
static BYTE CanBusOffMaxCnt;    /* (copy in EEPROM) */

/* Bus-off events counter */
BYTE        CanBusOffCnt = 0;

/* Help variables for RTR reception */
static BYTE RtrIdHi;     /* (stored in EEPROM) */
static BYTE RtrIdLo;     /* (stored in EEPROM) */

/* Help variables for CAN message reception */
static BYTE ObjectMask1;
static BYTE ObjectMask2;

/* Toggle bit for the Emergency CAN-message */
static BYTE CanEmgToggle = 0x80;

/* ------------------------------------------------------------------------ */
/* Local prototypes */

static BYTE can_check_for_msgs( void );
static BYTE get_buf_index     ( void );
static BYTE get_buf_cntr      ( void );
static BYTE bits_in_byte      ( BYTE val );

static void can_load_config   ( void );

#ifdef __CAN_REFRESH__
static void can_descriptor_refresh( BYTE object_no );
static void can_recv_descriptor_refresh( void );
#endif /* __CAN_REFRESH__ */

/* ------------------------------------------------------------------------ */

BYTE can_read_reg( BYTE regaddr )
{
  BYTE byt;

  /* Select CAN-controller and read-mode */
  CAN_SELECT();
  CAN_READ_ENABLE();

  spi_write( regaddr );
  byt = spi_read();

  /* Deselect CAN-controller */
  CAN_DESELECT();

  return byt;
}

/* ------------------------------------------------------------------------ */

void can_write_reg( BYTE regaddr, BYTE byt )
{
  /* Select CAN-controller and write-mode */
  CAN_SELECT();
  CAN_WRITE_ENABLE();

  spi_write( regaddr );
  spi_write( byt );

  /* Deselect CAN-controller */
  CAN_DESELECT();
}

/* ------------------------------------------------------------------------ */

void can_init( BOOL init_msg_buffer )
{
  BYTE baudrate;
  BYTE i, canaddr;
  BYTE id_hi, id_lo;
  BYTE bufno;

  /* Disable interrupt */
  CAN_INT_DISABLE();

  /* Initialise configuration parameters */
  can_load_config();

  /* Reset error counter */
  if( init_msg_buffer ) CanErrorCntr = 0;

  /* Initialize CANopen Error Register Object (Object 0x1001) */
  CANopenErrorReg = 0x00;

  /* Prepare PORTB for jumper read-out */
  DDRB  = PORTB_DDR_FOR_JUMPERS;
  PORTB = PORTB_DATA_FOR_JUMPERS;

  /* Read the jumper settings */
  NodeID   = read_nodeid();
  baudrate = read_baudrate();

  /* Set PORTB back to 'operational' setting */
  DDRB  = PORTB_DDR_OPERATIONAL;
  PORTB = PORTB_DATA_OPERATIONAL;

  /* Set CAN-controller in configuration mode */
  can_write_reg( C91_MODE_STATUS_I, C91_RES | C91_IM );

  /* Initialise registers 0 to 0x0A to zero */
  for( i=0; i<0x0B; ++i ) can_write_reg( i, 0x00 );

  /* Enable Monitor Mode to enable reception of RTRs (by CPU!) ?
     ###BEWARE: all messages not received by any of the other
                buffers are now accepted in buffer 0...
		could be a bit much if there are many nodes
		(many messages) on the bus... */
  can_write_reg( C91_CONTROL_I, 0x00 ); /* Do not enable Monitor Mode */

  /* Enable the 81C91 controller's Transmit Check feature */
  can_write_reg( C91_CONTROL_I, can_read_reg(C91_CONTROL_I) |
		 C91_TRANSMIT_CHECK_ENABLE );

  /* Reset bits in Interrupt Register */
  can_write_reg( C91_INTERRUPT_I, 0x00 );

  /* Output Control */
  can_write_reg( C91_OUTPUTCONTROL_I, 0x18 );

  /* Clock Control */
  can_write_reg( C91_CLOCKCONTROL_I, 0x80 );
  can_write_reg( C91_CLOCKCONTROL_I, 0x01 );

  /* Write the settings for the required CAN-bus baudrate */
  can_write_reg( C91_BRP_I, CAN_BAUDRATE_CONFIGS[baudrate][0] );
  can_write_reg( C91_BL1_I, CAN_BAUDRATE_CONFIGS[baudrate][1] );
  can_write_reg( C91_BL2_I, CAN_BAUDRATE_CONFIGS[baudrate][2] );

  /* Node-ID for Description Register is split over 2 bytes */
  id_hi = NodeID >> 3;
  id_lo = NodeID << 5;

  /* Initialise the Object Descriptor Registers */
  canaddr = C91_DR00_I;
  for( bufno=0; bufno<C91_MSG_BUFFERS; ++bufno )
    {
      BYTE desc_hi, desc_lo;

      /* Use the corresponding descriptor bytes */
      desc_hi = CAN_DESCRIPTOR[bufno][0];
      desc_lo = CAN_DESCRIPTOR[bufno][1];

      /* NMT and SYNC are broadcast messages: Node-ID is not in */
      if( bufno != C91_NMT && bufno != C91_SYNC )
	{
	  /* Node-ID is included in COB-ID */
	  desc_hi |= id_hi;
	  desc_lo |= id_lo;
	}

      /* Write the CAN-controller's Descriptor Registers */
      can_write_reg( canaddr, desc_hi );
      ++canaddr;
      can_write_reg( canaddr, desc_lo );
      ++canaddr;
    }

  /* Receive-Interrupt Mask Registers
     (in combination with the Interrupt Mask setting below) */
  can_write_reg( C91_RECV_INTERRUPT_MASK1_I, 0xFF );
  can_write_reg( C91_RECV_INTERRUPT_MASK2_I, 0xFF );

  /* Enable INT pin interrupt for received messages only */
  can_write_reg( C91_INTERRUPT_MASK_I, C91_RECV_INT ); 

  /* If Remote Frames are not required adjust
     the CAN-controller's configuration: disable Monitor Mode for buffer 0
     and enable automatic RTR for NODEGUARD messages */
  can_rtr_enable( pdo_rtr_required() );

  /* Set CAN-controller to operational mode */
  can_write_reg( C91_MODE_STATUS_I, 0x00 );

  /* Bit masks for objects received */
  ObjectMask1 = 0;
  ObjectMask2 = 0;

  /* Help variables for RTR reception */
  RtrIdHi = id_hi;
  RtrIdLo = id_lo | C91_DR_RTR_MASK;

#ifdef __VARS_IN_EEPROM__
  /* Create working copies of configuration globals in EEPROM */
  if( eeprom_read( EE_NODEID ) != NodeID )
    eeprom_write( EE_NODEID, NodeID );
  if( eeprom_read( EE_RTRIDHI ) != RtrIdHi )
    eeprom_write( EE_RTRIDHI, RtrIdHi );
  if( eeprom_read( EE_RTRIDLO ) != RtrIdLo )
    eeprom_write( EE_RTRIDLO, RtrIdLo );
#endif /* __VARS_IN_EEPROM__ */

  if( init_msg_buffer )
    {
      /* Initialize the CAN-message buffer and management stuff */
      for( bufno=0; bufno<CAN_BUFS; ++bufno )
	CanMsgBuf[bufno][MSG_VALID_I] = BUF_EMPTY;
      CanBufFull = FALSE;
      MsgOutIndex1 = 0; MsgOutIndex2 = 0; MsgOutIndex3 = 0;
      MsgCounter1  = 0; MsgCounter2  = 0; MsgCounter3  = 0;
    }

  /* Enable interrupt */
#ifndef __ELMB103__
  /* Low level of INT1 generates an interrupt
     (= default and unchangeable on ATmega103) */
  EICRA &= ~(BIT(ISC11) | BIT(ISC10));
#endif
  CAN_INT_ENABLE(); /* Enable interrupt from CAN-controller */

  CanRefreshIndex = C91_MSG_BUFFERS-1;
}

/* ------------------------------------------------------------------------ */

BOOL can_msg_available( void )
{
  BOOL available;

  /* Need undisturbed access to buffer management variables !
     (because even the get_buf_xxx() functions may alter variables,
     due to the (self-correcting) majority voting mechanism) */
  CAN_INT_DISABLE();

  /* CAN-message available in buffer ? */
  available = (get_buf_cntr() > 0);

  CAN_INT_ENABLE();

#ifdef __VARS_IN_EEPROM__
  /* EEPROM access removed from CAN interrupt handling, do it here instead ! */
  if( !available )
    {
      NodeID  = eeprom_read( EE_NODEID );
      RtrIdLo = eeprom_read( EE_RTRIDLO );
      RtrIdHi = eeprom_read( EE_RTRIDHI );
    }
#endif /* __VARS_IN_EEPROM__ */

  return available;
}

/* ------------------------------------------------------------------------ */

BYTE can_read( BYTE *pdlc, BYTE **ppmsg_data )
{
  BYTE cntr, object_no;

  /* Need undisturbed access to buffer management variables !
     (because even the get_buf_xxx() functions may alter variables,
     due to the (self-correcting) majority voting mechanism) */
  CAN_INT_DISABLE();

  cntr = get_buf_cntr();

  /* Is any message available in the buffer ? */
  if( cntr > 0 )
    {
      BYTE index, *msg;

      index     = get_buf_index();
      msg       = CanMsgBuf[index];
      *pdlc     = msg[MSG_DLC_I];
      object_no = msg[MSG_OBJECT_I];

      /* We don't make a copy of the data in the buffer
	 (so make sure it does not get overwritten: see 'canint_handler()');
	 return a pointer that points to the databytes in the message buffer */
      *ppmsg_data = msg;

      /* Make sure the message in the buffer is valid;
	 skip any 'empty' buffers (although this should be very unlikely...) */
      if( bits_in_byte(msg[MSG_VALID_I]) >= 6 )
	{
	  /* Not valid..., skip this message... */
	  object_no = NO_OBJECT;
	}

      /* Mark buffer as empty
	 (NB: the contained message has not been handled yet!) */
      msg[MSG_VALID_I] = BUF_EMPTY;

      /* Decrement the messages-in-buffer counter */
      --cntr;
      MsgCounter1 = cntr;  MsgCounter2 = cntr;  MsgCounter3 = cntr;

      /* Increment the buffer(-out) index to point to
	 the next message to be handled (next time around) */
      index = (index + 1) & CAN_BUFS_MASK;
      MsgOutIndex1 = index;  MsgOutIndex2 = index;  MsgOutIndex3 = index;

#ifdef __CAN_REFRESH__
      /* Refresh the descriptors of one of the receiving buffers */
      can_recv_descriptor_refresh();
#endif /* __CAN_REFRESH__ */
    }
  else
    {
      object_no = NO_OBJECT;
    }

  CAN_INT_ENABLE();

  return object_no;;
}

/* ------------------------------------------------------------------------ */

void can_write( BYTE object_no, BYTE len, BYTE *msg_data )
{
  BYTE addr;
  signed char byt;

  /* Legal message object ? */
  if( object_no > C91_MSG_BUFFERS-1 ) return;

  CAN_INT_DISABLE(); /* Need undisturbed access to CAN-controller ! */

#ifdef __CAN_REFRESH__
  /* Refresh descriptor registers for this CAN message */
  can_descriptor_refresh( object_no );
#endif /* __CAN_REFRESH__ */

  /* Determine object's message buffer address */
  addr = C91_MSGS_I + (object_no * C91_MSG_SIZE);

  /* Write the data bytes to the message buffer;
     go from MSB to byte 0...! */
  for( byt=len-1; byt>=0; --byt ) can_write_reg( addr+byt, msg_data[byt] );

  /* Set the appropriate transmission request bit */
  if( object_no > C91_MSG_BUFFERS_PER_RRR-1 )
    can_write_reg( C91_TRANSMIT_REQ2_I,
		   BIT(object_no - C91_MSG_BUFFERS_PER_RRR) );
  else
    can_write_reg( C91_TRANSMIT_REQ1_I, BIT(object_no) );

#ifdef __CAN_REFRESH__
  /* Refresh the descriptors of one of the receiving buffers */
  /* ### Don't: you may loose a message */
  //can_recv_descriptor_refresh();
#endif /* __CAN_REFRESH__ */

  CAN_INT_ENABLE();
}

/* ------------------------------------------------------------------------ */

void can_write_bootup( void )
{
  BYTE can_data[C91_BOOTUP_LEN];
  BYTE ng;

  /* RTR bit (for NodeGuard message) -if present- must be temporarily removed
     when sending the BOOTUP message (Bootup and NodeGuard use the same
     message buffer!) and is restored by can_rtr_enable() */
  CAN_INT_DISABLE();
  ng = can_read_reg( C91_DR00_I+C91_NODEGUARD*2+1 );
  can_write_reg( C91_DR00_I+C91_NODEGUARD*2+1, ng & (~C91_DR_RTR_MASK) );
  CAN_INT_ENABLE();

  /* Send the CANopen Bootup message */
  can_data[0] = 0;
  can_write( C91_BOOTUP, C91_BOOTUP_LEN, can_data );

  /* Restore RTR bit, if required */
  can_rtr_enable( pdo_rtr_required() );
}

/* ------------------------------------------------------------------------ */

void can_write_emergency( BYTE err_low,
			  BYTE err_high,
			  BYTE mfct_field_0,
			  BYTE mfct_field_1,
			  BYTE mfct_field_2,
			  BYTE mfct_field_3,
			  BYTE canopen_err_bit )
{
  BYTE msg_data[8];

  /* CANopen error code */
  msg_data[0] = err_low;
  msg_data[1] = err_high;

  /* Update CANopen Error Register */
  CANopenErrorReg |= canopen_err_bit;

  /* Add CANopen Error Register (OD object 0x1001) to message */
  msg_data[2] = CANopenErrorReg;

  /* CANopen manufacturer specific error field */
  msg_data[3] = mfct_field_0;
  msg_data[4] = mfct_field_1;
  msg_data[5] = mfct_field_2;
  msg_data[6] = mfct_field_3;
  msg_data[7] = (CanEmgToggle & 0x80);

  /* Make sure the message does not get lost:
     wait for the previous one to be sent */
  {
    BYTE delay=0;
    while( can_transmitting(C91_EMERGENCY) && delay < 50 )
      {
	/* So wait, but not forever... */
	timer2_delay_ms( 1 );
	++delay;
      }
  }

  can_write( C91_EMERGENCY, C91_EMERGENCY_LEN, msg_data );

  /* Toggle the toggle bit */
  CanEmgToggle ^= 0x80;
}

/* ------------------------------------------------------------------------ */

void can_check_for_errors( void )
{
  BYTE interrupts, status;

  CAN_INT_DISABLE();

  interrupts = can_read_reg( C91_INTERRUPT_I );

  /* Detect whether a Nodeguarding message has been (automatically) serviced
     and if so, toggle the toggle-bit ! */
  if( interrupts & C91_REMOTE_FRAME_INT )
    {
      /* Invert the toggle bit */
      NodeGuardToggle ^= 0x80;

      /* Reset the RTR interrupt bit */
      can_write_reg( C91_INTERRUPT_I, (BYTE)(~C91_REMOTE_FRAME_INT) );

      /* Reset the Life Guarding time-out counter */
      TIMER1_DISABLE();
      LifeGuardCntr = 0;
      TIMER1_ENABLE();

      /* Update toggle bit in NodeGuard message buffer */
      can_write_reg( C91_MSGS_I + (C91_NODEGUARD*C91_MSG_SIZE),
		     NodeState | (NodeGuardToggle & 0x80) );
    }

  CAN_INT_ENABLE();

  /* Filter out error bits */
  interrupts &= (C91_WARNING_LEVEL_INT | C91_BUS_OFF_INT |
		 C91_ERROR_PASSIVE_INT | C91_TRANSM_CHECK_INT);

  if( interrupts )
    {
      BYTE err_cntr;

      ++CanErrorCntr;

      /* Save error counter: restored in case of initalization */
      err_cntr = CanErrorCntr;

      CAN_INT_DISABLE();
      status = can_read_reg( C91_MODE_STATUS_I );
      CAN_INT_ENABLE();

      if( interrupts & C91_BUS_OFF_INT )
	{
	  /* Aye... we're off the bus, try to recover,
	     but not before taking a break... */
	  timer2_delay_ms( 50 );
	  timer2_delay_ms( 50 );

	  ++CanBusOffCnt;
#ifdef __VARS_IN_EEPROM__
	  CanBusOffMaxCnt = eeprom_read( EE_CAN_BUSOFF_MAXCNT );
#endif /* __VARS_IN_EEPROM__ */

	  /* Regain access to CAN-bus,
	     unless the number of re-inits exceeds a preset value */
	  if( CanBusOffCnt <= CanBusOffMaxCnt ) can_init( FALSE );

	  /* Restore error counter */
	  CanErrorCntr = err_cntr;
	}

      can_write_emergency( 0x00, 0x81, interrupts, status,
			   err_cntr, CanBusOffCnt, ERRREG_COMMUNICATION );

      /* Reset interrupt bits (except the Remote Frame interrupt) */
      CAN_INT_DISABLE();
      can_write_reg( C91_INTERRUPT_I, (~interrupts) | C91_REMOTE_FRAME_INT );
      CAN_INT_ENABLE();
    }

  if( (CanBufFull & TRUE) == TRUE )
    {
      /* Report it: 'CAN overrun' emergency type */
      can_write_emergency( 0x10, 0x81, 0, 0, 0, 0, ERRREG_COMMUNICATION );
      CanBufFull = FALSE;
    }
}

/* ------------------------------------------------------------------------ */

BOOL can_transmitting( BYTE object_no )
{
  BOOL not_ready;
  not_ready = TRUE;

  CAN_INT_DISABLE();

  if( object_no < C91_MSG_BUFFERS_PER_RRR )
    {
      not_ready = can_read_reg(C91_TRANSMIT_REQ1_I) & BIT(object_no);
    }
  else
    {
      object_no -= C91_MSG_BUFFERS_PER_RRR;
      not_ready = can_read_reg(C91_TRANSMIT_REQ2_I) & BIT(object_no);
    }

  CAN_INT_ENABLE();

  return not_ready;
}

/* ------------------------------------------------------------------------ */

void can_rtr_enable( BOOL enable )
{
  BYTE ctrl, ng, delay;

  CAN_INT_DISABLE();

  ctrl = can_read_reg( C91_CONTROL_I );
  ng   = can_read_reg( C91_DR00_I + C91_NODEGUARD*2+1 );

#ifdef __VARS_IN_EEPROM__
  /* Refresh variable with copy in EEPROM */
  RtrDisabled = eeprom_read( EE_RTR_DISABLED );
#endif

  /* Set or reset Monitor Mode bit and at the same time
     reset or set RTR bit for automatic NodeGuard reply */
  if( enable == FALSE || RtrDisabled )
    {
      ctrl &= ~C91_MONITOR_MODE;
      ng   |= C91_DR_RTR_MASK;
    }
  else
    {
      ctrl |= C91_MONITOR_MODE;
      ng   &= ~C91_DR_RTR_MASK;
    }
  can_write_reg( C91_CONTROL_I, ctrl );

  /* Refresh the corresponding Descriptor Registers */
  can_write_reg( C91_DR00_I + C91_RTR*2,   CAN_DESCRIPTOR[C91_RTR][0] );
  can_write_reg( C91_DR00_I + C91_RTR*2+1, CAN_DESCRIPTOR[C91_RTR][1] );

  CAN_INT_ENABLE();

  /* Don't touch BOOTUP/NODEGUARD buffer until *after* a message has been sent.
     The following doesn't fully guarantee the message has been sent,
     but we'd like to keep it simple... */
  delay = 0;
  while( can_transmitting(C91_BOOTUP) && delay < 50 )
    {
      timer2_delay_ms( 1 );
      ++delay;
    }

  CAN_INT_DISABLE();

  can_write_reg( C91_DR00_I + C91_NODEGUARD*2+1, ng );

  /* Keep node state in NodeGuard message buffer up-to-date
     (in case we now switch from non-automatic (see guarding.c) to
     automatic reply, and at boot-up) */
  can_write_reg( C91_MSGS_I + (C91_NODEGUARD*C91_MSG_SIZE),
		 NodeState | (NodeGuardToggle & 0x80) );

  CAN_INT_ENABLE();
}

/* ------------------------------------------------------------------------ */

BOOL can_set_rtr_disabled( BOOL disable )
{
  if( disable > 1 ) return FALSE;
  if( disable )
    RtrDisabled = TRUE;
  else
    RtrDisabled = FALSE;

#ifdef __VARS_IN_EEPROM__
  if( eeprom_read( EE_RTR_DISABLED ) != RtrDisabled )
    eeprom_write( EE_RTR_DISABLED, RtrDisabled );
#endif /* __VARS_IN_EEPROM__ */

  /* Configure the CAN-controller */
  can_rtr_enable( pdo_rtr_required() );

  return TRUE;
}

/* ------------------------------------------------------------------------ */

BOOL can_get_rtr_disabled( void )
{
#ifdef __VARS_IN_EEPROM__
  RtrDisabled = eeprom_read( EE_RTR_DISABLED );
#endif /* __VARS_IN_EEPROM__ */

  return RtrDisabled;
}

/* ------------------------------------------------------------------------ */

#ifdef __CAN_REFRESH__
static void can_descriptor_refresh( BYTE object_no )
{
  BYTE desc_hi, desc_lo, addr;

  desc_hi = CAN_DESCRIPTOR[object_no][0];
  desc_lo = CAN_DESCRIPTOR[object_no][1];

  /* NMT and SYNC are broadcast messages: Node-ID is not in */
  if( object_no != C91_NMT && object_no != C91_SYNC )
    {
#ifdef __VARS_IN_EEPROM__
      /* ### Not in interrupt routine */
      //NodeID = eeprom_read( EE_NODEID );
#endif /* __VARS_IN_EEPROM__ */

      /* Node-ID is included in COB-ID */
      desc_hi |= (NodeID >> 3);
      desc_lo |= (NodeID << 5);
    }

  /* Write descriptor bytes */
  addr = C91_DR00_I + object_no*2;
  can_write_reg( addr, desc_hi );
  ++addr;
  can_write_reg( addr, desc_lo );
}
#endif /* __CAN_REFRESH__ */

/* ------------------------------------------------------------------------ */

#ifdef __CAN_REFRESH__
static void can_recv_descriptor_refresh( void )
{
  BYTE i;
  /* Find a next CAN-controller buffer which is a receiver
     and refresh its descriptor */
  for( i=0; i<C91_MSG_BUFFERS; ++i )
    {
      /* Modulo increment of index */
      CanRefreshIndex = (CanRefreshIndex + 1) & (C91_MSG_BUFFERS-1);

      if( CANBUF_IS_RECV[CanRefreshIndex] )
	{
	  can_descriptor_refresh( CanRefreshIndex );
	  /* Refresh only one descriptor at a time... */
	  break;
	}
    }
}
#endif /* __CAN_REFRESH__ */

/* ------------------------------------------------------------------------ */

BYTE canopen_init_state( void )
{
  BYTE state;

#ifdef __VARS_IN_EEPROM__
  CANopenOpStateInit = eeprom_read( EE_CANOPEN_OPSTATE_INIT );
#endif /* __VARS_IN_EEPROM__ */

  if( CANopenOpStateInit )
    state = NMT_OPERATIONAL;
  else
    state = NMT_PREOPERATIONAL;

  /* Update node state in NodeGuard message buffer
     (which may be sent automatically) */
  CAN_INT_DISABLE();
  can_write_reg( C91_MSGS_I + (C91_NODEGUARD*C91_MSG_SIZE),
		 state | (NodeGuardToggle & 0x80) );
  CAN_INT_ENABLE();

  return state;
}

/* ------------------------------------------------------------------------ */

BOOL can_set_opstate_init( BOOL enable )
{
  if( enable > 1 ) return FALSE;
  if( enable )
    CANopenOpStateInit = TRUE;
  else
    CANopenOpStateInit = FALSE;

#ifdef __VARS_IN_EEPROM__
  if( eeprom_read( EE_CANOPEN_OPSTATE_INIT ) != CANopenOpStateInit )
    eeprom_write( EE_CANOPEN_OPSTATE_INIT, CANopenOpStateInit );
#endif /* __VARS_IN_EEPROM__ */
  return TRUE;
}

/* ------------------------------------------------------------------------ */

BOOL can_get_opstate_init( void )
{
#ifdef __VARS_IN_EEPROM__
  CANopenOpStateInit = eeprom_read( EE_CANOPEN_OPSTATE_INIT );
#endif /* __VARS_IN_EEPROM__ */

  return CANopenOpStateInit;
}

/* ------------------------------------------------------------------------ */

BOOL can_set_busoff_maxcnt( BYTE cnt )
{
  CanBusOffMaxCnt = cnt;

#ifdef __VARS_IN_EEPROM__
  if( eeprom_read( EE_CAN_BUSOFF_MAXCNT ) != CanBusOffMaxCnt )
    eeprom_write( EE_CAN_BUSOFF_MAXCNT, CanBusOffMaxCnt );
#endif /* __VARS_IN_EEPROM__ */

  return TRUE;
}

/* ------------------------------------------------------------------------ */

BYTE can_get_busoff_maxcnt( void )
{
#ifdef __VARS_IN_EEPROM__
  CanBusOffMaxCnt = eeprom_read( EE_CAN_BUSOFF_MAXCNT );
#endif /* __VARS_IN_EEPROM__ */

  return CanBusOffMaxCnt;
}

/* ------------------------------------------------------------------------ */

/* Up to 16 bytes of configuration parameters can be stored */
#define CAN_STORE_SIZE 3

/* ------------------------------------------------------------------------ */

BOOL can_store_config( void )
{
  BYTE block[CAN_STORE_SIZE];

#ifdef __VARS_IN_EEPROM__
  RtrDisabled        = eeprom_read( EE_RTR_DISABLED );
  CANopenOpStateInit = eeprom_read( EE_CANOPEN_OPSTATE_INIT );
  CanBusOffMaxCnt    = eeprom_read( EE_CAN_BUSOFF_MAXCNT );
#endif /* __VARS_IN_EEPROM__ */

  block[0] = RtrDisabled;
  block[1] = CANopenOpStateInit;
  block[2] = CanBusOffMaxCnt;

  return( store_write_block( STORE_CAN, CAN_STORE_SIZE, block ) );
}

/* ------------------------------------------------------------------------ */

static void can_load_config( void )
{
  BYTE block[CAN_STORE_SIZE];

  /* Read the configuration from EEPROM, if any
     (errors in reading this datablock are caught and
      reported by functions in store.c...) */
  if( store_read_block( STORE_CAN, CAN_STORE_SIZE, block ) )
    {
      RtrDisabled        = block[0];
      CANopenOpStateInit = block[1];
      CanBusOffMaxCnt    = block[2];
    }
  else
    {
      /* No valid parameters in EEPROM: use defaults */
      RtrDisabled        = FALSE;
      CANopenOpStateInit = FALSE;
      CanBusOffMaxCnt    = 5;
    }

#ifdef __VARS_IN_EEPROM__
  /* Create working copies of configuration globals in EEPROM */
  if( eeprom_read( EE_RTR_DISABLED ) != RtrDisabled )
    eeprom_write( EE_RTR_DISABLED, RtrDisabled );
  if( eeprom_read( EE_CANOPEN_OPSTATE_INIT ) != CANopenOpStateInit )
    eeprom_write( EE_CANOPEN_OPSTATE_INIT, CANopenOpStateInit );
  if( eeprom_read( EE_CAN_BUSOFF_MAXCNT ) != CanBusOffMaxCnt )
    eeprom_write( EE_CAN_BUSOFF_MAXCNT, CanBusOffMaxCnt );
#endif /* __VARS_IN_EEPROM__ */
}

/* ------------------------------------------------------------------------ */
/* CAN INT interrupt handler */

#pragma interrupt_handler canint_handler:3

void canint_handler( void )
{
  BYTE object_no;

  object_no = can_check_for_msgs();

  if( object_no != NO_OBJECT )
    {
      /* CAN message received: copy it to the CAN message buffer */
      BYTE        cntr;
      BYTE        index;
      BYTE        *msg;
      BYTE        dlc;
      signed char byt;

      cntr = get_buf_cntr();

      /* If buffer is full (keep one buffer 'free', it might be the one
	 currently being processed by the application, and since the data
	 bytes are not copied they should not be overwritten yet; also the
	 buffer space is used for assembling a reply, by the SDO server),
	 disable further interrupts to prevent overwriting message buffers */
      if( cntr == CAN_BUFS-1 )
	{
	  CAN_INT_DISABLE();

	  /* A message is lost: get it reported; also because the interrupt
	     has been disabled more messages might get lost! */
	  CanBufFull = TRUE;

	  return;
	}

      /* Calculate index (of first empty buffer) from the 'done-reading-until'
	 index (MsgOutIndex_) and the 'number-of-full-buffers' counter
	 (MsgCounter_) parameters, which are stored in a fault-tolerant way
	 (3 copies of each parameter, majority voting mechanism) */
      index = (get_buf_index() + cntr) & CAN_BUFS_MASK;

      /* Location to copy CAN message to */
      msg = CanMsgBuf[index];

      /* Get received DLC */
      if( object_no > C91_MSG_BUFFERS-1 )
	{
	  dlc = 0; /* These object_no values are reserved for RTRs */
	}
      else
	{
	  BYTE addr;

	  dlc = (can_read_reg( C91_DR00_I+1+(object_no<<1) ) &
		 C91_DR_DLC_MASK);

	  /* Read the data bytes from the message buffer;
	     NB: always read the 8th databyte to guarantee a reload of
	     the message buffer to the Shadow Register !!!
	     If 2 messages with the same COB-ID arrive one after the other and
	     have less than 8 bytes, then the second 'read' would otherwise
	     *NOT* result in the new databytes !!!) */

	  /* Determine object's message buffer address in CAN-controller */
	  addr = C91_MSGS_I + (object_no * C91_MSG_SIZE);

	  /* Force transfer to Shadow Register */
	  if( dlc < 8 )
	    can_read_reg( addr + 7 );
	  else
	    dlc = 8;

	  /* Copy data bytes */
	  for( byt=dlc-1; byt>=0; --byt ) msg[byt] = can_read_reg( addr+byt );
	}

      /* Store Object ID, DLC and mark buffer as 'not empty' */
      msg[MSG_OBJECT_I] = object_no;
      msg[MSG_DLC_I]    = dlc;
      msg[MSG_VALID_I]  = BUF_NOT_EMPTY;

      /* Increment the CAN-message-in-buffer counter */
      ++cntr;
      MsgCounter1 = cntr;  MsgCounter2 = cntr;  MsgCounter3 = cntr;
    }
}

/* ------------------------------------------------------------------------ */

static BYTE can_check_for_msgs( void )
{
  BYTE object_no, bitmask;

  /* Each time 'can_check_for_msgs()') is called
     'ObjectMask1' and 'ObjectMask2' are updated */

  /* Check Receive-Ready register 1 */
  ObjectMask1 |= can_read_reg( C91_RECV_READY1_I );

  /* There is at least one message in buffers 0-7 received
     and/or still to be handled */
  if( ObjectMask1 != 0 )
    {
      bitmask = 0x01;
      for( object_no=0; object_no<C91_MSG_BUFFERS_PER_RRR; ++object_no )
	{
	  if( ObjectMask1 & bitmask )
	    {
	      /* Clear the bit in the Receive Ready register */
	      can_write_reg( C91_RECV_READY1_I, ~bitmask );

	      /* Reset the RI interrupt bit (if possible) */
	      // ###Not necessary ?
	      //can_write_reg( C91_INTERRUPT_I, (BYTE)(~C91_RECV_INT) );

	      /* Clear the bit in 'ObjectMask1' */
	      ObjectMask1 &= ~bitmask;

	      /* Remote Transmission Requests require extra work... */
	      if( object_no == C91_RTR )
		{
		  BYTE id_lo, id_hi;

		  /* Check if RTR received is for this node */

		  id_lo = can_read_reg( C91_DR00_I+(2*C91_RTR)+1 );

#ifdef __VARS_IN_EEPROM__
		  /* ### Not in interrupt routine */
		  //RtrIdLo = eeprom_read( EE_RTRIDLO );
#endif
		  if( (id_lo &
		       (NODEID_MASK_LOW_BYTE | C91_DR_RTR_MASK)) == RtrIdLo )
		    {
		      id_hi = can_read_reg( C91_DR00_I+(2*C91_RTR) );

#ifdef __VARS_IN_EEPROM__
		      /* ### Not in interrupt routine */
		      //RtrIdHi = eeprom_read( EE_RTRIDHI );
#endif
		      if( (id_hi & NODEID_MASK_HIGH_BYTE) == RtrIdHi )
			{
			  /* Okay, this is an RTR for me;
			     find out which object is requested... */

			  id_hi &= OBJECT_MASK;

			  switch( id_hi )
			    {
			    case TPDO1_OBJ:
			      return C91_TPDO1_RTR;
			    case TPDO2_OBJ:
			      return C91_TPDO2_RTR;
			    case TPDO3_OBJ:
			      return C91_TPDO3_RTR;
			    case NODEGUARD_OBJ:
			      return C91_NODEGUARD_RTR;
			    default:
			      /* Don't service this message */
			      return NO_OBJECT;
			    }
			}
		      else
			{
			  /* Don't service this message */
			  return NO_OBJECT;
			}
		    }
		  else
		    {
		      /* Don't service this message */
		      return NO_OBJECT;
		    }
		}
	      else
		{
		  /* This corresponds to the message buffer number
		     containing a message to service */
		  return object_no;
		}
	    }
	  bitmask <<= 1;
	}
    }

  /* Check Receive-Ready register 2 */
  ObjectMask2 |= can_read_reg( C91_RECV_READY2_I );

  /* There is at least one message in buffers 8-15 received
     and/or still to be handled */
  if( ObjectMask2 != 0 )
    {
      bitmask = 0x01;
      for( object_no=0; object_no<C91_MSG_BUFFERS_PER_RRR; ++object_no )
	{
	  if( ObjectMask2 & bitmask )
	    {
	      /* Clear the bit in the Receive Ready register */
	      can_write_reg( C91_RECV_READY2_I, ~bitmask );

	      /* Reset the RI interrupt bit (if possible) */
	      // ###Not necessary ?
	      //can_write_reg( C91_INTERRUPT_I, (BYTE)(~C91_RECV_INT) );

	      /* Clear the bit in 'ObjectMask2' */
	      ObjectMask2 &= ~bitmask;

	      /* This corresponds to the message buffer number
		 containing a message to service */
	      return( C91_MSG_BUFFERS_PER_RRR + object_no );
	    }
	  bitmask <<= 1;
	}
    }

  /* No message to service */
  return NO_OBJECT;
}

/* ------------------------------------------------------------------------ */

static BYTE get_buf_index( void )
{
  /* Majority voting */
  if( MsgOutIndex1 == MsgOutIndex2 ) MsgOutIndex3 = MsgOutIndex1;
  else if( MsgOutIndex1 == MsgOutIndex3 ) MsgOutIndex2 = MsgOutIndex1;
  else if( MsgOutIndex2 == MsgOutIndex3 ) MsgOutIndex1 = MsgOutIndex3;
  else
    {
      /* All 3 are different: do a majority vote on a bit-by-bit basis... */
      BYTE byt, bitmask, bits, i;
      byt     = 0x00; /* Start value */
      bitmask = 0x80; /* Start with MSB */
      for( i=0; i<8; ++i )
	{
	  bits = 0;
	  if( MsgOutIndex1 & bitmask ) ++bits;
	  if( MsgOutIndex2 & bitmask ) ++bits;
	  if( MsgOutIndex3 & bitmask ) ++bits;
	  byt <<= 1; /* Shift value one bit up*/
	  /* Bit is set or not */
	  if( bits > 1 ) ++byt; /* 2 or 3 of the bits (majority) are set */
	  bitmask >>= 1; /* Next bit to check */
	}
      /* Set the new MsgOutIndex1/2/3 value */
      MsgOutIndex1 = byt;  MsgOutIndex2 = byt;  MsgOutIndex3 = byt;
    }
  return MsgOutIndex1;
}

/* ------------------------------------------------------------------------ */

static BYTE get_buf_cntr( void )
{
  /* Majority voting */
  if( MsgCounter1 == MsgCounter2 ) MsgCounter3 = MsgCounter1;
  else if( MsgCounter1 == MsgCounter3 ) MsgCounter2 = MsgCounter1;
  else if( MsgCounter2 == MsgCounter3 ) MsgCounter1 = MsgCounter3;
  else
    {
      /* All 3 are different: do a majority vote on a bit-by-bit basis... */
      BYTE byt, bitmask, bits, i;
      byt     = 0x00; /* Start value */
      bitmask = 0x80; /* Start with MSB */
      for( i=0; i<8; ++i )
	{
	  bits = 0;
	  if( MsgCounter1 & bitmask ) ++bits;
	  if( MsgCounter2 & bitmask ) ++bits;
	  if( MsgCounter3 & bitmask ) ++bits;
	  byt <<= 1; /* Shift value one bit up*/
	  if( bits > 1 ) ++byt; /* 2 or 3 of the bits (majority) are set */
	  bitmask >>= 1; /* Next bit to check */
	}
      /* Set the new MsgCounter1/2/3 value */
      MsgCounter1 = byt;  MsgCounter2 = byt;  MsgCounter3 = byt;
    }
  return MsgCounter1;
}

/* ------------------------------------------------------------------------ */

static BYTE bits_in_byte( BYTE val )
{
  /* Calculate the number of bits set in a byte
     (method taken from www.devx.com, 'C++ tips'):
     Add up 4 pairs of 1-bit numbers (resulting in four 2-bit numbers):
     val = ((val >> 1) & 0x55) + (val & 0x55);
     Optimisation: save an AND (&) instruction by exploiting this clever
     relationship for each pair of bits: the two bits 'ab' represent the
     number 2a+b; to count the bits we subtract 'a' (i.e. 2a+b - a = a+b) */

  val = val - ((val >> 1) & 0x55);

  /* Add up four 2-bit numbers (resulting in two 4-bit numbers */
  val = ((val >> 2) & 0x33) + (val & 0x33);

  /* Add up two 4-bit numbers resulting in one 8-bit number */
  val = ((val >> 4) & 0x0F) + (val & 0x0F);

  return val;
}

/* ------------------------------------------------------------------------ */
