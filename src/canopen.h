/* ------------------------------------------------------------------------
File   : canopen.h

Descr  : Definitions for CANopen and CANopen messages.

History: 29JUN.98; Henk B&B; Definition.
--------------------------------------------------------------------------- */

#ifndef CANOPEN_H
#define CANOPEN_H

/* ------------------------------------------------------------------------ */

/* The PDO Communication Parameter object contains:
   number of entries (U8), COB-ID used (U32), transmission type (U8),
   inhibit time (U16), compatibility entry (U8) and event timer (U16).
   Of these we will only put what is configurable in our local definition
   of this object and also adjust the data type according to our needs */

typedef struct pdo_comm_par
{
  BYTE   transmission_type;
  UINT16 event_timer;
} PDO_COMM_PAR;


/* The PDO Mapping object potentially has 1+64 entries;
   we show here a possible C structure to contain a PDO Mapping
   with 2 mapped objects; we probably won't instantiate such objects */

typedef struct pdo_mapping
{
  BYTE   no_of_entries;
  UINT32 entry[2];
} PDO_MAPPING;

/* ------------------------------------------------------------------------ */
/* Communication Objects for the Predefined Connection Set */

/* The Object and Module-ID mask when the COB-ID+DLC is present
   in 2 separate bytes */
#define OBJECT_MASK                     0xF0
#define NODEID_MASK_HIGH_BYTE           0x0F
#define NODEID_MASK_LOW_BYTE            0xE0

/* CANopen object definitions in a one-byte variable:
   these are the upper 8 bits of the 11-bit CAN-identifier (bits 3-10) */
#define NMT_OBJ                         0x00
#define SYNC_OBJ                        0x10
#define TIMESTAMP_OBJ                   0x20
#define EMERGENCY_OBJ                   0x10
#define TPDO1_OBJ                       0x30
#define RPDO1_OBJ                       0x40
#define TPDO2_OBJ                       0x50
#define RPDO2_OBJ                       0x60
#define TPDO3_OBJ                       0x70
#define RPDO3_OBJ                       0x80
#define TPDO4_OBJ                       0x90
#define RPDO4_OBJ                       0xA0
#define SDOTX_OBJ                       0xB0
#define SDORX_OBJ                       0xC0
#define NODEGUARD_OBJ                   0xE0

/* ------------------------------------------------------------------------ */
/* NMT Start/Stop Service command specifiers */

#define NMT_START_REMOTE_NODE               1
#define NMT_STOP_REMOTE_NODE                2
#define NMT_ENTER_PREOPERATIONAL_STATE    128
#define NMT_RESET_NODE                    129
#define NMT_RESET_COMMUNICATION           130

/* ------------------------------------------------------------------------ */
/* NMT Slave state */

#define NMT_INITIALISING                    0
#define NMT_DISCONNECTED                    1
#define NMT_CONNECTING                      2
#define NMT_PREPARING                       3
#define NMT_STOPPED                         4
#define NMT_OPERATIONAL                     5
#define NMT_PREOPERATIONAL                127

/* ------------------------------------------------------------------------ */
/* SDO command specifiers and other bits */

#define SDO_COMMAND_SPECIFIER_MASK     (7<<5)

/* Client command specifiers */
#define SDO_INITIATE_DOWNLOAD_REQ      (1<<5)
#define SDO_DOWNLOAD_SEGMENT_REQ       (0<<5)
#define SDO_INITIATE_UPLOAD_REQ        (2<<5)
#define SDO_UPLOAD_SEGMENT_REQ         (3<<5)

/* Server command specifiers */
#define SDO_INITIATE_DOWNLOAD_RESP     (3<<5)
#define SDO_DOWNLOAD_SEGMENT_RESP      (1<<5)
#define SDO_INITIATE_UPLOAD_RESP       (2<<5)
#define SDO_UPLOAD_SEGMENT_RESP        (0<<5)

/* Client or Server command specifiers */
#define SDO_ABORT_TRANSFER             (4<<5)

/* Other bits and stuff */
#define SDO_EXPEDITED                  (1<<1)
#define SDO_TOGGLE_BIT                 (1<<4)
#define SDO_LAST_SEGMENT               (1<<0)
#define SDO_SEGMENT_SIZE_INDICATED     (1<<0)
#define SDO_DATA_SIZE_INDICATED        (1<<0)
#define SDO_SEGMENT_SIZE_MASK        (0x7<<1)
#define SDO_DATA_SIZE_MASK           (0x3<<2)
#define SDO_SEGMENT_SIZE_SHIFT              1
#define SDO_DATA_SIZE_SHIFT                 2

/* ------------------------------------------------------------------------ */
/* SDO Abort Domain Transfer protocol: abort codes */

/* Error classes (MSB) */
#define SDO_ECLASS_SERVICE                  5
#define SDO_ECLASS_ACCESS                   6
#define SDO_ECLASS_OTHER                    8

/* Error codes (MSB-1) */
#define SDO_ECODE_PAR_INCONSISTENT          3
#define SDO_ECODE_PAR_ILLEGAL               4
#define SDO_ECODE_ACCESS                    1
#define SDO_ECODE_NONEXISTENT               2
#define SDO_ECODE_HARDWARE                  6
#define SDO_ECODE_TYPE_CONFLICT             7
#define SDO_ECODE_ATTRIBUTE                 9

/* ------------------------------------------------------------------------ */
/* Error Register bits */

#define ERRREG_GENERIC                      0x01
#define ERRREG_CURRENT                      0x02
#define ERRREG_VOLTAGE                      0x04
#define ERRREG_TEMPERATURE                  0x08
#define ERRREG_COMMUNICATION                0x10
#define ERRREG_DEVICEPROFILE                0x20
#define ERRREG_MANUFACTURER                 0x80

#endif /* CANOPEN_H */
/* ------------------------------------------------------------------------ */
