/* ------------------------------------------------------------------------
File   : download.h

Descr  : Definitions and declarations for AVR-processor serial programming
         functions.

History: 21MAR.00; Henk B&B; Definition.
--------------------------------------------------------------------------- */

#ifndef DOWNLOAD_H
#define DOWNLOAD_H

/* ------------------------------------------------------------------------ */

/* First byte */
#define ISP_ENABLE_OR_ERASE            0xAC
#define ISP_READ_FLASH_LO_BYTE         0x20
#define ISP_READ_FLASH_HI_BYTE         0x28
#define ISP_WRITE_FLASH_LO_BYTE        0x40
#define ISP_WRITE_FLASH_HI_BYTE        0x48
#define ISP_READ_EEPROM_BYTE           0xA0
#define ISP_WRITE_EEPROM_BYTE          0xC0
#define ISP_READ_SIGNATURE_BYTE        0x30
#define ISP_READ_LOCK_BITS             0x58
#define ISP_READ_FUSE_BITS             0x50
#define ISP_READ_LOCK_AND_FUSE_BITS    0x58
/* Custom identifier */
#define ISP_END_OF_PROGRAMMING         0xBB

/* Second byte, after an ISP_ENABLE_OR_ERASE first byte */
#define ISP_PROGRAMMING_ENABLE         0x53
#define ISP_CHIP_ERASE                 0x80
#define ISP_WRITE_LOCK_BITS            0xF9
#define ISP_WRITE_FUSE_BITS            0xB4
#define ISP_WRITE_FUSE_2343_BIT        0xBE

/* ------------------------------------------------------------------------ */
/* Function prototypes */

BOOL do_serial_instruction( BYTE *instr );

#endif /* DOWNLOAD_H */
/* ------------------------------------------------------------------------ */
