/* ------------------------------------------------------------------------
File   : spi.h

Descr  : Declarations of (software) SPI serial interface functions
         for general-purpose applications.

History: 20OCT.03; Henk B&B; Definition.
--------------------------------------------------------------------------- */

#ifndef SPI_GP_H
#define SPI_GP_H

/* SPI serial interface (!NB: same as ELMBio ADC/DAC */
#define SPI_SET_SCLK()              SETBIT( PORTD, 4 )
#define SPI_CLEAR_SCLK()            CLEARBIT( PORTD, 4 )
#define SPI_SET_SDI()               SETBIT( PORTD, 5 )
#define SPI_CLEAR_SDI()             CLEARBIT( PORTD, 5 )
#define SPI_SDO_HIGH()              (PIND & BIT( 6 ))
#define SPI_SDO_LOW()               (!SPI_SDO_HIGH())

/* Device chip-selects (!NB: same as ELMBio DAC selects !) */
#define SPI_SELECT_1()              CLEARBIT( PORTE, 3 )
#define SPI_DESELECT_1()            SETBIT( PORTE, 3 )
#define SPI_SELECT_2()              CLEARBIT( PORTE, 4 )
#define SPI_DESELECT_2()            SETBIT( PORTE, 4 )
#define SPI_SELECT_3()              CLEARBIT( PORTE, 5 )
#define SPI_DESELECT_3()            SETBIT( PORTE, 5 )
#define SPI_SELECT_4()              CLEARBIT( PORTE, 6 )
#define SPI_DESELECT_4()            SETBIT( PORTE, 6 )

/* Make sure this corresponds to the stuff above ! */
#define SPI_INIT_DDR()              {SETBIT(DDRD,4);SETBIT(DDRD,5); \
                                     CLEARBIT(DDRD,6);SETBIT(PORTD,6); \
                                     SETBIT(DDRE,3);SETBIT(DDRE,4); \
				     SETBIT(DDRE,5);SETBIT(DDRE,6);}

/* ------------------------------------------------------------------------ */
/* Function prototypes */

BYTE spi_read_byte     ( void );
void spi_write_byte    ( BYTE byt );

BOOL spi_set_chipselect( BYTE select );
BYTE spi_get_chipselect( void );

BOOL spi_set_holdtime  ( BYTE select );
BYTE spi_get_holdtime  ( void );

BOOL spi_set_rising_clk( BOOL rising );
BOOL spi_get_rising_clk( void );

#endif /* SPI_GP_H */
/* ------------------------------------------------------------------------ */
