/* ------------------------------------------------------------------------
File   : timer103.h

Descr  : Headerfile for ATMEL AVR ATmega1xx microcontroller
         Timer/Counter routines.

History: 16.09.99; Henk B&B; Definitions for AVR micros.
         27APR.00; Henk B&B; Additions/changes to match the ATmega103 micro.
--------------------------------------------------------------------------- */

#ifndef TIMER103_H
#define TIMER103_H

/* ------------------------------------------------------------------------ */
/* Timer initialisations:

   An AVR timer can increment at CK, CK/8, CK/64, CK/256, CK/1024,
   with CK is the processor clock frequency,
   so with CK=4MHz an AVR timer increments every 0.25, 2, 16, 64 or
   256 microseconds.

   (in the ATmega103 Timer0 can increment at
    CK, CK/8, CK/32, CK/64, CK/128, CK/256, CK/1024;
    CK=4MHz: 0.25, 2, 8, 16, 32, 64 or 256 microseconds)

   **OVERFLOW**
   ============
   A timer overflows every MAXCOUNT/I microseconds,
   with MAXCOUNT=256 for an 8-bit timer and MAXCOUNT=65536 for a 16-bit timer
   and with 'I' equal to the timer increments per microseconds, corresponding
   to the available clock prescale selections of CK, CK/8, CK/64, CK/256 and
   CK/1024 resp.

   For CK=4 MHz:
   =============
                 I=4/1, 4/8, 4/64, 4/256 and 4/1024
                 I=4,   1/2, 1/16, 1/64  and 1/256
   so an 8-bit timer overflows every
                   256/4, 256*2, 256*16, 256*64 and 256*256 microseconds
                 = 64,    512,   4096,  16384   and 65536   microseconds
                 = 0.064, 0.512, 4.096, 16.384  and 65.536  ms

   and in addition for CK/32 and CK/128:
                 I=4/32, 4/128
                 I=1/8,  1/32
   so the 8-bit ATmega103 Timer0 overflows also every
                   256*8 and 256*32 microseconds
		 = 2048  and 8192   microseconds
                 = 2.048 and 8.192  ms

   a 16-bit timer overflows every
                   65536/4, 65536*2, 65536*16, 65536*64 and 6536*256  microsecs
                 = 16384,  131072,  1048576,   4194304  and 16777216  microsecs
                 = 16.384, 131.072, 1048.567,  4194.304 and 16777.216 ms
                 =  0.016,   0.131,    1.049,     4.194 and    16.777 s

   **PRELOADING**
   ==============
   If we preload the timer registers TCNT0 (8-bit Timer0) or
   TCNT1H and TCNT1L (16-bit Timer1) we get:
     freq    = XTALFREQ / (prescale*(MAX_COUNT - preload))
     ( ==> time = (prescale*(MAX_COUNT - preload)) / XTALFREQ )
   or
     preload = MAX_COUNT - XTALFREQ/(prescale*freq)
   or
     preload = MAX_COUNT - time*XTALFREQ/prescale

   **TIMER0**
   ==========
   With a 4 MHz clock:
   ===================
   Frequency:
   ----------
     preload = 256 - 4000000/(s*f)
           f = 4000000/(s(256-preload)) with s=1,8,64,256 or 1024
   Exact frequencies (4 MHz clock):
   f= 250 Hz, s=  64 ==> preload =     6 = 0x06
   f= 500 Hz, s=  64 ==> preload =   131 = 0x83
   f= 625 Hz, s=  64 ==> preload =   156 = 0x9C
   f= 125 Hz, s= 256 ==> preload =   131 = 0x83
   f= 625 Hz, s= 256 ==> preload =   231 = 0xE7

   **TIMER1**
   ==========
   With a 4 MHz clock:
   ===================
   Frequency:
   ----------
     preload = 65536 - 4000000/(s*f)
           f = 4000000/(s(65536-preload)) with s=1,8,64,256 or 1024

   So the frequency ranges (min, max) covered by different settings are:
     s=   1: 61.035 < f < 4000000.0 Hz
     s=   8:  7.629 < f <  500000.0 Hz
     s=  64:  0.954 < f <   62500.0 Hz
     s= 256:  0.238 < f <   15625.0 Hz
     s=1024:  0.060 < f <    3906.2 Hz

   Examples of exact frequencies (4 MHz clock):
   ...more here...
   f=1000 Hz, s=   8 ==> preload = 65036 = 0xFE0C
   f=   1 Hz, s=  64 ==> preload =  3036 = 0x0BDC
   f=   2 Hz, s=  64 ==> preload = 34286 = 0x85EE
   f=   4 Hz, s=  64 ==> preload = 49911 = 0xC2F7
   f=   5 Hz, s=  64 ==> preload = 53036 = 0xCF2C
   f=  10 Hz, s=  64 ==> preload = 59286 = 0xE796
   f=  20 Hz, s=  64 ==> preload = 62411 = 0xF3CB
   f=  25 Hz, s=  64 ==> preload = 63036 = 0xF63C
   f=  50 Hz, s=  64 ==> preload = 64286 = 0xFB1E
   f= 100 Hz, s=  64 ==> preload = 64911 = 0xFD8F
   f= 125 Hz, s=  64 ==> preload = 65036 = 0xFE0C
   f= 250 Hz, s=  64 ==> preload = 65286 = 0xFF06
   f= 500 Hz, s=  64 ==> preload = 65411 = 0xFF83
   f= 625 Hz, s=  64 ==> preload = 65436 = 0xFF9C
   f= 753 Hz, s=  64 ==> preload = 65453 = 0xFFAD
   f=   1 Hz, s= 256 ==> preload = 49911 = 0xC2F7
   f=   5 Hz, s= 256 ==> preload = 62411 = 0xF3CB
   f=  25 Hz, s= 256 ==> preload = 64911 = 0xFD8F
   f= 125 Hz, s= 256 ==> preload = 65411 = 0xFF83
   f= 558 Hz, s= 256 ==> preload = 65508 = 0xFFE4
   f= 601 Hz, s= 256 ==> preload = 65510 = 0xFFE6
   f= 625 Hz, s= 256 ==> preload = 65511 = 0xFFE7
   f= 651 Hz, s= 256 ==> preload = 65512 = 0xFFE8
   f= 744 Hz, s= 256 ==> preload = 65515 = 0xFFEB
   f= 868 Hz, s= 256 ==> preload = 65518 = 0xFFEE
   f= 186 Hz, s=1024 ==> preload = 65515 = 0xFFEB
   ...and more here...

   Period until overflow:
   ----------------------
     preload = 256 - 4000000t/s
           t = (256-preload)s/4000000

   t= 100 ms, s=1024 ==> preload = 65145.4 = 0xFE79
   t= 200 ms, s=1024 ==> preload = 64754.8 = 0xFCF3
   t= 300 ms, s=1024 ==> preload = 64364.1 = 0xFB6C
   t= 400 ms, s=1024 ==> preload = 63973.5 = 0xF9E6
   t= 500 ms, s=1024 ==> preload = 63582.9 = 0xF85F
   t= 600 ms, s=1024 ==> preload = 63192.2 = 0xF6D8
   t= 700 ms, s=1024 ==> preload = 62801.6 = 0xF552
   t= 800 ms, s=1024 ==> preload = 62411.0 = 0xF3CB
   t= 900 ms, s=1024 ==> preload = 62020.4 = 0xF244
   t=1000 ms, s=1024 ==> preload = 61629.8 = 0xF0BE
   t=1100 ms, s=1024 ==> preload = 61239.1 = 0xEF37
   t=1200 ms, s=1024 ==> preload = 60848.5 = 0xEDB1
   t=1300 ms, s=1024 ==> preload = 60457.9 = 0xEC2A
   t=1400 ms, s=1024 ==> preload = 60067.2 = 0xEAA3
   t=1500 ms, s=1024 ==> preload = 59676.6 = 0xE91D
   t=1600 ms, s=1024 ==> preload = 59286.0 = 0xE796
   t=1700 ms, s=1024 ==> preload = 58895.4 = 0xE60F
   t=1800 ms, s=1024 ==> preload = 58504.8 = 0xE489
   t=1900 ms, s=1024 ==> preload = 58114.1 = 0xE302
   t=2000 ms, s=1024 ==> preload = 57723.5 = 0xE17C
   t=2100 ms, s=1024 ==> preload = 57332.9 = 0xDFF5
   t=2200 ms, s=1024 ==> preload = 56942.2 = 0xDE6E
   t=2300 ms, s=1024 ==> preload = 56551.6 = 0xDCE8
   t=2400 ms, s=1024 ==> preload = 56161.0 = 0xDB61
   t=2500 ms, s=1024 ==> preload = 55770.4 = 0xD9DA
   t=2600 ms, s=1024 ==> preload = 55379.8 = 0xD854
   t=2700 ms, s=1024 ==> preload = 54989.1 = 0xD6CD
   t=2800 ms, s=1024 ==> preload = 54598.5 = 0xD547
   t=2900 ms, s=1024 ==> preload = 54207.9 = 0xD3C0
   t=3000 ms, s=1024 ==> preload = 53817.2 = 0xD239
   t=3100 ms, s=1024 ==> preload = 53426.6 = 0xD0B3
   t=3200 ms, s=1024 ==> preload = 53036.0 = 0xCF2C
   t=3300 ms, s=1024 ==> preload = 52645.4 = 0xCDA5
   t=3400 ms, s=1024 ==> preload = 52254.8 = 0xCC1F
   t=3500 ms, s=1024 ==> preload = 51864.1 = 0xCA98
   t=3600 ms, s=1024 ==> preload = 51473.5 = 0xC912
   t=3700 ms, s=1024 ==> preload = 51082.9 = 0xC78B
   t=3800 ms, s=1024 ==> preload = 50692.2 = 0xC604
   t=3900 ms, s=1024 ==> preload = 50301.6 = 0xC47E
   t=4000 ms, s=1024 ==> preload = 49911.0 = 0xC2F7
*/

/* ------------------------------------------------------------------------ */
/* Some Timer/Counter Register bits and settings */

/* Timer/Counter Interrupt Mask Register bits */
#define T0_OVERFLOW_IE   TOIE0
#define T1_OVERFLOW_IE   TOIE1
#define T2_OVERFLOW_IE   TOIE2

/* Timer/Counter Interrupt FLAG Register bits */
#define T0_OVERFLOW      TOV0
#define T1_OVERFLOW      TOV1
#define T2_OVERFLOW      TOV2

/* Timer/Counter0 Control Register clock prescale select */
#define T0_STOP          0x00
#define T0_CK_DIV_1      0x01
#define T0_CK_DIV_8      0x02
#define T0_CK_DIV_32     0x03
#define T0_CK_DIV_64     0x04
#define T0_CK_DIV_128    0x05
#define T0_CK_DIV_256    0x06
#define T0_CK_DIV_1024   0x07

/* Timer/Counter1 Control Register clock prescale select */
#define T1_STOP          0x00
#define T1_CK_DIV_1      0x01
#define T1_CK_DIV_8      0x02
#define T1_CK_DIV_64     0x03
#define T1_CK_DIV_256    0x04
#define T1_CK_DIV_1024   0x05
#define T1_FALLING_EDGE  0x06
#define T1_RISING_EDGE   0x07

/* Timer/Counter2 Control Register clock prescale select */
#define T2_STOP          0x00
#define T2_CK_DIV_1      0x01
#define T2_CK_DIV_8      0x02
#define T2_CK_DIV_64     0x03
#define T2_CK_DIV_256    0x04
#define T2_CK_DIV_1024   0x05
#define T2_FALLING_EDGE  0x06
#define T2_RISING_EDGE   0x07

/* ------------------------------------------------------------------------ */
/* Timer1 stuff */

/* Timer1 settings for defined delays at 4 MHz */
#define SET_TIMER1_1500MS() {TCNT1H=0xE9; TCNT1L=0x1D; TCCR1B=T1_CK_DIV_1024;}

/* Timer1 settings for defined delays at 4 MHz */
#define SET_TIMER1_1000MS() {TCNT1H=0xC2; TCNT1L=0xF7; TCCR1B=T1_CK_DIV_256;}

#define TIMER1_ENABLE()     {TIMSK |= BIT( T1_OVERFLOW_IE );}
#define TIMER1_DISABLE()    {TIMSK &= ~BIT( T1_OVERFLOW_IE );}

/* ------------------------------------------------------------------------ */
/* Timer0 time-out stuff */

/* Timer0 settings for defined delays at 4 MHz */
#define SET_TIMER0_10MS()   {TCNT0=217; TCCR0=T0_CK_DIV_1024;}

/* Number of clients for time-out services */
#define T0_CLIENTS          1

/* Client identifiers */
#define ADC_ELMB            0

/* ------------------------------------------------------------------------ */
/* Function prototypes */

/* Timer2 used for busy-wait delays */
void timer2_delay_mus       ( BYTE microseconds );
void timer2_delay_ms        ( BYTE milliseconds );

/* Timer1 used as a clock for periodic actions:
   microcontroller activity monitoring, Node Guarding, PDOs */
void timer1_init            ( void );
void timer1_stop            ( void );

/* Timer0 is used for time-outs on operations */
void timer0_init            ( void );
void timer0_set_timeout_10ms( BYTE client, BYTE ticks );
BOOL timer0_timeout         ( BYTE client );

/* ------------------------------------------------------------------------ */
#endif
