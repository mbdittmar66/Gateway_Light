//**********************************************************************
// Filename:      c8051f040.h
// Rights:        Copyright (c) 2006 The Carlisle and Finch Company
//                All Rights Reserved
// Creation Date: 9/11/06
// Compiler:      Tasking cc51
// Version:       v7.1r1
// Purpose:       This module contains register/bit definitions for the 
//                C8051F04x product family.
// Modification History:
// Date         Author      Description
// ----         ------      -----------
// 9-11-06		JRN			Created
//**********************************************************************


// ===========================================
// Data types definitions
// ===========================================

#define SHORT   int
#define USHORT  unsigned int
#define BOOLEAN unsigned char
#define CHAR    char
#define UCHAR   unsigned char
#define INT     int
#define UINT    unsigned int
#define LONG    long
#define ULONG   unsigned long
#define VOID    void
#define FLOAT   float


#define ON      1
#define OFF     0

#define TRUE    1
#define FALSE   0

#ifndef NULL 
#define NULL	0
#endif



// ===========================================
//  BYTE Registers
// ===========================================

_sfrbyte P0       _at ( 0x80 );    /* PORT 0                                       */
_sfrbyte SP       _at ( 0x81 );    /* STACK POINTER                                */
_sfrbyte DPL      _at ( 0x82 );    /* DATA POINTER - LOW BYTE                      */
_sfrbyte DPH      _at ( 0x83 );    /* DATA POINTER - HIGH BYTE                     */
_sfrbyte SFRPAGE  _at ( 0x84 );    /* SFR PAGE SELECT                              */
_sfrbyte SFRNEXT  _at ( 0x85 );    /* SFR STACK NEXT PAGE                          */
_sfrbyte SFRLAST  _at ( 0x86 );    /* SFR STACK LAST PAGE                          */
_sfrbyte PCON     _at ( 0x87 );    /* POWER CONTROL                                */
_sfrbyte TCON     _at ( 0x88 );    /* TIMER CONTROL                                */
_sfrbyte CPT0CN   _at ( 0x88 );    /* COMPARATOR 0 CONTROL                         */
_sfrbyte CPT1CN   _at ( 0x88 );    /* COMPARATOR 1 CONTROL                         */
_sfrbyte CPT2CN   _at ( 0x88 );    /* COMPARATOR 2 CONTROL                         */
_sfrbyte TMOD     _at ( 0x89 );    /* TIMER MODE                                   */
_sfrbyte CPT0MD   _at ( 0x89 );    /* COMPARATOR 0 MODE                            */
_sfrbyte CPT1MD   _at ( 0x89 );    /* COMPARATOR 1 MODE                            */
_sfrbyte CPT2MD   _at ( 0x89 );    /* COMPARATOR 2 MODE                            */
_sfrbyte TL0      _at ( 0x8A );    /* TIMER 0 - LOW BYTE                           */
_sfrbyte OSCICN   _at ( 0x8A );    /* INTERNAL OSCILLATOR CONTROL                  */
_sfrbyte TL1      _at ( 0x8B );    /* TIMER 1 - LOW BYTE                           */
_sfrbyte OSCICL   _at ( 0x8B );    /* INTERNAL OSCILLATOR CALIBRATION              */
_sfrbyte TH0      _at ( 0x8C );    /* TIMER 0 - HIGH BYTE                          */
_sfrbyte OSCXCN   _at ( 0x8C );    /* EXTERNAL OSCILLATOR CONTROL                  */
_sfrbyte TH1      _at ( 0x8D );    /* TIMER 1 - HIGH BYTE                          */
_sfrbyte CKCON    _at ( 0x8E );    /* TIMER 0/1 CLOCK CONTROL                      */
_sfrbyte PSCTL    _at ( 0x8F );    /* FLASH WRITE/ERASE CONTROL                    */
_sfrbyte P1       _at ( 0x90 );    /* PORT 1                                       */
_sfrbyte SSTA0    _at ( 0x91 );    /* UART 0 STATUS                                */
_sfrbyte SFRPGCN  _at ( 0x96 );    /* SFR PAGE CONTROL                             */
_sfrbyte CLKSEL   _at ( 0x97 );    /* SYSTEM CLOCK SELECT                          */
_sfrbyte SCON0    _at ( 0x98 );    /* UART 0 CONTROL                               */
_sfrbyte SCON1    _at ( 0x98 );    /* UART 1 CONTROL                               */
_sfrbyte SBUF0    _at ( 0x99 );    /* UART 0 BUFFER                                */
_sfrbyte SBUF1    _at ( 0x99 );    /* UART 1 BUFFER                                */
_sfrbyte SPI0CFG  _at ( 0x9A );    /* SPI 0 CONFIGURATION                          */
_sfrbyte SPI0DAT  _at ( 0x9B );    /* SPI 0 DATA                                   */
_sfrbyte P4MDOUT  _at ( 0x9C );    /* PORT 4 OUTPUT MODE                           */
_sfrbyte SPI0CKR  _at ( 0x9D );    /* SPI 0 CLOCK RATE CONTROL                     */
_sfrbyte P5MDOUT  _at ( 0x9D );    /* PORT 5 OUTPUT MODE                           */
_sfrbyte P6MDOUT  _at ( 0x9E );    /* PORT 6 OUTPUT MODE                           */
_sfrbyte P7MDOUT  _at ( 0x9F );    /* PORT 7 OUTPUT MODE                           */
_sfrbyte P2       _at ( 0xA0 );    /* PORT 2                                       */
_sfrbyte EMI0TC   _at ( 0xA1 );    /* EMIF TIMING CONTROL                          */
_sfrbyte EMI0CN   _at ( 0xA2 );    /* EMIF CONTROL                                 */
_sfrbyte EMI0CF   _at ( 0xA3 );    /* EMIF CONFIGURATION                           */
_sfrbyte P0MDOUT  _at ( 0xA4 );    /* PORT 0 OUTPUT MODE                           */
_sfrbyte P1MDOUT  _at ( 0xA5 );    /* PORT 1 OUTPUT MODE                           */
_sfrbyte P2MDOUT  _at ( 0xA6 );    /* PORT 2 OUTPUT MODE CONFIGURATION             */
_sfrbyte P3MDOUT  _at ( 0xA7 );    /* PORT 3 OUTPUT MODE CONFIGURATION             */
_sfrbyte IE       _at ( 0xA8 );    /* INTERRUPT ENABLE                             */
_sfrbyte SADDR0   _at ( 0xA9 );    /* UART 0 SLAVE ADDRESS                         */
_sfrbyte SADDR1   _at ( 0xA9 );    /* UART 1 SLAVE ADDRESS                         */
_sfrbyte P1MDIN   _at ( 0xAD );    /* PORT 1 INPUT MODE                            */
_sfrbyte P2MDIN   _at ( 0xAE );    /* PORT 2 INPUT MODE                            */
_sfrbyte P3MDIN   _at ( 0xAF );    /* PORT 3 INPUT MODE                            */
_sfrbyte P3       _at ( 0xB0 );    /* PORT 3                                       */
_sfrbyte FLSCL    _at ( 0xB7 );    /* FLASH TIMING PRESCALAR                       */
_sfrbyte FLACL    _at ( 0xB7 );    /* FLASH ACCESS LIMIT                           */
_sfrbyte IP       _at ( 0xB8 );    /* INTERRUPT PRIORITY                           */
_sfrbyte SADEN0   _at ( 0xB9 );    /* UART 0 SLAVE ADDRESS MASK                    */
_sfrbyte AMX2CF   _at ( 0xBA );    /* ADC 2 MUX CONFIGURATION                      */
_sfrbyte AMX0PRT  _at ( 0xBD );    /* ADC 0 MUX PORT PIN SELECT REGISTER           */
_sfrbyte AMX0CF   _at ( 0xBA );    /* ADC 0 CONFIGURATION REGISTER                 */
_sfrbyte AMX0SL   _at ( 0xBB );    /* ADC 0 AND ADC 1 MODE SELECTION               */
_sfrbyte AMX2SL   _at ( 0xBB );    /* ADC 2 MUX CHANNEL SELECTION                  */
_sfrbyte ADC0CF   _at ( 0xBC );    /* ADC 0 CONFIGURATION                          */
_sfrbyte ADC2CF   _at ( 0xBC );    /* ADC 2 CONFIGURATION                          */
_sfrbyte ADC0L    _at ( 0xBE );    /* ADC 0 DATA - LOW BYTE                        */
_sfrbyte ADC2     _at ( 0xBE );    /* ADC 2 DATA - LOW BYTE                        */
_sfrbyte ADC0H    _at ( 0xBF );    /* ADC 0 DATA - HIGH BYTE                       */
_sfrbyte SMB0CN   _at ( 0xC0 );    /* SMBUS 0 CONTROL                              */
_sfrbyte CAN0STA  _at ( 0xC0 );    /* CAN 0 STATUS                                 */
_sfrbyte SMB0STA  _at ( 0xC1 );    /* SMBUS 0 STATUS                               */
_sfrbyte SMB0DAT  _at ( 0xC2 );    /* SMBUS 0 DATA                                 */
_sfrbyte SMB0ADR  _at ( 0xC3 );    /* SMBUS 0 SLAVE ADDRESS                        */
_sfrbyte ADC0GTL  _at ( 0xC4 );    /* ADC 0 GREATER-THAN REGISTER - LOW BYTE       */
_sfrbyte ADC2GT   _at ( 0xC4 );    /* ADC 2 GREATER-THAN REGISTER - LOW BYTE       */
_sfrbyte ADC0GTH  _at ( 0xC5 );    /* ADC 0 GREATER-THAN REGISTER - HIGH BYTE      */
_sfrbyte ADC0LTL  _at ( 0xC6 );    /* ADC 0 LESS-THAN REGISTER - LOW BYTE          */
_sfrbyte ADC2LT   _at ( 0xC6 );    /* ADC 2 LESS-THAN REGISTER - LOW BYTE          */
_sfrbyte ADC0LTH  _at ( 0xC7 );    /* ADC 0 LESS-THAN REGISTER - HIGH BYTE         */
_sfrbyte TMR2CN   _at ( 0xC8 );    /* TIMER 2 CONTROL                              */
_sfrbyte TMR3CN   _at ( 0xC8 );    /* TIMER 3 CONTROL                              */
_sfrbyte TMR4CN   _at ( 0xC8 );    /* TIMER 4 CONTROL                              */
_sfrbyte P4       _at ( 0xC8 );    /* PORT 4                                       */
_sfrbyte TMR2CF   _at ( 0xC9 );    /* TIMER 2 CONFIGURATION                        */
_sfrbyte TMR3CF   _at ( 0xC9 );    /* TIMER 3 CONFIGURATION                        */
_sfrbyte TMR4CF   _at ( 0xC9 );    /* TIMER 4 CONFIGURATION                        */
_sfrbyte RCAP2L   _at ( 0xCA );    /* TIMER 2 CAPTURE REGISTER - LOW BYTE          */
_sfrbyte RCAP3L   _at ( 0xCA );    /* TIMER 3 CAPTURE REGISTER - LOW BYTE          */
_sfrbyte RCAP4L   _at ( 0xCA );    /* TIMER 4 CAPTURE REGISTER - LOW BYTE          */
_sfrbyte RCAP2H   _at ( 0xCB );    /* TIMER 2 CAPTURE REGISTER - HIGH BYTE         */
_sfrbyte RCAP3H   _at ( 0xCB );    /* TIMER 3 CAPTURE REGISTER - HIGH BYTE         */
_sfrbyte RCAP4H   _at ( 0xCB );    /* TIMER 4 CAPTURE REGISTER - HIGH BYTE         */
_sfrbyte TMR2L    _at ( 0xCC );    /* TIMER 2 - LOW BYTE                           */
_sfrbyte TMR3L    _at ( 0xCC );    /* TIMER 3 - LOW BYTE                           */
_sfrbyte TMR4L    _at ( 0xCC );    /* TIMER 4 - LOW BYTE                           */
_sfrbyte TMR2H    _at ( 0xCD );    /* TIMER 2 - HIGH BYTE                          */
_sfrbyte TMR3H    _at ( 0xCD );    /* TIMER 3 - HIGH BYTE                          */
_sfrbyte TMR4H    _at ( 0xCD );    /* TIMER 4 - HIGH BYTE                          */
_sfrbyte SMB0CR   _at ( 0xCF );    /* SMBUS 0 CLOCK RATE                           */
_sfrbyte PSW      _at ( 0xD0 );    /* PROGRAM STATUS WORD                          */
_sfrbyte REF0CN   _at ( 0xD1 );    /* VOLTAGE REFERENCE 0 CONTROL                  */
_sfrbyte DAC0L    _at ( 0xD2 );    /* DAC 0 REGISTER - LOW BYTE                    */
_sfrbyte DAC1L    _at ( 0xD2 );    /* DAC 1 REGISTER - LOW BYTE                    */
_sfrbyte DAC0H    _at ( 0xD3 );    /* DAC 0 REGISTER - HIGH BYTE                   */
_sfrbyte DAC1H    _at ( 0xD3 );    /* DAC 1 REGISTER - HIGH BYTE                   */
_sfrbyte DAC0CN   _at ( 0xD4 );    /* DAC 0 CONTROL                                */
_sfrbyte DAC1CN   _at ( 0xD4 );    /* DAC 1 CONTROL                                */
_sfrbyte HVA0CN   _at ( 0xD6 );    /* HVDA CONTROL REGISTER                        */
_sfrbyte PCA0CN   _at ( 0xD8 );    /* PCA 0 COUNTER CONTROL                        */
_sfrbyte CAN0DATL _at ( 0xD8 );    /* CAN 0 DATA - LOW BYTE                        */
_sfrbyte P5       _at ( 0xD8 );    /* PORT 5                                       */
_sfrbyte PCA0MD   _at ( 0xD9 );    /* PCA 0 COUNTER MODE                           */
_sfrbyte CAN0DATH _at ( 0xD9 );    /* CAN 0 DATA - HIGH BYTE                       */
_sfrbyte PCA0CPM0 _at ( 0xDA );    /* PCA 0 MODULE 0 CONTROL                       */
_sfrbyte CAN0ADR  _at ( 0xDA );    /* CAN 0 ADDRESS                                */
_sfrbyte PCA0CPM1 _at ( 0xDB );    /* PCA 0 MODULE 1 CONTROL                       */
_sfrbyte CAN0TST  _at ( 0xDB );    /* CAN 0 TEST                                   */
_sfrbyte PCA0CPM2 _at ( 0xDC );    /* PCA 0 MODULE 2 CONTROL                       */
_sfrbyte PCA0CPM3 _at ( 0xDD );    /* PCA 0 MODULE 3 CONTROL                       */
_sfrbyte PCA0CPM4 _at ( 0xDE );    /* PCA 0 MODULE 4 CONTROL                       */
_sfrbyte PCA0CPM5 _at ( 0xDF );    /* PCA 0 MODULE 5 CONTROL                       */
_sfrbyte ACC      _at ( 0xE0 );    /* ACCUMULATOR                                  */
_sfrbyte PCA0CPL5 _at ( 0xE1 );    /* PCA 0 MODULE 5 CAPTURE/COMPARE - LOW BYTE    */
_sfrbyte XBR0     _at ( 0xE1 );    /* CROSSBAR CONFIGURATION REGISTER 0            */
_sfrbyte PCA0CPH5 _at ( 0xE2 );    /* PCA 0 MODULE 5 CAPTURE/COMPARE - HIGH BYTE   */
_sfrbyte XBR1     _at ( 0xE2 );    /* CROSSBAR CONFIGURATION REGISTER 1            */
_sfrbyte XBR2     _at ( 0xE3 );    /* CROSSBAR CONFIGURATION REGISTER 2            */
_sfrbyte XBR3     _at ( 0xE4 );    /* CROSSBAR CONFIGURATION REGISTER 3            */
_sfrbyte EIE1     _at ( 0xE6 );    /* EXTERNAL INTERRUPT ENABLE 1                  */
_sfrbyte EIE2     _at ( 0xE7 );    /* EXTERNAL INTERRUPT ENABLE 2                  */
_sfrbyte ADC0CN   _at ( 0xE8 );    /* ADC 0 CONTROL                                */
_sfrbyte ADC2CN   _at ( 0xE8 );    /* ADC 2 CONTROL                                */
_sfrbyte P6       _at ( 0xE8 );    /* PORT 6                                       */
_sfrbyte PCA0CPL2 _at ( 0xE9 );    /* PCA 0 MODULE 2 CAPTURE/COMPARE - LOW BYTE    */
_sfrbyte PCA0CPH2 _at ( 0xEA );    /* PCA 0 MODULE 2 CAPTURE/COMPARE - HIGH BYTE   */
_sfrbyte PCA0CPL3 _at ( 0xEB );    /* PCA 0 MODULE 3 CAPTURE/COMPARE - LOW BYTE    */
_sfrbyte PCA0CPH3 _at ( 0xEC );    /* PCA 0 MODULE 3 CAPTURE/COMPARE - HIGH BYTE   */
_sfrbyte PCA0CPL4 _at ( 0xED );    /* PCA 0 MODULE 4 CAPTURE/COMPARE - LOW BYTE    */
_sfrbyte PCA0CPH4 _at ( 0xEE );    /* PCA 0 MODULE 4 CAPTURE/COMPARE - HIGH BYTE   */
_sfrbyte RSTSRC   _at ( 0xEF );    /* RESET SOURCE                                 */
_sfrbyte B        _at ( 0xF0 );    /* B REGISTER                                   */
_sfrbyte EIP1     _at ( 0xF6 );    /* EXTERNAL INTERRUPT PRIORITY REGISTER 1       */
_sfrbyte EIP2     _at ( 0xF7 );    /* EXTERNAL INTERRUPT PRIORITY REGISTER 2       */
_sfrbyte SPI0CN   _at ( 0xF8 );    /* SPI 0 CONTROL                                */
_sfrbyte CAN0CN   _at ( 0xF8 );    /* CAN 0 CONTROL                                */
_sfrbyte P7       _at ( 0xF8 );    /* PORT 7                                       */
_sfrbyte PCA0L    _at ( 0xF9 );    /* PCA 0 TIMER - LOW BYTE                       */
_sfrbyte PCA0H    _at ( 0xFA );    /* PCA 0 TIMER - HIGH BYTE                      */
_sfrbyte PCA0CPL0 _at ( 0xFB );    /* PCA 0 MODULE 0 CAPTURE/COMPARE - LOW BYTE    */
_sfrbyte PCA0CPH0 _at ( 0xFC );    /* PCA 0 MODULE 0 CAPTURE/COMPARE - HIGH BYTE   */
_sfrbyte PCA0CPL1 _at ( 0xFD );    /* PCA 0 MODULE 1 CAPTURE/COMPARE - LOW BYTE    */
_sfrbyte PCA0CPH1 _at ( 0xFE );    /* PCA 0 MODULE 1 CAPTURE/COMPARE - HIGH BYTE   */
_sfrbyte WDTCN    _at ( 0xFF );    /* WATCHDOG TIMER CONTROL                       */


/*  BIT Registers  */

/*  TCON  0x88 */
_sfrbit TF1   _atbit ( TCON, 7 );              /* TIMER 1 OVERFLOW FLAG      */
_sfrbit TR1   _atbit ( TCON, 6 );              /* TIMER 1 ON/OFF CONTROL     */
_sfrbit TF0   _atbit ( TCON, 5 );              /* TIMER 0 OVERFLOW FLAG      */
_sfrbit TR0   _atbit ( TCON, 4 );              /* TIMER 0 ON/OFF CONTROL     */
_sfrbit IE1   _atbit ( TCON, 3 );              /* EXT. INTERRUPT 1 EDGE FLAG */
_sfrbit IT1   _atbit ( TCON, 2 );              /* EXT. INTERRUPT 1 TYPE      */
_sfrbit IE0   _atbit ( TCON, 1 );              /* EXT. INTERRUPT 0 EDGE FLAG */
_sfrbit IT0   _atbit ( TCON, 0 );              /* EXT. INTERRUPT 0 TYPE      */

/*  CPT0CN  0x88 */
_sfrbit CP0EN   _atbit ( CPT0CN, 7 );          /* COMPARATOR 0 ENABLE                 */
_sfrbit CP0OUT  _atbit ( CPT0CN, 6 );          /* COMPARATOR 0 OUTPUT                 */
_sfrbit CP0RIF  _atbit ( CPT0CN, 5 );          /* COMPARATOR 0 RISING EDGE INTERRUPT  */
_sfrbit CP0FIF  _atbit ( CPT0CN, 4 );          /* COMPARATOR 0 FALLING EDGE INTERRUPT */
_sfrbit CP0HYP1 _atbit ( CPT0CN, 3 );          /* COMPARATOR 0 POSITIVE HYSTERESIS 1  */
_sfrbit CP0HYP0 _atbit ( CPT0CN, 2 );          /* COMPARATOR 0 POSITIVE HYSTERESIS 0  */
_sfrbit CP0HYN1 _atbit ( CPT0CN, 1 );          /* COMPARATOR 0 NEGATIVE HYSTERESIS 1  */
_sfrbit CP0HYN0 _atbit ( CPT0CN, 0 );          /* COMPARATOR 0 NEGATIVE HYSTERESIS 0  */

/*  CPT1CN  0x88 */
_sfrbit CP1EN   _atbit ( CPT1CN, 7 );          /* COMPARATOR 1 ENABLE                 */
_sfrbit CP1OUT  _atbit ( CPT1CN, 6 );          /* COMPARATOR 1 OUTPUT                 */
_sfrbit CP1RIF  _atbit ( CPT1CN, 5 );          /* COMPARATOR 1 RISING EDGE INTERRUPT  */
_sfrbit CP1FIF  _atbit ( CPT1CN, 4 );          /* COMPARATOR 1 FALLING EDGE INTERRUPT */
_sfrbit CP1HYP1 _atbit ( CPT1CN, 3 );          /* COMPARATOR 1 POSITIVE HYSTERESIS 1  */
_sfrbit CP1HYP0 _atbit ( CPT1CN, 2 );          /* COMPARATOR 1 POSITIVE HYSTERESIS 0  */
_sfrbit CP1HYN1 _atbit ( CPT1CN, 1 );          /* COMPARATOR 1 NEGATIVE HYSTERESIS 1  */
_sfrbit CP1HYN0 _atbit ( CPT1CN, 0 );          /* COMPARATOR 1 NEGATIVE HYSTERESIS 0  */

/*  CPT2CN  0x88 */
_sfrbit CP2EN   _atbit ( CPT2CN, 7 );          /* COMPARATOR 2 ENABLE                 */
_sfrbit CP2OUT  _atbit ( CPT2CN, 6 );          /* COMPARATOR 2 OUTPUT                 */
_sfrbit CP2RIF  _atbit ( CPT2CN, 5 );          /* COMPARATOR 2 RISING EDGE INTERRUPT  */
_sfrbit CP2FIF  _atbit ( CPT2CN, 4 );          /* COMPARATOR 2 FALLING EDGE INTERRUPT */
_sfrbit CP2HYP1 _atbit ( CPT2CN, 3 );          /* COMPARATOR 2 POSITIVE HYSTERESIS 1  */
_sfrbit CP2HYP0 _atbit ( CPT2CN, 2 );          /* COMPARATOR 2 POSITIVE HYSTERESIS 0  */
_sfrbit CP2HYN1 _atbit ( CPT2CN, 1 );          /* COMPARATOR 2 NEGATIVE HYSTERESIS 1  */
_sfrbit CP2HYN0 _atbit ( CPT2CN, 0 );          /* COMPARATOR 2 NEGATIVE HYSTERESIS 0  */

/*  SCON0  0x98 */
_sfrbit SM00 _atbit ( SCON0, 7 );              /* UART 0 MODE 0            */
_sfrbit SM10 _atbit ( SCON0, 6 );              /* UART 0 MODE 1            */
_sfrbit SM20 _atbit ( SCON0, 5 );              /* UART 0 MCE               */
_sfrbit REN0 _atbit ( SCON0, 4 );              /* UART 0 RX ENABLE         */
_sfrbit TB80 _atbit ( SCON0, 3 );              /* UART 0 TX BIT 8          */
_sfrbit RB80 _atbit ( SCON0, 2 );              /* UART 0 RX BIT 8          */
_sfrbit TI0  _atbit ( SCON0, 1 );              /* UART 0 TX INTERRUPT FLAG */
_sfrbit RI0  _atbit ( SCON0, 0 );              /* UART 0 RX INTERRUPT FLAG */

/*  SCON1  0x98 */
_sfrbit S0MODE _atbit ( SCON1, 7 );            /* UART 1 MODE              */
_sfrbit MCE1   _atbit ( SCON1, 5 );            /* UART 1 MCE               */
_sfrbit REN1   _atbit ( SCON1, 4 );            /* UART 1 RX ENABLE         */
_sfrbit TB81   _atbit ( SCON1, 3 );            /* UART 1 TX BIT 8          */
_sfrbit RB81   _atbit ( SCON1, 2 );            /* UART 1 RX BIT 8          */
_sfrbit TI1    _atbit ( SCON1, 1 );            /* UART 1 TX INTERRUPT FLAG */
_sfrbit RI1    _atbit ( SCON1, 0 );            /* UART 1 RX INTERRUPT FLAG */
                
/*  IE  0xA8 */
_sfrbit EA    _atbit ( IE, 7 );                /* GLOBAL INTERRUPT ENABLE      */
_sfrbit ET2   _atbit ( IE, 5 );                /* TIMER 2 INTERRUPT ENABLE     */
_sfrbit ES0   _atbit ( IE, 4 );                /* UART0 INTERRUPT ENABLE       */
_sfrbit ET1   _atbit ( IE, 3 );                /* TIMER 1 INTERRUPT ENABLE     */
_sfrbit EX1   _atbit ( IE, 2 );                /* EXTERNAL INTERRUPT 1 ENABLE  */
_sfrbit ET0   _atbit ( IE, 1 );                /* TIMER 0 INTERRUPT ENABLE     */
_sfrbit EX0   _atbit ( IE, 0 );                /* EXTERNAL INTERRUPT 0 ENABLE  */

/*  IP  0xB8 */
_sfrbit PT2   _atbit ( IP, 5 );                /* TIMER 2 PRIORITY					*/
_sfrbit PS    _atbit ( IP, 4 );                /* SERIAL PORT PRIORITY				*/
_sfrbit PT1   _atbit ( IP, 3 );                /* TIMER 1 PRIORITY					*/
_sfrbit PX1   _atbit ( IP, 2 );                /* EXTERNAL INTERRUPT 1 PRIORITY	*/
_sfrbit PT0   _atbit ( IP, 1 );                /* TIMER 0 PRIORITY					*/
_sfrbit PX0   _atbit ( IP, 0 );                /* EXTERNAL INTERRUPT 0 PRIORITY	*/
               
/* SMB0CN 0xC0 */
_sfrbit BUSY   _atbit ( SMB0CN, 7 );           /* SMBUS 0 BUSY                    */
_sfrbit ENSMB  _atbit ( SMB0CN, 6 );           /* SMBUS 0 ENABLE                  */
_sfrbit STA    _atbit ( SMB0CN, 5 );           /* SMBUS 0 START FLAG              */
_sfrbit STO    _atbit ( SMB0CN, 4 );           /* SMBUS 0 STOP FLAG               */
_sfrbit SI     _atbit ( SMB0CN, 3 );           /* SMBUS 0 INTERRUPT PENDING FLAG  */
_sfrbit AA     _atbit ( SMB0CN, 2 );           /* SMBUS 0 ASSERT/ACKNOWLEDGE FLAG */
_sfrbit SMBFTE _atbit ( SMB0CN, 1 );           /* SMBUS 0 FREE TIMER ENABLE       */
_sfrbit SMBTOE _atbit ( SMB0CN, 0 );           /* SMBUS 0 TIMEOUT ENABLE          */

/* CAN0STA 0xC0 */
//_sfrbit BOFF   _atbit ( CAN0STA, 7 );          /* Bus Off Status                  */
//_sfrbit EWARN  _atbit ( CAN0STA, 6 );          /* Warning Status                  */
//_sfrbit EPASS  _atbit ( CAN0STA, 5 );          /* Error Passive                   */
//_sfrbit RXOK   _atbit ( CAN0STA, 4 );          /* Received Message Successfully   */
//_sfrbit TXOK   _atbit ( CAN0STA, 3 );          /* Transmit a Message Successfully */
//_sfrbit LEC2   _atbit ( CAN0STA, 2 );          /* Last error code bit 2           */
//_sfrbit LEC1   _atbit ( CAN0STA, 1 );          /* Last error code bit 1           */
//_sfrbit LEC0   _atbit ( CAN0STA, 0 );          /* Last error code bit            */

/*  TMR2CN  0xC8 */
_sfrbit TF2   _atbit ( TMR2CN, 7 );            /* TIMER 2 OVERFLOW FLAG        */
_sfrbit EXF2  _atbit ( TMR2CN, 6 );            /* TIMER 2 EXTERNAL FLAG        */
_sfrbit EXEN2 _atbit ( TMR2CN, 3 );            /* TIMER 2 EXTERNAL ENABLE FLAG */
_sfrbit TR2   _atbit ( TMR2CN, 2 );            /* TIMER 2 ON/OFF CONTROL       */
_sfrbit CT2   _atbit ( TMR2CN, 1 );            /* TIMER 2 COUNTER SELECT       */
_sfrbit CPRL2 _atbit ( TMR2CN, 0 );            /* TIMER 2 CAPTURE SELECT       */

/*  TMR3CN  0xC8 */
_sfrbit TF3   _atbit ( TMR3CN, 7 );            /* TIMER 3 OVERFLOW FLAG        */
_sfrbit EXF3  _atbit ( TMR3CN, 6 );            /* TIMER 3 EXTERNAL FLAG        */
_sfrbit EXEN3 _atbit ( TMR3CN, 3 );            /* TIMER 3 EXTERNAL ENABLE FLAG */
_sfrbit TR3   _atbit ( TMR3CN, 2 );            /* TIMER 3 ON/OFF CONTROL       */
_sfrbit CT3   _atbit ( TMR3CN, 1 );            /* TIMER 3 COUNTER SELECT       */
_sfrbit CPRL3 _atbit ( TMR3CN, 0 );            /* TIMER 3 CAPTURE SELECT       */

/*  TMR4CN  0xC8 */
_sfrbit TF4   _atbit ( TMR4CN, 7 );            /* TIMER 4 OVERFLOW FLAG        */
_sfrbit EXF4  _atbit ( TMR4CN, 6 );            /* TIMER 4 EXTERNAL FLAG        */
_sfrbit EXEN4 _atbit ( TMR4CN, 3 );            /* TIMER 4 EXTERNAL ENABLE FLAG */
_sfrbit TR4   _atbit ( TMR4CN, 2 );            /* TIMER 4 ON/OFF CONTROL       */
_sfrbit CT4   _atbit ( TMR4CN, 1 );            /* TIMER 4 COUNTER SELECT       */
_sfrbit CPRL4 _atbit ( TMR4CN, 0 );            /* TIMER 4 CAPTURE SELECT       */

/*  PSW 0xD0 */
_sfrbit CY  _atbit ( PSW, 7 );                 /* CARRY FLAG              */
_sfrbit AC  _atbit ( PSW, 6 );                 /* AUXILIARY CARRY FLAG    */
_sfrbit F0  _atbit ( PSW, 5 );                 /* USER FLAG 0             */
_sfrbit RS1 _atbit ( PSW, 4 );                 /* REGISTER BANK SELECT 1  */
_sfrbit RS0 _atbit ( PSW, 3 );                 /* REGISTER BANK SELECT 0  */
_sfrbit OV  _atbit ( PSW, 2 );                 /* OVERFLOW FLAG           */
_sfrbit F1  _atbit ( PSW, 1 );                 /* USER FLAG 1             */
_sfrbit P   _atbit ( PSW, 0 );                 /* ACCUMULATOR PARITY FLAG */

/* PCA0CN 0xD8 */
_sfrbit CF   _atbit ( PCA0CN, 7 );             /* PCA 0 COUNTER OVERFLOW FLAG   */
_sfrbit CR   _atbit ( PCA0CN, 6 );             /* PCA 0 COUNTER RUN CONTROL BIT */
_sfrbit CCF5 _atbit ( PCA0CN, 5 );             /* PCA 0 MODULE 5 INTERRUPT FLAG */
_sfrbit CCF4 _atbit ( PCA0CN, 4 );             /* PCA 0 MODULE 4 INTERRUPT FLAG */
_sfrbit CCF3 _atbit ( PCA0CN, 3 );             /* PCA 0 MODULE 3 INTERRUPT FLAG */
_sfrbit CCF2 _atbit ( PCA0CN, 2 );             /* PCA 0 MODULE 2 INTERRUPT FLAG */
_sfrbit CCF1 _atbit ( PCA0CN, 1 );             /* PCA 0 MODULE 1 INTERRUPT FLAG */
_sfrbit CCF0 _atbit ( PCA0CN, 0 );             /* PCA 0 MODULE 0 INTERRUPT FLAG */


/* ADC0CN 0xE8 */
_sfrbit AD0EN   _atbit ( ADC0CN, 7 );          /* ADC 0 ENABLE                   */
_sfrbit AD0TM   _atbit ( ADC0CN, 6 );          /* ADC 0 TRACK MODE               */
_sfrbit AD0INT  _atbit ( ADC0CN, 5 );          /* ADC 0 EOC INTERRUPT FLAG       */
_sfrbit AD0BUSY _atbit ( ADC0CN, 4 );          /* ADC 0 BUSY FLAG                */
_sfrbit AD0CM1  _atbit ( ADC0CN, 3 );          /* ADC 0 CONVERT START MODE BIT 1 */
_sfrbit AD0CM0  _atbit ( ADC0CN, 2 );          /* ADC 0 CONVERT START MODE BIT 0 */
_sfrbit AD0WINT _atbit ( ADC0CN, 1 );          /* ADC 0 WINDOW INTERRUPT FLAG    */
                 
/* ADC2CN 0xE8 */
_sfrbit AD2EN   _atbit ( ADC2CN, 7 );          /* ADC 2 ENABLE                   */
_sfrbit AD2TM   _atbit ( ADC2CN, 6 );          /* ADC 2 TRACK MODE               */
_sfrbit AD2INT  _atbit ( ADC2CN, 5 );          /* ADC 2 EOC INTERRUPT FLAG       */
_sfrbit AD2BUSY _atbit ( ADC2CN, 4 );          /* ADC 2 BUSY FLAG                */
_sfrbit AD2WINT _atbit ( ADC2CN, 3 );          /* ADC 2 WINDOW INTERRUPT FLAG    */
_sfrbit AD2CM2  _atbit ( ADC2CN, 2 );          /* ADC 2 CONVERT START MODE BIT 2 */
_sfrbit AD2CM1  _atbit ( ADC2CN, 1 );          /* ADC 2 CONVERT START MODE BIT 1 */
_sfrbit AD2CM0  _atbit ( ADC2CN, 0 );          /* ADC 2 CONVERT START MODE BIT 0 */

/* SPI0CN 0xF8 */
_sfrbit SPIF   _atbit ( SPI0CN, 7 );           /* SPI 0 INTERRUPT FLAG          */
_sfrbit WCOL   _atbit ( SPI0CN, 6 );           /* SPI 0 WRITE COLLISION FLAG    */
_sfrbit MODF   _atbit ( SPI0CN, 5 );           /* SPI 0 MODE FAULT FLAG         */
_sfrbit RXOVRN _atbit ( SPI0CN, 4 );           /* SPI 0 RX OVERRUN FLAG         */
_sfrbit NSSMD1 _atbit ( SPI0CN, 3 );    	   /* SPI 0 SLAVE SELECT MODE BIT 1 */
_sfrbit NSSMD0 _atbit ( SPI0CN, 2 );		   /* SPI 0 SLAVE SELECT MODE BIT 0 */
_sfrbit TXBMT  _atbit ( SPI0CN, 1 );           /* SPI 0 TX BUFFER EMPTY         */
_sfrbit SPIEN  _atbit ( SPI0CN, 0 );           /* SPI 0 SPI ENABLE              */

/* CAN0CN 0xF8 */
_sfrbit CANINIT _atbit ( CAN0CN, 0 );          /* CAN Initialization bit */
_sfrbit CANIE   _atbit ( CAN0CN, 1 );          /* CAN Module Interrupt Enable Bit */
_sfrbit CANSIE  _atbit ( CAN0CN, 2 );          /* CAN Status change Interrupt Enable Bit */
_sfrbit CANEIE  _atbit ( CAN0CN, 3 );          /* CAN Module Error Interrupt Enable Bit */
_sfrbit CANIF   _atbit ( CAN0CN, 4 );          /* CAN Module Interrupt Flag */
_sfrbit CANDAR  _atbit ( CAN0CN, 5 );          /* CAN Disable Automatic Retransmission bit */
_sfrbit CANCCE  _atbit ( CAN0CN, 6 );          /* CAN Configuration Change Enable bit */
_sfrbit CANTEST _atbit ( CAN0CN, 7 );          /* CAN Test Mode Enable bit */


_sfrbit AUX2_IRQ        _atbit(P3, 0);    // P3.3  - Push-Pull , Digital
_sfrbit PRB_485EN       _atbit(P3, 1);    // P3.3  - Push-Pull , Digital
_sfrbit HBRIDGE_FAULT   _atbit(P3, 2);    // P3.3  - Push-Pull , Digital
_sfrbit MAIN_485EN      _atbit(P3, 3);    // P3.3  - Push-Pull , Digital
_sfrbit UART2_CS        _atbit(P3, 5);    // P3.5  -  Push-Pull , Digital
_sfrbit UART3_CS        _atbit(P3, 6);    // P3.6  -  Push-Pull , Digital
_sfrbit ETH_RST         _atbit(P3, 7);    // P3.6  -  Push-Pull , Digital
//_sfrbit LAMP_OVERTEMP   _atbit(P4, 0);    // P4.0  - Push-Pull , Digital
_sfrbit LAMP_OVERTEMP   _atbit(P5, 6);    //  rev H SLC board P5.6  - Push-Pull , Digital
//_sfrbit AUX_DIGITAL_INPUT _atbit(P4, 1);    // P4.1  - Push-Pull , Digital
_sfrbit AUX_DIGITAL_INPUT _atbit(P5, 7);    // rev H SLC board P5.7  - Push-Pull , Digital
// _sfrbit DEBUG_PIN		 _atbit(P4, 2);    // P4.2  - Push-Pull , Digital

// _sfrbit LCD_RS_SLC_B   _atbit(P4, 7);    // P4.7  -  LCD_RS for SLC B boards,      Push-Pull , Digital

// ===========================================
// SFR PAGE DEFINITIONS
// ===========================================

#define  CONFIG_PAGE       0x0F     /* SYSTEM AND PORT CONFIGURATION PAGE */
#define  LEGACY_PAGE       0x00     /* LEGACY SFR PAGE                    */
#define  TIMER01_PAGE      0x00     /* TIMER 0 AND TIMER 1                */
#define  CPT0_PAGE         0x01     /* COMPARATOR 0                       */
#define  CPT1_PAGE         0x02     /* COMPARATOR 1                       */
#define  CPT2_PAGE         0x03     /* COMPARATOR 2                       */
#define  UART0_PAGE        0x00     /* UART 0                             */
#define  UART1_PAGE        0x01     /* UART 1                             */
#define  SPI0_PAGE         0x00     /* SPI 0                              */
#define  EMI0_PAGE         0x00     /* EXTERNAL MEMORY INTERFACE          */
#define  ADC0_PAGE         0x00     /* ADC 0                              */
#define  ADC2_PAGE         0x02     /* ADC 2                              */
#define  SMB0_PAGE         0x00     /* SMBUS 0                            */
#define  TMR2_PAGE         0x00     /* TIMER 2                            */
#define  TMR3_PAGE         0x01     /* TIMER 3                            */
#define  TMR4_PAGE         0x02     /* TIMER 4                            */
#define  DAC0_PAGE         0x00     /* DAC 0                              */
#define  DAC1_PAGE         0x01     /* DAC 1                              */
#define  PCA0_PAGE         0x00     /* PCA 0                              */
#define  CAN0_PAGE         0x01     /* CAN 0                              */


// ===========================================
// Additional includes
// ===========================================

//#define KICK_WDT WDTCN=0xA5
#define KICK_WDT

#if 0  // // mbd need test points with Rev H SLC so do not compile below
#define DEBUG_HIGH	{							\
						SFRPAGE = CONFIG_PAGE;	\
						DEBUG_PIN = 1;			\
					}

#define	DEBUG_LOW	{							\
						SFRPAGE = CONFIG_PAGE;	\
						DEBUG_PIN = 0;			\
					}
#endif

// #include "timers.h"
// #include "stdarg.h"
// #include "stdlib.h"
// #include "i2c.h"
// #include "memory.h"
// #include "rtc.h"
// #include "serial.h"
// #include "spi.h"
// #include "lcd.h"
