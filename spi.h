//**********************************************************************
// Filename:      spi.h
// Rights:          Copyright (c) 2006 The Carlisle and Finch Company
//                   All Rights Reserved
// Creation Date: 7/05/06
// Compiler:      Tasking cc51
// Version:       v7.1r1
// Purpose:       This module contains SPI constants and function prototypes.
// Modification History:
// Date            Author        Description
// ----         ------      -----------
// 7-05-06        JRN            Created
//**********************************************************************

#ifndef _SPI_H_
#define _SPI_H_
     
//================================================================================================================
// Philips SC16IS740/50/60 Single SPI / UART with 64-byte FIFOs.
//
// SPI Register Address Byte
//  [ R/W ] [ A3 ] [ A2 ] [ A1 ] [ A0 ] [ CH1 ] [ CH0 ] [ NC ]
//

#define SPI_UART_READ       0x80
#define SPI_UART_WRITE      0x00

//
//       Register                   Address                             Description                        R / W
//---------------------------------------------------------------------------------------------------------------
#define SPI_UART_R_RHR        ( SPI_UART_READ  | ( 0x00 << 3 ) )      // Receive Holding Register            R
#define SPI_UART_W_THR        ( SPI_UART_WRITE | ( 0x00 << 3 ) )      // Transmit Holding Register           W
#define SPI_UART_R_IER        ( SPI_UART_READ  | ( 0x01 << 3 ) )      // Interrupt Enable Register           R
#define SPI_UART_W_IER        ( SPI_UART_WRITE | ( 0x01 << 3 ) )      // Interrupt Enable Register           W
#define SPI_UART_R_IIR        ( SPI_UART_READ  | ( 0x02 << 3 ) )      // Interrupt Identification Register   R
#define SPI_UART_W_FCR        ( SPI_UART_WRITE | ( 0x02 << 3 ) )      // FIFO Control Register               W
#define SPI_UART_EFR          ( SPI_UART_WRITE | ( 0x02 << 3 ) )      // Enhanced Function Register          W
#define SPI_UART_W_LCR        ( SPI_UART_WRITE | ( 0x03 << 3 ) )      // Line Control Register               W
#define SPI_UART_W_MCR        ( SPI_UART_WRITE | ( 0x04 << 3 ) )      // Modem Control Register              W
#define SPI_UART_R_LSR        ( SPI_UART_READ  | ( 0x05 << 3 ) )      // Line Status Register                R
#define SPI_UART_R_MSR        ( SPI_UART_READ  | ( 0x06 << 3 ) )      // Modem Status Register               R
#define SPI_UART_R_TXLVL      ( SPI_UART_READ  | ( 0x08 << 3 ) )      // Transmitter FIFO Level Register     R
#define SPI_UART_R_RXLVL      ( SPI_UART_READ  | ( 0x09 << 3 ) )      // Receiver FIFO Level Register        R
#define SPI_UART_W_DLL        ( SPI_UART_WRITE | ( 0x00 << 3 ) )      // Divisor Latch LSB Register          W
#define SPI_UART_W_DLH        ( SPI_UART_WRITE | ( 0x01 << 3 ) )      // Divisor Latch MSB Register          W
#define SPI_UART_W_IO_DIR     ( SPI_UART_WRITE | ( 0x0A << 3 ) )      // IO Control Register                 W
#define SPI_UART_R_IOSTATE    ( SPI_UART_READ  | ( 0x0B << 3 ) )      // IO Control Register                 W
#define SPI_UART_W_IO_INT_ENA ( SPI_UART_WRITE | ( 0x0C << 3 ) )      // IO Control Register                 W
#define SPI_UART_W_IOCTRL     ( SPI_UART_WRITE | ( 0x0E << 3 ) )      // IO Control Register                 W
#define SPI_UART_W_EFCR       ( SPI_UART_WRITE | ( 0x0F << 3 ) )      // Enhanced Function Register          W



//
// Interrupt Identification Values
//

#define SPI_UART_THR_IRQ    0x02        // Transmit Holding Register Interrupt
#define SPI_UART_RHR_IRQ    0x04        // Receive Holding Register Interrupt

//
// Baud Rate Divisor Values
//

#define SPI_UART_19200      0x24        // SPI / UART Baud Rate 19200  ( XTAL1 = 11.05MHz, divide by 1 )
#define SPI_UART_115200     0x06        // SPI / UART Baud Rate 115200 ( XTAL1 = 11.05MHz, divide by 1 )

//================================================================================================================



// ===========================================
// Function prototypes
// ===========================================

VOID  SpiInitialize      ( VOID );
VOID  SpiWriteByteUart2  ( UCHAR Address, UCHAR byte );
VOID  SpiInitializeUart2 ( VOID );
VOID  SpiInitializeUart3 ( VOID );
UCHAR SpiReadByteUart2   ( UCHAR Address );
UCHAR SpiReadByteUart3   ( UCHAR Address );
VOID  SpiWriteByteUart2  ( UCHAR Address, UCHAR byte );
VOID  SpiWriteByteUart3  ( UCHAR Address, UCHAR byte );
VOID  SpiCheckUart2      ( VOID );
VOID  SpiCheckUart3      ( VOID );

// ===========================================
// Extern declarations
// ===========================================


#endif // #ifndef _SPI_H_