//**********************************************************************
// Filename:      spi.c
// Rights:        Copyright (c) 2006 The Carlisle and Finch Company
//                All Rights Reserved
// Creation Date: 7/05/06
// Compiler:      Tasking cc51
// Version:       v7.1r1
// Purpose:       This module contains SPI interface.
//                Interface includes SC16IS740/50/60 Single SPI/UART with 64-byte FIFOs.
// Modification History:
// Date         Author      Description
// ----         ------      -----------
// 7-05-06		JRN			Created
//**********************************************************************

#include "c8051f040.h"
#include "SlProtocol.h"
#include "spi.h"
#include "serial.h"
#include "timers.h"

//=============================================================================
// Function   : SpiInitialize ()
// Purpose    : This function initializes the Serial Peripheral Interface.
// Parameters : Nothing is passed. 
// Returns    : Nothing is returned.
//=============================================================================

VOID SpiInitialize ( VOID )
{
    SFRPAGE   = SPI0_PAGE;
    
    //
    // SPI0CFG: SPI0 Configuration Register
    //
    // Bit6: MSTEN: Master Mode Enable
    //      0: Disable master mode. Operate in slave mode.
    //      1: Enable master mode. Operate as master.
    // Bit5: CKPHA: SPI0 Clock Phase
    //       This bit controls the SPI0 clock phase.
    //      0: Data centered on first edge of SCK period.
    //      1: Data centered on second edge of SCK period.
    // Bit4: CKPOL: SPI0 Clock Polarity
    //       This bit controls the SPI0 clock polarity.
    //      0: SCK line low in idle state.
    //      1: SCK line high in idle state.
    //
    
    SPI0CFG   = 0x40;   // SPI Master Mode 0
    
    //
    // SPI0CN: SPI0 Control Register
    //
    // Bit7: SPIF: SPI0 Interrupt Flag
    //       This bit is set to logic 1 by hardware at the end of a data transfer.
    //       If interrupts are enabled, setting this bit causes the CPU to vector
    //       to the SPI0 interrupt routine. This bit MUST be cleared by software.
    // Bits 3-2: NSSMD1-NSSMD0: Slave Select Mode
    //       Selects between the following NSS operation modes:
    //      00: 3-wire slave or 3-wire master mode. NSS is not routed to port pin.
    //      01: 4-wire slave or multi-master mode (default). NSS is always input to device.
    //      1x: 4-wire single-master mode. NSS signal is mapped as an output from the device
    //          and will assume the value of NSSMD0.
    // Bit0: SPIEN: SPI0 Enable
    //      0: SPI disabled.
    //      1: SPI enabled.
    //

//    SPI0CN    = 0x01;   // 3-wire Master SPI Enable
    SPI0CN    = 0x09;   // 4-wire Master SPI Enable
    
    //
    // SPI0CKR: SPI0 Clock Rate Register
    //
    // Bits7-0: SCR7-SCR0: SPI0 Clock Rate
    //       These bits determine the frequency of the SCK output when the SPI0 module
    //       is configured for master mode operation. The SCK clock frequency is a divided
    //       version of the system clock, and is given in the following equation, where SYSCLK
    //       is the system clock frequency and SPI0CKR is the 8-bit value held in the SPI0CKR register.
    //
    //       FSCK = SYSCLK / (2 * (SPI0CKR + 1))
    //
    
    SPI0CKR   = 0x18;   // 500KHz SPI Clock

} // SpiInitialize ()

//=====================================================================
// Function Name: SpiInitializeUart2 ()                                      
// Description:   This function initializes the external SPI interface
//                serial port.                               
//=====================================================================

VOID SpiInitializeUart2 ( VOID )
{
    SpiWriteByteUart2 ( SPI_UART_W_IOCTRL, 0x04 ); // Reset the SPI UART ( via software ).
    TimeWaitTicks ( 50 );                       // Allow some delay.
    SpiWriteByteUart2 ( SPI_UART_W_LCR, 0x80 );    // 0x80 to program baud rate
    TimeWaitTicks ( 50 );                       // Allow some delay.
    SpiWriteByteUart2 ( SPI_UART_W_DLL, 0x0E );    // 0x24=19.2K, 0x06 =115.2K with X1=11.0592 MHz
    TimeWaitTicks ( 50 );                       // Allow some delay.
    SpiWriteByteUart2 ( SPI_UART_W_DLH, 0x00 );    // divisor = 0x0024 for 19200 bps
    TimeWaitTicks ( 50 );                       // Allow some delay.
    // SpiWriteByteUart2 ( SPI_UART_EFR,   0 );    // Enable enhanced registers
    SpiWriteByteUart2 ( SPI_UART_EFR,   0x10 );    // Enable enhanced registers
    TimeWaitTicks ( 50 );                       // Allow some delay.
    SpiWriteByteUart2 ( SPI_UART_W_EFCR,   0x30 );   // Enable RS485 transmit enable on RTS pin
    TimeWaitTicks ( 50 );                       // Allow some delay.
    SpiWriteByteUart2 ( SPI_UART_W_LCR, 0x03 );    // 8 data bit, 1 stop bit, no parity
    TimeWaitTicks ( 50 );                       // Allow some delay.
    SpiWriteByteUart2 ( SPI_UART_W_IER, 0x03 );    // Enable Tx/Rx interrupts
     TimeWaitTicks ( 50 );                       // Allow some delay.
    SpiWriteByteUart2 ( SPI_UART_W_FCR, 0x07 );    // Reset TXFIFO, reset RXFIFO, FIFO mode
    TimeWaitTicks ( 50 );                       // Allow some delay.
    SpiWriteByteUart2 ( SPI_UART_W_IO_DIR, 0xff); // All GPIO pins on UART are set to outputs
    TimeWaitTicks ( 50 );                       // Allow some delay.
    SpiWriteByteUart2 ( SPI_UART_W_IO_INT_ENA, 0); // A change in input pin will not generate an interrupt
    TimeWaitTicks ( 50 );                       // Allow some delay.
        
    
    gSerUartQues[SER_UART_2].pTxQ->ucUart = (UINT) SER_UART_2;
    gSerUartQues[SER_UART_2].pRxQ->ucUart = (UINT) SER_UART_2;
    
} // SpiInitializeUart2 ()




//=====================================================================
// Function Name: SpiInitializeUart3 ()                                      
// Description:   This function initializes the external SPI interface
//                serial port.                               
//=====================================================================

VOID SpiInitializeUart3 ( VOID )
{
    SpiWriteByteUart3 ( SPI_UART_W_IOCTRL, 0x04 ); // Reset the SPI UART ( via software ).
    TimeWaitTicks ( 50 );                       // Allow some delay.
    SpiWriteByteUart3 ( SPI_UART_W_LCR, 0x80 );    // 0x80 to program baud rate
    TimeWaitTicks ( 50 );                       // Allow some delay.
    SpiWriteByteUart3 ( SPI_UART_W_DLL, 0x0E );    // 0x24=19.2K, 0x06 =115.2K with X1=11.0592 MHz
    TimeWaitTicks ( 50 );                       // Allow some delay.
    SpiWriteByteUart3 ( SPI_UART_W_DLH, 0x00 );    // divisor = 0x0024 for 19200 bps
    TimeWaitTicks ( 50 );                       // Allow some delay.
    // SpiWriteByteUart2 ( SPI_UART_EFR,   0 );    // Enable enhanced registers
    SpiWriteByteUart3 ( SPI_UART_EFR,   0x10 );    // Enable enhanced registers
    TimeWaitTicks ( 50 );                       // Allow some delay.
    SpiWriteByteUart3 ( SPI_UART_W_EFCR,   0x30 );   // Enable RS485 transmit enable on RTS pin
    TimeWaitTicks ( 50 );                       // Allow some delay.
    SpiWriteByteUart3 ( SPI_UART_W_LCR, 0x03 );    // 8 data bit, 1 stop bit, no parity
    TimeWaitTicks ( 50 );                       // Allow some delay.
    SpiWriteByteUart3 ( SPI_UART_W_IER, 0x03 );    // Enable Tx/Rx interrupts
    TimeWaitTicks ( 50 );                       // Allow some delay.
    SpiWriteByteUart3 ( SPI_UART_W_FCR, 0x07 );    // Reset TXFIFO, reset RXFIFO, FIFO mode
    TimeWaitTicks ( 50 );                       // Allow some delay.
    SpiWriteByteUart3 ( SPI_UART_W_IO_DIR, 0xff); // All GPIO pins on UART are set to outputs
    TimeWaitTicks ( 50 );                       // Allow some delay.
    SpiWriteByteUart3 ( SPI_UART_W_IO_INT_ENA, 0); // A change in input pin will not generate an interrupt
    TimeWaitTicks ( 50 );                       // Allow some delay.
        
    gSerUartQues[SER_UART_3].pTxQ->ucUart = (UINT) SER_UART_3;
    gSerUartQues[SER_UART_3].pRxQ->ucUart = (UINT) SER_UART_3;
    gSerUartQues[SER_UART_3].pTxQ->pIsr   = (UINT) SpiCheckUart3;
    
} // SpiInitializeUart3 ()




//=============================================================================
// Function   : SpiReadByteUart2 ( )
// Purpose    : Reads a byte from SPI slave device.
// Parameters : 8-bit address of SPI slave register to read.
// Returns    : 8-bit data byte returned from SPI slave.
//=============================================================================

UCHAR SpiReadByteUart2 ( UCHAR Address )
{
    UCHAR ret_val;
    
    SFRPAGE  = CONFIG_PAGE;
    UART2_CS = 0;
    
    //
    // Transmit the 8-bit register address of 
    // the SPI slave device to read.
    //
    
    SFRPAGE = SPI0_PAGE;                // Set SFRPAGE register to SPI0 page.
    while ( SPI0CFG & 0x80 ) KICK_WDT;           // Make sure there are no transactions in progress.
   	SPI0DAT = Address;                  // Transmit the 8-bit address to the SPI slave device.
    
    //
    // Wait for the SPI write operation to complete
    // before attempting to perform the SPI read operation.
    //
    
    SFRPAGE = SPI0_PAGE;                // Set SFRPAGE register to SPI0 page.
    while ( SPI0CFG & 0x80 )KICK_WDT;           // Make sure there are no transactions in progress.
    SPI0DAT = 0x00;                     // Provide clock pulses for SPI slave device to shift data out on.
    
    //
    // Wait for transfer to complete before
    // retrieving received data byte.
    //
    
    SFRPAGE = SPI0_PAGE;                // Set SFRPAGE register to SPI0 page.
    while ( SPI0CFG & 0x80 )KICK_WDT;           // Make sure there are no transactions in progress.
    ret_val = SPI0DAT;                  // Read 8-bit data byte from SPI slave device.
    
    SFRPAGE  = CONFIG_PAGE;
    UART2_CS = 1;
    
    return ret_val;
                     
} // SpiReadByteUart2 ()



//=============================================================================
// Function   : SpiWriteByteUart2 ()
// Purpose    : Writes a byte to SPI slave device at a specific address.
// Parameters : 8-bit register address of SPI slave device, 8-bit data byte.
// Returns    : Nothing is returned.
//=============================================================================

VOID SpiWriteByteUart2 ( UCHAR Address, UCHAR byte )
{                
    SFRPAGE  = CONFIG_PAGE;
    UART2_CS = 0;
    
    //
    // Transmit the 8-bit register address of 
    // the SPI slave device to read.
    //
    
    SFRPAGE = SPI0_PAGE;                // Set SFRPAGE register to SPI0 page.
    while ( SPI0CFG & 0x80 )KICK_WDT;           // Make sure there are no transactions in progress.
   	SPI0DAT = Address;                  // Transmit the 8-bit address to the SPI slave device.
    
    //
    // Wait for transfer to complete before
    // attempting to write data byte.
    //
    
    SFRPAGE = SPI0_PAGE;                // Set SFRPAGE register to SPI0 page.
    while ( SPI0CFG & 0x80 )KICK_WDT;           // Make sure there are no transactions in progress.
    SPI0DAT = byte;	                    // Write 8-bit data byte to SPI slave device.
    
    //
    // Wait for transfer to complete before returning.
    //
    
    SFRPAGE = SPI0_PAGE;                // Set SFRPAGE register to SPI0 page.
    while ( SPI0CFG & 0x80 )KICK_WDT;           // Make sure there are no transactions in progress.

    SFRPAGE  = CONFIG_PAGE;
    UART2_CS = 1;
    
} // SpiWriteByteUart2 ()




//=============================================================================
// Function   : SpiReadByteUart3 ( )
// Purpose    : Reads a byte from SPI slave device.
// Parameters : 8-bit address of SPI slave register to read.
// Returns    : 8-bit data byte returned from SPI slave.
//=============================================================================

UCHAR SpiReadByteUart3 ( UCHAR Address )
{
    UCHAR ret_val;
    
    SFRPAGE  = CONFIG_PAGE;
    UART3_CS = 0;
    
    //
    // Transmit the 8-bit register address of 
    // the SPI slave device to read.
    //
    
    SFRPAGE = SPI0_PAGE;                // Set SFRPAGE register to SPI0 page.
    while ( SPI0CFG & 0x80 )KICK_WDT;           // Make sure there are no transactions in progress.
   	SPI0DAT = Address;                  // Transmit the 8-bit address to the SPI slave device.
    
    //
    // Wait for the SPI write operation to complete
    // before attempting to perform the SPI read operation.
    //
    
    SFRPAGE = SPI0_PAGE;                // Set SFRPAGE register to SPI0 page.
    while ( SPI0CFG & 0x80 )KICK_WDT;           // Make sure there are no transactions in progress.
    SPI0DAT = 0x00;                     // Provide clock pulses for SPI slave device to shift data out on.
    
    //
    // Wait for transfer to complete before
    // retrieving received data byte.
    //
    
    SFRPAGE = SPI0_PAGE;                // Set SFRPAGE register to SPI0 page.
    while ( SPI0CFG & 0x80 )KICK_WDT;           // Make sure there are no transactions in progress.
    ret_val = SPI0DAT;                  // Read 8-bit data byte from SPI slave device.
    
    SFRPAGE  = CONFIG_PAGE;
    UART3_CS = 1;
    
    return ret_val;
                     
} // SpiReadByteUart3 ()



//=============================================================================
// Function   : SpiWriteByteUart3 ()
// Purpose    : Writes a byte to SPI slave device at a specific address.
// Parameters : 8-bit register address of SPI slave device, 8-bit data byte.
// Returns    : Nothing is returned.
//=============================================================================

VOID SpiWriteByteUart3 ( UCHAR Address, UCHAR byte )
{                
    SFRPAGE  = CONFIG_PAGE;
    UART3_CS = 0;
    
    //
    // Transmit the 8-bit register address of 
    // the SPI slave device to read.
    //
    
    SFRPAGE = SPI0_PAGE;                // Set SFRPAGE register to SPI0 page.
    while ( SPI0CFG & 0x80 )KICK_WDT;           // Make sure there are no transactions in progress.
   	SPI0DAT = Address;                  // Transmit the 8-bit address to the SPI slave device.
    
    //
    // Wait for transfer to complete before
    // attempting to write data byte.
    //
    
    SFRPAGE = SPI0_PAGE;                // Set SFRPAGE register to SPI0 page.
    while ( SPI0CFG & 0x80 )KICK_WDT;           // Make sure there are no transactions in progress.
    SPI0DAT = byte;	                    // Write 8-bit data byte to SPI slave device.
    
    //
    // Wait for transfer to complete before returning.
    //
    
    SFRPAGE = SPI0_PAGE;                // Set SFRPAGE register to SPI0 page.
    while ( SPI0CFG & 0x80 )KICK_WDT;           // Make sure there are no transactions in progress.

    SFRPAGE  = CONFIG_PAGE;
    UART3_CS = 1;
    
} // SpiWriteByteUart3 ()


//=============================================================================
// Function   : SpiCheckUart2 ()
// Purpose    : This function polls the SPI / UART Interrupt Identification Register ( IIR ).
// Parameters : Nothing is passed.
// Returns    : Nothing is returned.
// Note       : * Supports SC16IS750 UART
//              * Takes processor approx. 70us to complete.
//=============================================================================

#define SC16IS750_IIR_INT_PENDING       0x01

#define SC16IS750_IIR_MASK              0x3E
#define SC16IS750_IIR_RX_LINE_STATUS    0x06
#define SC16IS750_IIR_RX_TIMEOUT_INT    0x0c
#define SC16IS750_IIR_RHR_INT           0x04
#define SC16IS750_IIR_THR_INT           0x02
#define SC16IS750_IIR_MODEM_INT         0x00
#define SC16IS750_IIR_INPUT_PIN         0x30
#define SC16IS750_IIR_RX_XOFF_SPECIAL   0x10
#define SC16IS750_IIR_MODEM_INT         0x00
#define SC16IS750_IIR_CTS_RTS_CHANGE    0x20

VOID SpiCheckUart2 ( VOID )
{
    UCHAR UartByte;
    UCHAR RxLvl;
    UCHAR ucLsr;
    
    SFRPAGE  = CONFIG_PAGE;
    UART2_CS = 0;
    RxLvl = SpiReadByteUart2 ( SPI_UART_R_RXLVL );
    
    if ( RxLvl )
    {
RxAgain:
        KICK_WDT;
        UartByte = SpiReadByteUart2 ( SPI_UART_R_RHR );
        SerQueInsertMacro ( &g_SerRxQueMain, UartByte );
        --RxLvl;
        if (RxLvl)
            goto RxAgain;
        UART2_CS = 1;
        *g_SerRxQueMain.puiTimeout = 1000;
        return;
    }

    //
    // Be sure we're in RX mode if no data is TX 
    // data is pending at the UART.
    // 
    ucLsr = SpiReadByteUart2 ( SPI_UART_R_LSR );
    if (ucLsr & 0x40)
    {
        MAIN_485EN = 0;
    }
    
    UART2_CS = 1;
    
} // SpiCheckUart2 ()


//=============================================================================
// Function   : SpiCheckUart3 ()
// Purpose    : This function polls the SPI / UART Interrupt Identification Register ( IIR ).
// Parameters : Nothing is passed.
// Returns    : Nothing is returned.
// Note       : * Supports SC16IS750 UART
//              * Takes processor approx. 70us to complete.
//=============================================================================


VOID SpiCheckUart3 ( VOID )
{
    UCHAR UartByte;
    UCHAR TxLvl;
    UCHAR RxLvl;
    
    SFRPAGE  = CONFIG_PAGE;
    UART3_CS = 0;

    //
    // Receive data if available.
    //
        
    RxLvl = SpiReadByteUart3 ( SPI_UART_R_RXLVL );
    if ( RxLvl )
    {
RxAgain:
        KICK_WDT;
        UartByte = SpiReadByteUart3 ( SPI_UART_R_RHR );
        SerQueInsertMacro ( &g_SerRxQueEthPort, UartByte );
        --RxLvl;
        if (RxLvl)
            goto RxAgain;
        *g_SerRxQueMain.puiTimeout = 1000;
    }
    
    //
    // Transmit data if available...
    //
    
    TxLvl = SpiReadByteUart3 ( SPI_UART_R_TXLVL );
    while (TxLvl <= 64)
    {
        if ( !SerQueSpaceUsed (&g_SerTxQueEthPort))
            break;
        SerQueRemoveMacro ( &g_SerTxQueEthPort, &UartByte );
        SpiWriteByteUart3 ( SPI_UART_W_THR, UartByte );
        --TxLvl;
        
    } // while (TxLvl < 63)
    
    UART3_CS = 1;
    
} // SpiCheckUart3 ()



