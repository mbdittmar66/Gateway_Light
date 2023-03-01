//**********************************************************************
// Filename:      serial.c
// Rights:        Copyright (c) 2006 The Carlisle and Finch Company
//                All Rights Reserved
// Creation Date: 7/05/06
// Compiler:      Tasking cc51
// Version:       v7.1r1
// Purpose:       This module contains serial UART interface.
// Modification History:
// Date         Author      Description
// ----         ------      -----------
// 7-05-06      JRN         Created
//**********************************************************************

#include "c8051f040.h"
#include "SlProtocol.h"
#include "serial.h"
#include "timers.h"
#include "main.h"
#include "stdlib.h"
#include "spi.h"
#include "utils.h"
#include "SLC_Init.h"  

SER_UART_QUE     g_SerRxQueMain;     // Uart 0
SER_UART_QUE     g_SerTxQueMain;     // Uart 0

SER_UART_QUE     g_SerRxQuePrb;      // Uart 1
SER_UART_QUE     g_SerTxQuePrb;      // Uart 1

SER_UART_QUE     g_SerRxQueSerPort;    // Uart 2
SER_UART_QUE     g_SerTxQueSerPort;

SER_UART_QUE     g_SerRxQueEthPort;    // Uart 3
SER_UART_QUE     g_SerTxQueEthPort;

tSerUartQues     gSerUartQues [SER_NUMBER_UARTS] = 
{
    { "SerPort",   &g_SerRxQueSerPort   , &g_SerTxQueSerPort   }, // Indexed by: SER_UART_0 at lowest level
    { "Prb" ,   &g_SerRxQuePrb    , &g_SerTxQuePrb    }, // Indexed by: SER_UART_1 at lowest level
    { "Main",   &g_SerRxQueMain   , &g_SerTxQueMain   }, // Indexed by: SER_UART_2 at lowest level
    { "EthPort",   &g_SerRxQueEthPort   , &g_SerTxQueEthPort   }, // Indexed by: SER_UART_3 at lowest level
};

tSER_PACKET    g_PreviousMessage; 

UCHAR          g_DEBUG = FALSE;
UINT           uiSerTxPacketTypeCount [32];
UINT           uiSerRxPacketTypeCount [32];

static UCHAR   g_Uart0IsrByte;
static UCHAR   g_Uart1IsrByte;

//
// Global Preloaded / stored payloads and packets
//
                                    
tPAYLOAD_SLC_EXT_STATUS              SerSlcExtStatusPayload      ;   
tPAYLOAD_SLC_MISC_SETUP              SerSlcMiscSetupPayload      ;   
tPAYLOAD_SLC_DEVICE_NAME_ETC         SerSlcDeviceNameEtcPayload  ;   
tPAYLOAD_DO_CALIBRATION              SerDoCalibrationPayload     ;
tPAYLOAD_TOUR_POINT                  SerTourPointPayload         ;

UCHAR                                SerTourPointDestAddr        ;
UCHAR                                SerRequestSlcDevNameEtc     ;
UCHAR                                SerRequestSlcMiscSetup      ;

//=============================================================================
// Function   : SerInitializeUARTs  ()
// Purpose    : This function initializes Universal Asynchronous Receiver/Transmitter.
// Parameters : Nothing is passed. 
// Returns    : Nothing is returned.
//=============================================================================

VOID SerInitializeUARTs ( VOID )
{
    UCHAR       n;

    SFRPAGE   = UART0_PAGE;
    SCON0     = 0x70;
    SSTA0     = 0x15;   // UART0 uses Timer2 for buad rate generation    
                        // UART1 uses TIMER1 for baud rate generation
                           
    SFRPAGE   = UART1_PAGE;
    SCON1     = 0x70;

    //
    // Setup ucUart field in each Que
    //

    for (n = 0; n < hbound1(gSerUartQues); n++)
    {
        gSerUartQues[n].pTxQ->ucUart = n;
        gSerUartQues[n].pRxQ->ucUart = n;
        gSerUartQues[n].pRxQ->puiTimeout = &g_TimeMs.RxTimeout[n];
    }
    
} // InitializeUARTs ()


//=====================================================================
// Function   : SerTransmitPacket ()                                           
// Purpose    : Transmits a packet out the desired QUE, using the 
//              protocol.  
// Parameters : A packet built with addresses, payload length,
//              packet type, and payload. 
// Note       : STX / ETX and checksum are internally generated,
//              and need not be setup by the caller.
// Returns    : Nothing is returned.
//=====================================================================
                  
VOID  SerTransmitPacket ( SER_UART_QUE *pQue, VOID *pTx)
{
    UCHAR CalculatedChecksum;
    UCHAR i;
    UCHAR *c;
    tSER_PACKET *pTxPacket;
    UCHAR DestAddr;
    
    pTxPacket = (tSER_PACKET *) pTx;
    
    //
    // If a packet is routed, then override the destination queue
    //
    
    DestAddr = pTxPacket->Preamble.DestAddr;
    if (g_BoardNumber == ( DestAddr & MASK_ADDRESS ))
    {
        //
        // Route / Reroute to SerPort ?
        //
        
        if (SLC_SERPORT_PROTOCOL_OFFSET == (MASK_DEVICE_TYPE & DestAddr))
        { 
            pQue = &g_SerTxQueSerPort;
        }
            
        //
        // Route / Reroute to EthPort?
        //
        
        if (SLC_ETHPORT_PROTOCOL_OFFSET == (MASK_DEVICE_TYPE & DestAddr) )
        { 
            pQue = &g_SerTxQueEthPort;
        }
    }
    
    //
    // Populate STX / ETX and checksum...
    //
    
    pTxPacket->Preamble.Stx = STX;
    
    CalculatedChecksum = 0;
    
    c = (UCHAR *) pTxPacket + 1;
    for (i = 1; i < (5+pTxPacket->Preamble.PayloadLength); i++)
    {
        CalculatedChecksum += *c;
        ++c;
    }
    pTxPacket->pPayload[pTxPacket->Preamble.PayloadLength  ] = CalculatedChecksum;
    
    pTxPacket->pPayload[pTxPacket->Preamble.PayloadLength+1] = ETX;
    SerTransmit ( pQue , (UCHAR *) pTxPacket, pTxPacket->Preamble.PayloadLength + 7);
    
    // pTxPacket->pPayload[pTxPacket->PayloadLength+2] = ETX;
    // SerTransmit ( pQue , (UCHAR *) pTxPacket, pTxPacket->PayloadLength + 8); // Add in 2nd ETX
    
    
    //
    // Accumulate TX Statistics...
    //
    
    if ( pTxPacket->Preamble.DestAddr & BROADCAST_BIT)
    {                                                                               
        ++(pQue->Stats.uiAddrBroadcast);
    } else
    {
        ++(pQue->Stats.uiAddrMatch);
    }
    ++pQue->Stats.uiNumPacketsOk;
    
    //
    // Accumulate TX stats if addressed for JCS or SLC boards
    //
    
    if (pTxPacket->Preamble.DestAddr < hbound1(g_TxPacketCount)) 
    {
        ++g_TxPacketCount[pTxPacket->Preamble.DestAddr];  // Total # packets sent to JCS
    }
    
    //
    // Accumulate a stat on type of transmitted packet.
    //
    
    if (pTxPacket->Preamble.PacketType < hbound1(uiSerTxPacketTypeCount))
    {
        ++uiSerTxPacketTypeCount[pTxPacket->Preamble.PacketType];
    }
    	    	    
} // SerTransmitPacket ( )

//=====================================================================
// Function   : SerStrTransmit ()                                        
// Purpose    : This function writes a zero terminated string 
//              as a block of data to a serial port.                     
// Parameters : 8-bit character to write.
// Returns    : Nothing is returned.
//=====================================================================

VOID SerStrTransmit ( SER_UART_QUE *pQue, UCHAR *c)
{
    SerTransmit ( pQue, c, (UCHAR) StdStrLen(c) );
}

//=====================================================================
// Function   : SerRomStrTransmit ()                                        
// Purpose    : This function writes a zero terminated _rom string 
//              as a block of data to a serial port.                     
// Parameters : 8-bit character to write.
// Returns    : Nothing is returned.
//=====================================================================

VOID SerRomStrTransmit ( SER_UART_QUE *pQue, _rom UCHAR *pucRom)
{
    UCHAR ucRam;
    
    while (*pucRom)
    {
        ucRam = *pucRom;
        SerTransmit ( pQue, &ucRam, 1);
        ++pucRom;
    }
}

//=====================================================================
// Function   : SerTransmit ()                                        
// Purpose    : This function writes a block of data to a serial port.                     
// Parameters : 8-bit character to write.
// Returns    : Nothing is returned.
//=====================================================================

VOID SerTransmit ( SER_UART_QUE *pQue, UCHAR *c, UCHAR ucLength)
{
    UCHAR ucC;
	static UCHAR toggle1, toggle2;
    //
    // Buffer all data for transmission...
    //
    
    while (ucLength)
    {
        //
        // If the buffer is full, then spin until there is space available. 
        // 
        
        if ( 0 == SerQueSpaceAvailable ( pQue ) )
        {
            ++pQue->Stats.uiErrorQueFull;

            //
            // On protocol channels, a full buffer is non recoverable, so
            // zap the buffer and continue...
            //

            if ( (SER_UART_2 == pQue->ucUart) ||
                 (SER_UART_3 == pQue->ucUart) )
            {
                SerQueFlush ( pQue );
            }
            while ( 0 == SerQueSpaceAvailable ( pQue ) );
        }
        
        //
        // Space is avaliable in the que, insert, adjust and
        // continue.
        
        SerQueInsert(pQue, *c);
        ++c;
        --ucLength;
    } 

    //
    // Kick Appropriate UART(s) into gear (if so needed).
    //
    
    if (pQue->LineStatus != LINE_TRANSMIT)
    {
        if (pQue->ucUart == SER_UART_0)
        {

            if (SerQueRemove (pQue, &ucC))
            {
                SFRPAGE = UART0_PAGE;
                SBUF0 = ucC;
                pQue->LineStatus = LINE_TRANSMIT;
            }
        }
    
        if (pQue->ucUart == SER_UART_1)
        {
            if (SerQueRemove (pQue, &ucC))
            {
                PRB_485EN = 1;
				            
			    if (toggle1)	 // mbd 2/28/2019 - out pulse on port 4.6
		    	{
//		        	MTR_CS1 = 0;	  // mbd need test points with Rev H SLC
		    	}
		    	else
		    	{
//		        	MTR_CS1 = 1;	   // mbd need test points with Rev H SLC
		    	}
		    	toggle1 = ~toggle1;


                SFRPAGE   = UART1_PAGE;
                SBUF1 = ucC;
                pQue->LineStatus = LINE_TRANSMIT;
            }
        }
        
        if (pQue->ucUart == SER_UART_2)
        {
            UCHAR ucLsr;
            
            SFRPAGE  = CONFIG_PAGE;
            MAIN_485EN = 1;
			if (toggle2)	 // mbd 2/28/2019 - out pulse on port 4.6
		    {
//		       	TestBit1 = 0;  // mbd need test points with Rev H SLC // mbd 2/28/2019 
		    }
		    else
		    {
//		       	TestBit1 = 1;	  // mbd need test points with Rev H SLC
		    }
		    toggle2 = ~toggle2;


            UART2_CS = 0;
            // SpiWriteByteUart2( SPI_UART_W_THR, 0);
            pQue->LineStatus = LINE_TRANSMIT;
            
            while (SerQueRemove(pQue, &ucC))
            {
                SpiWriteByteUart2 ( SPI_UART_W_THR, ucC);
            }
                        
            //
            // Be sure we're in RX mode if no data is TX 
            // data is pending at the UART.
            // 
            do 
            {
                ucLsr = SpiReadByteUart2 ( SPI_UART_R_LSR );
                
            } while (0 == (ucLsr & 0x40));
            
            pQue->LineStatus = LINE_RECEIVE;
            SFRPAGE  = CONFIG_PAGE;
            MAIN_485EN = 0;

        }
        
        if (pQue->ucUart == SER_UART_3)
        {
            if (SerQueRemove (pQue, &ucC))
            {
                SpiWriteByteUart3 ( SPI_UART_W_THR, ucC );
                pQue->LineStatus = LINE_TRANSMIT;
            }
        }
    
    }
    
} // SerTransmit ()







//=====================================================================
// Function   : UART0_ISR  
// Purpose    : This routine checks transmit/receive interrupts       
//              for UART 0 and services those respective interrupts.             
// Parameters : Nothing is passed.
// Returns    : Nothing is returned.
//=====================================================================

_interrupt ( 4 ) _large
VOID UART0_ISR ( VOID )
{
    static UCHAR ucSSTA0;    
    
    static SER_UART_QUE *pRxQ;
    static SER_UART_QUE *pTxQ;

    pRxQ = gSerUartQues [SER_UART_0].pRxQ;
    pTxQ = gSerUartQues [SER_UART_0].pTxQ;
    
    
    //
    // Accumulate statistics...
    //
    
    ++MainStats.uiUart0Isr;
    
    if ( RI0 ) 
    {
        RI0 = 0;
        
        ++(pRxQ->Stats.uiIntsRx);
    
        g_Uart0IsrByte = SBUF0;
        
        SerQueInsertMacro ( pRxQ, g_Uart0IsrByte );
        
        *pRxQ->puiTimeout = 1000;
    
        ucSSTA0 = SSTA0;
        if (ucSSTA0 & 0x80)   // Inspect FE0
        {
            ++(pRxQ->Stats.uiIntsModem);
            SSTA0 &= 0x7F;          // Clear FE0
            // Uart0IsrByte = SBUF0;   // Read byte which may have generated the error
        }
        
        ucSSTA0 = SSTA0;
        if (ucSSTA0 & 0x40)  // Inspect RXOV0
        {
            ++(pRxQ->Stats.uiIntsUnknown);
            SSTA0 &= 0xBF;              // Clear RXOV0
            // Uart0IsrByte = SBUF0;       // Read byte which may have generated the error
        }
        
        ucSSTA0 = SSTA0;
        
    } // if (RI0)
     
    if ( TI0 ) 
    {
        ++(pTxQ->Stats.uiIntsTx);
        
        //
        // Check for transmit collision...
        //
        
        ucSSTA0 = SSTA0;
        if ( ucSSTA0 & 0x20 )
        {
            ++(pTxQ->Stats.uiIntsModem);
            SSTA0 &= 0xDF;          // Clear FE0
        }
        ucSSTA0 = SSTA0;
    
        //
        // If the TX que is populated, then transmit a byte...
        //
    
        if ( pTxQ->Head != pTxQ->Tail )
        {
            SerQueRemoveMacro ( pTxQ, &g_Uart0IsrByte );
            SBUF0 = g_Uart0IsrByte;
        }
        else
        {
            //
            // No more data is available for transmission,
            // flag the s/w fifo as showing the UART0 as
            // being not busy, and turn off the RS-485.
            //
            
            SFRPAGE = CONFIG_PAGE;
            MAIN_485EN = 0;
            pTxQ->LineStatus = LINE_RECEIVE;
            // g_SerTxQueMain.LineStatus = LINE_TRANSMIT_ENDING;
            SFRPAGE = UART0_PAGE;
        }
    
        TI0 = 0;
                                     
    } // if (TI0)

} // UART0_ISR ()


//=====================================================================
// Function   : UART1_ISR  
// Purpose    : This routine checks transmit/receive interrupts       
//              for UART 1 and services those respective interrupts.             
// Parameters : Nothing is passed.
// Returns    : Nothing is returned.
//=====================================================================

_interrupt ( 20 ) _large
VOID UART1_ISR ( VOID ) 
{
    static SER_UART_QUE *pRxQ;
    static SER_UART_QUE *pTxQ;

    //
    // Accumulate statistics...
    //
    ++MainStats.uiUart1Isr;
    
    pRxQ = gSerUartQues [SER_UART_1].pRxQ;
    pTxQ = gSerUartQues [SER_UART_1].pTxQ;
    
    
    if ( RI1 ) 
    {
        RI1 = 0;
        ++(pRxQ->Stats.uiIntsRx);
        
        g_Uart1IsrByte = SBUF1;
    
        SerQueInsertMacro ( pRxQ, g_Uart1IsrByte );
        
        *pRxQ->puiTimeout = 1000;
    
    } // if (RI0)
     
    if ( TI1 ) 
    {
        TI1 = 0;
        
        ++(pTxQ->Stats.uiIntsTx);
        
        if ( pTxQ->Head != pTxQ->Tail )
        {
            SerQueRemoveMacro( pTxQ, &g_Uart1IsrByte );
            SBUF1 = g_Uart1IsrByte;
        }
        else
        {
            SFRPAGE = CONFIG_PAGE;
            PRB_485EN = 0;
            pTxQ->LineStatus = LINE_RECEIVE;
            // g_SerTxQuePrb.LineStatus = LINE_TRANSMIT_ENDING;
            SFRPAGE = UART1_PAGE;
        }
                    
    } // if (TI1)

    return;
    
} // UART1_ISR ()


// ===========================================================================
// Function:    SerQueFlush()
// Purpose:     Zaps all data in a queue.
// Parameters:  Pointer to UART queue to flush.
// Returns:     Nothing is returned.
// ===========================================================================

VOID SerQueFlush ( pSER_UART_QUE q )
{
    q->Head = 0;
    q->Tail = 0;
    q->TailInspect = 0;

} // SerQueFlush ()


// ===========================================================================
// Function:    SerQueInspectEmpty()
// Purpose:     Returns TRUE if the queue is empty for inspection, FALSE otherwise.
// Parameters:  Pointer to UART queue to check for empty.
// Returns:     TRUE if the queue is empty, FALSE otherwise.
// ===========================================================================

BOOLEAN SerQueInspectEmpty ( pSER_UART_QUE q )
{
    return ((q->Head) == (q->TailInspect));

} // SerQueInspectEmpty()



// ===========================================================================
// Function:    SerQueInspect ()
// Purpose:     Inspects (temporary remove) a character from the desired que. 
// Parameters:  Pointer to UART queue
// Returns:     TRUE if succesful, FALSE otherwise.
// ===========================================================================

BOOLEAN SerQueInspect ( pSER_UART_QUE q, UCHAR *c )
{
    static UCHAR TailInspect;             
    
    //
    // Leave if no new data...
    //

    if ( SerQueInspectEmpty ( q ) )
    {
        return FALSE;
    }

    //
    // Get byte from inspection tail.
    //
                                                                
    TailInspect = q->TailInspect;                         
    *c = q->Buffer[TailInspect];                             
                                                                
                                                                
    /* Adjust tail pointer, wrap pointer as required. */        
                                                                
    TailInspect++;        
    if (TailInspect >= sizeof(q->Buffer) )                       
    {                                                           
        TailInspect = 0;                                               
    }                                                           
                                                                
    q->TailInspect = TailInspect;                                       
    return TRUE;
                                                                
} // SerQueInspect ( )

// ===========================================================================
// Function:    SerQueSyncToInspectionPoint  ( )
// Purpose:     Adjusts tail to inspection tail, thus fixing 
//              the "SerQueInspect ( ) as que remove operations.
// Parameters:  Pointer to UART queue to check for empty.
// Returns:     TRUE if the queue is empty, FALSE otherwise.
// ===========================================================================

VOID SerQueSyncToInspectionPoint ( pSER_UART_QUE q )
{
    q->Tail = q->TailInspect;

} // SerQueSyncToInspectionPoint ( )


// ===========================================================================
// Function:    SerQueEmpty()
// Purpose:     Returns TRUE if the queue is empty, FALSE otherwise.
// Parameters:  Pointer to UART queue to check for empty.
// Returns:     TRUE if the queue is empty, FALSE otherwise.
// ===========================================================================

BOOLEAN SerQueEmpty ( pSER_UART_QUE q )
{
    return ((q->Head) == (q->Tail));

} // SerQueEmpty()



// ===========================================================================
// Function:    SerQueSpaceAvailable()
// Purpose:     Provides a mechanism for seeing how much space is still
//              available in a que.
// Parameters:  Pointer to UART queue to check for space available.
// Returns:     Count of bytes available.
// ===========================================================================

UCHAR SerQueSpaceAvailable ( pSER_UART_QUE q )
{
    int Used, Available;
    
    Used = (int) q->Head - (int) q->Tail;

    if (Used < 0)
        Used += sizeof(q->Buffer);

    Available = (sizeof(q->Buffer)-1) - Used;

    return (UCHAR) Available;

} // SerQueSpaceAvailable()


// ===========================================================================
// Function:    SerQueSpaceUsed()
// Purpose:     Provides a mechanism for seeing how much space has
//              been used up in a que.
// Parameters:  Pointer to UART queue to check for space used.
// Returns:     Count of bytes used.
// ===========================================================================

UCHAR SerQueSpaceUsed ( pSER_UART_QUE q )
{
    UCHAR Available;

    Available = sizeof(q->Buffer) -1 - SerQueSpaceAvailable(q);

    return  Available;

} // SerQueSpaceUsed()




// ===========================================================================
// Function:    SerQueInsert()
// Purpose:     Inserts a character to the desired que in binary.
// Parameters:  Pointer to UART queue to insert character, character to insert.
// Returns:     TRUE if succesful, FALSE otherwise.
// ===========================================================================

BOOLEAN SerQueInsert ( pSER_UART_QUE q, UCHAR c )
{
    if ( SerQueSpaceAvailable ( q ) <= 0 ) 
    {   
        ++(q->Stats.uiErrorQueFull);
        return FALSE;
    }
    
    SerQueInsertMacro ( q, c );

    return TRUE;

} // SerQueInsert()




// ===========================================================================
// Function:    SerQueRemove()
// Purpose:     Removes a character from the desired que. Que remove
//              operations are always in binary.
// Parameters:  Pointer to UART queue to remove character from, pointer to 
//              memory location to place removed character.
// Returns:     TRUE if succesful, FALSE otherwise.
// ===========================================================================

BOOLEAN SerQueRemove ( pSER_UART_QUE q, UCHAR *c )
{
    //
    // Leave if no new data...
    //

    if ( SerQueEmpty ( q ) )
    {
        return FALSE;
    }

    SerQueRemoveMacro ( q, c );

    return TRUE;

} // SerQueRemove()








