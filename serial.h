//**********************************************************************
// Filename:      serial.h
// Rights:        Copyright (c) 2006 The Carlisle and Finch Company
//                All Rights Reserved
// Creation Date: 7/05/06
// Compiler:      Tasking cc51
// Version:       v7.1r1
// Purpose:       This module contains UART constants and function prototypes.
// Modification History:
// Date         Author      Description
// ----         ------      -----------
// 7-05-06      JRN         Created
//**********************************************************************

#ifndef _SERIAL_H_
#define _SERIAL_H_

#include "SlProtocol.h"

//==========================================
// Constants
//==========================================

// #define SER_MAX_PAYLOAD             70
#define SER_NUM_TRACKED_PACKETS     32

#define STR_OFF             "OFF"
#define STR_ON              "ON"
#define STR_OK              "OK"
#define STR_PROMPT          "SLC %d> "
#define STR_QUESTION        "?"
#define STR_NEW_LINE        "\r\n"

// #define RETURN              0x0d    // Return

#define DECIMAL             10
#define HEX                 16

//
// These fields play with "tSerUartQues gSerUartQues []"
//

#define SER_NUMBER_UARTS    4
#define SER_UART_0          0
#define SER_UART_1          1
#define SER_UART_2          2
#define SER_UART_3          3

//
// ASCII code used for data handling.
//

#define STX     0x02    // Start of Text
#define ETX     0x03    // End of Text
#define ACK     0x06    // Acknowledge
#define NAK     0x15    // Negative Ackowledge
#define ASCII_SPACE     0x20    // Space

// ===========================================
// Error Codes
// ===========================================

#define ERROR_SUCCESS               0
#define ERROR_I2C_FAILED            1
#define ERROR_NO_STX                2
#define ERROR_NO_SRC_ADDR           3
#define ERROR_NO_DEST_ADDR          4
#define ERROR_NO_TYPE               5
#define ERROR_NO_LENGTH             6
#define ERROR_NO_DATA               7
#define ERROR_NO_CHECKSUM           8
#define ERROR_INVALID_CHECKSUM      9
#define ERROR_NO_ETX               10
#define ERROR_INVALID_ETX          11
#define ERROR_TILT                 12

typedef struct SER_UART_QUE
{
    UCHAR               ucQueOrSprintfRam   ;  // Must be 1st in list (see tStdprintfSupport in Stdlib.c)
    UINT                pIsr                ;  // Must be 2nd in list (see tStdprintfSupport in Stdlib.c)
    unsigned char       ucUart              ;
    unsigned char       Head                ;          
    unsigned char       Tail                ;
    unsigned char       TailInspect         ;
    UCHAR               LineStatus          ;
    unsigned char       Buffer[90]          ;
    UCHAR               ucStateProtocol     ;
    UINT               *puiTimeout          ;

    struct 
    {    
        UINT                uiErrorQueFull  ;
        UINT                uiInsertCount   ;
        UINT                uiIntsLsr       ;
        UINT                uiIntsModem     ;
        UINT                uiIntsTx        ;
        UINT                uiIntsRx        ;
        UINT                uiIntsUnknown   ;
        UINT                uiNumPacketsOk  ;
        UINT                uiErrorChecksumOrLength;
		UINT                uiErrorStx      ;
		UINT                uiErrorEtx      ;
        UINT                uiAddrBroadcast ;
        UINT                uiAddrMatch     ;
    } Stats;

} SER_UART_QUE;

typedef SER_UART_QUE *pSER_UART_QUE;

//
// Applicable to LineStatus field of type SER_UART_QUE
//

#define LINE_RECEIVE			0
#define LINE_TRANSMIT       	1
#define LINE_TRANSMIT_ENDING 	2

typedef struct
{
    _rom char       *pDesc;
    SER_UART_QUE    *pRxQ;
    SER_UART_QUE    *pTxQ;
    
} tSerUartQues;

extern tSerUartQues gSerUartQues [SER_NUMBER_UARTS]; 

//==========================================
// Macro definitions
//==========================================

// ===========================================================================
// Function:    SerQueInsertMacro ()
// Purpose:     Inserts a character to the desired que in binary
// Assumption:  This macro assumes that there is space in the que for 
//              at least one byte to be inserted.
// Important:   Check to see that there is space available in the que 
//              before invoking this macro.
// Returns:     TRUE if succesful, FALSE otherwise.
// ===========================================================================

#define SerQueInsertMacro(_Que_,_c_)                       \
{                                                          \
    static UCHAR Head;                                     \
                                                           \
    /* Insert byte at Head */                              \
                                                           \
    Head = (_Que_)->Head;                                  \
    (_Que_)->Buffer[Head] = _c_;                           \
                                                           \
    /* Adjust Head pointer, wrap pointer as required. */   \
    Head++;                                                \
                                                           \
    if (  Head >= sizeof(( _Que_ )->Buffer )  )            \
    {                                                      \
        Head = 0;                                          \
    }                                                      \
    (_Que_)->Head = Head;                                  \
    (_Que_)->Stats.uiInsertCount++;                        \
                                                           \
} /* SerQueInsertMacro ( ) */

// ===========================================================================
// Function:    SerQueRemoveMacro()
// Purpose:     Removes a character from the desired que. Que remove
//              operations are always in binary.
// Assumption:  This macro assumes that there is at least one byte in
//              the Que which can be removed.  
// Important:   Check to see that data is in the que before invoking 
//              this macro.
// Returns:     TRUE if succesful, FALSE otherwise.
// ===========================================================================

#define SerQueRemoveMacro(_Que_,_c_)                            \
{                                                               \
    static UCHAR Tail;                                          \
                                                                \
    /* remove from tail. */                                     \
                                                                \
    Tail = (_Que_)->Tail;                                       \
    *(_c_) = (_Que_)->Buffer[Tail];                             \
                                                                \
                                                                \
    /* Adjust tail pointer, wrap pointer as required. */        \
                                                                \
    Tail++;                                                     \
    if (Tail >= sizeof((_Que_)->Buffer) )                       \
    {                                                           \
        Tail = 0;                                               \
    }                                                           \
                                                                \
    (_Que_)->Tail = Tail;                                       \
    (_Que_)->TailInspect = Tail;                                \
                                                                \
} /* SerQueRemoveMacro ( ) */

//==========================================
// Function prototypes
//==========================================

VOID    SerInitializeUARTs          ( VOID );
VOID    SerQueFlush                 ( pSER_UART_QUE q );
BOOLEAN SerQueRemove                ( pSER_UART_QUE q, UCHAR *c );
BOOLEAN SerQueInsert                ( pSER_UART_QUE q, UCHAR  c );
BOOLEAN SerQueEmpty                 ( pSER_UART_QUE q );
UCHAR   SerQueSpaceAvailable        ( pSER_UART_QUE q );
UCHAR   SerQueSpaceUsed             ( pSER_UART_QUE q );
                                    
VOID    SerReceiveSerPort           ( UCHAR *pCmdStr, UINT *pArgs, UINT *pNumArgs );
VOID    SerReceiveEthPort           ( UCHAR *pCmdStr, UINT *pArgs, UINT *pNumArgs );
                                    
VOID    SerRomTransmit              ( SER_UART_QUE *pTxQ, _rom UCHAR *p, UCHAR ucLength);
VOID    SerTransmit                 ( SER_UART_QUE *pQue,      UCHAR *p, UCHAR ucLength);
                                    
VOID    SerStrTransmit              ( SER_UART_QUE *pTxQ, UCHAR *p);
VOID    SerRomStrTransmit           ( SER_UART_QUE *pTxQ, _rom UCHAR *p);
                                    
VOID    SerTransmitPacket           ( SER_UART_QUE *pQue, VOID *pTxPacket);
VOID 	SerCheckLineStatus          ( VOID );
BOOLEAN SerQueInspectEmpty          ( pSER_UART_QUE q ) ;
BOOLEAN SerQueInspect               ( pSER_UART_QUE q, UCHAR *c );
VOID    SerQueSyncToInspectionPoint ( pSER_UART_QUE q ) ;

                                  
//==========================================
// Global variables
//==========================================

extern UCHAR   g_PollLowThreshold;                         // # of polls which place device in "low poll" mode
extern UCHAR   g_PollTable[JCS_MAX_NUMBER+SLC_MAX_NUMBER]; // Poll table, indexed by protocol address


extern UINT    g_TxPacketCount[JCS_MAX_NUMBER+SLC_MAX_NUMBER]; // Total # packets sent to JCS
extern UINT    g_RxPacketCount[JCS_MAX_NUMBER+SLC_MAX_NUMBER]; // Total # packets received from SLC


extern SER_UART_QUE     g_SerRxQueMain   ;     // Uart 0
extern SER_UART_QUE     g_SerTxQueMain   ;     // Uart 0

extern SER_UART_QUE     g_SerRxQuePrb ;     // Uart 1
extern SER_UART_QUE     g_SerTxQuePrb ;     // Uart 1

extern SER_UART_QUE     g_SerRxQueSerPort;
extern SER_UART_QUE     g_SerTxQueSerPort;

extern SER_UART_QUE     g_SerRxQueEthPort;
extern SER_UART_QUE     g_SerTxQueEthPort;
                                  
extern tSER_PACKET 		g_PreviousMessage;

extern UCHAR            g_EnableRs485; 
extern UINT             uiSerTxPacketTypeCount [SER_NUM_TRACKED_PACKETS];
extern UINT             uiSerRxPacketTypeCount [SER_NUM_TRACKED_PACKETS];


//
// Global Preloaded / stored payloads and packets
//
                                    
extern tPAYLOAD_SLC_EXT_STATUS              SerSlcExtStatusPayload      ;   
extern tPAYLOAD_SLC_MISC_SETUP              SerSlcMiscSetupPayload      ;   
extern tPAYLOAD_SLC_DEVICE_NAME_ETC         SerSlcDeviceNameEtcPayload  ;   
extern tPAYLOAD_DO_CALIBRATION              SerDoCalibrationPayload     ;
extern tPAYLOAD_TOUR_POINT                  SerTourPointPayload         ;


extern UCHAR                                SerTourPointDestAddr        ;
extern UCHAR                                SerRequestSlcDevNameEtc     ;
extern UCHAR                                SerRequestSlcMiscSetup      ;

#endif // #ifndef _SERIAL_H_


