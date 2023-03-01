
//**********************************************************************
// Project:       Searchlight Controller Board
// Filename:      utils.c
// Rights:        Copyright (c) 2006 The Carlisle and Finch Company
//                All Rights Reserved
// Creation Date: 9/11/06
// Compiler:      Tasking cc51
// Version:       v7.1r1
// Purpose:       This module contains additional SLC interfaces.
// Modification History:
// Date         Author      Description
// ----         ------      -----------
// 9-11-06		JRN			Created
//**********************************************************************

#include "c8051F040.h"
#include "SLC_Init.h"
#include "SlProtocol.h"
#include "serial.h"
#include "utils.h"
#include "main.h"
#include "stdlib.h"
#include "timers.h"
#include "version.h"
#include "flash.h"
#include "pelco.h"

// -----------------------
// Compile time constants.
// -----------------------
#define POLL_RESPONSE_WAIT_TIME_MS  15
#define PRB_COMM_FAULT_TIME_MS      2000
#define JCS_ACTIVE_TIME             3000

#define  INCR_MULT  100

#define  HARD_STOP  100     // How far from the ends (defined by prbconfig command) 
                            // to not allow more commanded movement
#define  SOFT_STOP  5 * 100 / 127      // The first number is the number of degrees 
                                        // away from the ends to start slowing down (
                                        // if at full speed).

//
// The Main Channel, PRB Channel, and EthPort Channel each have a
// a similar protocol, but independently operationg protocol state.
// This protocol state relates to the "Packet Parsing Technique", as
// described in the CSS theory of operations manual.
//

#define NOT_TTY_CHANNEL     FALSE
#define IS_TTY_CHANNEL      TRUE
typedef struct
{
    SER_UART_QUE        *pRxQue;
    SER_UART_QUE        *pTxQue;

    //
    // For those channels which allow TTY debugging operations,
    // the "command" will be stored (as it is received) in 
    // ucPayload, with the index into that command buffer being
    // ucTtyIndex
    //
    BOOLEAN             bTtyChannel;
    UCHAR               ucTtyIndex;

    //
    // RX packet (on RX channel) data...
    //

    UCHAR               ucSourceAddress;
    UCHAR               ucDestinationAddress;
    UCHAR               ucPacketType;
    UCHAR               ucCalculatedCheckSum;
    UCHAR               ucPayloadLength;
    UCHAR               ucPayloadIdx;
    UCHAR               ucPayload[50];
} tProtocolChannels;

tProtocolChannels sProtocolChannels[] =
{
    { &g_SerRxQueMain, &g_SerTxQueMain , NOT_TTY_CHANNEL },
    { &g_SerRxQuePrb , &g_SerTxQuePrb  , NOT_TTY_CHANNEL },
    { &g_SerRxQueSerPort, &g_SerTxQueSerPort , IS_TTY_CHANNEL  },
    { &g_SerRxQueEthPort, &g_SerTxQueEthPort , IS_TTY_CHANNEL  },
};

//
// Packets received on AUX, but which need to be 
// routed are stored here.
// 

tSER_PACKET g_RoutedRxSerPortPacket;
tSER_PACKET g_RoutedRxEthPortPacket;

// UCHAR ucStateMain;
// UCHAR ucStatePrb;
// UCHAR ucStateEthPort;

// ========================================================
// Clearly labelled PID parameters for different lamp bases
// ========================================================

#define DIP_BIG_BASE    0
#define DIP_MED_BASE    1
#define DIP_SLEEK_BASE  2

tBaseConfig g_BaseConfig[3] = { //     PnPNrm, PnINrm, PnDNrm, TltPNrm, TltINrm, TltDNrm, PnTgt, TltTgt, PnDB, TltDB, PnEndDB, TltEndDB, StRmpCyc, EndRmpCyc, AccDump, MaxPanRateOffset, MaxTiltRateOffset, JcsAccelLimit
                                   {       16,    100,     25,      10,     100,      50,    10,      5,     7,    7,      11,       11,       20,        40,   5000,   1000,   500,    6 },
                                   {        4,     75,    100,	     3,		100,	 100,	 20,	  5,	12,	  12,	   14,	     14,	   20,		  20,	1000,   -150,  -150,    0 },
                                   {	    4,     75,    100,		 3,		100,	 100,	 20,	  5,	25,	  15,	   30,		 20,	   20,		  20,	1000,      0,     0,    0 }
                              };

SER_UART_QUE *UtlQueDebugPrint;     // Operates with ucUtlDebugMask
UCHAR         ucUtlDebugMask;       // Operates with UtlQueDebugPrint
UCHAR         ucUtlDebugDetails;    // Operates with UtlQueDebugPrint

UCHAR g_PolarCoordMode;
UCHAR g_DynTrackingMode;  // mbdtest experimental
UCHAR g_NewTargetDynPosition; // mbdtest experimental

BOOLEAN g_SmartView1; // v1.59
BOOLEAN sendNameOnce; // Used to send the SLC Device Name packet once during startup.
tMotor  g_PanMotor;
tMotor  g_TiltMotor;
tMotor  g_FocusMotor;
tRelays g_Relays;

UCHAR g_MasterProtocolAddress;
UCHAR g_BoardNumber;
UCHAR g_BaseType;

// Polar coordinate mode
UCHAR gUtlHeightAboveSurface;  // mbdtest needed for radar tracking

ULONG g_TargetAbsPosPan;        // Needs to be long for offset on sleek base
UINT  g_TargetAbsPosTilt;
UINT  g_TargetAbsPosFocus;

INT  g_TargetRelPosPan;
INT  g_TargetRelPosTilt;
INT  g_TargetRelPosFocus;

UINT   g_FocusHomeOffset;

USHORT usUtlAutoReturnCountdown;    
           
INT   g_PolarRate;

ULONG  g_PanPolarPos;
ULONG  g_TiltPolarPos;
ULONG  g_FocusPolarPos;

UINT   g_PolarIter;
UINT   g_TargetPolarIter;

INT PanSpeed;
INT g_temp;
INT g_temp2;      
UCHAR g_PanSpeedDebug;
UCHAR g_PanLastSpeedDebug;
UCHAR g_iMotorDebug;
UINT g_PanDebug;
UINT g_PanDebug2;
UINT g_TiltDebug;
UINT g_TiltDebug2;
UINT g_LastPanPos;
UINT g_LastTiltPos;
UINT g_PanPos;
UINT g_TiltPos;
UINT g_RPanLast;
UINT g_RTiltLast;
UINT uiIterRemaining; //MAS!!!!!!!!!!!!!!!!!!!!!!!!

static UINT debugState0;  // MAS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
UINT debugState1;  // MAS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
UINT debugState2;  // MAS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
UINT debugState3;  // MAS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
UINT debugState4;  // MAS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
UINT debugState5;  // MAS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

// Added by ACM!! per MAS!! in v1.68 to allow SerPort or EthPort interfaces to inhibit SLC BASIC STATUS response
UCHAR ucJcsInhibitSlcResponse;

UCHAR g_MotorAccl = 5;
UCHAR g_ucMotorTimedOut = 0;

UCHAR g_ucJcsTiltCommandSpeed = 128;
UCHAR g_ucJcsPanCommandSpeed = 128;

UCHAR g_PrbSlaveSpeakPayload;

UINT  uiPollResponseWaitTimeMs = POLL_RESPONSE_WAIT_TIME_MS;

tPotReader g_PotReader;

//
// The poll table is a list of protocol addresses.  Devices
// which are online and functioning will have a zero in the
// corresponding entry in g_PollTable[], as indexed by g_PollTableIdx.
// Each time a SLAVE_SPEAK packet is not responded to by a 
// device, that device's entry in g_PollTable[] is incremented.
// Values in g_PollTable greater than 10 put the corresponding device
// into "low poll" mode.
//

UCHAR      g_PollLowThreshold = 10                          ; // # of polls which place device in "low poll" mode
UCHAR      g_PollTableIdx                                   ; // Index into the poll table
UCHAR      g_PollTableLowPollIdx                            ; // Last device "low polled"
UCHAR      g_PollTable    [JCS_MAX_NUMBER+SLC_MAX_NUMBER]   ; // Poll table, indexed by protocol address
UINT       g_TxPacketCount[JCS_MAX_NUMBER+SLC_MAX_NUMBER]   ; // Total # packets sent to JCS
UINT       g_RxPacketCount[SLC_MAX_NUMBER+SLC_MAX_NUMBER]   ; // Total # packets received from SLC

#define MAX_ARGS 5

UCHAR ucNumArgs;
UINT  uiArgs     [MAX_ARGS];
UCHAR *pucArgStr [MAX_ARGS];

UCHAR ucUtlJcsStatByte3;
UCHAR ucUtlJcsStatDestAddress;

//
// Counts upward when lamp is ON
//

USHORT      usUtlLampHours  ;
UCHAR       ucUtlLampMinutes;
UCHAR       ucUtlLampSeconds;

//
// Handles Tours (etc).
//

// UCHAR               ucUtlActivePointNumber ;
UCHAR               ucUtlActiveTourNext    ;
tTourPt             UtlActivePoint         ;

UINT g_ActivePresetPanPos;
UINT g_ActivePresetTiltPos;

// 
// Global Forward prototypes
//

UINT UtilAdc0GetChannel  ( UCHAR channel, UCHAR gain );
UINT UtilGetPanDegrees   ( VOID ); 
UINT UtilGetTiltDegrees  ( VOID );
INT  UtilGetPanRate      ( VOID );
INT  UtilGetTiltRate     ( VOID );
UINT UtilSqrt ( LONG );
UCHAR UtilMotorDeadband ( UCHAR Speed, UCHAR Deadband );
VOID UtilPolarStop      ( VOID );
static VOID UtilPolarUpdateMotor ( tMotor *pMotor, ULONG uiFeedback, ULONG uiTargetPosition, INT CalcSpeed, UCHAR FinalPos, UCHAR Dir, UCHAR Ramp );
_reentrant VOID PelcoProcessCommand ( tProtocolChannels *pChannel );	// mbd
static VOID UtilDynTrackingUpdateMotor ( tMotor *pMotor, ULONG ulFeedback, ULONG ulTargetPosition );  // mbdtest experimental

//
// Module local ("STATIC") prototyes
//

static _reentrant VOID lTourCommand                 ( SER_UART_QUE * );
static _reentrant VOID lNameCommand                 ( SER_UART_QUE * );
static _reentrant VOID lHelpCommand                 ( SER_UART_QUE * );
static _reentrant VOID lCalibratePotCommand         ( SER_UART_QUE * );
static _reentrant VOID lMotorStopCommand            ( SER_UART_QUE * );
static _reentrant VOID lPrbPrintCommand             ( SER_UART_QUE * );
static _reentrant VOID lPrbConfigCommand            ( SER_UART_QUE * );
static _reentrant VOID lStatusCommand               ( SER_UART_QUE * );
static _reentrant VOID lPidCommand                  ( SER_UART_QUE * );
static _reentrant VOID lCommCommand                 ( SER_UART_QUE * );
static _reentrant VOID lVersionCommand              ( SER_UART_QUE * );
static _reentrant VOID lBoardIdCommand              ( SER_UART_QUE * );
static _reentrant VOID lSetupCommand                ( SER_UART_QUE * );
static _reentrant VOID lMotorCommand                ( SER_UART_QUE * );
static _reentrant VOID lRelayCommand                ( SER_UART_QUE * );
static _reentrant VOID lPollTableCommand            ( SER_UART_QUE * );
static _reentrant VOID lPolarCommand                ( SER_UART_QUE * );
static _reentrant VOID lPacketsCommand              ( SER_UART_QUE * );
static _reentrant VOID lSetHomeCommand              ( SER_UART_QUE * );
static _reentrant VOID lBaseTypeCommand             ( SER_UART_QUE * );
static _reentrant VOID lDebugMaskCommand            ( SER_UART_QUE * );
static _reentrant VOID lUtilParseCommandStrIntoArgs ( UCHAR * );
static _reentrant VOID UtlThisSlcSpeaksOnMain       ( VOID    );
static _reentrant VOID UtilSendSlaveSpeakOnMain     ( UCHAR ucProtocolAddress );


//
// ------------------------------------------------------
// Module "local" constants (in ROM)...
// ------------------------------------------------------
//
typedef struct 
{
    _rom UCHAR *Name;
    _rom UINT   pFunction;
    _rom UCHAR *Help;
} tCommand;

_rom tCommand Command[] =
{
    {   (_rom UCHAR *) "?"              , (UINT) &lHelpCommand         , (_rom UCHAR *) ""                                             },
    {   (_rom UCHAR *) "calpot"         , (UINT) &lCalibratePotCommand , (_rom UCHAR *) "[1/0]"                                        },
    {   (_rom UCHAR *) "ms"             , (UINT) &lMotorStopCommand    , (_rom UCHAR *) ""                                             },
    {   (_rom UCHAR *) "c"              , (UINT) &lCommCommand         , (_rom UCHAR *) ""                                             },
    {   (_rom UCHAR *) "help"           , (UINT) &lHelpCommand         , (_rom UCHAR *) ""                                             },
    {   (_rom UCHAR *) "id"             , (UINT) &lBoardIdCommand      , (_rom UCHAR *) ""                                             },
    {   (_rom UCHAR *) "m"              , (UINT) &lMotorCommand        , (_rom UCHAR *) "m [state]"                                    },
    {   (_rom UCHAR *) "tour"           , (UINT) &lTourCommand         , (_rom UCHAR *) "tour number"                                  },
    {   (_rom UCHAR *) "name"           , (UINT) &lNameCommand         , (_rom UCHAR *) "name [main|SerPort|EthPort] [name]"                 },
    {   (_rom UCHAR *) "setup"          , (UINT) &lSetupCommand        , (_rom UCHAR *) ""                                             },
    {   (_rom UCHAR *) "pid"            , (UINT) &lPidCommand          , (_rom UCHAR *) "pgain igain dgain"                            },
    {   (_rom UCHAR *) "prbconfig"      , (UINT) &lPrbConfigCommand    , (_rom UCHAR *) "panrange tiltrange focusrange adcavg panpots" },
    {   (_rom UCHAR *) "pa"             , (UINT) &lPacketsCommand      , (_rom UCHAR *) "REM: Shows packets by type"                   },
    {   (_rom UCHAR *) "pp"             , (UINT) &lPrbPrintCommand     , (_rom UCHAR *) ""                                             },
    {   (_rom UCHAR *) "p"              , (UINT) &lPollTableCommand    , (_rom UCHAR *) "[low poll threshold]"                         },
    {   (_rom UCHAR *) "polar"          , (UINT) &lPolarCommand        , (_rom UCHAR *) "panpos tiltpos rate"                          },
    {   (_rom UCHAR *) "relay"          , (UINT) &lRelayCommand        , (_rom UCHAR *) "id [state]"                                   },
    {   (_rom UCHAR *) "status"         , (UINT) &lStatusCommand       , (_rom UCHAR *) ""                                             },
    {   (_rom UCHAR *) "version"        , (UINT) &lVersionCommand      , (_rom UCHAR *) ""                                             },
    {   (_rom UCHAR *) "sethome"        , (UINT) &lSetHomeCommand      , (_rom UCHAR *) "panoffset tiltoffset"                         },
    {   (_rom UCHAR *) "basetype"       , (UINT) &lBaseTypeCommand     , (_rom UCHAR *) ""                                             },
    {   (_rom UCHAR *) "debug"          , (UINT) &lDebugMaskCommand    , (_rom UCHAR *) "debugmask debugdetails"                       },
    {   NULL  , NULL                                                   , (_rom UCHAR *) ""                                             },
};

//=====================================================================
// Function   : lNameCommand ()                                           
// Purpose    : This function sets or retrieves the name of this SLC 
//              device.(up to 8 characters in length)
// Parameters : 
// Returns    : Nothing is returned.
// Important  : Observe the heavy stack footprint!
//=====================================================================
static _reentrant VOID lNameCommand ( SER_UART_QUE *pQue  )
{
    tFlashMiscPage   FlashMisc;     // Heavy stack footprint!

    FlashRead(&FlashMisc, FLASH_MISC_PTR, sizeof (FlashMisc));
    
    if ( pucArgStr[1] )
    {
        if (    ( 0 == StdRomStrComp( pucArgStr[1], (_rom UCHAR *) "main" )   ) &&
                (StdStrLen(pucArgStr[2]) < sizeof(FlashMisc.szMainDeviceName) ) )
        {
            StdStrCopy ( FlashMisc.szMainDeviceName, pucArgStr[2]);
        }
        if (    ( 0 == StdRomStrComp( pucArgStr[1], (_rom UCHAR *) "SerPort")    ) &&
                (StdStrLen(pucArgStr[2]) < sizeof(FlashMisc.szSerPortDeviceName) ) )
        {
            StdStrCopy ( FlashMisc.szSerPortDeviceName, pucArgStr[2]);
        }
        if (    ( 0 == StdRomStrComp( pucArgStr[1], (_rom UCHAR *) "EthPort")    ) &&
                (StdStrLen(pucArgStr[2]) < sizeof(FlashMisc.szEthPortDeviceName) ) )
        {
            StdStrCopy ( FlashMisc.szEthPortDeviceName, pucArgStr[2]);
        }
        
        FlashErasePage ( FLASH_MISC_PAGE );
        FlashWrite     ( FLASH_MISC_PTR, &FlashMisc, sizeof(FlashMisc));
        
        StdStrCopy(SerSlcDeviceNameEtcPayload.szMainDeviceName, FlashMisc.szMainDeviceName);
        StdStrCopy(SerSlcDeviceNameEtcPayload.szSerPortDeviceName, FlashMisc.szSerPortDeviceName);
        StdStrCopy(SerSlcDeviceNameEtcPayload.szEthPortDeviceName, FlashMisc.szEthPortDeviceName);
    }
    
    StdPrintf (pQue, STR_NEW_LINE "name main = '%s'", FlashMisc.szMainDeviceName );
    StdPrintf (pQue, STR_NEW_LINE "name SerPort = '%s'", FlashMisc.szSerPortDeviceName );
    StdPrintf (pQue, STR_NEW_LINE "name EthPort = '%s'", FlashMisc.szEthPortDeviceName );
    
    
    
} // lNameCommand ( VOID )



//=====================================================================
// Function   : lTourCommand ()                                           
// Purpose    : This function prints tour data for a specified tour. 
// Parameters : 
// Returns    : Nothing is returned.
//=====================================================================
static _reentrant VOID lTourCommand ( SER_UART_QUE *pQue  )
{
    UCHAR            ucTour;
    tTourPt          Point;
    UCHAR            ucPointNumber;
    UCHAR            ucNextTourNumber;
    
    //
    // validate tour number
    //
    
    ucTour = uiArgs[1];
    if (    ( ucTour >  FLASH_TOTAL_NUM_TOURS) ||
            ( ucTour <= 0                    ) )
         
    {
        StdPrintf (pQue, STR_NEW_LINE  "?");
        return;
    }
    
    StdPrintf (pQue, STR_NEW_LINE  "Tour %d", ucTour);
    StdPrintf (pQue, STR_NEW_LINE  "#  Az    Elev  Rate  Options");
    StdPrintf (pQue, STR_NEW_LINE  "== ===== ===== ===== =======");
    for (ucPointNumber = 0; ucPointNumber < TOUR_MAX_NUMBER_OF_POINTS;++ucPointNumber)
    {
        StdPrintf (pQue, STR_NEW_LINE  "%02d", ucPointNumber+1);
        
        FlashRetrieveTourPoint  ( ucTour-1, ucPointNumber, &Point, &ucNextTourNumber);
        
        if (TOUR_POINT_NOT_SET(Point))
        {
            StdPrintf (pQue, "   NOT SET");
        } else
        {
            StdPrintf (pQue, "%6d", Point.sAzimuth                );
            StdPrintf (pQue, "%6d", Point.sElevation              );
            StdPrintf (pQue, "%6d", Point.usRate & TOUR_RATE_MASK );
        
            if (Point.ucOptions & TOUR_OPT_TO_PT_AUX1_ON     ) StdPrintf (pQue, " TO_AUX1"     );
            if (Point.ucOptions & TOUR_OPT_TO_PT_AUX2_ON     ) StdPrintf (pQue, " TO_AUX2"     );
            if (Point.ucOptions & TOUR_OPT_TO_PT_BEAM_ON    ) StdPrintf (pQue, " TO_BEAM"    );
            if (Point.ucOptions & TOUR_OPT_TO_PT_AUX3_ON    ) StdPrintf (pQue, " TO_AUX3"    );
            if (Point.ucOptions & TOUR_OPT_DWELL_PT_AUX1_ON  ) StdPrintf (pQue, " DW_AUX1"  );
            if (Point.ucOptions & TOUR_OPT_DWELL_PT_AUX2_ON  ) StdPrintf (pQue, " DW_AUX2"  );
            if (Point.ucOptions & TOUR_OPT_DWELL_PT_BEAM_ON ) StdPrintf (pQue, " DW_BEAM" );
            if (Point.ucOptions & TOUR_OPT_DWELL_PT_AUX3_ON ) StdPrintf (pQue, " DW_AUX3" ); 
        }
    }
    if (TOUR_INVALID == ucNextTourNumber)
        StdPrintf (pQue, STR_NEW_LINE  "No Next Tour");
    else 
        StdPrintf (pQue, STR_NEW_LINE  "Next Tour %d", ucNextTourNumber+1);
    
    
} // lTourCommand ( VOID )

//=====================================================================
// Function   : lBaseTypeCommand ()                                           
// Purpose    : This function returns the numerical base type.
// Parameters : Nothing is passed.
// Returns    : Nothing is returned.
//=====================================================================
static _reentrant VOID lBaseTypeCommand ( SER_UART_QUE *pQue )
{
    
    StdPrintf (pQue, STR_NEW_LINE "Base Type: %u", g_BaseType );
    
} // lBaseTypeCommand ( VOID )
    
//=====================================================================
// Function   : lMotorStopCommand ()                                           
// Purpose    : This function stops the motors.
// Parameters : Nothing is passed.
// Returns    : Nothing is returned.
//=====================================================================
static _reentrant VOID lMotorStopCommand ( SER_UART_QUE *pQue )
{
    UtilSetMotorSpeed ( PAN_MOTOR,      128 );
    UtilSetMotorSpeed ( TILT_MOTOR,     128 );
    UtilSetMotorSpeed ( FOCUS_MOTOR,    128 );
    
    StdPrintf (pQue, STR_NEW_LINE STR_OK);
    
} // lStdoutCommand ( ) 

//=====================================================================
// Function   : lHelpCommand ()                                           
// Purpose    : This function handles the "help" command.
// Parameters : Nothing is passed.
// Returns    : Nothing is returned.
//=====================================================================
static _reentrant VOID lHelpCommand ( SER_UART_QUE *pQue   )
{
    _rom tCommand  *pComm;
    
    pComm = Command;
    
    StdPrintf (pQue, STR_NEW_LINE "Command    Syntax");
    StdPrintf (pQue, STR_NEW_LINE "=======    ======");
    while (pComm->Name)
    {
        StdPrintf (pQue, STR_NEW_LINE "%8S %S", 
                    pComm->Name,
                    pComm->Help);
        ++pComm;
    }

} // lHelpCommand ( ) 

//=====================================================================
// Function   : lCalibratePotCommand ()                                           
// Purpose    : This function tells the PRB to begin/continue calibration.
// Parameters : 0 or 1 is passed (exit or enter cal mode).
// Returns    : Nothing is returned.
//=====================================================================
static _reentrant VOID lCalibratePotCommand ( SER_UART_QUE *pQue  )
{

    if ( ucNumArgs == 2 )
    {
        if ( uiArgs[1] )
        {
            StdPrintf (pQue, STR_NEW_LINE STR_ON);
            g_PrbSlaveSpeakPayload |= IN_CAL_MODE_BIT;       // Turn on calibration mode flag bit in PRB Slave Speak packets
        }
        else
        {
            StdPrintf (pQue, STR_NEW_LINE STR_OFF);
            g_PrbSlaveSpeakPayload &= ~IN_CAL_MODE_BIT;
        }
        
    } // if ( ucNumArgs == 2 )
    
} // lCalibratePotCommand ( VOID )

//=====================================================================
// Function   : lWriteHomeToMiscPage ()                                           
// Purpose    : This function stores the home offsets to flash misc page.
// Parameters : The pan and tilt offsets are passed through globals.
// Returns    : Nothing is returned.
// Important  : Heavy stack footprint!
//=====================================================================
static _reentrant VOID lWriteHomeToMiscPage ( void  )
{
    tFlashMiscPage  FlashMisc;
    
    //
    // Read / modify / write "misc" flash page.
    //
    
    FlashRead      (&FlashMisc, FLASH_MISC_PTR, sizeof (FlashMisc));
    
    FlashMisc.SlcMiscSetupPayload.usPanHomeOffset  = SerSlcMiscSetupPayload.usPanHomeOffset;
    FlashMisc.SlcMiscSetupPayload.usTiltHomeOffset = SerSlcMiscSetupPayload.usTiltHomeOffset ;
   
    FlashErasePage ( FLASH_MISC_PAGE );
    FlashWrite     ( FLASH_MISC_PTR, &FlashMisc, sizeof(FlashMisc));
    
} // lWriteHomeToMiscPage ( )
    																				  
//=====================================================================
// Function   : lSetHomeCommand ()                                           
// Purpose    : This function sets the offsets for relative / HOME positioning.
// Parameters : The pan and tilt offsets are passed in hundredths of degrees.
// Returns    : Nothing is returned.
// Important  : Heavy stack footprint in call to lWriteHomeToMiscPage ( )
//=====================================================================
static _reentrant VOID lSetHomeCommand ( SER_UART_QUE *pQue  )
{
    //
    // Set home position as indicated by user
    //
    
    if ( ( ucNumArgs == 2 ) && ( 0 == StdRomStrComp( pucArgStr[1], (_rom UCHAR *) "current")) )
    {
        SerSlcMiscSetupPayload.usPanHomeOffset  = g_PotReader.PanFb;
        SerSlcMiscSetupPayload.usTiltHomeOffset = g_PotReader.TiltFb;
        lWriteHomeToMiscPage ( );
    } 
    else if ( ucNumArgs == 3 )
    {
        SerSlcMiscSetupPayload.usPanHomeOffset  = uiArgs[1];
        SerSlcMiscSetupPayload.usTiltHomeOffset = uiArgs[2];
        lWriteHomeToMiscPage ( );
    }
    
    StdPrintf (pQue, STR_NEW_LINE "Pan  Home = %d", SerSlcMiscSetupPayload.usPanHomeOffset  );
    StdPrintf (pQue, STR_NEW_LINE "Tilt Home = %d", SerSlcMiscSetupPayload.usTiltHomeOffset );
    
} // lSetHomeCommand ( VOID )
    
//=====================================================================
// Function   : lDebugMaskCommand ()                                           
// Purpose    : Sets / clears Debug Mask.
// Returns    : Nothing is returned.
//=====================================================================
static _reentrant VOID lDebugMaskCommand ( SER_UART_QUE *pQue )
{
    ucUtlDebugMask    = (UCHAR) uiArgs[1];
    ucUtlDebugDetails = (UCHAR) uiArgs[2];
    if (ucUtlDebugMask)
    {
        UtlQueDebugPrint = pQue;
    } 

    StdPrintf (pQue, STR_NEW_LINE "Mask: %d, Details: %d", ucUtlDebugMask, ucUtlDebugDetails);
    
} // lDebugMaskCommand ( VOID )

//=====================================================================
// Function   : lPrbPrintCommand ()                                           
// Purpose    : Sets / clears PRB print bit in debug mask.
// Returns    : Nothing is returned.
//=====================================================================
static _reentrant VOID lPrbPrintCommand ( SER_UART_QUE *pQue )
{
    if ( ucUtlDebugMask & DEBUG_PRB_PRINT )
    {
        ucUtlDebugMask &= ~DEBUG_PRB_PRINT;
        StdPrintf (pQue, STR_NEW_LINE "Prb Print OFF");
    }
    else
    {
        ucUtlDebugMask |= DEBUG_PRB_PRINT;
        UtlQueDebugPrint = pQue;
        StdPrintf (pQue, STR_NEW_LINE "Prb Print ON");
    }

} // lPrbPrintCommand ( VOID )

//=====================================================================
// Function   : lPrbConfigCommand ()                                           
// Purpose    : This function sends config data to the PRB.
// Parameters : Nothing is passed.
// Returns    : Nothing is returned.
//=====================================================================
static _reentrant VOID lPrbConfigCommand ( SER_UART_QUE *pQue  )
{
    tPAYLOAD_PRB_CONFIG *TxConfigPayload;
    tSER_PACKET TxPacket;

    StdMemClr(&TxPacket, sizeof(TxPacket));
    
    TxConfigPayload = (tPAYLOAD_PRB_CONFIG *) TxPacket.pPayload;

    //
    // PRBCONFIG
    // Command string : "prbconfig [panrange] [tiltrange] [focusrange] [adcavgs] [panpots]"
    // Arguments      : panrange    - Range of pan axis in hundredths of a degree (36,000 = 360deg).
    //                  tiltrange   - Range of tilt axis in hundredths of a degree.
    //                  focusrange  - Range of focus axis in hundredths of a degree.
    //                  adcavgs     - Number of ADC samples to average.
    // Returns        : Nothing is returned. Specified motor is put into desired run state. 
    //
    
    //
    // Up to 5 arguments (not including command), variable arguments
    //
    
    if ( ucNumArgs >= 3 )
    {
        TxConfigPayload->PanRange    = uiArgs[1];
        TxConfigPayload->TiltRange   = uiArgs[2];
    }

    if ( ucNumArgs >= 4 )
    {
        TxConfigPayload->FocusRange  = uiArgs[3];
    }
    
    if ( ucNumArgs >= 5 )
    {
        TxConfigPayload->AdcAvgs     = uiArgs[4];
    }

    TxConfigPayload->SLBaseType      = g_BaseType;
    
    TxPacket.Preamble.SrcAddr  = g_BoardNumber + SLC_PROTOCOL_OFFSET;
    TxPacket.Preamble.DestAddr = g_BoardNumber + PRB_PROTOCOL_OFFSET;
    TxPacket.Preamble.PacketType = PKT_TYPE_PRB_CONFIG;
    TxPacket.Preamble.PayloadLength = 14;

    SerTransmitPacket ( &g_SerTxQuePrb , &TxPacket);
    
    StdPrintf (pQue, STR_NEW_LINE STR_OK);
    
} // lPrbConfigCommand ( VOID )

//=====================================================================
// Function   : lPolarCommand ()                                           
// Purpose    : This function commands the searchlight to go to a 
//            : specified polar location at a specified rate.
// Parameters : Nothing is passed.
// Returns    : Nothing is returned.
//=====================================================================
static _reentrant VOID lPolarCommand ( SER_UART_QUE *pQue  )
{
    LONG    lPositionTemp;

    //
    // POLAR
    // Command string : "polar [panpos] [tiltpos] [rate]"
    // Arguments      : panpos      - Commanded pan axis position in hundredths of a degree (36,000 = 360deg).
    //                  tiltpos     - Commanded tilt axis position in hundredths of a degree.
    //                  rate        - Rate of motion .
    // Returns        : Nothing is returned. Specified motor is put into desired run state. 
    //
    
    if ( ucNumArgs == 3 )
    {
        if ( g_BaseType == 2 )
        {
            lPositionTemp          = (INT) uiArgs[1] + (LONG) SerSlcMiscSetupPayload.usPanHomeOffset;
            
            if ( ( (INT) uiArgs[1] > 18000 ) || ( (INT) uiArgs[1] < -18000 ) )
            {
                StdPrintf (pQue, STR_NEW_LINE "OUT OF RANGE: +/-18000");
                return;
            }

            g_TargetAbsPosPan = lPositionTemp + 36000;
                                
        } // if ( g_BaseType == 2 )
        else
        {
            g_TargetAbsPosPan       = uiArgs[1] + SerSlcMiscSetupPayload.usPanHomeOffset;
        }
        g_TargetAbsPosTilt      = uiArgs[2] + SerSlcMiscSetupPayload.usTiltHomeOffset;
        g_PolarRate             = 1000;
        g_PolarCoordMode        = TRUE;
        StdPrintf (pQue, STR_NEW_LINE STR_ON);
    }

    if ( ucNumArgs == 4 )
    {
        if ( g_BaseType == 2 )
        {
            lPositionTemp          = (INT) uiArgs[1] + (LONG) SerSlcMiscSetupPayload.usPanHomeOffset;
            
            if ( (INT) uiArgs[1] > 18000 || (INT) uiArgs[1] < -18000 )
            {
                StdPrintf (pQue, STR_NEW_LINE "OUT OF RANGE: +/-18000");
                return;
            }

            g_TargetAbsPosPan = lPositionTemp + 36000;
                                
        } // if ( g_BaseType == 2 )
        else
            g_TargetAbsPosPan       = uiArgs[1] + SerSlcMiscSetupPayload.usPanHomeOffset;
        
        g_TargetAbsPosTilt      = uiArgs[2] + SerSlcMiscSetupPayload.usTiltHomeOffset;
        g_PolarRate             = uiArgs[3];
        g_PolarCoordMode        = TRUE;
        StdPrintf (pQue, STR_NEW_LINE STR_ON);
    }

    if ( ucNumArgs == 1 )
    {
        UtilPolarStop ( );
        StdPrintf (pQue, STR_NEW_LINE STR_OFF);
    }
    
} // lPolarCommand ( VOID )

//=====================================================================
// Function   : lPidCommand ()                                           
// Purpose    : This function updates the feedback PID coefficients. 
// Parameters : Nothing is passed.
// Returns    : Nothing is returned.
//=====================================================================
static _reentrant VOID lPidCommand ( SER_UART_QUE *pQue  )
{
    //
    // PID
    // Command string : "pid [pgain] [igain] [dgain]"
    // Arguments      : pgain       - P term gain (smaller number is higher gain).
    //                  igain       - I term gain (smaller number is higher gain).
    //                  dgain       - D term gain (smaller number is higher gain).
    // Returns        : Nothing is returned.
    //
    
    if ( ucNumArgs == 4 )
    {
        g_PanMotor.Gain.P = uiArgs[1];
        g_PanMotor.Gain.I = uiArgs[2];
        g_PanMotor.Gain.D = uiArgs[3];
        g_TiltMotor.Gain.P = uiArgs[1];
        g_TiltMotor.Gain.I = uiArgs[2];
        g_TiltMotor.Gain.D = uiArgs[3];
        g_FocusMotor.Gain.P = uiArgs[1];
        g_FocusMotor.Gain.I = uiArgs[2];
        g_FocusMotor.Gain.D = uiArgs[3]; 
        StdPrintf (pQue, STR_NEW_LINE STR_OK);
    }
    if ( ucNumArgs == 5 )
    {
        g_PanMotor.Gain.P = uiArgs[1];
        g_PanMotor.Gain.I = uiArgs[2];
        g_TiltMotor.Gain.P = uiArgs[3];
        g_TiltMotor.Gain.I = uiArgs[4];
        StdPrintf (pQue, STR_NEW_LINE STR_OK);
    }
    if ( ucNumArgs == 2 )
    {
        g_PanMotor.Gain.I = uiArgs[1];
        StdPrintf (pQue, STR_NEW_LINE STR_OK);
    }

} // lPolarCommand ( VOID )

//
//=====================================================================
// Function   : lStatusCommand ()                                           
// Purpose    : This function handles the command.
// Parameters : Nothing is passed.
// Returns    : Nothing is returned.
//=====================================================================
static _reentrant VOID lStatusCommand ( SER_UART_QUE *pQue  )
{
    StdPrintf (pQue, STR_NEW_LINE "uiMainStatus: %u"  , MainStats.uiMainStatus);
    StdPrintf (pQue, STR_NEW_LINE "uiMainCycles: %u"  , MainStats.uiMainCycles);
    StdPrintf (pQue, STR_NEW_LINE "uiUart0Isr  : %u"  , MainStats.uiUart0Isr  );
    StdPrintf (pQue, STR_NEW_LINE "uiUart1Isr  : %u"  , MainStats.uiUart1Isr  );
    StdPrintf (pQue, STR_NEW_LINE "uiI2cIsr    : %u"  , MainStats.uiI2cIsr    );
    StdPrintf (pQue, STR_NEW_LINE "PRB Blanking: %u"  , g_TimeMs.PrbBlanked   );
    
    StdMemClr(&MainStats, sizeof(MainStats));
    
} // lStatusCommand ( )


//
//=====================================================================
// Function   : lPollTableCommand ()                                           
// Purpose    : This function handles the command.
// Parameters : Nothing is passed.
// Returns    : Nothing is returned.
//=====================================================================
static _reentrant VOID lPollTableCommand ( SER_UART_QUE *pQue  )
{
    UCHAR i;

    if (ucNumArgs > 1)
    {
        g_PollLowThreshold = uiArgs[1];
    }
    
    if (ucNumArgs > 2)
    {
        uiPollResponseWaitTimeMs = uiArgs[2];
    }

    // StdPrintf (pQue, STR_NEW_LINE "Poll Idx: %d"          , g_PollTableIdx          );
    // StdPrintf (pQue, STR_NEW_LINE "Low Poll Idx: %d"      , g_PollTableLowPollIdx   );
    // StdPrintf (pQue, STR_NEW_LINE "Master Device     : %d", g_MasterProtocolAddress+1 );
    StdPrintf (pQue, STR_NEW_LINE "Low Poll Threshold: %d", g_PollLowThreshold    );
    StdPrintf (pQue, STR_NEW_LINE "Poll Response Wait: %d ms", uiPollResponseWaitTimeMs  );
    
    StdPrintf (pQue, STR_NEW_LINE "+-----+----------------------------+-----------------------------+");
    StdPrintf (pQue, STR_NEW_LINE "|     | Joystick                   | Searchlight                 |");
    StdPrintf (pQue, STR_NEW_LINE "|Addr | Status            Tx    Rx | Status           Tx    Rx   |");
    StdPrintf (pQue, STR_NEW_LINE "+-----+----------------------------+-----------------------------+");
    
    for (i = 0; i < JCS_MAX_NUMBER; i++)
    {
        UCHAR ucSlcIdx;
        
        char  szJcsMiss[20];
        char  szSlcMiss[20];

        //
        // "i" is used to index out the JCS
        // values, and this index is used to        
        // is used to index out the SLC values.
        //
        
        ucSlcIdx = i+JCS_MAX_NUMBER;
        
        //
        // Prepare JCS Status
        //
        
        if (g_BoardNumber != g_MasterProtocolAddress)
            StdSprintf(szJcsMiss, "--------     ");
        else if ( (0 == g_PollTable[i]) || (1 == g_PollTable[i]) )
            StdSprintf(szJcsMiss, "ON  LINE     ", g_PollTable[i]);
        else if (g_PollLowThreshold == g_PollTable[i])
            StdSprintf(szJcsMiss, "off line     ");                       
        else
            StdSprintf(szJcsMiss, "Marginal (%2u)", g_PollTable[i]);
        
        //
        // Prepare SLC Status
        //
        
        if (i == g_MasterProtocolAddress)
            StdSprintf(szSlcMiss, "MASTER->     ");
        else if (g_BoardNumber != g_MasterProtocolAddress)
            StdSprintf(szSlcMiss, "--------     ");
        else if ( (0 == g_PollTable[ucSlcIdx]) || (1 == g_PollTable[ucSlcIdx]) )
            StdSprintf(szSlcMiss, "ON  LINE     ", g_PollTable[ucSlcIdx]);
        else if (g_PollLowThreshold == g_PollTable[ucSlcIdx])
            StdSprintf(szSlcMiss, "off line     ");                           
        else
            StdSprintf(szSlcMiss, "MARGINAL (%2u)", g_PollTable[ucSlcIdx]);
        
        
     // StdPrintf (pQue, STR_NEW_LINE " %4u | %s %6u%6u | %s%6u%6u    ", 
        StdPrintf (pQue, STR_NEW_LINE " %02u     %s %6u%6u   %s%6u%6u", 
            i+1,
            szJcsMiss, g_TxPacketCount [i       ], g_RxPacketCount[i       ],
            szSlcMiss, g_TxPacketCount [ucSlcIdx], g_RxPacketCount[ucSlcIdx]);
            
        //
        // Zeroize stats after printing them.
        //
        
        g_TxPacketCount[i       ] = g_RxPacketCount[i       ] =
        g_TxPacketCount[ucSlcIdx] = g_RxPacketCount[ucSlcIdx] = 0;
    }
    
} // lPollTableCommand ( )

//
//=====================================================================
// Function   : lPacketsCommand ()                                           
// Purpose    : This function handles the command.
// Parameters : Nothing is passed.
// Returns    : Nothing is returned.
//=====================================================================
static _reentrant VOID lPacketsCommand ( SER_UART_QUE *pQue  )
{
    UCHAR i;
    
    StdPrintf (pQue, STR_NEW_LINE "Type    Tx    Rx     Type    Tx    Rx");
    StdPrintf (pQue, STR_NEW_LINE "====  ====  ====     ====  ====  ====");
    
    for (i = 0; i < (SER_NUM_TRACKED_PACKETS/2); i++)
    {
        StdPrintf (pQue, STR_NEW_LINE "  %02x %5u %5u     ", i,
                                                               uiSerTxPacketTypeCount[i                          ], uiSerRxPacketTypeCount[i                          ]);
        StdPrintf (pQue,              "  %02x %5u %5u     ", i+SER_NUM_TRACKED_PACKETS/2, 
                                                               uiSerTxPacketTypeCount[i+SER_NUM_TRACKED_PACKETS/2], uiSerRxPacketTypeCount[i+SER_NUM_TRACKED_PACKETS/2]);
    }
        
    //
    // Zeroize stats after printing them.
    //
    
    StdMemClr(uiSerTxPacketTypeCount, sizeof(uiSerTxPacketTypeCount));
    StdMemClr(uiSerRxPacketTypeCount, sizeof(uiSerRxPacketTypeCount));
        
} // lPacketsCommand (   )

//=====================================================================
// Function   : lRelayCommand ()                                           
// Purpose    : Specified relay is put into desired state. 
// Parameters : Nothing is passed.
// Returns    : Nothing is returned.
// Command string : "relay [id] [state]"
// Arguments      : id    - Relay to modify ( 1...7 ).
//                  state - State to put relay into ( on / off ).
//=====================================================================
static _reentrant VOID lRelayCommand ( SER_UART_QUE *pQue  )
{
    //UCHAR *pRelay;

    if ( ( ucNumArgs < 3 ) || (uiArgs[1] > sizeof(g_Relays)) )
    {
        StdPrintf (pQue, STR_NEW_LINE STR_QUESTION);
        return;
    }

    //
    // Relay ON?
    //
    
    if ( uiArgs[2] )
        UtlRelayOn  ( uiArgs[1]);
    else
        UtlRelayOff ( uiArgs[1]);
    
    StdPrintf (pQue, STR_NEW_LINE STR_OK);
    
} // lRelayCommand ( )

//=====================================================================
// Function   : lMotorCommand ()                                           
// Purpose    : This function handles the command.
// Parameters : Nothing is passed.
// Returns    : Nothing is returned.
//=====================================================================
static _reentrant VOID lMotorCommand ( SER_UART_QUE *pQue  )
{
    //
    // MOTOR
    // Command string : "motor [id] [state]"
    // Arguments      : id    - Motor to modify ( 1 / 2 / 3 ).
    //                  state - State to put motor into ( on / off ).
    // Returns        : Nothing is returned. Specified motor is put into desired run state. 
    //
    
    if ( ucNumArgs == 3 )
    {
        UtilSetMotorSpeed ( uiArgs[1], uiArgs[2] );
        StdPrintf (pQue, STR_NEW_LINE STR_OK);
    }
    
} // lMotorCommand ( VOID )
        
//=====================================================================
// Function   : lBoardIdCommand ()                                           
// Purpose    : This function handles the command.
// Parameters : Nothing is passed.
// Returns    : Nothing is returned.
//=====================================================================
static _reentrant VOID lBoardIdCommand ( SER_UART_QUE *pQue  )
{
    //
    // BOARD ID
    // Command string : "id"
    // Arguments      : No arguments.
    // Returns        : Identification number of board; derived from DIP switch.
    //
    
    StdPrintf (pQue, STR_NEW_LINE "%u", g_BoardNumber);
        
} // lBoardIdCommand ( ) 

//=====================================================================
// Function   : lSetupCommand ()                                           
// Purpose    : This function handles the setup command.
// Parameters : Nothing is passed.
// Returns    : Nothing is returned.
//=====================================================================
static _reentrant VOID lSetupCommand ( SER_UART_QUE *pQue  )
{
    UCHAR  ucLampType;
    
    StdPrintf (pQue, STR_NEW_LINE "Details              %02x", SerSlcMiscSetupPayload.ucByte1Details       );
    
    ucLampType = SerSlcMiscSetupPayload.ucByte1Details & SLC_SETUP_B1_LAMP_TYPE_MASK;
    
    
    if (ucLampType == SLC_SETUP_B1_LAMP_TYPE_NOT_SETUP)
    {
    StdPrintf (pQue, STR_NEW_LINE "Lamp                 Not Setup");
    }
    
    if (ucLampType == SLC_SETUP_B1_LAMP_TYPE_METAL_HALIDE)
    {
    StdPrintf (pQue, STR_NEW_LINE "Lamp                 Metal Halide");
    }
    if (ucLampType == SLC_SETUP_B1_LAMP_TYPE_XENON       )
    {
    StdPrintf (pQue, STR_NEW_LINE "Lamp                 Xenon");
    }
    if (ucLampType == SLC_SETUP_B1_LAMP_TYPE_HALOGEN     )
    {
    StdPrintf (pQue, STR_NEW_LINE "Lamp                 Halogen");
    }
    
    if ( SerSlcMiscSetupPayload.ucByte1Details & SLC_SETUP_B1_ENABLE_END_OF_LIFE_MSG )
    {
    StdPrintf (pQue, STR_NEW_LINE "End of Life Message  Enabled");
    } else
    {
    StdPrintf (pQue, STR_NEW_LINE "End of Life Message  Disabled");
    }

    StdPrintf (pQue, STR_NEW_LINE "Lamp Model Number    '%s'", SerSlcMiscSetupPayload.szLampModelNumber    );
    StdPrintf (pQue, STR_NEW_LINE "Lamp Serial Number   '%s'", SerSlcMiscSetupPayload.szLampSerialNumber   );
    StdPrintf (pQue, STR_NEW_LINE "Auto Return Seconds  %u"  , SerSlcMiscSetupPayload.usAutoReturnSeconds  );
    StdPrintf (pQue, STR_NEW_LINE "Firmware Version     "      VERSION_STR                                 );
    StdPrintf (pQue, STR_NEW_LINE "                     %u"  , SerSlcMiscSetupPayload.usRoFirmwareVersion  );
    
    StdPrintf (pQue, STR_NEW_LINE "Main Device Name     '%s'", SerSlcDeviceNameEtcPayload.szMainDeviceName         );
    StdPrintf (pQue, STR_NEW_LINE "SerPort Device Name     '%s'", SerSlcDeviceNameEtcPayload.szSerPortDeviceName         );
    StdPrintf (pQue, STR_NEW_LINE "EthPort Device Name     '%s'", SerSlcDeviceNameEtcPayload.szEthPortDeviceName         );
    
} // lSetupCommand ( ) 


//=====================================================================
// Function   : lVersionCommand ()                                           
// Purpose    : This function handles the "stat" command.
// Parameters : Nothing is passed.
// Returns    : Nothing is returned.
//=====================================================================
static _reentrant VOID lVersionCommand ( SER_UART_QUE *pQue  )
{
    StdPrintf (pQue, STR_NEW_LINE VERSION_STR);
    
    
} // lVersionCommand ( ) 


//=====================================================================
// Function   : lCommCommandPrintf ()                                           
// Purpose    : This function helps with the printf of the "comm" command.
// Parameters : Nothing is passed.
// Returns    : Nothing is returned.
//=====================================================================
static _reentrant VOID lCommCommandPrintf ( SER_UART_QUE *pQue, _rom char *pType, _rom char *pDesc, SER_UART_QUE *pQ)
{
        StdPrintf (pQue, STR_NEW_LINE "%S%5S", pType, pDesc);

        StdPrintf (pQue, "%3u%6u |%6u%6u%6u%6u%6u",
            (UINT) pQ->ucStateProtocol       ,
            (UINT) SerQueSpaceUsed (pQ)      ,
            (UINT) pQ->Stats.uiNumPacketsOk  ,
            (UINT) pQ->Stats.uiErrorStx      ,
            (UINT) pQ->Stats.uiErrorChecksumOrLength ,
            (UINT) pQ->Stats.uiErrorEtx      ,
            (UINT) pQ->Stats.uiInsertCount   );
            
        StdPrintf (pQue, "%6u%6u%6u%6u%6u", 
            (UINT) pQ->Stats.uiErrorQueFull  ,
            (UINT) pQ->Stats.uiAddrMatch     ,
            (UINT) pQ->Stats.uiIntsTx        ,
            (UINT) pQ->Stats.uiIntsRx        ,
            (UINT) pQ->Stats.uiAddrBroadcast );
            
        StdMemClr(&pQ->Stats, sizeof(pQ->Stats));
        
} // lCommCommandPrintf ( )

//=====================================================================
// Function   : lCommCommand ()                                           
// Purpose    : This function handles the "comm" command.
// Parameters : Nothing is passed.
// Returns    : Nothing is returned.
//=====================================================================
static _reentrant VOID lCommCommand ( SER_UART_QUE *pQue  )
{
    UCHAR i;
    tSerUartQues *pSerUartQue;
            
    pSerUartQue = gSerUartQues;
    
    StdPrintf (pQue, STR_NEW_LINE "Ty Descr St CurrQ | PktOK ErStx ErCkL ErEtx Total #Full Match TxInt RxInt Broad");
    StdPrintf (pQue, STR_NEW_LINE "== ===== == ===== | ===== ===== ===== ===== ===== ===== ===== ===== ===== =====");
    
    for (i = 0; i < StdHbound1(gSerUartQues); i++)
    {
        SER_UART_QUE    *pQ;

        //
        // Print TX stats followed by RX stats.
        //
        pQ = pSerUartQue->pTxQ;
        lCommCommandPrintf (pQue, "TX ", pSerUartQue->pDesc, pQ);
        
        pQ = pSerUartQue->pRxQ;
        lCommCommandPrintf (pQue, "RX ", pSerUartQue->pDesc, pQ);
        
        ++pSerUartQue;
    }
    
} // lCommCommand ( )
 
//=====================================================================
//                                    
// Function   : UtilProcessTty ()       
//                                    
// Purpose    : This function process a character received on SerPort
//              or EthPort as a TTY debugging byte.
//                                    
// Parameters : * PTR to tProtocolChannel which received the byte 
//              * Received byte
//                                    
// Returns    : Nothing is returned.
//                                    
// Note       : Recieved TTY data is stored in ucPayload[] until
//              carriage return ("ASCII_CR") is received.
//                                    
//=====================================================================

VOID _reentrant UtilProcessTty( tProtocolChannels *pChannel, UCHAR c )
{
    VOID _reentrant (*function) ( SER_UART_QUE * ); 
    UINT            *pF;

    //
    // Take receive from SerPort (passed as 'c'), and store it in the
    // command buffer.
    //
    
    //
    // Protect against buffer over run 
    //
    
    if (pChannel->ucTtyIndex >= sizeof (pChannel->ucPayload))
    {
        StdPrintf (pChannel->pTxQue, STR_NEW_LINE STR_QUESTION);
        pChannel->ucTtyIndex = 0;
    }
    
    //
    // When a delete or backspace is seen, delete the 
    // character in the buffer and on the screen.
    //
    
    if ( (c == ASCII_BS) || (c == ASCII_DEL) )
    {
        if ( pChannel->ucTtyIndex )
        {
            --pChannel->ucTtyIndex;
            
            //
            // Backspace to delete character typed...
            //
            
            StdPutc ( pChannel->pTxQue, ASCII_BS);
            StdPutc ( pChannel->pTxQue, ' '     );
            StdPutc ( pChannel->pTxQue, ASCII_BS);
        }
        else
        {
            // 
            // Sound bell to indicate beginning of line.
            //
            
            StdPutc ( pChannel->pTxQue, ASCII_BELL);
        }
        return;
    }
    
    
    //
    // When a return is seen, process the packet.
    //
    
    if (ASCII_CR == c)
    {
        _rom tCommand  *pComm;
        
        //
        // Terminate command read into a string...
        //
        
        pChannel->ucPayload[pChannel->ucTtyIndex] = 0;
        
        //
        // Reset packet index...
        //
        
        lUtilParseCommandStrIntoArgs ( pChannel->ucPayload ); 
        
        //
        // Find command and execute it.  If the command
        // is not found, print question mark and prompt.
        //
        
    
        pComm = Command;
    
        while ( (pucArgStr[0]) && (pComm->Name) )
        {
            if (0 == StdRomStrComp(pucArgStr [0], pComm->Name))
            {
                pF = (UINT *) &function;
                *pF = pComm->pFunction;                
                if (*pF)
                {
                    function ( pChannel->pTxQue );
                }
    
                break;
            }
            ++pComm;
        }
        
        //
        // If the command was entered, and no matching
        // command was found in the "Command" structure,
        // then print a question mark.
        //
        
        if (pucArgStr[0] && (!pComm->Name))
        {
            StdPrintf (pChannel->pTxQue, STR_NEW_LINE STR_QUESTION);
        }
        
        StdPrintf ( pChannel->pTxQue, STR_NEW_LINE STR_PROMPT, g_BoardNumber+1);

        pChannel->ucTtyIndex = 0;
        
        return;

    } // if (ASCII_CR == c)
    
    //
    // Store byte just received in the command buffer.
    //
    
    pChannel->ucPayload[pChannel->ucTtyIndex++] = c;
    
    //
    // Echo byte just received to the screen
    //
    
    StdPrintf (pChannel->pTxQue, "%c", c);
    

} // UtilProcessTty ()


//=====================================================================
// Function   : lUtilParseCommandStrIntoArgs ()                                        
// Purpose    : This function parses the incoming command.
// Parameters : Pointer to UART Rx buffer for appropriate hardware UART.
// Returns    : Nothing is returned.
//=====================================================================

static _reentrant VOID lUtilParseCommandStrIntoArgs ( UCHAR *pCommandString )
{
    UCHAR SpaceDetected;
    UCHAR i;
    UCHAR length;
    UCHAR *c;
    
    length = StdStrLen (pCommandString );
    
    //
    // Parse the command, and seperate into individual arguments.
    //
    
    //
    // Initialize all pointers to NULL and values to zero.
    // 
    
    for (i = 0; i < hbound1(uiArgs); i++)
    {
        uiArgs     [i] = 0;
        pucArgStr  [i] = NULL;
    }
    

    ucNumArgs = 0;
    SpaceDetected = TRUE;
    
    //
    // Replace all white space with zeros.
    //
    
    c = pCommandString;
    for ( i = 0; i < length; i++, c++) 
    {        
        if ( *c == ASCII_SPACE ) 
        {
            *c = 0;
        }
    }
    
    //
    // Convert all parameters...
    //
    
    c = pCommandString;
    for ( i = 0; i < length; i++, c++) 
    {        
   
        if ( 0 == *c ) 
        {
            //
            // If a NULL is detected immediately after a valid character,
            // set the space flag, and increment the number of arguments.
            //

            SpaceDetected = TRUE;
        } 
        else 
        {
            //
            // If a valid character is detected immediately after a space,
            // reset the space flag, and set the next argument pointer to that location.
            //

            if ( SpaceDetected ) 
            {
                uiArgs     [ucNumArgs] = StdAsciiToUlongAutoBase ( c );
                pucArgStr  [ucNumArgs] = c;
                ucNumArgs++;
                SpaceDetected = FALSE;
            }
        
        } // if ( Buffer[i] == SPACE ) 
    
    } // for ( i = 0; i < length; i++ )

} // lUtilParseCommandStrIntoArgs ()


//=====================================================================
// Function   : UtilGotoHomePosition ()
// Purpose    : Just like it sounds
// Parameters : Nothing is passed.
// Returns    : Nothing is returned.
//=====================================================================

VOID UtilGotoHomePosition ( VOID )
{

    //
    // Stop tour if in progress.
    //
    
    if (SerSlcExtStatusPayload.BasicStatus.ucActiveTour != TOUR_INVALID)
    {
        if ( ucUtlDebugMask & DEBUG_TOUR_EXCEPTION )
        {
                StdPrintf (UtlQueDebugPrint,  STR_NEW_LINE "TOUR STOPPED BY HOME POSITION COMMAND"); 
        }
        SerSlcExtStatusPayload.BasicStatus.ucActiveTour = TOUR_INVALID;
        UtilPolarStop ( );
    }

    UtilGotoPolarPosition ( 0, 0, 1500, 0);
    
} // UtilGotoHomePosition ( ) 


//=====================================================================
// Function   : UtilCloseToPoint ( )
// Purpose    : Returns indication if SL is close to passed pan/tilt.
//=====================================================================
BOOLEAN UtilCloseToPoint (LONG lPan, LONG lTilt)
{
    BOOLEAN bResult;
    
    if ( g_BaseType == DIP_SLEEK_BASE )
    {
        bResult = ( ( g_PotReader.PanFb + 36000  > lPan -  50 ) && 
                    ( g_PotReader.PanFb + 36000  < lPan +  50 ) && 
                    ( g_PotReader.TiltFb > lTilt - 50 ) && 
                    ( g_PotReader.TiltFb < lTilt + 50 ) );
    } 
    else
    {
        bResult = ( ( g_PotReader.PanFb  > lPan -  50 ) && 
                    ( g_PotReader.PanFb  < lPan +  50 ) && 
                    ( g_PotReader.TiltFb > lTilt - 50 ) && 
                    ( g_PotReader.TiltFb < lTilt + 50 ) );
    }
                    
    return bResult;
         
} // UtilCloseToPoint ( )

//=====================================================================
// Function   : UtilGotoPolarPosition ()
// Purpose    : Just like it sounds
// Parameters : Nothing is passed.
// Returns    : True if placed in motion to the point,
//              False if already stopped on the point.
//=====================================================================

BOOLEAN UtilGotoPolarPosition ( LONG lPan, SHORT sTilt, USHORT usRate, UINT uiFocus)
{
    LONG            lTempPan    ;
    LONG            lTempTilt   ;
    USHORT         usTempRate   ;
    
    static LONG     lLastPan    ;
    static LONG     lLastTilt   ;
    static USHORT  usLastRate   ;
            
    // 
	// Make sure PRB is functioning in normal position feedback mode
	//

	if ( g_PotReader.OperationStatus == NEEDS_CAL_MODE || 
         g_PotReader.CalStatus || 
         (SerSlcExtStatusPayload.ucByte10 & SLC_EXT_STAT_B10_PRB_COMM_FAULT) )
	{
		return FALSE;
	}
    
    //
    // Convert input to internal values (same as what comes from PRB) for PAN axis.
    //

    lTempPan = lPan + (LONG) SerSlcMiscSetupPayload.usPanHomeOffset;
    
    // 
    // Process small base (continuous rotation) differently than others.
    //

    if ( g_BaseType == 2 )
    {
        //
        // Make lTempPan greater than or equal to zero,
        // while conforming to the realities of 360 degree
        // compass rose.
        //

        while ( lTempPan > 18000)
        {
            lTempPan -= 36000;
        }
        
        while (lTempPan < -18000)
        {
            lTempPan += 36000;
        }

        // 
        // Offset start position by full turn for proper "crossing 0" operation
        // as implemented in UtilPolarUpdateTargetPos()
        //

        lTempPan += 36000;
            
    } // if ( g_BaseType == 2 )

    else
    {
        //
        // Check to make sure commanded input is not out of range, else don't go.
        //

        if ( lTempPan < 0 )
            lTempPan = 0;
        if ( lTempPan > g_PotReader.PanRange )
            lTempPan = g_PotReader.PanRange ;

        // g_TargetAbsPosPan = (UINT) lPositionTemp;
    }
    
    //
    // Do the same for TILT axis. 
    //

    lTempTilt = sTilt + (LONG) SerSlcMiscSetupPayload.usTiltHomeOffset;
    
    //
    // Cap values
    //
    
    if ( lTempTilt < 0 )
        lTempTilt = 0;
    if ( lTempTilt > g_PotReader.TiltRange )
        lTempTilt = g_PotReader.TiltRange ;
        
    //
    // Do same (sic) for Rate.
    //
    
    usTempRate = usRate;
    
    //
    // Azimuth and Tilt are now conditioned.  If the SL is not in motion, then check
    // to see if the SL is already at the conditioned point.
    //
    
    if (    ( !g_PolarCoordMode                       ) &&   // Not in motion
            (  UtilCloseToPoint (lTempPan, lTempTilt) ) )   // Close to point
    {
        return FALSE;
    }

    //
    // v1.53 - Prevent stutter when a command to goto the point already in
    // motion is received.
    //
    
    if (    (     g_PolarCoordMode     ) &&
            ( lTempPan   == lLastPan   ) &&
            ( lTempTilt  == lLastTilt  ) &&
            ( usTempRate == usLastRate ) )
    {
        return TRUE;
    }
    
    //
    // OK, a new valid point has been provided, make
    // motion to that point.
    //
            
    UtilPolarStop ( );

    g_TargetAbsPosPan       = lLastPan   = (UINT) lTempPan ;
    g_TargetAbsPosTilt      = lLastTilt  = (UINT) lTempTilt;
    g_PolarRate             = usLastRate =       usTempRate;
    g_TargetAbsPosFocus     = uiFocus;
    g_PolarCoordMode        = TRUE;

    usUtlAutoReturnCountdown = 0;
    
    return TRUE;

} // UtilGotoPolarPosition ( )

// mbdtest experimental tracking mode
//=====================================================================
// Function   : UtilGotoDynTrackingPosition ()
// Purpose    : Just like it sounds
// Parameters : Nothing is passed.
// Returns    : True if placed in motion to the point,
//              False if already stopped on the point.
//=====================================================================

BOOLEAN  UtilGotoDynTrackingPosition( LONG lPan, SHORT sTilt, USHORT usRate)
{
    LONG            lTempPan    ;
    LONG            lTempTilt   ;
    USHORT         usTempRate   ;
    
    static LONG     lLastPan    ;
    static LONG     lLastTilt   ;
    static USHORT  usLastRate   ;
            
    // 
	// Make sure PRB is functioning in normal position feedback mode
	//

	if ( g_PotReader.OperationStatus == NEEDS_CAL_MODE || 
         g_PotReader.CalStatus || 
         (SerSlcExtStatusPayload.ucByte10 & SLC_EXT_STAT_B10_PRB_COMM_FAULT) )
	{
		return FALSE;
	}
    
    //
    // Convert input to internal values (same as what comes from PRB) for PAN axis.
    //

    lTempPan = lPan + (LONG) SerSlcMiscSetupPayload.usPanHomeOffset;
    
    // 
    // Process small base (continuous rotation) differently than others.
    //

    if ( g_BaseType == 2 )
    {
        //
        // Make lTempPan greater than or equal to zero,
        // while conforming to the realities of 360 degree
        // compass rose.
        //

        while ( lTempPan > 18000)
        {
            lTempPan -= 36000;
        }
        
        while (lTempPan < -18000)
        {
            lTempPan += 36000;
        }

        // 
        // Offset start position by full turn for proper "crossing 0" operation
        // as implemented in UtilPolarUpdateTargetPos()
        //

        lTempPan += 36000;
            
    } // if ( g_BaseType == 2 )

    else
    {
        //
        // Check to make sure commanded input is not out of range, else don't go.
        //

        if ( lTempPan < 0 )
            lTempPan = 0;
        if ( lTempPan > g_PotReader.PanRange )
            lTempPan = g_PotReader.PanRange ;

        // g_TargetAbsPosPan = (UINT) lPositionTemp;
    }
    
    //
    // Do the same for TILT axis. 
    //

    lTempTilt = sTilt + (LONG) SerSlcMiscSetupPayload.usTiltHomeOffset;
    
    //
    // Cap values
    //
    
    if ( lTempTilt < 0 )
        lTempTilt = 0;
    if ( lTempTilt > g_PotReader.TiltRange )
        lTempTilt = g_PotReader.TiltRange ;
        
    //
    // Do same (sic) for Rate.
    //
    
    usTempRate = usRate;
    
   
    //
    // OK, a new valid point has been provided, make
    // motion to that point.
    //
            
    g_TargetAbsPosPan       = lLastPan   = (UINT) lTempPan ;
    g_TargetAbsPosTilt      = lLastTilt  = (UINT) lTempTilt;
	if ( g_TargetAbsPosPan <= 50 )
	{
		g_TargetAbsPosPan = 50 ;
	}
    g_PolarRate             = usLastRate =       usTempRate;
    g_PolarCoordMode        = FALSE;
	g_DynTrackingMode		= TRUE;

    usUtlAutoReturnCountdown = 0;
    
    return TRUE;
}

                 
//=====================================================================
// Function   : UtilNextTourPoint ()
// Purpose    : Starts next tour point, if a tour is running.
// Assumption : Called immediatly after the last tour point is finished.
// Parameters : Nothing is passed.
// Returns    : Nothing is returned.
//=====================================================================

VOID    UtilNextTourPoint ( VOID )
{
    
 
} // UtilNextTourPoint ( )


//=====================================================================
// Function   : UtilBeginDwelling ()
// Purpose    : Starts next tour point, if a tour is running.
// Assumption : Called immediatly after movement to a tour 
//              point has stopped
// Parameters : Nothing is passed.
// Returns    : Nothing is returned.
//=====================================================================

VOID UtilBeginDwelling ( VOID )
{
        
} // VOID UtilBeginDwelling ( VOID )
                
//=====================================================================
// Function   : UtilPolarStop ()
// Purpose    : This function stops polar coordinate motion. 
// Parameters : Nothing is passed.
// Returns    : Nothing is returned.
//=====================================================================

VOID    UtilPolarStop ( VOID )
{
        usUtlAutoReturnCountdown = SerSlcMiscSetupPayload.usAutoReturnSeconds;

        g_PolarCoordMode = FALSE;
        g_PolarIter = 0;
        
        UtilSetMotorSpeed ( PAN_MOTOR, 127 );
        UtilSetMotorSpeed ( TILT_MOTOR, 127 );
}

//=====================================================================
// Function   : UtilPolarUpdateTargetPos ()
// Purpose    : This function updates the "stitching" polar coordinate 
//            : positions.
//            : The motor control only uses a position loop, so the 
//            : position must be updated each time through the feedback
//            : loop when the motors are in motion. 
// Parameters : Nothing is passed.
// Returns    : Nothing is returned.
//=====================================================================

static VOID UtilPolarUpdateTargetPos ( VOID )
{
    LONG     iPanDegrees;
    LONG     iTiltDegrees;
    LONG     iPathDegrees;

    static   INT   iPanPhaseEnd;
    static   INT   iTiltPhaseEnd;

    static   ULONG  ulPanStartDeg;
    static   ULONG  ulTiltStartDeg;

    static   UINT  PreviousPanPos;

    LONG     iTempPanDeg;
    LONG     iTempTiltDeg;

    static LONG     iPanPosHundredthIncrement;
    static LONG     iTiltPosHundredthIncrement;
//    static LONG     iFocusPosHundredthIncrement;

    ULONG           ulTimeIn100ms;     // Length of polar movement in units of 100mS
    LONG            iSpeedTemp;
//    static INT      PanSpeed;    
    static INT      TiltSpeed;
    UCHAR           ucPanDir;
    UCHAR           ucTiltDir;
 //   UINT            uiIterRemaining; //MAS!!!!!!!!!!!!!!!!!!!!!!!!
    UCHAR           PanAccumFlag;
    UCHAR           TiltAccumFlag;
    UCHAR           Pan2AccumFlag;
    UCHAR           Tilt2AccumFlag;

    UCHAR           ucPolarOverrunCount;
    
    INT             temp;
    INT             iPanTemp;
    INT             iTiltTemp;

    static UCHAR    ExtRangeOffset;

    ULONG           PanPotFeedback;
    UINT            TiltPotFeedback;

	debugState1 = 0;  // MAS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	debugState2 = 0;  // MAS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	debugState3 = 0;  // MAS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	debugState4 = 0;  // MAS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	debugState5 = 0;  // MAS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

    //
    // If not in polar coordinate mode, return from function.
    //

    if ( ! g_PolarCoordMode )
    {
        g_PolarIter = 0;
        return;
    }
    
    //
    // Init Pan and Tilt Pot FB registers (with offset on sleek base)
    //    

    if ( g_BaseType == DIP_SLEEK_BASE )
    {    
        if ( 0 == g_PolarIter )
        {
            PreviousPanPos = g_PotReader.PanFb;
            ExtRangeOffset = 1;
        }
            
        if ( g_PotReader.PanFb > 30000 && g_PotReader.PanFb <= 36000 && PreviousPanPos < 6000 && PreviousPanPos >= 0 )
            ExtRangeOffset = ExtRangeOffset - 1;
        else if ( g_PotReader.PanFb < 6000 && g_PotReader.PanFb >= 0 && PreviousPanPos > 30000 && PreviousPanPos <= 36000 )
            ExtRangeOffset = ExtRangeOffset + 1;     

        PanPotFeedback = (ULONG) g_PotReader.PanFb + ( ExtRangeOffset * 36000 );
    }
    else
        PanPotFeedback = (ULONG) g_PotReader.PanFb;

    TiltPotFeedback = g_PotReader.TiltFb;
    
    PreviousPanPos = g_PotReader.PanFb;

    g_PanDebug = PanPotFeedback;
    g_TiltDebug = TiltPotFeedback;

    //
    // If it is the first time through in polar mode, set up the parameters.
    // 

    if ( 0 == g_PolarIter )
    {
        
        //
        //  Make sure input values are sane.
        //

        if ( g_TargetAbsPosPan > g_PotReader.PanRange && g_BaseType != 2 )
        {
            g_TargetAbsPosPan = g_PotReader.PanRange;
        }
        if ( g_TargetAbsPosPan < 0 )
        {
            g_TargetAbsPosPan = 0;
        }
        
        if ( g_TargetAbsPosTilt > g_PotReader.TiltRange )
        {
            g_TargetAbsPosTilt = g_PotReader.TiltRange;
        }
        if ( g_TargetAbsPosTilt < 0 )
        {
            g_TargetAbsPosTilt = 0;
        }
         
        if ( g_TargetAbsPosFocus > g_PotReader.FocusRange )
        {
            g_TargetAbsPosFocus = g_PotReader.FocusRange;
        }
        if ( g_TargetAbsPosFocus < 0 )
        {
            g_TargetAbsPosFocus = 0;
        }
        
        if ( g_PolarRate > 32000 )
        {
            g_PolarRate = 32000;
        }
        if ( g_PolarRate < 0 )
        {
            g_PolarRate = 32000;
        }
        else if ( g_PolarRate < 40 )
        {
            g_PolarRate = 40;
        }

        //
        // Init beginning position, offset pan on sleek base for continuous rotation.
        //

        ulPanStartDeg = PanPotFeedback;
        ulTiltStartDeg = TiltPotFeedback;

        //
        // Calculate length of path in degrees, if sleek base, make sure it is the shortest path
        //
        
        iPanDegrees = (ULONG) g_TargetAbsPosPan - ulPanStartDeg ;

        if ( iPanDegrees < -18000 && g_BaseType == 2 )
        {
            g_TargetAbsPosPan = g_TargetAbsPosPan + 36000;
        }
        if ( iPanDegrees > 18000 && g_BaseType == 2 )
        {
            g_TargetAbsPosPan = g_TargetAbsPosPan - 36000;
        }
        
        iPanDegrees = (ULONG) g_TargetAbsPosPan - ulPanStartDeg ;

        iTiltDegrees = (ULONG) g_TargetAbsPosTilt - ulTiltStartDeg ;
        
        if ( iPanDegrees < 50 && iPanDegrees > -50 && iTiltDegrees < 50 && iTiltDegrees > -50 )
        {
            UtilPolarStop( );
            return;
        }

        iPathDegrees = 100 * ( UtilSqrt ( ( iPanDegrees / 100 ) * ( iPanDegrees / 100 ) + ( iTiltDegrees / 100 ) * ( iTiltDegrees / 100 ) ) - 1 );
        
        // 
        // Get absolute values of pan/tilt distances to simplify conditional logic and math for rate calculation.
        //

        iTempPanDeg = iPanDegrees;
        iTempTiltDeg = iTiltDegrees;
        
        if ( iTempPanDeg < 0 )
            iTempPanDeg = - iTempPanDeg;

        if ( iTempTiltDeg < 0 )
            iTempTiltDeg = - iTempTiltDeg;
        
        //
        // Intelligently limit rates so that the maximum speed in each axis can be achieved while 
        // still maintaining straight line motion.
        //
        
        ulTimeIn100ms = ( iPathDegrees * 10 ) / g_PolarRate;

        if ( ( iTempPanDeg * 10 / ulTimeIn100ms ) > g_PotReader.PanMaxRate + g_BaseConfig[g_BaseType].ucMaxPanRateOffset )
        {
            ulTimeIn100ms = ( iTempPanDeg * 10 ) / ( g_PotReader.PanMaxRate + g_BaseConfig[g_BaseType].ucMaxPanRateOffset ); 
            
            if ( ( iTempTiltDeg * 10 / ulTimeIn100ms ) > g_PotReader.TiltMaxRate + g_BaseConfig[g_BaseType].ucMaxTiltRateOffset )
            {
                ulTimeIn100ms = ( iTempTiltDeg * 10 ) / ( g_PotReader.TiltMaxRate + g_BaseConfig[g_BaseType].ucMaxTiltRateOffset );
            }

        } // if ( ( iPanDegrees * 10 / uiTimeIn100ms ) > g_PotReader.PanMaxRate )
        else if ( ( iTempTiltDeg * 10 / ulTimeIn100ms ) > g_PotReader.TiltMaxRate + g_BaseConfig[g_BaseType].ucMaxTiltRateOffset )
        {
            ulTimeIn100ms = ( iTempTiltDeg * 10 ) / ( g_PotReader.TiltMaxRate + g_BaseConfig[g_BaseType].ucMaxTiltRateOffset );
        }

        //
        // Determine number of points (feedback loop iterations) needed to meet specified rate.
        //
        
        g_TargetPolarIter = ( ulTimeIn100ms * PRB_POLLS_PER_SEC ) / 10;
        
        //
        // Make sure there are enough iterations for the ramp up and ramp down routines.
        //
        
        if ( g_TargetPolarIter < g_BaseConfig[g_BaseType].ucStartRampCycles + g_BaseConfig[g_BaseType].ucEndRampCycles + 10 )
        {
            g_TargetPolarIter = g_BaseConfig[g_BaseType].ucStartRampCycles + g_BaseConfig[g_BaseType].ucEndRampCycles + 20;
        }

        g_PolarRate = ( iPathDegrees * 10 ) / ulTimeIn100ms;
        
        g_PanPolarPos = PanPotFeedback;
        g_TiltPolarPos = TiltPotFeedback;

        g_PanMotor.IntegralAccum = 0;
        g_TiltMotor.IntegralAccum = 0;
        g_FocusMotor.IntegralAccum = 0;

        iPanPosHundredthIncrement   = ( iPanDegrees * INCR_MULT ) / ( INT ) g_TargetPolarIter;
        iTiltPosHundredthIncrement  = ( iTiltDegrees * INCR_MULT ) / ( INT ) g_TargetPolarIter;
//        iFocusPosIncrement      = iFocusDegrees / ( INT ) g_TargetPolarIter;

        //
        // If the increment values are less than 0.1 per loop, make them 0.
        //
        
        if ( iPanPosHundredthIncrement < 10 && iPanPosHundredthIncrement > -10 )
        {
            iPanPosHundredthIncrement = 0;
        }
        if ( iTiltPosHundredthIncrement < 10 && iTiltPosHundredthIncrement > -10 )
        {
            iTiltPosHundredthIncrement = 0;
        }

        g_TargetPolarIter = g_TargetPolarIter + ( g_BaseConfig[g_BaseType].ucStartRampCycles / 2 );

#if 0
        if ( ucUtlDebugMask & POLAR_POINT )
        {
            StdPrintf ( g_QuePrbPrint, STR_NEW_LINE "Targets: P:%5u T:%5u R:%5u I:%5u", g_TargetAbsPosPan, g_TargetAbsPosPan, g_PolarRate, g_TargetPolarIter);
        }
#endif

        //
        // Predict the speed the motors will need to go so that the feedback loop
        // has a good starting point, it only fine tunes the speed. (velocity feed-forward)
        //
        
        if ( iPanPosHundredthIncrement > INCR_MULT || iPanPosHundredthIncrement < -INCR_MULT )
        {
            iSpeedTemp = g_PotReader.PanMaxRate / PRB_POLLS_PER_SEC;
            iSpeedTemp = ( iSpeedTemp * 100 ) / ( iPanPosHundredthIncrement / INCR_MULT );
            //iSpeedTemp = ( ( 127 - g_PanMotor.Deadband ) * 100 ) / iSpeedTemp;
            //iSpeedTemp = ( 16 * 100 ) / iSpeedTemp;
            iSpeedTemp = ( 64 * 100 ) / iSpeedTemp;
            PanSpeed = iSpeedTemp;
        }
        else
        {
            PanSpeed = 0;
        }
        
        if ( iTiltPosHundredthIncrement > INCR_MULT || iTiltPosHundredthIncrement < -INCR_MULT )
        {
            iSpeedTemp = g_PotReader.TiltMaxRate / PRB_POLLS_PER_SEC;
            iSpeedTemp = ( iSpeedTemp * 100 ) / ( iTiltPosHundredthIncrement / INCR_MULT );
            //iSpeedTemp = ( ( 127 - g_TiltMotor.Deadband ) * 100 ) / iSpeedTemp;
            //iSpeedTemp = ( 16 * 100 ) / iSpeedTemp;
            iSpeedTemp = ( 64 * 100 ) / iSpeedTemp;
            TiltSpeed = iSpeedTemp;
        }
        else
        {
            TiltSpeed = 0;
        }
        
        //
        // Set limits on accumulator values.
        //

        g_PanMotor.AccumLimit = 200;
        g_TiltMotor.AccumLimit = 200;
        
        //
        // Pre-load first motor speed to fool acceleration routine on the first pass.
        //

        g_PanMotor.LastSpeed = 128;

        g_TiltMotor.LastSpeed = 128;

        //
        // Set up "normal polar mode" PID coefficients
        //

        g_PanMotor.Gain.P = g_BaseConfig[g_BaseType].ucPanPNorm;
        g_PanMotor.Gain.I = g_BaseConfig[g_BaseType].ucPanINorm;
        g_PanMotor.Gain.D = g_BaseConfig[g_BaseType].ucPanDNorm;
        g_TiltMotor.Gain.P = g_BaseConfig[g_BaseType].ucTiltPNorm;
        g_TiltMotor.Gain.I = g_BaseConfig[g_BaseType].ucTiltINorm;
        g_TiltMotor.Gain.D = g_BaseConfig[g_BaseType].ucTiltDNorm;
        
        //
        // Set up boolean data type indicating motor direction ( TRUE = Forward (+), FALSE = Reverse (-) )
        //

        if ( PanSpeed < 0 )
        {
            ucPanDir = FALSE;
        }
        else
        {
            ucPanDir = TRUE;
        }

        if ( TiltSpeed < 0 )
        {
            ucTiltDir = FALSE;
        }
        else
        {
            ucTiltDir = TRUE;
        }

        //
        // Set up speed to start at if going straight to final positioning mode (less than 2 sec traverse).
        //

        if ( ucPanDir )
        {
            g_PanMotor.FinalPosStartSpeed = 158;
        }
        else
        {
            g_PanMotor.FinalPosStartSpeed = 98;
        }

        if ( ucTiltDir )
        {
            g_TiltMotor.FinalPosStartSpeed = 158;
        }
        else
        {
            g_TiltMotor.FinalPosStartSpeed = 98;
        }

        g_PanMotor.Deadband = g_BaseConfig[g_BaseType].ucPanDeadband;
        g_TiltMotor.Deadband = g_BaseConfig[g_BaseType].ucTiltDeadband;
        
        ++g_PolarIter;

		ucPolarOverrunCount = 0;  // ACM!! Was previously only cleared in a later "else" statement
		debugState0 = 0;  // MAS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

    } // if ( 0 == g_PolarIter )

    //
    // Start Ramp segment
    //

    if ( g_PolarIter <= g_BaseConfig[g_BaseType].ucStartRampCycles && g_TargetPolarIter >= ( 20 + g_BaseConfig[g_BaseType].ucStartRampCycles + g_BaseConfig[g_BaseType].ucEndRampCycles ) )
    {   
        // 
        // Ramp up motor speed from start to 1/2 second into motion.
        //

		debugState0 = 10;  // MAS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		debugState1 = debugState1 | 0x01;  // MAS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!


        g_temp = ( iPanPosHundredthIncrement * g_PolarIter ) / g_BaseConfig[g_BaseType].ucStartRampCycles;
        // g_temp = ( iTiltPosHundredthIncrement * g_PolarIter ) / START_RAMP_CYCLES;

        g_PanPolarPos   = ( g_PanPolarPos * INCR_MULT + ( ( iPanPosHundredthIncrement * g_PolarIter ) / g_BaseConfig[g_BaseType].ucStartRampCycles ) ) / INCR_MULT;
        g_TiltPolarPos  = ( g_TiltPolarPos * INCR_MULT + ( ( iTiltPosHundredthIncrement * g_PolarIter ) / g_BaseConfig[g_BaseType].ucStartRampCycles ) ) / INCR_MULT;
        
        g_MotorAccl = 20;
        
        temp = ( PanSpeed * (INT) g_PolarIter ) / g_BaseConfig[g_BaseType].ucStartRampCycles;
        UtilPolarUpdateMotor ( &g_PanMotor, PanPotFeedback, g_PanPolarPos, temp, (UCHAR) FALSE, ucPanDir, TRUE );
        g_temp2 = temp;

        temp = ( TiltSpeed * (INT) g_PolarIter ) / g_BaseConfig[g_BaseType].ucStartRampCycles;
        UtilPolarUpdateMotor ( &g_TiltMotor, TiltPotFeedback, g_TiltPolarPos, temp, (UCHAR) FALSE, ucTiltDir, TRUE );
        // g_temp2 = temp;
        
        if ( g_BaseConfig[g_BaseType].ucStartRampCycles == g_PolarIter )
        {
        debugState0 = 11;  // MAS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
   		debugState1 = debugState1 | 0x02;  // MAS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	  	   
            iPanPhaseEnd = ulPanStartDeg - g_PanPolarPos;
            iTiltPhaseEnd = ulTiltStartDeg - g_TiltPolarPos;
        }

    }

    //
    // Constant Rate segment (middle)
    //																												

    else if ( g_PolarIter <= ( g_TargetPolarIter - ( g_BaseConfig[g_BaseType].ucEndRampCycles / 2 ) ) && g_TargetPolarIter >= ( 20 + g_BaseConfig[g_BaseType].ucStartRampCycles + g_BaseConfig[g_BaseType].ucEndRampCycles ) )//MAS!!ACM!! changed 20 to 10, changed to >=
    {
        debugState0 = 20;  // MAS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
   		debugState2 = debugState2 | 0x01;  // MAS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	
        //
        // Calculate new position.
        //
        
        uiIterRemaining = g_TargetPolarIter - g_PolarIter;
        
        g_PanPolarPos   = g_TargetAbsPosPan - ( ( iPanPosHundredthIncrement * uiIterRemaining ) / INCR_MULT );
        g_TiltPolarPos  = g_TargetAbsPosTilt - ( ( iTiltPosHundredthIncrement * uiIterRemaining ) / INCR_MULT );

        g_temp = iPanPosHundredthIncrement;  // for debugging
        // g_temp = iTiltPosHundredthIncrement;  // for debugging
        
        g_MotorAccl = 5;
        
        if ( iPanPosHundredthIncrement < g_BaseConfig[g_BaseType].uiSlowAccumDump && iPanPosHundredthIncrement > - g_BaseConfig[g_BaseType].uiSlowAccumDump )
        {
        debugState0 = 21;  // MAS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		debugState2 = debugState2 | 0x02;  // MAS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	    
            g_PanMotor.IntegralAccum = 0;
            g_PanMotor.Gain.P = 10;
        }

        UtilPolarUpdateMotor ( &g_PanMotor, PanPotFeedback, (UINT) g_PanPolarPos, PanSpeed, (UCHAR) FALSE, ucPanDir, FALSE );
        UtilPolarUpdateMotor ( &g_TiltMotor, TiltPotFeedback, (UINT) g_TiltPolarPos, TiltSpeed, (UCHAR) FALSE, ucTiltDir, FALSE );

        if ( g_PolarIter == ( g_TargetPolarIter - ( g_BaseConfig[g_BaseType].ucEndRampCycles / 2 ) ) )
        {
        debugState0 = 22;  // MAS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  		debugState2 = debugState2 | 0x04;  // MAS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	   
           
            g_PanMotor.FinalPosStartSpeed = g_PanMotor.LastSpeed;
            g_TiltMotor.FinalPosStartSpeed = g_TiltMotor.LastSpeed;
            g_PanPolarPos = g_PanPolarPos * INCR_MULT;
            g_TiltPolarPos = g_TiltPolarPos * INCR_MULT;
        }

        g_temp2 = PanSpeed;  // for debugging
        // g_temp2 = TiltSpeed;  // for debugging
        
    }

    //
    // Update motors until position is within 0.15 (PAN) / 0.05 (TILT) degrees of desired.
    // Includes End Ramp in middle 
    //

    else if ( PanPotFeedback < g_TargetAbsPosPan - g_BaseConfig[g_BaseType].ucPanTarget || PanPotFeedback > g_TargetAbsPosPan + g_BaseConfig[g_BaseType].ucPanTarget || 
              TiltPotFeedback < g_TargetAbsPosTilt - g_BaseConfig[g_BaseType].ucTiltTarget || TiltPotFeedback > g_TargetAbsPosTilt + g_BaseConfig[g_BaseType].ucTiltTarget )
    {
		debugState0 = 3;  // MAS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  		debugState3 = debugState3 | 0x01;  // MAS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	

        g_PanMotor.Deadband = g_BaseConfig[g_BaseType].ucPanEndDeadband;
        g_TiltMotor.Deadband = g_BaseConfig[g_BaseType].ucTiltEndDeadband;
        
        if (g_PolarIter < g_TargetPolarIter + 50)	// Need to determine origin of 50 ACM!!  was( g_TargetPolarIter > g_PolarIter - 50 )
        {
        debugState0 = 31;  // MAS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 		debugState3 = debugState3 | 0x02;  // MAS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	
            uiIterRemaining = ( g_TargetPolarIter + ( g_BaseConfig[g_BaseType].ucEndRampCycles / 2 ) ) - g_PolarIter + 1;
        }
        else
        {
        debugState0 = 32;  // MAS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		debugState3 = debugState3 | 0x04;  // MAS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	
            uiIterRemaining = 0;
        }
        
        //
        // If the base does not get to the setpoint within 2 seconds of the end of the motor ramp down, stop motors and end
        // polar position mode.
        //

        if ( uiIterRemaining == 0 )
        {
		debugState0 = 33;  // MAS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		debugState3 = debugState3 | 0x08;  // MAS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

            //
            // Only call UtilBeginDwelling() once, when it "appears" that light has stopped moving.            
            // It may not actually be "at" the desired endpoints, but should be within 1/2 deg or less.
            //
            
            if (ucPolarOverrunCount == 1 )
            {
			debugState0 = 34;  // MAS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
			debugState3 = debugState3 | 0x10;  // MAS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

                if ( ucUtlDebugMask & DEBUG_POLAR_POINT )
                {
				debugState3 = debugState3 | 0x20;  // MAS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                    if (TOUR_INVALID == SerSlcExtStatusPayload.BasicStatus.ucActiveTour)
                    { 
                        StdPrintf ( UtlQueDebugPrint, STR_NEW_LINE "Polar Point Complete" );
			         }
                    else
                    { 
                        StdPrintf ( UtlQueDebugPrint, STR_NEW_LINE "Tour Point %i Complete", SerSlcExtStatusPayload.BasicStatus.ucActiveTourPoint );
                    }
                }

                UtilBeginDwelling ( );          
            }
            
            ucPolarOverrunCount += 1;
            if ( ucPolarOverrunCount > 40 )//Wait for two seconds before making full stop.
            { 
            debugState0 = 35;  // MAS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	      	debugState3 = debugState3 | 0x40;  // MAS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

                if ( ucUtlDebugMask & DEBUG_PRB_PRINT)
			    {
			    	StdPrintf (UtlQueDebugPrint,  STR_NEW_LINE "St:%u S1:%x S2:%x S3:%x S4:%x S5:%x I:%4u TI:%4u", debugState0,debugState1,debugState2,debugState3,debugState4,debugState5, g_PolarIter, g_TargetPolarIter ); //MAS!!!!!!!!!!
				}

				UtilPolarStop ( );

                return; 
            }
        }        
        else
		{
		debugState0 = 36;  // MAS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		debugState3 = debugState3 | 0x80;  // MAS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        
            ucPolarOverrunCount = 0;
        }
        //
        // Set limits on accumulator values.

        //

//        g_PanMotor.AccumLimit = 200;
//        g_TiltMotor.AccumLimit = 200;
        
        //
        // End Ramp segment
        // "Slowly" decrease how much commanded position is changing.
        //

        if ( ( g_PolarIter - 1 ) < g_TargetPolarIter + ( g_BaseConfig[g_BaseType].ucEndRampCycles / 2 ) )
        {
        debugState0 = 4;  // MAS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		debugState4 = debugState4 | 0x01;  // MAS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
     
           
            // g_temp = ( iPanPosHundredthIncrement * uiIterRemaining ) / ( END_RAMP_CYCLES + 1 );
            g_temp = ( iTiltPosHundredthIncrement * uiIterRemaining ) / ( g_BaseConfig[g_BaseType].ucEndRampCycles + 1 );

            g_PanPolarPos   = g_PanPolarPos + ( ( iPanPosHundredthIncrement * uiIterRemaining ) / ( g_BaseConfig[g_BaseType].ucEndRampCycles + 1 ) );
            g_TiltPolarPos  = g_TiltPolarPos + ( ( iTiltPosHundredthIncrement * uiIterRemaining ) / ( g_BaseConfig[g_BaseType].ucEndRampCycles + 1 ) );
            
            if ( uiIterRemaining > 5 )
            {
            debugState4 = debugState4 | 0x02;  // MAS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!    
                iPanTemp = PanSpeed * ( uiIterRemaining - 5 );
                iPanTemp  = iPanTemp / ( g_BaseConfig[g_BaseType].ucEndRampCycles - 5 );
            }
            else
            {
			debugState4 = debugState4 | 0x04;  // MAS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                iPanTemp = 0;
            }
            
            iTiltTemp = TiltSpeed * uiIterRemaining;
            iTiltTemp = iTiltTemp / g_BaseConfig[g_BaseType].ucEndRampCycles;
        }
        else
        {
        debugState4 = debugState4 | 0x08;  // MAS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!    

            iPanTemp = 0;
            iTiltTemp = 0;
            g_temp = 0;

            g_PanPolarPos   = (ULONG) g_TargetAbsPosPan * INCR_MULT;
            g_TiltPolarPos  = (ULONG) g_TargetAbsPosTilt * INCR_MULT;
        }
        
        g_temp2 = iPanTemp;
        // g_temp2 = iTiltTemp;
        
        //
        // If the error is less than 50, use the I term.
        //

        if ( PanPotFeedback > ( g_TargetAbsPosPan - 20 ) && PanPotFeedback < ( g_TargetAbsPosPan + 20 ) )
        {
         debugState4 = debugState4 | 0x10;  // MAS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            if ( Pan2AccumFlag )
            {
            debugState4 = debugState4 | 0x20;  // MAS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
   
                g_PanMotor.IntegralAccum = 0;
                Pan2AccumFlag = FALSE;
            }
            PanAccumFlag = TRUE;
            g_PanMotor.Gain.I = g_BaseConfig[g_BaseType].ucPanINorm * 2;
        }
        else if ( PanPotFeedback > g_TargetAbsPosPan - 100 && PanPotFeedback < g_TargetAbsPosPan + 100 )
        {
        debugState4 = debugState4 | 0x40;  // MAS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!  
            if ( PanAccumFlag )
            {
            debugState4 = debugState4 | 0x80;  // MAS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

                g_PanMotor.IntegralAccum = 0;
                PanAccumFlag = FALSE;
            }
            Pan2AccumFlag = TRUE;
            g_PanMotor.Gain.P = 10;
        }
        else
        {
        debugState4 = debugState4 | 0x100;  // MAS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!  
            g_PanMotor.Gain.I = g_BaseConfig[g_BaseType].ucPanINorm;  
            PanAccumFlag = TRUE;
            Pan2AccumFlag = TRUE;
        }
#if 1
        if ( TiltPotFeedback > g_TargetAbsPosTilt - 20 && TiltPotFeedback < g_TargetAbsPosTilt + 20 )
        {
         debugState4 = debugState4 | 0x200;  // MAS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            if ( Tilt2AccumFlag )
            {
            debugState4 = debugState4 | 0x400;  // MAS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
     
                g_TiltMotor.IntegralAccum = 0;
                Tilt2AccumFlag = FALSE;
            }
            TiltAccumFlag = TRUE;
        }
        else if ( TiltPotFeedback > g_TargetAbsPosTilt - 100 && TiltPotFeedback < g_TargetAbsPosTilt + 100 )
        {
        debugState4 = debugState4 | 0x800;  // MAS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! 
            if ( TiltAccumFlag )
            {
 	        debugState4 = debugState4 | 0x1000;  // MAS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                g_TiltMotor.IntegralAccum = 0;
                TiltAccumFlag = FALSE;
            }
            Tilt2AccumFlag = TRUE;
        }
        else
        {
        debugState4 = debugState4 | 0x2000;  // MAS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            g_TiltMotor.Gain.I = g_BaseConfig[g_BaseType].ucTiltINorm;  
            TiltAccumFlag = TRUE;
            Tilt2AccumFlag = TRUE;
        }
#endif        
//        g_PanMotor.Gain.P = PAN_P_NORM;
        g_PanMotor.Gain.D = g_BaseConfig[g_BaseType].ucPanDNorm;
        g_TiltMotor.Gain.P = g_BaseConfig[g_BaseType].ucTiltPNorm;
        g_TiltMotor.Gain.D = g_BaseConfig[g_BaseType].ucTiltDNorm;

        g_MotorAccl = 10;
                
        //
        // If the motor has reached the desired stop point, put on the brakes, the other motor isn't quite there.
        //

        if ( g_TargetAbsPosPan - PanPotFeedback > 30 && g_TargetAbsPosPan - PanPotFeedback < -30 ) // ACM!! Change this to reference g_BaseConfig struct
        {
      debugState0 = 50;  // MAS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
     	debugState5 = debugState5 | 0x01;  // MAS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
			UtilPolarUpdateMotor ( &g_PanMotor, PanPotFeedback, ( g_PanPolarPos / INCR_MULT ), iPanTemp, (UCHAR) TRUE, ucPanDir, TRUE );
        }
        else
        {
		debugState0 = 51;  // MAS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	   	debugState5 = debugState5 | 0x02;  // MAS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            UtilSetMotorSpeed ( PAN_MOTOR, 127 );
        }
                
        if ( g_TargetAbsPosTilt - TiltPotFeedback > 20 && g_TargetAbsPosTilt - TiltPotFeedback < -20 ) // ACM!! Change this to reference g_BaseConfig struct
        {
      	debugState0 = 52;  // MAS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		debugState5 = debugState5 | 0x04;  // MAS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! 
            UtilPolarUpdateMotor ( &g_TiltMotor, TiltPotFeedback, ( g_TiltPolarPos / INCR_MULT ), iTiltTemp, (UCHAR) TRUE, ucTiltDir, TRUE );
        }
        else
        {
		debugState0 = 53;  // MAS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
   		debugState5 = debugState5 | 0x08;  // MAS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

            UtilSetMotorSpeed ( TILT_MOTOR, 127 );
        }

    }

    else
    {          
    debugState0 = 6;  // MAS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
   	debugState5 = debugState5 | 0x10;  // MAS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        
        if ( ucUtlDebugMask & DEBUG_PRB_PRINT)
		    {
		    	StdPrintf (UtlQueDebugPrint,  STR_NEW_LINE "St:%u S1:%x S2:%x S3:%x S4:%x S5:%x I:%4u TI:%4u", debugState0,debugState1,debugState2,debugState3,debugState4,debugState5, g_PolarIter, g_TargetPolarIter ); //MAS!!!!!!!!!!
			}
				
        UtilPolarStop ( );  // MAS!!
        
        //
        // Begin dwelling.  Dwell for at least 1 second.
        //
        
        UtilBeginDwelling ( );
//        SerSlcExtStatusPayload.usTourDwellCountdown = 1 | UtlActivePoint.ucDwellSec;
        
		return; // v1.42
        
        // v1.42 g_PolarCoordMode = FALSE;
        // v1.42 g_PolarIter = 0;
        // v1.42 UtilSetMotorSpeed ( PAN_MOTOR, 127 );
        // v1.42 UtilSetMotorSpeed ( TILT_MOTOR, 127 );
    }    
           
    ++g_PolarIter;

	if ( ucUtlDebugMask & DEBUG_PRB_PRINT)
    {
    	StdPrintf (UtlQueDebugPrint,  STR_NEW_LINE "St:%u S1:%x S2:%x S3:%x S4:%x S5:%x I:%4u TI:%4u", debugState0,debugState1,debugState2,debugState3,debugState4,debugState5, g_PolarIter, g_TargetPolarIter ); //MAS!!!!!!!!!!
	}
} // UtilPolarUpdateTargetPos (  )


//=====================================================================
// Function   : UtilDynTrackingUpdateTargetPos ()
// Purpose    : This function updates the "stitching" polar coordinate 
//            : positions.
//            : The motor control only uses a position loop, so the 
//            : position must be updated each time through the feedback
//            : loop when the motors are in motion. 
// Parameters : Nothing is passed.
// Returns    : Nothing is returned.
//=====================================================================
static VOID UtilDynTrackingUpdateTargetPos ( VOID )
{
    LONG     iPanDegrees;
    LONG     iTiltDegrees;

    static   ULONG  ulPanStartDeg;
    static   ULONG  ulTiltStartDeg;

    static   UINT  PreviousPanPos;

//    LONG     iTempPanDeg;
//    LONG     iTempTiltDeg;

//    LONG            iSpeedTemp;
//    static INT      PanSpeed;    
//    static INT      TiltSpeed;
//    UCHAR           PanAccumFlag;
//    UCHAR           TiltAccumFlag;
//    UCHAR           Pan2AccumFlag;
//    UCHAR           Tilt2AccumFlag;

    
//    INT             temp;
//    INT             iPanTemp;
//    INT             iTiltTemp;
 
    ULONG           PanPotFeedback;
    UINT            TiltPotFeedback;

    //
    // If not in dyn tracking mode, return from function.
    //

    if ( ! g_DynTrackingMode )
    {
        return;
    }

    PanPotFeedback = (ULONG) g_PotReader.PanFb;

    TiltPotFeedback = g_PotReader.TiltFb;
    
    PreviousPanPos = g_PotReader.PanFb;

	ulPanStartDeg = PanPotFeedback;
    ulTiltStartDeg = TiltPotFeedback;

        //
        // Calculate length of path in degrees.  If either length >10 degs = 1000 "hundredths" degrees goto polar mode        //
        
    iPanDegrees = (ULONG) g_TargetAbsPosPan - ulPanStartDeg ;
       
    iTiltDegrees = (ULONG) g_TargetAbsPosTilt - ulTiltStartDeg ;
        
    if ( iPanDegrees < -1000 || iPanDegrees > 1000 )   
    {
	 	g_PanMotor.Gain.P = 4; // full speed ahead
    }
	else
	{
		g_PanMotor.Gain.P = 12; // better for tracking
	}

	if ( iTiltDegrees < -1000 || iTiltDegrees > 1000 )
	{
	 	g_TiltMotor.Gain.P = 4; // full speed ahead
	}
	else
	{
		g_TiltMotor.Gain.P = 12;
	}


//    g_PanMotor.IntegralAccum = 0;			mbdtest doing this in the motor update instead 
//    g_TiltMotor.IntegralAccum = 0;
//    g_FocusMotor.IntegralAccum = 0;
//    g_PanMotor.AccumLimit = 200;
//    g_TiltMotor.AccumLimit = 200;
    g_PanMotor.AccumLimit = 200;
    g_TiltMotor.AccumLimit = 200;
        
        //
        // Pre-load first motor speed to fool acceleration routine on the first pass.
        //

     g_PanMotor.LastSpeed = 128;

     g_TiltMotor.LastSpeed = 128;

        //
        // Set up "normal polar mode" PID coefficients
        //

    // g_PanMotor.Gain.P = g_BaseConfig[g_BaseType].ucPanPNorm;
    // g_PanMotor.Gain.I = g_BaseConfig[g_BaseType].ucPanINorm;
    // g_PanMotor.Gain.D = g_BaseConfig[g_BaseType].ucPanDNorm;
    // g_TiltMotor.Gain.P = g_BaseConfig[g_BaseType].ucTiltPNorm;
    // g_TiltMotor.Gain.I = g_BaseConfig[g_BaseType].ucTiltINorm;
    // g_TiltMotor.Gain.D = g_BaseConfig[g_BaseType].ucTiltDNorm;
     
	 //g_PanMotor.Gain.P = 4;


     g_PanMotor.Gain.I = 100;
     g_PanMotor.Gain.D = 100;
     //g_TiltMotor.Gain.P = g_BaseConfig[g_BaseType].ucTiltPNorm;
     g_TiltMotor.Gain.I = g_BaseConfig[g_BaseType].ucTiltINorm;
     g_TiltMotor.Gain.D = g_BaseConfig[g_BaseType].ucTiltDNorm;
       
     g_PanMotor.Deadband = g_BaseConfig[g_BaseType].ucPanDeadband;
     g_TiltMotor.Deadband = g_BaseConfig[g_BaseType].ucTiltDeadband;

     UtilDynTrackingUpdateMotor ( &g_PanMotor, PanPotFeedback, g_TargetAbsPosPan );
     UtilDynTrackingUpdateMotor ( &g_TiltMotor, TiltPotFeedback, g_TargetAbsPosTilt );


}	   

//=====================================================================
// Function   : UtilDynTrackingUpdateMotor ()
// Purpose    : This function updates a motor using PID control for 
//             dynamic tracking positioning.
// Parameters : A tMotor struct pointer, a calculated initial speed,
//              and a flag byte indicating if the mode is final 
//              positioning are passed.
// Returns    : Nothing is returned.
//=====================================================================

static VOID UtilDynTrackingUpdateMotor ( tMotor *pMotor, ULONG ulFeedback, ULONG ulTargetPosition )
{
    INT     iMotorCommandSpeed;
    LONG    lTemp;
 
    
    //
    // Calculate Proportional term.
    //
																																				    
    lTemp = ulTargetPosition - ulFeedback;
	if ( lTemp >= 30000 )		 // saturate here so conversion to INT works properly with pMotor->DegreeError saturation.. Saturating at 30000 is fine since we saturate below at 1000..doesn't really matter
	{
		lTemp = 30000;
	}
	if ( lTemp <= -30000 )
	{
		lTemp = -30000;
	}
	 
    pMotor->DegreeError = (INT) lTemp;
    if ( pMotor->DegreeError >= 1000 )
    {
        pMotor->DegreeError = 1000;
    }
    
    if ( pMotor->DegreeError <= -1000 )
    {
        pMotor->DegreeError = -1000;
    }

    // debug error
    if ( pMotor->ID == PAN_MOTOR )
    {
        g_PanDebug = (UINT) pMotor->DegreeError;
        g_PanPos = ulFeedback;
    }
    if ( pMotor->ID == TILT_MOTOR )
    {
        g_TiltDebug2 = (UINT) pMotor->DegreeError;
        g_TiltPos = ulFeedback;
    }
    
    pMotor->PidTerms.P  = pMotor->DegreeError / pMotor->Gain.P;                    // Calculate proportional.

//    if ( pMotor->PidTerms.P < - 127 )
//        pMotor->PidTerms.P = -127;
//    if ( pMotor->PidTerms.P > 127 )
//        pMotor->PidTerms.P = 127;
    // 
    // Calculate Integral term.
    // if a new target position has been sent, make sure reset the integral accumulator first time through

	if (g_NewTargetDynPosition == 1)
	{
		pMotor->IntegralAccum = 0;
		g_NewTargetDynPosition = 0;
	}
    pMotor->IntegralAccum += pMotor->DegreeError;                       // Add error value to integral accumulator.
    
    if ( pMotor->IntegralAccum > pMotor->AccumLimit )                         // Cap off the integral accumulator above 1000.
    {
        pMotor->IntegralAccum = pMotor->AccumLimit;
    }
    else if ( pMotor->IntegralAccum < -pMotor->AccumLimit )                   // Cap off the integral accumulator below -1000.
    {
        pMotor->IntegralAccum = -pMotor->AccumLimit;
    }

    pMotor->PidTerms.I = pMotor->IntegralAccum / pMotor->Gain.I;            // Calculate the integral.

    // 
    // Calculate Differential term.
    //
    
    pMotor->PidTerms.D = ( ulFeedback - pMotor->DiffError ) / pMotor->Gain.D;
    pMotor->DiffError = ulFeedback;    
    
    //
    // Add up motor speed inputs, make sure they are in range.
    //
    // iMotorCommandSpeed = 128 + CalcSpeed;
    // iMotorCommandSpeed = iMotorCommandSpeed + pMotor->PidTerms.P + pMotor->PidTerms.I;
    //StdPrintf     ( &g_SerTxQueSerPort,STR_NEW_LINE "MTR = %c, P= %d, I=%d, D=%d, ERR = %d, CMD = %d ",  pMotor->ID, pMotor->PidTerms.P,  pMotor->PidTerms.I, pMotor->PidTerms.D,  pMotor->DegreeError,iMotorCommandSpeed ); 
	   //STR_NEW_LINE "name main = '%s'", FlashMisc.szMainDeviceName

    iMotorCommandSpeed = 128 + pMotor->PidTerms.P + pMotor->PidTerms.I; //    iMotorCommandSpeed = iMotorCommandSpeed + pMotor->PidTerms.P + pMotor->PidTerms.I - pMotor->PidTerms.D; 

    //
    // Limit motor speed values to within 0 - 255.
    //
    
    if ( iMotorCommandSpeed < 0 )
    {
        iMotorCommandSpeed = 0;
    }
    if ( iMotorCommandSpeed > 255 )
    {
        iMotorCommandSpeed = 255;
    }
   
    //
    // Control the acceleration of the motor.
    //
    
   /* if ( iMotorCommandSpeed < pMotor->LastSpeed )
    {
        if ( iMotorCommandSpeed < pMotor->LastSpeed - g_MotorAccl )
        {
            iMotorCommandSpeed = pMotor->LastSpeed - g_MotorAccl;
        }
    }
    else if ( iMotorCommandSpeed > pMotor->LastSpeed )
    {
        if ( iMotorCommandSpeed > pMotor->LastSpeed + g_MotorAccl )
        {
            iMotorCommandSpeed = pMotor->LastSpeed + g_MotorAccl;
        }
    }
  */  
    //
    // Don't let the motor stop or brake if it lands on 127 or 128, don't let the pan motor run backward.
    //

    if ( iMotorCommandSpeed == 127 )
    {
        iMotorCommandSpeed = 126;
    }

    pMotor->Speed = ( iMotorCommandSpeed  + pMotor->LastSpeed ) / 2;

    pMotor->LastSpeed = iMotorCommandSpeed;
        
    pMotor->Speed = UtilMotorDeadband ( pMotor->Speed, pMotor->Deadband );

    if (pMotor->ID == PAN_MOTOR)
	{
	 StdPrintf     ( &g_SerTxQueSerPort,STR_NEW_LINE "%d, %d, %d, %d, %d", pMotor->PidTerms.P,pMotor->PidTerms.I, pMotor->IntegralAccum, pMotor->DegreeError,iMotorCommandSpeed );
//    StdPrintf     ( &g_SerTxQueSerPort,STR_NEW_LINE " P= %d, I=%d, IAC = %d, D=%d, ERR = %d, CMD = %d, SPEED = %d ", pMotor->PidTerms.P,  pMotor->PidTerms.I, pMotor->IntegralAccum, pMotor->PidTerms.D,  pMotor->DegreeError,iMotorCommandSpeed, pMotor->Speed );
   	}

    if ( pMotor->DegreeError < 25 && pMotor->DegreeError > -25 )  // if within +/- 0.5 degs don't keep driving motor
    {
//        pMotor->Speed = 127;
    }
	    
    UtilSetMotorSpeed ( pMotor->ID, pMotor->Speed );            // Set motor speed.

                   
} //UtilDynTrackingUpdateMotor ()


//=====================================================================
// Function   : UtilPolarUpdateMotor ()
// Purpose    : This function updates a motor using PID control for 
//              polar coordinate positioning.
// Parameters : A tMotor struct pointer, a calculated initial speed,
//              and a flag byte indicating if the mode is final 
//              positioning are passed.
// Returns    : Nothing is returned.
//=====================================================================

static VOID UtilPolarUpdateMotor ( tMotor *pMotor, ULONG ulFeedback, ULONG ulTargetPosition, INT CalcSpeed, UCHAR FinalPos, UCHAR Dir, UCHAR Ramp )
{
    INT     iMotorCommandSpeed;
    INT     iFinalSpeedLimit;
    LONG    lTemp;
    
    //
    // Make sure CalcSpeed is between -127 and 127.
    //

    if ( CalcSpeed < -127 )
    {
        CalcSpeed = -127;
    }
    if ( CalcSpeed > 127 )
    {
        CalcSpeed = 127;
    }
    
    //
    // Calculate Proportional term.
    //

    lTemp = ulTargetPosition - ulFeedback;
    pMotor->DegreeError = (INT) lTemp;
    
    if ( pMotor->DegreeError > 1000 )
    {
        pMotor->DegreeError = 1000;
    }
    
    if ( pMotor->DegreeError < -1000 )
    {
        pMotor->DegreeError = -1000;
    }

    // debug error
    if ( pMotor->ID == PAN_MOTOR )
    {
        g_PanDebug = (UINT) pMotor->DegreeError;
        g_PanPos = ulFeedback;
    }
    if ( pMotor->ID == TILT_MOTOR )
    {
        g_TiltDebug2 = (UINT) pMotor->DegreeError;
        g_TiltPos = ulFeedback;
    }
    
    pMotor->PidTerms.P  = pMotor->DegreeError / pMotor->Gain.P;                    // Calculate proportional.

    if ( pMotor->PidTerms.P < - 127 )
        pMotor->PidTerms.P = -127;
    if ( pMotor->PidTerms.P > 127 )
        pMotor->PidTerms.P = 127;
    // 
    // Calculate Integral term.
    
    pMotor->IntegralAccum += pMotor->DegreeError;                       // Add error value to integral accumulator.
    
    if ( pMotor->IntegralAccum > pMotor->AccumLimit )                         // Cap off the integral accumulator above 1000.
    {
        pMotor->IntegralAccum = pMotor->AccumLimit;
    }
    else if ( pMotor->IntegralAccum < -pMotor->AccumLimit )                   // Cap off the integral accumulator below -1000.
    {
        pMotor->IntegralAccum = -pMotor->AccumLimit;
    }

    pMotor->PidTerms.I = pMotor->IntegralAccum / pMotor->Gain.I;            // Calculate the integral.

    // 
    // Calculate Differential term.
    //
    
    pMotor->PidTerms.D = ( ulFeedback - pMotor->DiffError ) / pMotor->Gain.D;
    pMotor->DiffError = ulFeedback;    
    
    //
    // Add up motor speed inputs, make sure they are in range.
    //
    
    iMotorCommandSpeed = 128 + CalcSpeed;
    iMotorCommandSpeed = iMotorCommandSpeed + pMotor->PidTerms.P + pMotor->PidTerms.I; 
//    iMotorCommandSpeed = iMotorCommandSpeed + pMotor->PidTerms.P + pMotor->PidTerms.I - pMotor->PidTerms.D; 

    //
    // Limit motor speed values to within 0 - 255.
    //
    
    if ( iMotorCommandSpeed < 0 )
    {
        iMotorCommandSpeed = 0;
    }
    if ( iMotorCommandSpeed > 255 )
    {
        iMotorCommandSpeed = 255;
    }

    if (pMotor->ID == PAN_MOTOR)     
    {
        g_iMotorDebug = iMotorCommandSpeed;
    }
    
    //
    // If in final positioning mode and not the tilt motor,
    // make sure the motor doesn't go faster than it previously was.
    //

    if ( FinalPos && pMotor->ID != TILT_MOTOR )
    {
        iFinalSpeedLimit = pMotor->FinalPosStartSpeed - 128;
        if (iFinalSpeedLimit < 0 )
        {
            iFinalSpeedLimit = -iFinalSpeedLimit;
        }

        if ( iMotorCommandSpeed > 128 )
        {
            if ( iMotorCommandSpeed > 128 + iFinalSpeedLimit )
            {
                iMotorCommandSpeed = 128 + iFinalSpeedLimit;
            }
        }
        else 
        {
            if ( iMotorCommandSpeed < 128 - iFinalSpeedLimit )
            {
                iMotorCommandSpeed = 128 - iFinalSpeedLimit;
            }
        }
    } // if ( FinalPos )
    
    //
    // Control the acceleration of the motor.
    //
    
    if ( iMotorCommandSpeed < pMotor->LastSpeed )
    {
        if ( iMotorCommandSpeed < pMotor->LastSpeed - g_MotorAccl )
        {
            iMotorCommandSpeed = pMotor->LastSpeed - g_MotorAccl;
        }
    }
    else if ( iMotorCommandSpeed > pMotor->LastSpeed )
    {
        if ( iMotorCommandSpeed > pMotor->LastSpeed + g_MotorAccl )
        {
            iMotorCommandSpeed = pMotor->LastSpeed + g_MotorAccl;
        }
    }
    
    //
    // Don't let the motor stop or brake if it lands on 127 or 128, don't let the pan motor run backward.
    //

    if ( iMotorCommandSpeed == 127 )
    {
        iMotorCommandSpeed = 126;
    }

    if ( Ramp == FALSE )
    {
        pMotor->Speed = ( iMotorCommandSpeed  + pMotor->LastSpeed ) / 2;
//        pMotor->Speed = ( ( iMotorCommandSpeed * 2 ) + pMotor->LastSpeed ) / 3;
    }
    else
    {
        pMotor->Speed = iMotorCommandSpeed;
    }

    pMotor->LastSpeed = iMotorCommandSpeed;
        
    pMotor->Speed = UtilMotorDeadband ( pMotor->Speed, pMotor->Deadband );
    
    UtilSetMotorSpeed ( pMotor->ID, pMotor->Speed );            // Set motor speed.
                   
} //UtilPolarUpdateMotor ()

//=====================================================================
// Function   : UtilMotorDeadband ()                                           
// Purpose    : This function calculates the motor deadband.
// Parameters : Motor speed to process.
// Returns    : Processed motor speed.
//=====================================================================

UCHAR UtilMotorDeadband ( UCHAR Speed, UCHAR Deadband )
{
    // 
    // Account for motor/PWM deadband.
    //
    
    if ( Speed > 128 )
    {    
        if ( Speed + Deadband > 255 )
        {
            Speed = 255;
        }
        else
        { 
            Speed = Speed + Deadband;            // Calculate the motor speed.
        }
    }
    else if ( Speed < 127 )
    {
        if ( Speed - Deadband < 0 )
        {
            Speed = 0;
        }
        else
        { 
            Speed = Speed - Deadband ;            // Calculate the motor speed.
        }
    }

    return Speed;

} // UtilMotorDeadband ()

//=====================================================================
// Function   : UtilSetAllMotorsWithLimits ()                                           
// Purpose    : This function sets the speed of all three motors, depending
//            : on the position from the pots. If in the HARD_STOP range,
//            : brake the motor, if SOFT_STOP, slow down from full speed.
// Parameters : Pan motor speed, tilt motor speed, focus motor speed
// Returns    : Nothing is returned.
//=====================================================================

VOID UtilSetAllMotorsWithLimits ( UCHAR PanSpeed, UCHAR TiltSpeed, UCHAR FocusSpeed )
{
    UCHAR ucTempAbsVal;
    UCHAR ucTempSpeed;

    //
    // Don't limit rotation on continuous rotation base, or a base that needs cal, a disconnected PRB, or SmartView 1 base.
    //

    if ( g_BaseType == DIP_SLEEK_BASE || 
         g_PotReader.OperationStatus == NEEDS_CAL_MODE || 
         g_SmartView1 || 
       ( SerSlcExtStatusPayload.ucByte10 & SLC_EXT_STAT_B10_PRB_COMM_FAULT ) )
    {    
        ucTempSpeed = PanSpeed;
    }
    else
    {
        if ( PanSpeed >= 128 )
            ucTempAbsVal = PanSpeed - 128;
        else
            ucTempAbsVal = 128 - PanSpeed;
        
        // Checks both position and commanded speed and direction so it can drive out of the limit condition.
		// v1.73: Simplify stops to slow motors with 3 degree of end stops but allow motion past hard stops (clutches will sleep which is fine)
        if ( g_PotReader.PanFb < (HARD_STOP * 3) && PanSpeed < 128 )
        {
            ucTempSpeed = 128 - (ucTempAbsVal / 4);
        }
        else if ( g_PotReader.PanFb > ( g_PotReader.PanRange - (HARD_STOP * 3)) && PanSpeed > 128 )
        {
            ucTempSpeed = 128 + (ucTempAbsVal / 4);
        }
        else
		{
			ucTempSpeed = PanSpeed;
		}
    }
    
	g_ucJcsPanCommandSpeed = ucTempSpeed;

    //
    // Tilt Motor
    //
    
    if ( g_PotReader.OperationStatus == NEEDS_CAL_MODE || 
         g_SmartView1 || 
         ( SerSlcExtStatusPayload.ucByte10 & SLC_EXT_STAT_B10_PRB_COMM_FAULT ) )
    {
        ucTempSpeed = TiltSpeed;
    }
    else
    {
        if ( TiltSpeed >= 128 )
            ucTempAbsVal = TiltSpeed - 128;
        else
            ucTempAbsVal = 128 - TiltSpeed;

		// v1.73: Simplify stops to slow motors with 3 degree of end stops but allow motion past hard stops (clutches will sleep which is fine)
        if ( g_PotReader.TiltFb < (HARD_STOP * 3) && TiltSpeed < 128 )
        {
            ucTempSpeed = 128 - (ucTempAbsVal / 4);
        }
        else if ( g_PotReader.TiltFb > ( g_PotReader.TiltRange - (HARD_STOP * 3) ) && TiltSpeed > 128 )
        {
            ucTempSpeed = 128 + (ucTempAbsVal / 4);
        }
        else
        {
            ucTempSpeed = TiltSpeed;
        }
    }

	g_ucJcsTiltCommandSpeed = ucTempSpeed;

	UtilSetMotorSpeed ( FOCUS_MOTOR, FocusSpeed );

} // UtilSetAllMotorsWithLimits ( UCHAR PanMotor, UCHAR TiltMotor, UCHAR FocusMotor )

//=====================================================================
// Function   : UtilCheckMotorAccel ()                                           
// Purpose    : This function is called from main based on a mS timer counting down.  ACM!! added function v1.68
// Parameters : Nothing is passed.
// Returns    : Nothing is returned.
//=====================================================================

void UtilCheckMotorAccel ( VOID )
{
	UCHAR ucTempSpeed;
	static UCHAR ucPrevPanSpeed = 128;  
	static UCHAR ucPrevTiltSpeed = 128;	  
	static UCHAR ucLastMinPanSpeed = 128;	// v1.73
	static UCHAR ucLastMinTiltSpeed = 128;	// v1.73
	static UCHAR ucLastMinFocusSpeed = 128;	// v1.73

	//
	// Stop motors if in polar coordinate mode and PRB loses communication
	//																	  

	if (SerSlcExtStatusPayload.ucByte10 & SLC_EXT_STAT_B10_PRB_COMM_FAULT && g_PolarCoordMode)
	{
	    UtilPolarStop();
		return;
	}

	// v1.73 Don't let motors run continuously if this fault happens
	// Stop motors and get out of cal mode if in calibration mode and PRB loses communication
	//																	  

	if (g_PotReader.CalStatus)
	{
	    if(SerSlcExtStatusPayload.ucByte10 & SLC_EXT_STAT_B10_PRB_COMM_FAULT)
	    {
	    	UtilSetMotorSpeed ( PAN_MOTOR,   128 );
			UtilSetMotorSpeed ( TILT_MOTOR,  128 );
			UtilSetMotorSpeed ( FOCUS_MOTOR, 128 );
			g_PotReader.CalStatus = 0;
			ucUtlJcsStatByte3 |= JCS_STAT_B3_REQ_SLC_EXT_STAT_PKT;
            SerSlcExtStatusPayload.ucByte9 &= ~SLC_EXT_STAT_B9_PRB_CAL_IN_PROCESS;
			return;
		}
	}

	// v1.73 - time out motor motion if speeds haven't changed in the last minute
	if(g_TimeMs.MotorTimeout == 0 || g_ucMotorTimedOut)		// this condition is somewhat "inside-out" - goal is to reduce normal operation time by avoiding too many conditions
	{
		if(	g_PanMotor.LoadedSpeed < 127	||	g_PanMotor.LoadedSpeed > 128	||
			g_TiltMotor.LoadedSpeed < 127	||	g_TiltMotor.LoadedSpeed > 128	||
			g_FocusMotor.LoadedSpeed < 127	||	g_FocusMotor.LoadedSpeed > 128)
		{
			if(	g_ucMotorTimedOut)		// continually check to see if commanded speed has changes so operator can immediately resume control
			{
				if(	g_PanMotor.LoadedSpeed < (ucLastMinPanSpeed - 3)	||	g_PanMotor.LoadedSpeed > (ucLastMinPanSpeed + 3)	||
					g_TiltMotor.LoadedSpeed < (ucLastMinTiltSpeed - 3)	||	g_TiltMotor.LoadedSpeed > (ucLastMinTiltSpeed + 3)	||
					g_FocusMotor.LoadedSpeed < (ucLastMinFocusSpeed - 3) ||	g_FocusMotor.LoadedSpeed > (ucLastMinFocusSpeed + 3))
				{
					// escape timeout state
					g_ucMotorTimedOut = 0;
					ucLastMinPanSpeed = 128;
					ucLastMinTiltSpeed = 128;
					ucLastMinFocusSpeed = 128;
					g_TimeMs.MotorTimeout = SIXTY_SECONDS;
				}
			}

			if(	g_TimeMs.MotorTimeout == 0)
			{
				if(	g_PanMotor.LoadedSpeed < (ucLastMinPanSpeed - 3)	||	g_PanMotor.LoadedSpeed > (ucLastMinPanSpeed + 3)	||
					g_TiltMotor.LoadedSpeed < (ucLastMinTiltSpeed - 3)	||	g_TiltMotor.LoadedSpeed > (ucLastMinTiltSpeed + 3)	||
					g_FocusMotor.LoadedSpeed < (ucLastMinFocusSpeed - 3) ||	g_FocusMotor.LoadedSpeed > (ucLastMinFocusSpeed + 3))
				{
					g_ucMotorTimedOut = 0;
				}
				else
				{
					g_ucMotorTimedOut = 1;
					SFRPAGE  = CONFIG_PAGE;                 
                	MTR1_DIS = 1;                           // Directly disable motor control in case UtilSetMotorSpeed is not called
                	MTR1_EN1 = 0;                           
                	MTR1_EN2 = 0;                           
				}

				g_TimeMs.MotorTimeout = SIXTY_SECONDS;
				ucLastMinPanSpeed = g_PanMotor.LoadedSpeed;
				ucLastMinTiltSpeed = g_TiltMotor.LoadedSpeed;
				ucLastMinFocusSpeed = g_FocusMotor.LoadedSpeed;
			}
		}
		else
		{
			// escape timeout state
			g_ucMotorTimedOut = 0;
			ucLastMinPanSpeed = 128;
			ucLastMinTiltSpeed = 128;
			ucLastMinFocusSpeed = 128;
			g_TimeMs.MotorTimeout = SIXTY_SECONDS;
		}
	}
	   
	//
	// Don't check acceleration if base is in polar coordinate or calibration mode
	//

//	if (g_PolarCoordMode || g_PotReader.CalStatus)			 mbdtest
	if (g_PolarCoordMode || g_DynTrackingMode || g_PotReader.CalStatus)		  // mbdtest

	{
		return;
	}
	
	//
	// ACM!! v1.68: Stop motors if joystick has lost communications
	//
	
	if (!g_TimeMs.ActiveJcs)
	{
		UtilSetMotorSpeed ( PAN_MOTOR,   128 );
		UtilSetMotorSpeed ( TILT_MOTOR,  128 );
		UtilSetMotorSpeed ( FOCUS_MOTOR, 128 );
		return;
	}

	// 
	// Limit pan motor acceleration
	//

	if ( g_BaseConfig[g_BaseType].ucJcsAccelLimit )	 // only do acceleration limits if ucJcsAccelLimit not 0 (only do on big base since big motors cause high current issues)
	{
		if ( g_ucJcsPanCommandSpeed < ucPrevPanSpeed - g_BaseConfig[g_BaseType].ucJcsAccelLimit && g_ucJcsPanCommandSpeed < 128)
		{
			ucTempSpeed = ucPrevPanSpeed - g_BaseConfig[g_BaseType].ucJcsAccelLimit;
		}
	    else if ( g_ucJcsPanCommandSpeed > ucPrevPanSpeed + g_BaseConfig[g_BaseType].ucJcsAccelLimit && g_ucJcsPanCommandSpeed > 128)
		{
			ucTempSpeed = ucPrevPanSpeed + g_BaseConfig[g_BaseType].ucJcsAccelLimit;
		}
		else
		{
			ucTempSpeed = g_ucJcsPanCommandSpeed;
		}
		ucPrevPanSpeed = ucTempSpeed;
	}
	else
	{
		ucTempSpeed = g_ucJcsPanCommandSpeed;
	}

	UtilSetMotorSpeed ( PAN_MOTOR, ucTempSpeed );

    // 
	// Limit tilt motor acceleration
	//

	if ( g_BaseConfig[g_BaseType].ucJcsAccelLimit )	 // only do acceleration limits if ucJcsAccelLimit not 0 (only do on big base since big motors cause high current issues)
	{
		if ( g_ucJcsTiltCommandSpeed < ucPrevTiltSpeed - g_BaseConfig[g_BaseType].ucJcsAccelLimit && g_ucJcsTiltCommandSpeed < 128)
		{
			ucTempSpeed = ucPrevTiltSpeed - g_BaseConfig[g_BaseType].ucJcsAccelLimit;
		}
	    else if ( g_ucJcsTiltCommandSpeed > ucPrevTiltSpeed + g_BaseConfig[g_BaseType].ucJcsAccelLimit && g_ucJcsTiltCommandSpeed > 128)
		{
			ucTempSpeed = ucPrevTiltSpeed + g_BaseConfig[g_BaseType].ucJcsAccelLimit;
		}
		else
		{
			ucTempSpeed = g_ucJcsTiltCommandSpeed;
		}
		ucPrevTiltSpeed = ucTempSpeed;
	}
	else
	{
		ucTempSpeed = g_ucJcsTiltCommandSpeed;
	}

	UtilSetMotorSpeed ( TILT_MOTOR, ucTempSpeed );
	
} // void UtilCheckMotorAccel ( VOID )

//=====================================================================
// Function   : UtilSetMotorSpeed ()                                           
// Purpose    : This function sets the speed of the desired motor.
// Parameters : Speed to set motor.
// Returns    : Nothing is returned.
//=====================================================================

VOID UtilSetMotorSpeed ( UCHAR MotorId, INT Speed )
{
    //
    // Make sure all the compare modules are set up correctly.
    //

    SFRPAGE = PCA0_PAGE;
    PCA0CPM0 = 0x42;                        
    PCA0CPM1 = 0x42;                        
    PCA0CPM2 = 0x42;                        


    //
    // Set the directional signals to the appropriate motor.
    //
    
    switch ( MotorId )
    {
        case MOTOR_1:
            
            g_PanLastSpeedDebug = g_PanSpeedDebug;
            g_PanSpeedDebug = Speed;
			
			g_PanMotor.LoadedSpeed = Speed;				// v1.73 - record loaded speed value for use in timeout calculation
            if(g_ucMotorTimedOut)							// v1.73 - reset speed if motion is commanded but has timed out (constant speed for more than one minute)
			{
				Speed = 128;
			}
            
            // FREEWHEEL
            if ( Speed == 128 )
            {              
                SFRPAGE  = CONFIG_PAGE;                 // Set SFRPAGE to Port I/O page.
                MTR1_DIS = 1;                           // Disable H-bridge	(and clear overcurrent fault if present)
                MTR1_EN1 = 0;                           // Forward and reverse disable.
                MTR1_EN2 = 0;                           // Forward and reverse disable.
            }
            
            // STOP (BRAKE)
            else if ( Speed == 127 ) // MAS!! v1.69 17 Jan 12
            {              
                SFRPAGE  = CONFIG_PAGE;                 // Set SFRPAGE to Port I/O page.
                MTR1_DIS = 0;                           // Enable H-bridge
                MTR1_EN1 = 1;                           // Turn on dynamic braking.
                MTR1_EN2 = 1;                          
                                
                SFRPAGE  = PCA0_PAGE;                   // Set SFRPAGE to PCA0 page.
                PCA0CPH0 = 0x7F;                        // 50% duty cycle braking.   
            }
            
            // REVERSE
            else if ( Speed < 127 )
            {
                CHAR c;

                //c = Speed;
                
                SFRPAGE  = CONFIG_PAGE;                 // Set SFRPAGE to Port I/O page.
                MTR1_DIS = 0;                           // Enable H-bridge.
                MTR1_EN1 = 0;                           // Forward disable.
                MTR1_EN2 = 1;                           // Reverse enable.
                // DEBUG_PIN = 0;							// mbd debug out on P4.2 , pin 5 of J5
                c = ( Speed - 128 ) * 2;                // Scale Speed value for PWM register.
                
                SFRPAGE  = PCA0_PAGE;                   // Set SFRPAGE to PCA0 page.
                PCA0CPH0 = ( c & 0xFF );                // Set reload value for PWM duty cycle.   

            }
            
            // FORWARD
            else if ( Speed > 128 )
            {
                CHAR c;
                
                //c = Speed;
                
                SFRPAGE  = CONFIG_PAGE;                 // Set SFRPAGE to Port I/O page.
                MTR1_DIS = 0;                           // Enable H-bridge.
                MTR1_EN1 = 1;                           // Forward enable.
                MTR1_EN2 = 0;                           // Reverse disable.
                
                c = ( 128 - Speed ) * 2;
                
                SFRPAGE  = PCA0_PAGE;                   // Set SFRPAGE to PCA0 page.
                PCA0CPH0 = ( c & 0xFF );                // Set reload value for PWM duty cycle.   
            }    
            else
            {
                SFRPAGE  = CONFIG_PAGE;                 // Set SFRPAGE to Port I/O page.
                MTR1_DIS = 1;                           // Disable H-bridge
                MTR1_EN1 = 0;                           // Forward and reverse disable.
                MTR1_EN2 = 0;                           // Forward and reverse disable.
            }
            //Done
            break;
        
        case MOTOR_2:
            
            g_TiltMotor.LoadedSpeed = Speed;			// v1.73 - record loaded speed value for use in timeout calculation
            if(g_ucMotorTimedOut)							// v1.73 - reset speed if motion is commanded but has timed out (constant speed for more than one minute)
			{
				Speed = 128;
			}

            // FREEWHEEL
            if ( Speed == 128 )
            {  
                       
                // Set SFRPAGE to Port I/O page. 
				SFRPAGE  = CONFIG_PAGE;                

                MTR2_DIS = 1; 							// Disable H-bridge(and clear overcurrent fault if present)// ACM!! v1.70 14 Nov 12
				                                        // v1.69 checked MTR3 status and held off if it was in use (MTR2/3 DIS was tied together on the PCB)
			                                 
                MTR2_EN1 = 0;                           // Forward and reverse disable.
                MTR2_EN2 = 0;                           // Forward and reverse disable.
            }
            
            // STOP (BRAKE)
            else if ( Speed == 127 )
            {              
                SFRPAGE  = CONFIG_PAGE;                 // Set SFRPAGE to Port I/O page.
                MTR2_DIS = 0;                           // Enable H-bridge
                MTR2_EN1 = 1;                           // Turn on dynamic braking.
                MTR2_EN2 = 1;                          
                                
                SFRPAGE  = PCA0_PAGE;                   // Set SFRPAGE to PCA0 page.
                PCA0CPH1 = 0x7F;                        // 50% duty cycle braking.   
            }
            
            // REVERSE
            else if ( Speed < 127 )
            {
                CHAR c;
                
                SFRPAGE  = CONFIG_PAGE;                 // Set SFRPAGE to Port I/O page.
                MTR2_DIS = 0;                           // Enable H-bridge.
                MTR2_EN1 = 0;                           // Forward disable.
                MTR2_EN2 = 1;                           // Reverse enable.
                
                c = ( Speed - 128 ) * 2;                // Scale Speed value for PWM register.
                
                SFRPAGE  = PCA0_PAGE;                   // Set SFRPAGE to PCA0 page.
                PCA0CPH1 = ( c & 0xFF );                // Set reload value for PWM duty cycle.   
            }
            
            // FORWARD
            else if ( Speed > 128 )
            {
                CHAR c;
                
                SFRPAGE  = CONFIG_PAGE;                 // Set SFRPAGE to Port I/O page.
                MTR2_DIS = 0;                           // Enable H-bridge.
                MTR2_EN1 = 1;                           // Forward enable.
                MTR2_EN2 = 0;                           // Reverse disable.
                
                c = ( 128 - Speed ) * 2;
                
                SFRPAGE  = PCA0_PAGE;                   // Set SFRPAGE to PCA0 page.
                PCA0CPH1 = ( c & 0xFF );                // Set reload value for PWM duty cycle.   
            }    
            else
            {
                
                SFRPAGE  = CONFIG_PAGE;                 // Set SFRPAGE to Port I/O page.
        
                MTR2_DIS = 1;                           // Disable H-bridge(and clear overcurrent fault if present)               
                MTR2_EN1 = 0;                           // Forward and reverse disable.
                MTR2_EN2 = 0;                           // Forward and reverse disable.
            }
            
            //Done        
            break;
        
        case MOTOR_3:
            
            g_FocusMotor.LoadedSpeed = Speed;				// v1.73 - record loaded speed value for use in timeout calculation
            if(g_ucMotorTimedOut)							// v1.73 - reset speed if motion is commanded but has timed out (constant speed for more than one minute)
			{
				Speed = 128;
			}

            // FREEWHEEL
            if ( Speed == 128 )
            {              
                SFRPAGE  = CONFIG_PAGE;                 // Set SFRPAGE to Port I/O page.
                MTR3_DIS = 1; 							// Disable H-bridge(and clear overcurrent fault if present)// ACM!! v1.70 14 Nov 12
														// MTR3_DIS (U8 pin 1) was previously tied (in hardware) to MTR2_DIS

                MTR3_EN1 = 0;                           // Forward and reverse disable.
                MTR3_EN2 = 0;                           // Forward and reverse disable.
            }
            
            
            // STOP (BRAKE)
            else if ( Speed == 127 )                    // MAS!! v1.69 17 Jan 12
            {              
                SFRPAGE  = CONFIG_PAGE;                 // Set SFRPAGE to Port I/O page.
                MTR3_DIS = 0;							// Enable MTR3 driver, previously shared with MTR2  // ACM!! v1.70  14 Nov 2012
                MTR3_EN1 = 1;                           // Turn on dynamic braking.
                MTR3_EN2 = 1;                          
                                
                SFRPAGE  = PCA0_PAGE;                   // Set SFRPAGE to PCA0 page.
                PCA0CPH2 = 0x7F;                        // 50% duty cycle braking.   
            }
            
            // REVERSE    
            else if ( Speed < 127 )
            {
                CHAR c;
                
                SFRPAGE  = CONFIG_PAGE;                 // Set SFRPAGE to Port I/O page.
                MTR3_DIS = 0;							// Enable MTR3 driver  ACM!! v1.70   14 Nov 12
                MTR3_EN1 = 0;                           // Forward disable.
                MTR3_EN2 = 1;                           // Reverse enable.
                
                c = ( Speed - 128 ) * 2;
                
                SFRPAGE  = PCA0_PAGE;                   // Set SFRPAGE to PCA0 page.
                PCA0CPH2 = ( c & 0xFF );                // Set reload value for PWM duty cycle.   
            }
            
            // FORWARD    
            else if ( Speed > 128 )
            {
                CHAR c;
                
                SFRPAGE  = CONFIG_PAGE;                 // Set SFRPAGE to Port I/O page.
                MTR3_DIS = 0;							// Enable MTR3 driver  ACM!! v1.70   14 Nov 12
                MTR3_EN1 = 1;                           // Forward enable.
                MTR3_EN2 = 0;                           // Reverse disable.

                c = ( 128 - Speed ) * 2;

                SFRPAGE  = PCA0_PAGE;                   // Set SFRPAGE to PCA0 page.
                PCA0CPH2 = ( c & 0xFF );                // Set reload value for PWM duty cycle.   
            }   
            else
            {
                SFRPAGE  = CONFIG_PAGE;                 // Set SFRPAGE to Port I/O page.
                MTR3_EN1 = 0;                           // Forward and reverse disable.
                MTR3_EN2 = 0;                           // Forward and reverse disable.
            }
            //Done 
            break;
            
    } // switch ( Id )
        
} // UtilSetMotorSpeed ()

//=====================================================================
// Function   : UtlRelayOff ()                                           
// Purpose    : This function turns OFF a relay and adjust data structures.
// Parameters : Relay to turn off is passed.
// Returns    : Nothing is returned.
//=====================================================================

VOID UtlRelayOff (UCHAR ucRelay)
{
    SFRPAGE = CONFIG_PAGE;
    
    switch (ucRelay)
    {
        case RELAY_LAMP       :
            
            
            //
            // Update FLASH record of lamp time, at the time the lamp
            // is switched OFF.
            //
            
            if (RLY_LAMP)
            {
                FlashUpdateLampTime ( );
            }
            
            ucUtlJcsStatByte3 |= JCS_STAT_B3_REQ_SLC_EXT_STAT_PKT;
            SerSlcExtStatusPayload.BasicStatus.ucByte7 &= ~SLC_STAT_B7_BEAM_ON;
            RLY_LAMP = 0;
            
            break;
            
        case RELAY_LAMP_START :
            RLY_LAMP_START = 0;
            break;
        case RELAY_FOCUS_OUT  :
            RLY_FOCUS_OUT = 0;
            break;
        case RELAY_FOCUS_IN   :
            RLY_FOCUS_IN = 0;
            break;
        case RELAY_AUX1       :
            ucUtlJcsStatByte3 |= JCS_STAT_B3_REQ_SLC_EXT_STAT_PKT;
            SerSlcExtStatusPayload.BasicStatus.ucByte7 &= ~SLC_STAT_B7_AUX1_ON;
            RLY_AUX1 = 0;
            break;
        case RELAY_AUX2       :
            ucUtlJcsStatByte3 |= JCS_STAT_B3_REQ_SLC_EXT_STAT_PKT;
            SerSlcExtStatusPayload.BasicStatus.ucByte7 &= ~SLC_STAT_B7_AUX2_ON;
            RLY_AUX2 = 0;
            break;
        case RELAY_AUX3       :
            ucUtlJcsStatByte3 |= JCS_STAT_B3_REQ_SLC_EXT_STAT_PKT;
            SerSlcExtStatusPayload.BasicStatus.ucByte7 &= ~SLC_STAT_B7_AUX3_ON;
            RLY_AUX3 = 0;
            break;
         case RELAY_AUX4       :                                                 
             ucUtlJcsStatByte3 |= JCS_STAT_B3_REQ_SLC_EXT_STAT_PKT;              
             SerSlcExtStatusPayload.ucByte22 &= ~SLC_EXT_STAT_B22_AUX4_ON; 
             RLY_AUX4 = 0;                                                       
             break;                                                              
         case AUX5       :                                                       
             ucUtlJcsStatByte3 |= JCS_STAT_B3_REQ_SLC_EXT_STAT_PKT;              
             SerSlcExtStatusPayload.ucByte22 &= ~SLC_EXT_STAT_B22_AUX5_ON; 
             HSS_OUT_3 = 0;	   // maps to AUX5                                 
             break;                                                              
         case AUX6       :                                                       
             ucUtlJcsStatByte3 |= JCS_STAT_B3_REQ_SLC_EXT_STAT_PKT;              
             SerSlcExtStatusPayload.ucByte22 &= ~SLC_EXT_STAT_B22_AUX6_ON; 
             HSS_OUT_2 = 0;	   // maps to AUX6                                 
             break;                                                              
         case AUX7       :                                                       
             ucUtlJcsStatByte3 |= JCS_STAT_B3_REQ_SLC_EXT_STAT_PKT;              
             SerSlcExtStatusPayload.ucByte22 &= ~SLC_EXT_STAT_B22_AUX7_ON; 
             HSS_OUT_1 = 0;	   // maps to AUX7                                 
             break;                                                              
         case AUX8       :                                                       
             ucUtlJcsStatByte3 |= JCS_STAT_B3_REQ_SLC_EXT_STAT_PKT;              
             SerSlcExtStatusPayload.ucByte22 &= ~SLC_EXT_STAT_B22_AUX8_ON; 
             HSS_OUT_4 = 0;	   // maps to AUX8                                 
             break;                                                              
         case AUX9_UP       :                                                    
             ucUtlJcsStatByte3 |= JCS_STAT_B3_REQ_SLC_EXT_STAT_PKT;              
             SerSlcExtStatusPayload.ucByte22 &= ~SLC_EXT_STAT_B22_AUX9_UP_ON; 
             HSS_OUT_7 = 0;	   // maps to AUX9_UP                              
             break;                                                              
         case AUX9_DOWN       :                                                  
             ucUtlJcsStatByte3 |= JCS_STAT_B3_REQ_SLC_EXT_STAT_PKT;              
             SerSlcExtStatusPayload.ucByte22 &= ~SLC_EXT_STAT_B22_AUX9_DOWN_ON; 
             HSS_OUT_6 = 0;	   // maps to AUX9_DOWN                            
             break;                                                              
         case AUX10_UP       :                                                   
             ucUtlJcsStatByte3 |= JCS_STAT_B3_REQ_SLC_EXT_STAT_PKT;              
             SerSlcExtStatusPayload.ucByte22 &= ~SLC_EXT_STAT_B22_AUX10_UP_ON; 
             HSS_OUT_5 = 0;	   // maps to AUX10_UP                             
             break;                                                              
         case AUX10_DOWN       :                                                 
             ucUtlJcsStatByte3 |= JCS_STAT_B3_REQ_SLC_EXT_STAT_PKT;              
             SerSlcExtStatusPayload.ucByte23 &= ~SLC_EXT_STAT_B23_AUX10_DOWN_ON; 
             HSS_OUT_8 = 0;	   // maps to AUX10_DOWN                           
             break;														       


    }
    
} // UtlRelayOff ( )

//=====================================================================
// Function   : UtlRelayOn ()                                           
// Purpose    : This function turns ON a relay and adjust data structures.
// Parameters : Relay to turn on is passed.
// Returns    : Nothing is returned.
//=====================================================================

VOID UtlRelayOn (UCHAR ucRelay)
{
    SFRPAGE = CONFIG_PAGE;
    
    switch (ucRelay)
    {
        case RELAY_LAMP       :
        
            //
            // Zeroize running timer at time of lamp is switch ON
            //
            
            if (0 == RLY_LAMP)
            {
                usUtlLampHours   = 0;
                ucUtlLampMinutes = 0; 
                ucUtlLampSeconds = 0;
            }
            ucUtlJcsStatByte3 |= JCS_STAT_B3_REQ_SLC_EXT_STAT_PKT;
            SerSlcExtStatusPayload.BasicStatus.ucByte7 |= SLC_STAT_B7_BEAM_ON;
            RLY_LAMP = 1;
            break;
            
        case RELAY_LAMP_START :
            g_TimeMs.PrbBlanked   =  100;                   // Blank PRB while lamp is starting
            g_TimeMs.PrbCommFault = PRB_COMM_FAULT_TIME_MS; // Blank PRB while lamp is starting
            RLY_LAMP_START = 1;
            break;
        case RELAY_FOCUS_OUT  :
            RLY_FOCUS_OUT = 1;
            break;
        case RELAY_FOCUS_IN   :
            RLY_FOCUS_IN = 1;
            break;
        case RELAY_AUX2       :
            ucUtlJcsStatByte3 |= JCS_STAT_B3_REQ_SLC_EXT_STAT_PKT;
            SerSlcExtStatusPayload.BasicStatus.ucByte7 |= SLC_STAT_B7_AUX2_ON;
            RLY_AUX2 = 1;
            break;
        case RELAY_AUX3       :
            ucUtlJcsStatByte3 |= JCS_STAT_B3_REQ_SLC_EXT_STAT_PKT;
            SerSlcExtStatusPayload.BasicStatus.ucByte7 |= SLC_STAT_B7_AUX3_ON;
            RLY_AUX3 = 1;
            break;
        case RELAY_AUX1       :
            ucUtlJcsStatByte3 |= JCS_STAT_B3_REQ_SLC_EXT_STAT_PKT;
            SerSlcExtStatusPayload.BasicStatus.ucByte7 |= SLC_STAT_B7_AUX1_ON;
            RLY_AUX1 = 1;
            break;
         case RELAY_AUX4       :                                                 
             ucUtlJcsStatByte3 |= JCS_STAT_B3_REQ_SLC_EXT_STAT_PKT;              
             SerSlcExtStatusPayload.ucByte22 |= SLC_EXT_STAT_B22_AUX4_ON; 
             RLY_AUX4 = 1;                                                       
             break;                                                              
         case AUX5       :                                                       
             ucUtlJcsStatByte3 |= JCS_STAT_B3_REQ_SLC_EXT_STAT_PKT;              
             SerSlcExtStatusPayload.ucByte22 |= SLC_EXT_STAT_B22_AUX5_ON; 
             HSS_OUT_3 = 1;	   // maps to AUX5 
             break;                                
         case AUX6       :                                                       
             ucUtlJcsStatByte3 |= JCS_STAT_B3_REQ_SLC_EXT_STAT_PKT;              
             SerSlcExtStatusPayload.ucByte22 |= SLC_EXT_STAT_B22_AUX6_ON; 
             HSS_OUT_2 = 1;	   // maps to AUX6                                 
             break;                                                              
         case AUX7       :                                                       
             ucUtlJcsStatByte3 |= JCS_STAT_B3_REQ_SLC_EXT_STAT_PKT;              
             SerSlcExtStatusPayload.ucByte22 |= SLC_EXT_STAT_B22_AUX7_ON; 
             HSS_OUT_1 = 1;	   // maps to AUX7                                 
             break;                                                              
         case AUX8       :                                                       
             ucUtlJcsStatByte3 |= JCS_STAT_B3_REQ_SLC_EXT_STAT_PKT;              
             SerSlcExtStatusPayload.ucByte22 |= SLC_EXT_STAT_B22_AUX8_ON; 
             HSS_OUT_4 = 1;	   // maps to AUX8                                 
             break;                                                              
         case AUX9_UP       :                                                    
             ucUtlJcsStatByte3 |= JCS_STAT_B3_REQ_SLC_EXT_STAT_PKT;              
             SerSlcExtStatusPayload.ucByte22 |= SLC_EXT_STAT_B22_AUX9_UP_ON; 
             HSS_OUT_7 = 1;	   // maps to AUX9_UP                              
             break;                                                              
         case AUX9_DOWN       :                                                  
             ucUtlJcsStatByte3 |= JCS_STAT_B3_REQ_SLC_EXT_STAT_PKT;              
             SerSlcExtStatusPayload.ucByte22 |= SLC_EXT_STAT_B22_AUX9_DOWN_ON; 
             HSS_OUT_6 = 1;	   // maps to AUX9_DOWN                            
             break;                                                              
         case AUX10_UP       :                                                   
             ucUtlJcsStatByte3 |= JCS_STAT_B3_REQ_SLC_EXT_STAT_PKT;              
             SerSlcExtStatusPayload.ucByte22 |= SLC_EXT_STAT_B22_AUX10_UP_ON; 
             HSS_OUT_5 = 1;	   // maps to AUX10_UP                             
             break;                                                              
         case AUX10_DOWN       :                                                 
             ucUtlJcsStatByte3 |= JCS_STAT_B3_REQ_SLC_EXT_STAT_PKT;              
             SerSlcExtStatusPayload.ucByte23 |= SLC_EXT_STAT_B23_AUX10_DOWN_ON; 
             HSS_OUT_8 = 1;	   // maps to AUX10_DOWN                           
             break;									                           

    }
    
} // UtlRelayOn ( )

//=====================================================================
// Function   : UtlLampOn ()                                           
// Purpose    : This function turns ON a lamp including momentary relay.
// Parameters : Nothing is passed.
// Returns    : Nothing is returned.
//=====================================================================

VOID UtlLampOn ( VOID )
{
        UtlRelayOn  (RELAY_LAMP);
        UtlRelayOn  (RELAY_LAMP_START);
        g_TimeMs.LampStart = 3100;  // v1.65 - Hold Lamp Start on for 1 second, with 100ms to turn it OFF.
    
} // UtlLampOn ( )

//=====================================================================
// Function   : UtlLampOff ()                                           
// Purpose    : This function turns OFF a lamp including momentary relay.
// Parameters : Nothing is passed.
// Returns    : Nothing is returned.
//=====================================================================

VOID UtlLampOff ( VOID )
{
        UtlRelayOff (RELAY_LAMP);
        UtlRelayOff (RELAY_LAMP_START);
    
} // UtlLampOff ( )

//=====================================================================
// Function   : UtilProcessPacket ()                                           
// Purpose    : This function checks to see if a message command 
//              has been received on MAIN, EthPort, SerPort, or PRB channels.
// Parameters : Nothing is passed.
// Returns    : Nothing is returned.
//=====================================================================

#define JS_ANA_ACTIVE_LOW   120
#define JS_ANA_ACTIVE_HIGH  136

VOID _reentrant UtilProcessPacket ( tProtocolChannels *pChannel )
{
    tSER_PACKET      TxPacket;
//	static unsigned int DebugRoutedSerToPc;
//	static unsigned int DebugRoutedEthToPc;
//	static unsigned int DebugRoutedSerFromPc;
//	static unsigned int DebugRoutedEthFromPc;
static	UCHAR	toggle;    
    //
    // Accumulate a stat on type of received packet.
    //
    
    if (pChannel->ucPacketType < hbound1(uiSerRxPacketTypeCount))
    {
        ++uiSerRxPacketTypeCount[pChannel->ucPacketType];
    }
    
    //
    // 1st route received packets destined for SerPort channel.
    //

    if ( (g_BoardNumber + SLC_SERPORT_PROTOCOL_OFFSET) ==  pChannel -> ucDestinationAddress )
    {
        StdMemClr(&TxPacket, sizeof(TxPacket));
        TxPacket.Preamble.SrcAddr  = pChannel->ucSourceAddress;
        TxPacket.Preamble.DestAddr = pChannel->ucDestinationAddress; 
        TxPacket.Preamble.PacketType = pChannel->ucPacketType;
        TxPacket.Preamble.PayloadLength = pChannel->ucPayloadLength;
        StdMemCopy(TxPacket.pPayload, pChannel->ucPayload, TxPacket.Preamble.PayloadLength);
        SerTransmitPacket ( &g_SerTxQueSerPort, &TxPacket);
//		DebugRoutedSerToPc++;
        return;
    }

    //
    // 2nd route received packets destined for CME channel.
    //				  

    if ( (g_BoardNumber + SLC_ETHPORT_PROTOCOL_OFFSET) ==  pChannel -> ucDestinationAddress )
    {
        StdMemClr(&TxPacket, sizeof(TxPacket));
        TxPacket.Preamble.SrcAddr  = pChannel->ucSourceAddress;
        TxPacket.Preamble.DestAddr = pChannel->ucDestinationAddress; 
        TxPacket.Preamble.PacketType = pChannel->ucPacketType;
        TxPacket.Preamble.PayloadLength = pChannel->ucPayloadLength;
        StdMemCopy(TxPacket.pPayload, pChannel->ucPayload, TxPacket.Preamble.PayloadLength);
        SerTransmitPacket ( &g_SerTxQueEthPort, &TxPacket);
//		DebugRoutedEthToPc++;
        return;
    }

    //
    // If the packet was received on SerPort, and if 
    // destined for a board other than the current SLC then,
    // route the packet to the main channel.
    //

    if (    ( pChannel->pRxQue                      == &g_SerRxQueSerPort                  ) &&
            ( (g_BoardNumber + SLC_PROTOCOL_OFFSET) != pChannel -> ucDestinationAddress ) 
       )
    {
         StdMemClr(&g_RoutedRxSerPortPacket, sizeof(g_RoutedRxSerPortPacket));
         g_RoutedRxSerPortPacket.Preamble.SrcAddr       = g_BoardNumber + SLC_SERPORT_PROTOCOL_OFFSET;
         g_RoutedRxSerPortPacket.Preamble.DestAddr      = pChannel->ucDestinationAddress; 
         g_RoutedRxSerPortPacket.Preamble.PacketType    = pChannel->ucPacketType;
         g_RoutedRxSerPortPacket.Preamble.PayloadLength = pChannel->ucPayloadLength;
         StdMemCopy(g_RoutedRxSerPortPacket.pPayload, pChannel->ucPayload, g_RoutedRxSerPortPacket.Preamble.PayloadLength);	 // ACM!! v1.68 5 Jan 12
//         DebugRoutedSerFromPc++;
         return;
    } 

    //
    // If the packet was received on EthPort (CME), and if 
    // destined for a board other than the current SLC then,
    // route the packet to the main channel.
    //

    if (    ( pChannel->pRxQue                      == &g_SerRxQueEthPort                  ) &&
            ( (g_BoardNumber + SLC_PROTOCOL_OFFSET) != pChannel -> ucDestinationAddress ) 
       )
    {
         StdMemClr(&g_RoutedRxEthPortPacket, sizeof(g_RoutedRxEthPortPacket));
         g_RoutedRxEthPortPacket.Preamble.SrcAddr       = g_BoardNumber + SLC_ETHPORT_PROTOCOL_OFFSET;
         g_RoutedRxEthPortPacket.Preamble.DestAddr      = pChannel->ucDestinationAddress; 
         g_RoutedRxEthPortPacket.Preamble.PacketType    = pChannel->ucPacketType;
         g_RoutedRxEthPortPacket.Preamble.PayloadLength = pChannel->ucPayloadLength;
         StdMemCopy(g_RoutedRxEthPortPacket.pPayload, pChannel->ucPayload, g_RoutedRxEthPortPacket.Preamble.PayloadLength);  // ACM!! v1.68 5 Jan 12
// 		 DebugRoutedEthFromPc++;
         return;
    } 

    //
    // Accumulate statistics...
    //

    if ( pChannel->ucDestinationAddress == (g_BoardNumber + SLC_PROTOCOL_OFFSET) )
    {                                                                               
        ++(pChannel->pRxQue->Stats.uiAddrMatch);
    }                                           
    
    if ( (pChannel->ucDestinationAddress) & BROADCAST_BIT)
    {                                                                               
        ++(pChannel->pRxQue->Stats.uiAddrBroadcast);
    }                                           
    
    //
    // Accumulate RX stats if packet came from a JCS or SLC board(s)...
    //
    
    if (pChannel->ucSourceAddress < hbound1(g_RxPacketCount))
    {
        ++g_RxPacketCount[pChannel->ucSourceAddress];  // Total # packets sent to JCS
    }

    //
    // At this point, a message command has been received.
    // Figure out what kind of message it is and proceed accordingly.
    //             

	switch ( pChannel->ucPacketType ) 
    {
        case PKT_TYPE_ZEROIZE_LAMP_HOURS:
        {
            tPAYLOAD_ZEROIZE_LAMP_HOURS *pPayloadZeroizeLampHours;
            
            //
            // If this board is not intended to speak, then
            // don't plow on.
            //
            
            if ( pChannel->ucDestinationAddress != (g_BoardNumber + SLC_PROTOCOL_OFFSET) )
            {
                break;
            }
			
            g_TimeMs.PollMain = 0;
            
            pPayloadZeroizeLampHours = (tPAYLOAD_ZEROIZE_LAMP_HOURS *) pChannel->ucPayload;

            //
            // Use extra caution zapping this field.  Also,
            // do not zeroize lamp hours when the beam is ON.
            //
                        
            if ( pPayloadZeroizeLampHours->usSignature != SIGNATURE_ZEROIZE_LAMP_HOURS)
            {
                break;
            }
            
            //
            // Store Total SL time field in Flash MISC page (from SerSlcExtStatusPayload fields).
            //
            
            FlashStoreMisc ( TRUE );
      
            break;
            
        } // case PKT_TYPE_ZEROIZE_LAMP_HOURS:
        
        case PKT_TYPE_DO_CALIBRATION :
        {
            SER_PRB_CONFIG_PACKET    TxPacket;
            tPAYLOAD_DO_CALIBRATION *pDoCalibration;
            
            //
            // If this board is not intended to speak, then
            // don't plow on.
            //
            
            if ( pChannel->ucDestinationAddress != (g_BoardNumber + SLC_PROTOCOL_OFFSET) )
            {
                break;
            }

            g_TimeMs.PollMain = 0;
            
            pDoCalibration = (tPAYLOAD_DO_CALIBRATION *) pChannel->ucPayload;
            
            StdMemClr(&TxPacket, sizeof(TxPacket));
            
            TxPacket.Payload.PanRange    = pDoCalibration->usPanRange;
            TxPacket.Payload.TiltRange   = pDoCalibration->usTiltRange;
            TxPacket.Payload.FocusRange  = pDoCalibration->usFocusRange;
            TxPacket.Payload.AdcAvgs     = pDoCalibration->ucAdcAvgs;
            TxPacket.Payload.SLBaseType  = g_BaseType;
    
            TxPacket.Preamble.SrcAddr  = g_BoardNumber + SLC_PROTOCOL_OFFSET;
            TxPacket.Preamble.DestAddr = g_BoardNumber + PRB_PROTOCOL_OFFSET;
            TxPacket.Preamble.PacketType = PKT_TYPE_PRB_CONFIG;
            TxPacket.Preamble.PayloadLength = 14;
            
            SerTransmitPacket ( &g_SerTxQuePrb , &TxPacket);
    
            g_PrbSlaveSpeakPayload |= IN_CAL_MODE_BIT;       // Turn on calibration mode flag bit in PRB Slave Speak packets
           
            break;
            
        } // case PKT_TYPE_DO_CALIBRATION :
        
        case PKT_TYPE_REQUEST_SLC_MISC_SETUP :
        {
            //
            // If this board is not intended to speak, then
            // don't plow on.
            //
            
            if ( pChannel->ucDestinationAddress != (g_BoardNumber + SLC_PROTOCOL_OFFSET) )
            {
                break;
            }

            g_TimeMs.PollMain = 0;
            
            SerRequestSlcMiscSetup = pChannel->ucSourceAddress;

            break;
            
        } // case PKT_TYPE_REQUEST_SLC_MISC_SETUP :
        
        case PKT_TYPE_REQUEST_SLC_DEVICE_NAME :
        {
            //
            // If this board is not intended to speak, then
            // don't plow on.
            //
            
            if ( pChannel->ucDestinationAddress != (g_BoardNumber + SLC_PROTOCOL_OFFSET) )
            {
                break;
            }

            g_TimeMs.PollMain = 0;
            
            SerRequestSlcDevNameEtc = pChannel->ucSourceAddress;

            break;
            
        } // case PKT_TYPE_REQUEST_SLC_DEVICE_NAME : 
        
        case PKT_TYPE_SLAVE_SPEAK:
        {
            //
            // Mark master device as being the source of the last (current)
            // slave speak packet.
            //
            
            g_MasterProtocolAddress = pChannel->ucSourceAddress - SLC_PROTOCOL_OFFSET;
            
            //
            // Holdoff this SLC from assuming the master role
            // for 5 seconds, plus some board specific timing offset.
            //
            
            g_TimeMs.HoldoffMastering = 5000 + 100*g_BoardNumber;
            
            //
            // If this board is not intended to speak, then
            // don't plow on.
            //
            
            if ( pChannel->ucDestinationAddress != (g_BoardNumber + SLC_PROTOCOL_OFFSET) )
            {
                break;
            }

            //
            // If the host SLC board is #1 (protocol address zero),
            // assume the role of the master at this time by responding
            // to SLAVE_SPEAK with SLAVE_SPEAK.  This will cause
            // the current master to relinquish control.
            //
            
            if (0 == g_BoardNumber)
            {
                g_MasterProtocolAddress = 0;
                g_TimeMs.PollMain = POLL_RESPONSE_WAIT_TIME_MS;
                g_TimeMs.HoldoffMastering = 0;
                UtilSendSlaveSpeakOnMain ( g_PollTableIdx );
                 break;
            }
    
            //
            // The host SLC board is not board #1 (protocol address zero),
            // so just respond with this board's conventional response to
            // SLAVE_SPEAK.
            //
            
            UtlThisSlcSpeaksOnMain ( );

            break;
 

        } // case PKT_TYPE_SLAVE_SPEAK:
        

        case PKT_TYPE_SLC_DEVICE_NAME_ETC:
        {
            tPAYLOAD_SLC_DEVICE_NAME_ETC *pNames;
        
            //
            // If this board is not intended to receive this packet,
            // then don't store it.
            //
            
            if ( pChannel->ucDestinationAddress != (g_BoardNumber + SLC_PROTOCOL_OFFSET) )
            {
                break;
            }

           
            pNames = (tPAYLOAD_SLC_DEVICE_NAME_ETC *) pChannel->ucPayload;
            
            if (pNames->ucByte0CommandAndControl & SLC_DEVICE_NAME_ETC_B0_STORE_FLASH)
            {
                //
                // Be safe to extreme with regards to a missing NULL string terminator
                //
                
                StdMemCopy(SerSlcDeviceNameEtcPayload.szMainDeviceName, pNames->szMainDeviceName, sizeof(SerSlcDeviceNameEtcPayload.szMainDeviceName));
                StdMemCopy(SerSlcDeviceNameEtcPayload.szSerPortDeviceName, pNames->szSerPortDeviceName, sizeof(SerSlcDeviceNameEtcPayload.szSerPortDeviceName));
                StdMemCopy(SerSlcDeviceNameEtcPayload.szEthPortDeviceName, pNames->szEthPortDeviceName, sizeof(SerSlcDeviceNameEtcPayload.szEthPortDeviceName));
                SerSlcDeviceNameEtcPayload.szMainDeviceName[sizeof(SerSlcDeviceNameEtcPayload.szMainDeviceName)-1] = 0;
                SerSlcDeviceNameEtcPayload.szSerPortDeviceName[sizeof(SerSlcDeviceNameEtcPayload.szMainDeviceName)-1] = 0;
                SerSlcDeviceNameEtcPayload.szEthPortDeviceName[sizeof(SerSlcDeviceNameEtcPayload.szMainDeviceName)-1] = 0;
                FlashStoreMisc ( FALSE );
            }
            
            //
            // Quickly turn around next mastered packet.
            //
            
            g_TimeMs.PollMain = 0;

			//
			// Transmit new names out to connected JCS boards.		  // ACM!! v1.68
            //

            SerRequestSlcDevNameEtc = JCS_BROADCAST;

            break;
            
        } // case PKT_TYPE_SLC_SETUP_NAMES:
        
        case PKT_TYPE_SLC_MISC_SETUP:
        {                                       
            tPAYLOAD_SLC_MISC_SETUP *pSlcMiscSetup;
			tFlashMiscPage   FlashMisc; // mbdtest       
            //
            // If this board is not intended to receive this packet,
            // then don't store it.
            //
            
            if ( pChannel->ucDestinationAddress != (g_BoardNumber + SLC_PROTOCOL_OFFSET) )
            {
                break;
            }

            
            //
            // Process incoming packet according to its content.
            //
            
            pSlcMiscSetup = (tPAYLOAD_SLC_MISC_SETUP *) pChannel->ucPayload;
            
            if (pSlcMiscSetup->ucByte0CommandAndControl & SLC_SETUP_B0_WRITE_TO_FLASH)
            {


                //
                // Store misc data just specified to RAM values.
                //
            
                SerSlcMiscSetupPayload = *pSlcMiscSetup;
                
                //
                // Overwrite any changes to read only values.
                //

				// mbdtest	don't want to lose the offsets if only change is AUTOHOME value 
				FlashRead      (&FlashMisc, FLASH_MISC_PTR, sizeof (FlashMisc));
    			SerSlcMiscSetupPayload.usPanHomeOffset = FlashMisc.SlcMiscSetupPayload.usPanHomeOffset;
    			SerSlcMiscSetupPayload.usTiltHomeOffset = FlashMisc.SlcMiscSetupPayload.usTiltHomeOffset;
				//mbdtest

                SerSlcMiscSetupPayload.usRoFirmwareVersion = VERSION_DECIMAL;
                
                //
                // Write Misc data just updated to flash.
                //
                
                FlashStoreMisc ( FALSE );
            }

            if (pSlcMiscSetup->ucByte0CommandAndControl & SLC_SETUP_B0_SET_HOME_CURRENT_POS)
            {
			// mbdtest
				FlashRead      (&FlashMisc, FLASH_MISC_PTR, sizeof (FlashMisc));
    			SerSlcMiscSetupPayload.usAutoReturnSeconds = FlashMisc.SlcMiscSetupPayload.usAutoReturnSeconds;
			// mbdtest
		        SerSlcMiscSetupPayload.usPanHomeOffset  = g_PotReader.PanFb;
		        SerSlcMiscSetupPayload.usTiltHomeOffset = g_PotReader.TiltFb;
        		lWriteHomeToMiscPage ( );
			}

            //
            // Quickly turn around next mastered packet.
            //
            
            g_TimeMs.PollMain = 0;
            break;
        }
        
        case PKT_TYPE_SLC_BASIC_STATUS:
        case PKT_TYPE_SLC_EXTENDED_STATUS:
            //
            // Quickly turn around next mastered packet.
            //
            
            g_TimeMs.PollMain = 0;

            break;
                         
        case PKT_TYPE_DO_POSITION_SL_REL:
        {
            tPAYLOAD_DO_POSITION_SL_REL *pPolarPosPayload;
            
           
            //
            // Quickly turn around next mastered packet.
            //
            
            g_TimeMs.PollMain = 0;
            
            //
            // If the packet was not intended for then, do not process it.
            //
            
            if ( pChannel->ucDestinationAddress != (g_BoardNumber + SLC_PROTOCOL_OFFSET) )
            {
                break;
            }

       
            //
            // Stop tour if in progress.
            //
    
            if (SerSlcExtStatusPayload.BasicStatus.ucActiveTour != TOUR_INVALID)
            {
                if ( ucUtlDebugMask & DEBUG_TOUR_EXCEPTION )
                {
                        StdPrintf (UtlQueDebugPrint,  STR_NEW_LINE "TOUR STOPPED BY POLAR COORD COMMAND"); 
                }
                SerSlcExtStatusPayload.BasicStatus.ucActiveTour = TOUR_INVALID;
                UtilPolarStop ( );
            }

            //
            // Go to a specific coordinate as commanded by the master station (on AUX ports).
            //

            pPolarPosPayload = (tPAYLOAD_DO_POSITION_SL_REL *) pChannel->ucPayload;
            
			// mbdtest
            if ( pPolarPosPayload->usGrowth0 & DYN_TRACKING_MODE )
            {
				g_NewTargetDynPosition == 1;		// first time target commanded, used as a flag to clear integral accum
 				UtilGotoDynTrackingPosition(10 * (LONG) pPolarPosPayload->sAzimuth   , 
                                    		10*pPolarPosPayload->sElevation , 
                							10*pPolarPosPayload->usRate     );
			}
			else 
			{
			g_DynTrackingMode = 0; // mbdtest make sure this is cleared
			UtilGotoPolarPosition ( 10 * (LONG) pPolarPosPayload->sAzimuth   , 
                                    10*pPolarPosPayload->sElevation , 
                                    10*pPolarPosPayload->usRate     ,
                                    pPolarPosPayload->usGrowth0     );
			}
			break;
            
        } // case PKT_TYPE_DO_POSITION_SL_REL:

        case PKT_TYPE_JCS_STATUS:
        {
			UCHAR extended_jcs_packet = 0; 
            tPAYLOAD_JCS_STATUS  *pJcsStatusPayload;
            pJcsStatusPayload = ( tPAYLOAD_JCS_STATUS * ) pChannel->ucPayload;
            if ( pChannel->ucPayloadLength > 5 )
				{
				extended_jcs_packet = 1;	// alert that this is an extended JCS packet with a larger payload
				}
            //
            // Quickly turn around next mastered packet.
            //
            
            g_TimeMs.PollMain = 0; // POLL_RESPONSE_QUICK_MS;
           
            //
            // Display analog data on port?
            //
            
            if (    (ucUtlDebugMask & DEBUG_JCS_ANALOG                   ) &&
                    (ucUtlDebugDetails == (pChannel->ucSourceAddress + 1))
               )
            {
                StdPrintf (UtlQueDebugPrint,  STR_NEW_LINE "JCS: %u X:%u Y:%u Z:%u", 
                    ucUtlDebugDetails + 1, 
                    pJcsStatusPayload->ucX,
                    pJcsStatusPayload->ucY,
                    pJcsStatusPayload->ucZ);
            }
        
            //
            // If the JCS status packet was not intended for
            // this SLC, then ignore it.
            //
            
            if ( pChannel->ucDestinationAddress != (g_BoardNumber + SLC_PROTOCOL_OFFSET) )
            {
                break; // Ignore Packet
            }

           
            //
            // Note if the JCS has requested either an SLC extended status packet
            // or an SLC SETUP NAMES packet, make note of that request.
            //
            
            if (pJcsStatusPayload->ucByte3 & (  JCS_STAT_B3_REQ_SLC_EXT_STAT_PKT        |
                                                JCS_STAT_B3_REQ_SLC_DEVICE_NAME_ETC_PKT |
                                                JCS_STAT_B3_REQ_SLC_MISC_SETUP_PKT      ) )
                                                // JCS_STAT_B3_ZERIOZE_LAMP_LIFE           ) )
            {
                ucUtlJcsStatByte3 |=  pJcsStatusPayload->ucByte3 & 
                                        ( JCS_STAT_B3_REQ_SLC_EXT_STAT_PKT        | 
                                          JCS_STAT_B3_REQ_SLC_DEVICE_NAME_ETC_PKT |
                                          JCS_STAT_B3_REQ_SLC_MISC_SETUP_PKT      );
                                          // JCS_STAT_B3_ZERIOZE_LAMP_LIFE           );
                                          
            }
            
            //
            // if an AUX channel sourced the packet, then
            // store address which sourced the PKT_TYPE_JCS_STATUS 
            // so that the SLC_STATUS packets (etc) may reply specifically to that
            // device.
            //
            
            if  ( pChannel->ucSourceAddress & (SLC_SERPORT_PROTOCOL_OFFSET | SLC_ETHPORT_PROTOCOL_OFFSET )) 
            {
                ucUtlJcsStatDestAddress = pChannel->ucSourceAddress;
            }

			//
			// Changed by ACM!! per MAS!! in v1.68 of SLC firmware
			//

			if ( pChannel->ucSourceAddress & (SLC_SERPORT_PROTOCOL_OFFSET | SLC_ETHPORT_PROTOCOL_OFFSET) )
			{
				if (pJcsStatusPayload->ucByte4 & JCS_STAT_B4_INHIBIT_SLC_RESPONSE)
				{
					ucJcsInhibitSlcResponse = TRUE;				
				}
				else
				{
	 				ucJcsInhibitSlcResponse = FALSE;
				}
			}	// end v1.68 changes

            //
            // SLC: v1.60, SDK v1.17: To implement an SLC status return without JCS
            // control, ignore controls if so indicated).
            //

            if (pJcsStatusPayload->ucByte3 & JCS_STAT_B3_IGNORE_CONTROLS)
            {
                break;
            }

			//
            // If a JCS is trying to drive the SL when a non JCS device is
            // in command, then pre-empt the non JCS device.
            //
            
            if (    ((SerSlcExtStatusPayload.BasicStatus.ucControllingDevice & MASK_DEVICE_TYPE) != JCS_PROTOCOL_OFFSET ) &&  // If a non JCS is in control, and
                    ((pChannel->ucSourceAddress                              & MASK_DEVICE_TYPE) == JCS_PROTOCOL_OFFSET ) && // If a JCS requesting control, and
                    (( pJcsStatusPayload->ucX < JS_ANA_ACTIVE_LOW ) || ( pJcsStatusPayload->ucX > JS_ANA_ACTIVE_HIGH ) ||
                     ( pJcsStatusPayload->ucY < JS_ANA_ACTIVE_LOW ) || ( pJcsStatusPayload->ucY > JS_ANA_ACTIVE_HIGH ) ||
                     ( pJcsStatusPayload->ucZ < JS_ANA_ACTIVE_LOW ) || ( pJcsStatusPayload->ucZ > JS_ANA_ACTIVE_HIGH ) )
               )
            {

                if ( ucUtlDebugMask & DEBUG_TOUR_EXCEPTION )
                {
                        StdPrintf (UtlQueDebugPrint,  STR_NEW_LINE "TOUR STOPPED BY JCS: %u X:%u Y:%u Z:%u", 
                            pChannel->ucSourceAddress + 1,
                            pJcsStatusPayload->ucX,
                            pJcsStatusPayload->ucY,
                            pJcsStatusPayload->ucZ);
                }

                SerSlcExtStatusPayload.BasicStatus.ucControllingDevice = pChannel->ucSourceAddress;
                g_TimeMs.ActiveJcs = JCS_ACTIVE_TIME; 
            }
            
            //
            // If another device is active, ignore this packet.
            //
            
            if ( ( SerSlcExtStatusPayload.BasicStatus.ucControllingDevice != pChannel->ucSourceAddress) && 
                 (g_TimeMs.ActiveJcs                                                                  ) )
            {
                break; // Ignore Packet
            }
            
            //
            // If no longer being controlled, then show no active device..
            //
            
            if (0 == g_TimeMs.ActiveJcs)
            {
                SerSlcExtStatusPayload.BasicStatus.ucControllingDevice = INVALID_PROTOCOL_ADDRESS;
            }
                                                                                                                            
            //
            // If in calibration, then do not proceed with any control...
            //
            
            if ( g_PotReader.CalStatus != 0 )
            {
                break; // Ignore Packet
            }
            
            //
            // As long as a SL is controlled by the active JCS, keep refreshing
            // the controlling device and the active jcs timer.
            //

            if ( ( pJcsStatusPayload->ucX < JS_ANA_ACTIVE_LOW ) || ( pJcsStatusPayload->ucX > JS_ANA_ACTIVE_HIGH ) ||
                 ( pJcsStatusPayload->ucY < JS_ANA_ACTIVE_LOW ) || ( pJcsStatusPayload->ucY > JS_ANA_ACTIVE_HIGH ) ||
                 ( pJcsStatusPayload->ucZ < JS_ANA_ACTIVE_LOW ) || ( pJcsStatusPayload->ucZ > JS_ANA_ACTIVE_HIGH ) 
               )
            {
                SerSlcExtStatusPayload.BasicStatus.ucControllingDevice = pChannel->ucSourceAddress;
                g_TimeMs.ActiveJcs = JCS_ACTIVE_TIME; 
                usUtlAutoReturnCountdown = SerSlcMiscSetupPayload.usAutoReturnSeconds;      // Mirrored value from flash.
                g_PolarCoordMode = FALSE;
                SerSlcExtStatusPayload.BasicStatus.ucActiveTour = TOUR_INVALID;
            }
            
 //           if (!g_PolarCoordMode)
            if ( !(g_PolarCoordMode || g_DynTrackingMode) )

            {
                UtilSetAllMotorsWithLimits ( pJcsStatusPayload->ucX, pJcsStatusPayload->ucY, pJcsStatusPayload->ucZ );         
            }
                        
            if ( pJcsStatusPayload->ucZ < (128 - FOCUS_RELAY_THRESHOLD) )
            {
                SerSlcExtStatusPayload.BasicStatus.ucControllingDevice = pChannel->ucSourceAddress;
                g_TimeMs.ActiveJcs = JCS_ACTIVE_TIME; 
                UtlRelayOn ( RELAY_FOCUS_IN );
            }
            else
            {
                UtlRelayOff ( RELAY_FOCUS_IN );
            }
            
            if ( pJcsStatusPayload->ucZ > (128 + FOCUS_RELAY_THRESHOLD) )
            {
                SerSlcExtStatusPayload.BasicStatus.ucControllingDevice = pChannel->ucSourceAddress;
                g_TimeMs.ActiveJcs = JCS_ACTIVE_TIME; 
                UtlRelayOn (RELAY_FOCUS_OUT );
            }
            else
            {
                UtlRelayOff (RELAY_FOCUS_OUT);
            }
             
            if ( pJcsStatusPayload->ucByte4 & JCS_STAT_B4_LAMP_ON_START )
            {
                SerSlcExtStatusPayload.BasicStatus.ucControllingDevice = pChannel->ucSourceAddress;
                g_TimeMs.ActiveJcs = JCS_ACTIVE_TIME; 
                usUtlAutoReturnCountdown = SerSlcMiscSetupPayload.usAutoReturnSeconds;  
                
                // 
                // If controlled over AUX port, use 1 second lamp start to command lamp on.
                //
                
                if  ( pChannel->ucSourceAddress & (SLC_SERPORT_PROTOCOL_OFFSET | SLC_ETHPORT_PROTOCOL_OFFSET )) 
                {
                    UtlLampOn ();
                }
                else
                {
                    UtlRelayOn (RELAY_LAMP      );
                    UtlRelayOn (RELAY_LAMP_START);
                }
            }                
            else 
            {
                if (g_TimeMs.LampStart < 100 )          // v1.65 - Only turn off relay if it is not being controlled by tour.
                    UtlRelayOff (RELAY_LAMP_START);
            }
            
            if ( pJcsStatusPayload->ucByte4 & JCS_STAT_B4_LAMP_OFF )
            {
                SerSlcExtStatusPayload.BasicStatus.ucControllingDevice = pChannel->ucSourceAddress;
                g_TimeMs.ActiveJcs = JCS_ACTIVE_TIME; 
                usUtlAutoReturnCountdown = SerSlcMiscSetupPayload.usAutoReturnSeconds;  
                UtlRelayOff (RELAY_LAMP);
            }

            if ( pJcsStatusPayload->ucByte4 & JCS_STAT_B4_AUX1_ON )
            {
                SerSlcExtStatusPayload.BasicStatus.ucControllingDevice = pChannel->ucSourceAddress;
                g_TimeMs.ActiveJcs = JCS_ACTIVE_TIME; 
                usUtlAutoReturnCountdown = SerSlcMiscSetupPayload.usAutoReturnSeconds;  
                UtlRelayOn (RELAY_AUX1);
            }

            if ( pJcsStatusPayload->ucByte4 & JCS_STAT_B4_AUX1_OFF )
            
            {
                SerSlcExtStatusPayload.BasicStatus.ucControllingDevice = pChannel->ucSourceAddress;
                g_TimeMs.ActiveJcs = JCS_ACTIVE_TIME; 
                usUtlAutoReturnCountdown = SerSlcMiscSetupPayload.usAutoReturnSeconds;  
                UtlRelayOff (RELAY_AUX1);
            }

			if ( pJcsStatusPayload->ucByte3 & JCS_STAT_B3_AUX2_ON )
            {
                SerSlcExtStatusPayload.BasicStatus.ucControllingDevice = pChannel->ucSourceAddress;
                g_TimeMs.ActiveJcs = JCS_ACTIVE_TIME; 
                usUtlAutoReturnCountdown = SerSlcMiscSetupPayload.usAutoReturnSeconds;  
                UtlRelayOn (RELAY_AUX2);
            }

            if ( pJcsStatusPayload->ucByte3 & JCS_STAT_B3_AUX2_OFF )
            {
                SerSlcExtStatusPayload.BasicStatus.ucControllingDevice = pChannel->ucSourceAddress;
                g_TimeMs.ActiveJcs = JCS_ACTIVE_TIME; 
                usUtlAutoReturnCountdown = SerSlcMiscSetupPayload.usAutoReturnSeconds;  
                UtlRelayOff (RELAY_AUX2);
            }
            
            if ( pJcsStatusPayload->ucByte3 & JCS_STAT_B3_AUX3_ON )
            {
                SerSlcExtStatusPayload.BasicStatus.ucControllingDevice = pChannel->ucSourceAddress;
                g_TimeMs.ActiveJcs = JCS_ACTIVE_TIME; 
                usUtlAutoReturnCountdown = SerSlcMiscSetupPayload.usAutoReturnSeconds;  
                UtlRelayOn (RELAY_AUX3);
            }

            if ( pJcsStatusPayload->ucByte3 & JCS_STAT_B3_AUX3_OFF )
            {
                SerSlcExtStatusPayload.BasicStatus.ucControllingDevice = pChannel->ucSourceAddress;
                g_TimeMs.ActiveJcs = JCS_ACTIVE_TIME; 
                usUtlAutoReturnCountdown = SerSlcMiscSetupPayload.usAutoReturnSeconds;  
                UtlRelayOff (RELAY_AUX3);
            }
            
            if ( pJcsStatusPayload->ucByte4 & JCS_STAT_B4_HOME_ON )
            {
                SerSlcExtStatusPayload.BasicStatus.ucControllingDevice = pChannel->ucSourceAddress;
                g_TimeMs.ActiveJcs = JCS_ACTIVE_TIME; 
//                SerSlcExtStatusPayload.BasicStatus.ucByte7 |= SLC_STAT_B7_AT_HOME;   // Removed by ACM!! in v1.68, dynamically set if at home
                
                //
                // Go to HOME position.
                //
                
                UtilGotoHomePosition ( );
                
            }

	if (extended_jcs_packet == 1)
	{
            if ( pJcsStatusPayload->ucByte5 & JCS_STAT_B5_AUX4_OFF )
            {
                SerSlcExtStatusPayload.BasicStatus.ucControllingDevice = pChannel->ucSourceAddress;
                g_TimeMs.ActiveJcs = JCS_ACTIVE_TIME; 
                usUtlAutoReturnCountdown = SerSlcMiscSetupPayload.usAutoReturnSeconds;  
                UtlRelayOff (RELAY_AUX4);
            }

            if ( pJcsStatusPayload->ucByte5 & JCS_STAT_B5_AUX4_ON )
            {
                SerSlcExtStatusPayload.BasicStatus.ucControllingDevice = pChannel->ucSourceAddress;
                g_TimeMs.ActiveJcs = JCS_ACTIVE_TIME; 
                usUtlAutoReturnCountdown = SerSlcMiscSetupPayload.usAutoReturnSeconds;  
                UtlRelayOn (RELAY_AUX4);
            }

             if ( pJcsStatusPayload->ucByte5 & JCS_STAT_B5_AUX5_OFF )
            {
                SerSlcExtStatusPayload.BasicStatus.ucControllingDevice = pChannel->ucSourceAddress;
                g_TimeMs.ActiveJcs = JCS_ACTIVE_TIME; 
                usUtlAutoReturnCountdown = SerSlcMiscSetupPayload.usAutoReturnSeconds;  
                UtlRelayOff (AUX5);
            }

            if ( pJcsStatusPayload->ucByte5 & JCS_STAT_B5_AUX5_ON )
            {
                SerSlcExtStatusPayload.BasicStatus.ucControllingDevice = pChannel->ucSourceAddress;
                g_TimeMs.ActiveJcs = JCS_ACTIVE_TIME; 
                usUtlAutoReturnCountdown = SerSlcMiscSetupPayload.usAutoReturnSeconds;  
                UtlRelayOn (AUX5);
            }

             if ( pJcsStatusPayload->ucByte5 & JCS_STAT_B5_AUX6_OFF )
            {
                SerSlcExtStatusPayload.BasicStatus.ucControllingDevice = pChannel->ucSourceAddress;
                g_TimeMs.ActiveJcs = JCS_ACTIVE_TIME; 
                usUtlAutoReturnCountdown = SerSlcMiscSetupPayload.usAutoReturnSeconds;  
                UtlRelayOff (AUX6);
            }

            if ( pJcsStatusPayload->ucByte5 & JCS_STAT_B5_AUX6_ON )
            {
                SerSlcExtStatusPayload.BasicStatus.ucControllingDevice = pChannel->ucSourceAddress;
                g_TimeMs.ActiveJcs = JCS_ACTIVE_TIME; 
                usUtlAutoReturnCountdown = SerSlcMiscSetupPayload.usAutoReturnSeconds;  
                UtlRelayOn (AUX6);
            }

             if ( pJcsStatusPayload->ucByte5 & JCS_STAT_B5_AUX7_OFF )
            {
                SerSlcExtStatusPayload.BasicStatus.ucControllingDevice = pChannel->ucSourceAddress;
                g_TimeMs.ActiveJcs = JCS_ACTIVE_TIME; 
                usUtlAutoReturnCountdown = SerSlcMiscSetupPayload.usAutoReturnSeconds;  
                UtlRelayOff (AUX7);
            }

            if ( pJcsStatusPayload->ucByte5 & JCS_STAT_B5_AUX7_ON )
            {
                SerSlcExtStatusPayload.BasicStatus.ucControllingDevice = pChannel->ucSourceAddress;
                g_TimeMs.ActiveJcs = JCS_ACTIVE_TIME; 
                usUtlAutoReturnCountdown = SerSlcMiscSetupPayload.usAutoReturnSeconds;  
                UtlRelayOn (AUX7);
            }
            
             if ( pJcsStatusPayload->ucByte6 & JCS_STAT_B6_AUX8_OFF )
            {
                SerSlcExtStatusPayload.BasicStatus.ucControllingDevice = pChannel->ucSourceAddress;
                g_TimeMs.ActiveJcs = JCS_ACTIVE_TIME; 
                usUtlAutoReturnCountdown = SerSlcMiscSetupPayload.usAutoReturnSeconds;  
                UtlRelayOff (AUX8);
            }

            if ( pJcsStatusPayload->ucByte6 & JCS_STAT_B6_AUX8_ON )
            {
                SerSlcExtStatusPayload.BasicStatus.ucControllingDevice = pChannel->ucSourceAddress;
                g_TimeMs.ActiveJcs = JCS_ACTIVE_TIME; 
                usUtlAutoReturnCountdown = SerSlcMiscSetupPayload.usAutoReturnSeconds;  
                UtlRelayOn (AUX8);
            }

             if ( pJcsStatusPayload->ucByte6 & JCS_STAT_B6_AUX9_UP_OFF )
            {
                SerSlcExtStatusPayload.BasicStatus.ucControllingDevice = pChannel->ucSourceAddress;
                g_TimeMs.ActiveJcs = JCS_ACTIVE_TIME; 
                usUtlAutoReturnCountdown = SerSlcMiscSetupPayload.usAutoReturnSeconds;  
                UtlRelayOff (AUX9_UP);
            }

            if ( pJcsStatusPayload->ucByte6 & JCS_STAT_B6_AUX9_UP_ON )
            {
                SerSlcExtStatusPayload.BasicStatus.ucControllingDevice = pChannel->ucSourceAddress;
                g_TimeMs.ActiveJcs = JCS_ACTIVE_TIME; 
                usUtlAutoReturnCountdown = SerSlcMiscSetupPayload.usAutoReturnSeconds;  
                UtlRelayOn (AUX9_UP);
            }

             if ( pJcsStatusPayload->ucByte6 & JCS_STAT_B6_AUX9_DOWN_OFF )
            {
                SerSlcExtStatusPayload.BasicStatus.ucControllingDevice = pChannel->ucSourceAddress;
                g_TimeMs.ActiveJcs = JCS_ACTIVE_TIME; 
                usUtlAutoReturnCountdown = SerSlcMiscSetupPayload.usAutoReturnSeconds;  
                UtlRelayOff (AUX9_DOWN);
            }

            if ( pJcsStatusPayload->ucByte6 & JCS_STAT_B6_AUX9_DOWN_ON )
            {
                SerSlcExtStatusPayload.BasicStatus.ucControllingDevice = pChannel->ucSourceAddress;
                g_TimeMs.ActiveJcs = JCS_ACTIVE_TIME; 
                usUtlAutoReturnCountdown = SerSlcMiscSetupPayload.usAutoReturnSeconds;  
                UtlRelayOn (AUX9_DOWN);
            }

            if ( pJcsStatusPayload->ucByte6 & JCS_STAT_B6_AUX10_UP_OFF )
            {
                SerSlcExtStatusPayload.BasicStatus.ucControllingDevice = pChannel->ucSourceAddress;
                g_TimeMs.ActiveJcs = JCS_ACTIVE_TIME; 
                usUtlAutoReturnCountdown = SerSlcMiscSetupPayload.usAutoReturnSeconds;  
                UtlRelayOff (AUX10_UP);
            }

            if ( pJcsStatusPayload->ucByte6 & JCS_STAT_B6_AUX10_UP_ON )
            {
                SerSlcExtStatusPayload.BasicStatus.ucControllingDevice = pChannel->ucSourceAddress;
                g_TimeMs.ActiveJcs = JCS_ACTIVE_TIME; 
                usUtlAutoReturnCountdown = SerSlcMiscSetupPayload.usAutoReturnSeconds;  
                UtlRelayOn (AUX10_UP);
            }

            if ( pJcsStatusPayload->ucByte7 & JCS_STAT_B7_AUX10_DOWN_OFF )
            {
                SerSlcExtStatusPayload.BasicStatus.ucControllingDevice = pChannel->ucSourceAddress;
                g_TimeMs.ActiveJcs = JCS_ACTIVE_TIME; 
                usUtlAutoReturnCountdown = SerSlcMiscSetupPayload.usAutoReturnSeconds;  
                UtlRelayOff (AUX10_DOWN);
            }

            if ( pJcsStatusPayload->ucByte7 & JCS_STAT_B7_AUX10_DOWN_ON )
            {
                SerSlcExtStatusPayload.BasicStatus.ucControllingDevice = pChannel->ucSourceAddress;
                g_TimeMs.ActiveJcs = JCS_ACTIVE_TIME; 
                usUtlAutoReturnCountdown = SerSlcMiscSetupPayload.usAutoReturnSeconds;  
                UtlRelayOn (AUX10_DOWN);
            }
	} // if (extended_jcs_packet == 1)


            
            break;

            
        } // case PKT_TYPE_JCS_STATUS:
    
        // POT FEEDBACK INFORMATION
        case PKT_TYPE_PRB_FEEDBACK_INFO:
        {
            UCHAR  ucByte9In ;
            UCHAR  ucByte9Out;
            static UCHAR counter;
            LONG   lTempPan;
            LONG   lTempTilt;
            
            tPAYLOAD_PRB_FB_INFO *PrbFbPayload;

            //
            // Blank PRB data while lamp is starting.
            //
            
//            if (g_TimeMs.PrbBlanked)
//                return;
            
            g_TimeMs.PrbCommFault = PRB_COMM_FAULT_TIME_MS;

            //
            // Clear CAL in progress 
            //
            if (SerSlcExtStatusPayload.ucByte9 & SLC_EXT_STAT_B9_PRB_CAL_IN_PROCESS)
            {
                ucUtlJcsStatByte3 |= JCS_STAT_B3_REQ_SLC_EXT_STAT_PKT;
                SerSlcExtStatusPayload.ucByte9 &= ~SLC_EXT_STAT_B9_PRB_CAL_IN_PROCESS;
            }

            PrbFbPayload = (tPAYLOAD_PRB_FB_INFO *) pChannel->ucPayload;

            //
            // Payload string consists of 7 bytes ( 2 bytes per POT x 3 POTs ), and operation status byte
            // <--  Pan FB  --><--  Tilt FB  --><--  Focus FB  --><-- PRB Mode -->
            //  [Byte0][Byte1]  [Byte2][Byte3]    [Byte4][Byte5]       [Byte6]
            //
    
            g_PotReader.PanFb                             = PrbFbPayload->PanPosition; 
            g_PotReader.TiltFb                            = PrbFbPayload->TiltPosition; 
            g_PotReader.PanRawData                        = PrbFbPayload->PanAdc;
            g_PotReader.TiltRawData                       = PrbFbPayload->TiltAdc;

            //
            // Handle continuous rotation base differently - always +/- 180deg
            //

            if ( (SerSlcExtStatusPayload.ucByte9 & SLC_EXT_STAT_B9_PRB_CAL_IN_PROCESS) || (SerSlcExtStatusPayload.ucByte9 & SLC_EXT_STAT_B9_PRB_NOT_CALIBRATED) )		// v1.73 - send raw ADC values of not calibrated or in calibration
			{
				SerSlcExtStatusPayload.BasicStatus.sAzimuth = (SHORT) g_PotReader.PanRawData;
			}
            else if ( g_BaseType == 2 )
            {
                lTempPan = (LONG) g_PotReader.PanFb - (LONG) SerSlcMiscSetupPayload.usPanHomeOffset; 
                
                if ( lTempPan > 18000 )
                    lTempPan = lTempPan - 36000;

                if ( lTempPan < -18000 )
                    lTempPan = lTempPan + 36000; 
                
                SerSlcExtStatusPayload.BasicStatus.sAzimuth   = lTempPan / 10;
                    
            } // if ( g_BaseType == 2 )
            
            else
            {
                //
                // Protocol expects units of tenths of degree.
                //
            
                lTempPan = (LONG) g_PotReader.PanFb - (LONG) SerSlcMiscSetupPayload.usPanHomeOffset; 
                SerSlcExtStatusPayload.BasicStatus.sAzimuth   = lTempPan / 10;
            } 
        
            if ( (SerSlcExtStatusPayload.ucByte9 & SLC_EXT_STAT_B9_PRB_CAL_IN_PROCESS) || (SerSlcExtStatusPayload.ucByte9 & SLC_EXT_STAT_B9_PRB_NOT_CALIBRATED) )		// v1.73 - send raw ADC values of not calibrated or in calibration
			{
				SerSlcExtStatusPayload.BasicStatus.sElevation = (SHORT) g_PotReader.TiltRawData;
			}
			else
			{
				lTempTilt = (LONG) g_PotReader.TiltFb - (LONG) SerSlcMiscSetupPayload.usTiltHomeOffset;
	            SerSlcExtStatusPayload.BasicStatus.sElevation = lTempTilt / 10; 
    		}
    		        
            g_PotReader.FocusFb = PrbFbPayload->FocusPosition;
            
			//
			// Added by ACM!! in v1.68
			// Set or clear AT_HOME bit in SLC BASIC STATUS packet depending on whether light is "close enough" to home (within one degree)
			// Also add hysteresis between set/clear conditions
			//

			if ( SerSlcExtStatusPayload.BasicStatus.ucByte7 & SLC_STAT_B7_AT_HOME )	 // if AT_HOME status bit is set
			{
				if ( SerSlcExtStatusPayload.BasicStatus.sAzimuth   < -15 ||	   // clear AT_HOME if any axis is beyond 3.0 degrees from "absolute Home"
				 	 SerSlcExtStatusPayload.BasicStatus.sAzimuth   >  15 ||
				 	 SerSlcExtStatusPayload.BasicStatus.sElevation < -15 ||
				 	 SerSlcExtStatusPayload.BasicStatus.sElevation >  15 )
				{
					SerSlcExtStatusPayload.BasicStatus.ucByte7 &= ~SLC_STAT_B7_AT_HOME;
				}
			}
			else															   // else AT_HOME is NOT set
			{
				if ( SerSlcExtStatusPayload.BasicStatus.sAzimuth   > -5 &&	   // indicate AT_HOME if within 1.0 degrees of "absolute Home"
					 SerSlcExtStatusPayload.BasicStatus.sAzimuth   <  5 &&
					 SerSlcExtStatusPayload.BasicStatus.sElevation > -5 &&
					 SerSlcExtStatusPayload.BasicStatus.sElevation <  5 )
				{
					SerSlcExtStatusPayload.BasicStatus.ucByte7 |= SLC_STAT_B7_AT_HOME;
				}
			}
			// end of v1.68 changes
			
            //
            // Track changes in HORIZ and VERT stops.
            //

            ucByte9In = SerSlcExtStatusPayload.ucByte9 &
                        ( SLC_EXT_STAT_B9_HORIZ_STOP_1 |
                          SLC_EXT_STAT_B9_HORIZ_STOP_2 |
                          SLC_EXT_STAT_B9_VERT_STOP_1  |
                          SLC_EXT_STAT_B9_VERT_STOP_2  );

            //
            // Zeroize initial stop values 
            // 

            SerSlcExtStatusPayload.ucByte9 &= ~( SLC_EXT_STAT_B9_HORIZ_STOP_1 |
                                                 SLC_EXT_STAT_B9_HORIZ_STOP_2 |
                                                 SLC_EXT_STAT_B9_VERT_STOP_1  |
                                                 SLC_EXT_STAT_B9_VERT_STOP_2  );

            //
            // Setup Pan Hard limit 1 flag unless it is on the small base (continuous rotation).
            //

            if ( g_PotReader.PanFb < HARD_STOP && g_BaseType != 2 )
            {
                SerSlcExtStatusPayload.ucByte9 |= SLC_EXT_STAT_B9_HORIZ_STOP_1;
            }

            //
            // Setup Pan Hard limit 2 flag.
            //

            if ( g_PotReader.PanFb > g_PotReader.PanRange - HARD_STOP && g_BaseType != 2 )
            {
                SerSlcExtStatusPayload.ucByte9 |= SLC_EXT_STAT_B9_HORIZ_STOP_2;
            }
    
            // Set Tilt Hard limit 1 flag.
            //

            if ( g_PotReader.TiltFb < HARD_STOP )
            {
                SerSlcExtStatusPayload.ucByte9 |= SLC_EXT_STAT_B9_VERT_STOP_1;

            }

            //
            // Set Tilt Hard limit 2 flag.
            //

            if ( g_PotReader.TiltFb > g_PotReader.TiltRange - HARD_STOP )
            {
                SerSlcExtStatusPayload.ucByte9 |= SLC_EXT_STAT_B9_VERT_STOP_2;
            }

            //
            // If the stop status bits have changed, 
            // then set indicaiton to send extended status
            // packet...
            //

            ucByte9Out = SerSlcExtStatusPayload.ucByte9 &             //
                          ( SLC_EXT_STAT_B9_HORIZ_STOP_1 |            // Track changes in HORIZ and VERT stops.
                            SLC_EXT_STAT_B9_HORIZ_STOP_2 |            //
                            SLC_EXT_STAT_B9_VERT_STOP_1  |
                            SLC_EXT_STAT_B9_VERT_STOP_2  );

            if (ucByte9In ^ ucByte9Out)                             // Has stop data changed?
            {
                ucUtlJcsStatByte3 |= JCS_STAT_B3_REQ_SLC_EXT_STAT_PKT;
            }

            //
            // Check for change in PRB status / fault indications.
            //

            if (g_PotReader.OperationStatus != PrbFbPayload->OperationStatus)
            {
                g_PotReader.OperationStatus = PrbFbPayload->OperationStatus;    // Store latest value
                
                //
                // Clear prior info latched in SLC
                //
                
                SerSlcExtStatusPayload.ucByte10 &= ~(SLC_EXT_STAT_B10_PRB_RESET      |
                                                     SLC_EXT_STAT_B10_PRB_CONT_FAULT |
                                                     SLC_EXT_STAT_B10_PRB_POT1_FAULT |
                                                     SLC_EXT_STAT_B10_PRB_POT2_FAULT |
                                                     SLC_EXT_STAT_B10_PRB_POT3_FAULT |
                                                     SLC_EXT_STAT_B10_PRB_POT4_FAULT );

                SerSlcExtStatusPayload.ucByte9 &=  ~SLC_EXT_STAT_B9_PRB_NOT_CALIBRATED;

                //
                // Setup new info in SLC.
                //
            
                if ( g_PotReader.OperationStatus & PRB_FB_OP_STAT_PRB_RESET )
                {
                    SerSlcExtStatusPayload.ucByte10 |= SLC_EXT_STAT_B10_PRB_RESET;
                    if ( ucUtlDebugMask & DEBUG_TOUR_EXCEPTION )
                    {
                            StdPrintf (UtlQueDebugPrint,  STR_NEW_LINE "PRB RESET"); 
                    }
                }
                    
                if ( g_PotReader.OperationStatus & PRB_FB_OP_STAT_PRB_CONT_FAULT )
                    SerSlcExtStatusPayload.ucByte10 |= SLC_EXT_STAT_B10_PRB_CONT_FAULT;

                if ( g_PotReader.OperationStatus & PRB_FB_OP_STAT_NEEDS_CAL_MODE )
                    SerSlcExtStatusPayload.ucByte9 |= SLC_EXT_STAT_B9_PRB_NOT_CALIBRATED;
                    
                if ( g_PotReader.OperationStatus & PRB_FB_OP_STAT_POT1_FAULT )
                    SerSlcExtStatusPayload.ucByte10 |= SLC_EXT_STAT_B10_PRB_POT1_FAULT;
                    
                if ( g_PotReader.OperationStatus & PRB_FB_OP_STAT_POT2_FAULT )
                    SerSlcExtStatusPayload.ucByte10 |= SLC_EXT_STAT_B10_PRB_POT2_FAULT;
                    
                if ( g_PotReader.OperationStatus & PRB_FB_OP_STAT_POT3_FAULT )
                    SerSlcExtStatusPayload.ucByte10 |= SLC_EXT_STAT_B10_PRB_POT3_FAULT;
                    
                if ( g_PotReader.OperationStatus & PRB_FB_OP_STAT_POT4_FAULT )
                    SerSlcExtStatusPayload.ucByte10 |= SLC_EXT_STAT_B10_PRB_POT4_FAULT;
                    
                //
                // Indicate the the SLC should send an extended status 
                // broadcast packet.
                //
                
                ucUtlJcsStatByte3 |= JCS_STAT_B3_REQ_SLC_EXT_STAT_PKT;
                
            } // if (g_PotReader.OperationStatus != PrbFbPayload->OperationStatus)
            

            UtilPolarUpdateTargetPos ( );
			UtilDynTrackingUpdateTargetPos ();

            ++counter;
            if ( counter >= 0 )            
            {
                g_PanPos = g_PotReader.PanFb;
                g_TiltPos = g_PotReader.TiltFb;
                if ( ucUtlDebugMask & DEBUG_PRB_PRINT)
                {
                    if ( g_PolarCoordMode )
                    {
                     //   StdPrintf (pQue,  NEW_LINE "PanP:%6u TiltP:%6u PanT:%6u TiltP:%6u PolarIncr:%6u", g_PotReader.PanFb, g_PotReader.TiltFb, g_PanPolarPos, g_TiltPolarPos, g_PolarIter );
                     // StdPrintf (g_QuePrbPrint,  STR_NEW_LINE "P P:%5u P T:%5u E:%5d P:%4d I:%4d MS:%4u %4u %4d %4d", g_PanDebug, g_PanDebug2, g_PanMotor.DegreeError, g_PanMotor.PidTerms.P, g_PanMotor.PidTerms.I, g_PanMotor.Speed, g_PanMotor.LastSpeed, g_temp2, g_temp );
                     // StdPrintf (UtlQueDebugPrint,  STR_NEW_LINE "PanEnd:%5u PanP:%5u PanT:%5u TiltEnd:%5u TiltP:%5u TiltT:%5u", g_TargetAbsPosPan, g_PanDebug, g_PanDebug2, g_TargetAbsPosTilt, g_TiltDebug, g_TiltDebug2);
                     // StdPrintf (UtlQueDebugPrint,  STR_NEW_LINE "PanErr: %6d TiltErr: %6d", g_PanDebug, g_TiltDebug);
             // StdPrintf (UtlQueDebugPrint,  STR_NEW_LINE "P AP:%6u CP:%6u E:%4d P:%3d I:%3d MS:%4u %4u", g_PotReader.PanFb, (UINT) g_PanPolarPos / INCR_MULT , g_PanMotor.DegreeError, g_PanMotor.PidTerms.P, g_PanMotor.PidTerms.I, g_PanSpeedDebug, g_iMotorDebug ); //MAS!!!!!!!!!!
              
             //StdPrintf (UtlQueDebugPrint,  STR_NEW_LINE "P AP:%6u CP:%6u E:%4d P:%3d I:%3d MS:%4u %4u", g_PotReader.PanFb, (UINT) g_PanPolarPos / INCR_MULT , g_PanMotor.DegreeError, g_PanMotor.PidTerms.P, g_PanMotor.PidTerms.I, g_PanSpeedDebug, uiIterRemaining ); //MAS!!!!!!!!!!
             //StdPrintf (UtlQueDebugPrint,  STR_NEW_LINE "State:%u S1:%x S2:%x S3:%x S4:%x S5:%x", debugState0,debugState1,debugState2,debugState3,debugState4,debugState5 ); //MAS!!!!!!!!!!
   
                     //   StdPrintf (g_QuePrbPrint,  STR_NEW_LINE "T P:%5u T:%5u E:%4d P:%3d I:%3d MS:%4u %4u", g_PotReader.TiltFb, g_TiltDebug, g_TiltMotor.DegreeError, g_TiltMotor.PidTerms.P, g_TiltMotor.PidTerms.I, g_TiltMotor.Speed, g_TiltMotor.LastSpeed );
                     //   StdPrintf (pQue,  NEW_LINE "TiltP:%6u TiltT:%6u Err:%6d P:%6d I:%6d D:%6d MS:%6u", g_PotReader.TiltFb, (UINT) g_TiltPolarPos, g_TiltMotor.DegreeError, g_TiltMotor.PidTerms.P, g_TiltMotor.PidTerms.I, g_TiltMotor.PidTerms.D , g_TiltMotor.Speed );
                     
                     //   StdPrintf (UtlQueDebugPrint,  STR_NEW_LINE "P:%6d RP:%5d RPC:%5d T:%6d RT:%5d RTC:%5d", (INT) lTempPan, g_PotReader.PanRawData, g_PotReader.PanRawData - g_RPanLast, (INT) lTempTilt, g_PotReader.TiltRawData, g_PotReader.TiltRawData - g_RTiltLast);
                     // StdPrintf (UtlQueDebugPrint,  STR_NEW_LINE "PC: %4d PE: %4d PTerm: %2d PS: %4d TC: %4d TE: %4d PTerm: %2d TS: %4d", (INT) g_PanPos - (INT) g_LastPanPos, g_PanMotor.DegreeError, g_PanMotor.PidTerms.P, g_PanMotor.Speed, g_TiltPos - g_LastTiltPos, g_TiltMotor.DegreeError, g_TiltMotor.PidTerms.P, g_TiltMotor.Speed);
                     // StdPrintf (UtlQueDebugPrint,  STR_NEW_LINE "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d", (INT) g_PanPos, (INT) lTempPan, (INT) g_PanPos - (INT) g_LastPanPos, g_PanMotor.DegreeError, g_PanMotor.PidTerms.P, g_PanMotor.Speed, g_TiltPos, g_TiltPos - g_LastTiltPos, g_TiltMotor.DegreeError, g_TiltMotor.PidTerms.P, g_TiltMotor.Speed);
                     //   StdPrintf (UtlQueDebugPrint,  STR_NEW_LINE "%d,%d", (INT) lTempPan, (INT) lTempTilt );
                     //   StdPrintf (UtlQueDebugPrint,  STR_NEW_LINE "%d,%u,    %d,%u", (INT) lTempPan, g_PotReader.PanFb, (INT) lTempTilt, g_PotReader.TiltFb );
                    }
                    else
                    {
                    //    StdPrintf (UtlQueDebugPrint,  STR_NEW_LINE "P:%6d RP:%5d RPC:%5d T:%6d RT:%5d RTC:%5d", (INT) lTempPan, g_PotReader.PanRawData, g_PotReader.PanRawData - g_RPanLast, (INT) lTempTilt, g_PotReader.TiltRawData, g_PotReader.TiltRawData - g_RTiltLast);
                    //    StdPrintf (UtlQueDebugPrint,  STR_NEW_LINE "P:%6d PC:%4d RP:%5d T:%6d TC:%4d RT:%5d", (INT) lTempPan, g_PanPos - g_LastPanPos, g_PotReader.PanRawData, (INT) lTempTilt, g_TiltPos - g_LastTiltPos, g_PotReader.TiltRawData);
                    //    StdPrintf (UtlQueDebugPrint,  STR_NEW_LINE "P:%6d T:%6d", (INT) lTempPan, (INT) lTempTilt );
                        StdPrintf (UtlQueDebugPrint,  STR_NEW_LINE "%d,%u,    %d,%u", (INT) lTempPan, g_PotReader.PanFb, (INT) lTempTilt, g_PotReader.TiltFb );
                    }
                }
                g_LastPanPos = g_PanPos;
                g_LastTiltPos = g_TiltPos;
                g_RPanLast = g_PotReader.PanRawData;
                g_RTiltLast = g_PotReader.TiltRawData;

                counter = 0;
            } // if ( counter >= 10 )

            break;

	    } // case PRB_FEEDBACK_INFO 
    
        // PRB Calibration Feedback
        case PKT_TYPE_PRB_CAL_FEEDBACK:
        {
            tPAYLOAD_PRB_CAL_FEEDBACK *PrbCalFbPayload;
            
            //
            // Track when PRB communicates in order to drive PRB COM fault
            // when comm quits.
            //

            g_TimeMs.PrbCommFault = PRB_COMM_FAULT_TIME_MS;

            //
            // Put indication of PRB calibration status in SLC Extended Status packet
            // Note: Extended Stat packets always sent when in CAL.
            // 

            ucUtlJcsStatByte3 |= JCS_STAT_B3_REQ_SLC_EXT_STAT_PKT;
            SerSlcExtStatusPayload.ucByte9 |= SLC_EXT_STAT_B9_PRB_CAL_IN_PROCESS;

            PrbCalFbPayload = (tPAYLOAD_PRB_CAL_FEEDBACK *) pChannel->ucPayload;

            //
            // Payload string consists of 12 bytes ( 2 bytes per ADC x 4 channels ), motor speeds, and calibration status byte
            // <--  Pan FB  --><-- Pan 2 FB --><--  Tilt FB  --><--  Focus FB  --><-- Pan Speed --><-- Tilt Speed --><-- Focus Speed --><-- Cal Mode -->
            //  [Byte0][Byte1]  [Byte2][Byte3]   [Byte4][Byte5]    [Byte6][Byte7]       [Byte8]          [Byte9]           [Byte10]         [Byte11]
            //
    
            // 
            // If calibration has not finished (CalProgress == 0), set motor speed based on packet data
            //

            
            UtilSetMotorSpeed ( PAN_MOTOR, PrbCalFbPayload->PanSpeed );
            UtilSetMotorSpeed ( TILT_MOTOR, PrbCalFbPayload->TiltSpeed );
            UtilSetMotorSpeed ( FOCUS_MOTOR, PrbCalFbPayload->FocusSpeed );
            
            if ( !PrbCalFbPayload->CalProgress )
            {
                g_PrbSlaveSpeakPayload &= ~IN_CAL_MODE_BIT;
            }

            g_PotReader.CalStatus = PrbCalFbPayload->CalProgress;

            // v1.73 - Update position being sent to JCS so raw values will display
			SerSlcExtStatusPayload.BasicStatus.sAzimuth = (SHORT) PrbCalFbPayload->Pan1Adc;
			if(g_BaseType == DIP_SLEEK_BASE)
			{
				SerSlcExtStatusPayload.BasicStatus.sElevation = (SHORT) PrbCalFbPayload->TiltAdc;
			}
			else
			{	// Pan2 for sleekbase (second pot on PRB) is used for tilt on medium and big bases
				SerSlcExtStatusPayload.BasicStatus.sElevation = (SHORT) PrbCalFbPayload->Pan2Adc;
            }

            if ( ucUtlDebugMask & DEBUG_PRB_PRINT)
            {
//                StdPrintf (UtlQueDebugPrint,  STR_NEW_LINE "ADC0 %5u %5u %5u %5u CalState: %2u", PrbCalFbPayload->Pan1Adc, PrbCalFbPayload->Pan2Adc, PrbCalFbPayload->TiltAdc, PrbCalFbPayload->FocusAdc, g_PotReader.CalStatus );
                StdPrintf (UtlQueDebugPrint,  STR_NEW_LINE "%5u,%5u,%5u,%5u", PrbCalFbPayload->Pan1Adc, PrbCalFbPayload->Pan2Adc, PrbCalFbPayload->TiltAdc, PrbCalFbPayload->FocusAdc);
            }

            break;
        } // case PRB_CAL_FEEDBACK

        // PRB Configuration Data
        case PKT_TYPE_PRB_CONFIG:
        {
            tPAYLOAD_PRB_CONFIG *PrbConfigPayload;

       
            g_TimeMs.PrbCommFault = PRB_COMM_FAULT_TIME_MS;
        
            PrbConfigPayload = (tPAYLOAD_PRB_CONFIG *) pChannel->ucPayload;

            // NEEDS UPDATING
            // Payload string consists of 12 bytes ( 2 bytes per ADC x 4 channels ), motor speeds, and calibration status byte
            // <--  Pan FB  --><-- Pan 2 FB --><--  Tilt FB  --><--  Focus FB  --><-- Pan Speed --><-- Tilt Speed --><-- Focus Speed --><-- Cal Mode -->
            //  [Byte0][Byte1]  [Byte2][Byte3]   [Byte4][Byte5]    [Byte6][Byte7]       [Byte8]          [Byte9]           [Byte10]         [Byte11]
            //
    
            g_PotReader.PanRange    = PrbConfigPayload->PanRange;
            g_PotReader.TiltRange   = PrbConfigPayload->TiltRange;
            g_PotReader.FocusRange  = PrbConfigPayload->FocusRange;

            g_PotReader.PanMaxRate      = PrbConfigPayload->PanMaxRate;
            g_PotReader.TiltMaxRate     = PrbConfigPayload->TiltMaxRate;
            g_PotReader.FocusMaxRate    = PrbConfigPayload->FocusMaxRate;

            g_PrbSlaveSpeakPayload &= ~SLC_POWER_FAILED_BIT;
            
            break;
            
        } // case PRB_CONFIG:
        
    } // switch ( PacketType ) 

} // UtilProcessPacket ( ) 

//=====================================================================
// Function   : UtilCheckProtocolChannels ()                                           
// Purpose    : This function checks to see if a message command 
//              has been received on MAIN, , or PRB channels.
// Parameters : Nothing is passed.
// Returns    : Nothing is returned.
//=====================================================================

_reentrant VOID UtilCheckProtocolChannels ( VOID )
{
    UCHAR i;
    tProtocolChannels *pChannel;
    static UCHAR ucPelcoByteCount;
    
    pChannel = sProtocolChannels;
    
    
    for (i = 0; i < StdHbound1(sProtocolChannels); i++)
    {
        UCHAR c;
          
       //
       // If an RX timeout is observed on a protocol channel,
       // zap the buffer and slam the protocol state to 
       // ground zero (EXPECTING_PACKET_TYPE).
       //
         if (1 == *pChannel->pRxQue->puiTimeout)
        {
         pChannel->pRxQue->ucStateProtocol = EXPECTING_STX;
         *pChannel->pRxQue->puiTimeout = 0;
        }
            
        //
        // Pull any data, and parse it according to the rules
        // outlined in "Packet Parsing Technique" section of
        // the CSS Theory of Operations Manual.
        //
  
        while( SerQueInspect (pChannel->pRxQue, &c) )
        {
//        TestBit1 = 1;	   // mbd need test points with Rev H SLC
            //
            // =====================================================
            // When expecting STX, as long as STX's are being received
            // keep consuming them.  When STX is no longer seen, 
            // store STX and advance to  next channel parsing state.  
            // =====================================================
            //
            
            if (EXPECTING_STX == pChannel->pRxQue->ucStateProtocol)
            {
                if (STX == c) 
                {
                    pChannel->ucCalculatedCheckSum = 0;
                    pChannel->pRxQue->ucStateProtocol = EXPECTING_PACKET_TYPE;
                 
                    continue;
                }
                
                //
                // When not "in packet" on a TTY capable channel, 
                // then process as a TTY device...
                //

                if (pChannel->bTtyChannel)
                {
                    if(0xFF == c)		  // 0xFF is always first byte of Pelco D command, it is consumed here and Pelco "Address" byte is processed next pass through "while" loop
					{
						pChannel->ucCalculatedCheckSum = 0;			// Pelco also has a checksum, use existing variable
						ucPelcoByteCount = 1;						// this is the first byte of the 7-byte Pelco D command
                    	pChannel->pRxQue->ucStateProtocol = PELCO_COMMAND;	
					}
					else
					{
                    	UtilProcessTty (pChannel, c );
					}
                    continue;
                }
                
                // StdPrintf(&g_SerTxQueSerPort, "<%02x>", c);
                ++(pChannel->pRxQue->Stats.uiErrorStx);                 // Tag it as being an STX error
                continue;        
                
            } //  if ( (EXPECTING_STX == pChannel->pRxQue->ucStateProtocol) || ... 
            
            //
            // =====================================================
            // When expecting packet type, simply
            // advance to expecting payload length.
            // =====================================================
            //
            if (EXPECTING_PACKET_TYPE  == pChannel->pRxQue->ucStateProtocol) 
            {
                //
                // Validate packet type ..
                //
                
                if ( ( c < FIRST_PKT_TYPE ) &&
                     ( c > LAST_PKT_TYPE  ) )
                {
                    //
                    // Consume what should have been the packet type byte and
                    // resynchronize...
                    //
                    
                    SerQueRemove (pChannel->pRxQue, &c);                    // Consume a single byte
                    pChannel->pRxQue->ucStateProtocol = EXPECTING_STX;  // Resynchronize
                    ++(pChannel->pRxQue->Stats.uiErrorEtx);                 // Tag it as being an STX error
                    continue;
                }
                
                pChannel->ucCalculatedCheckSum += c;
                pChannel->ucPacketType = c;
                pChannel->pRxQue->ucStateProtocol = EXPECTING_LENGTH;
                continue;
                
            } // if (EXPECTING_PACKET_TYPE  == pChannel->pRxQue->ucStateProtocol) 
            
            //
            // =====================================================
            // When expecting payload length, store payloand length
            // and if length is zero, then advance to expecting
            // checksum state.  Otherwise (length is non zero), 
            // in which case advance to expecting payload state.
            // =====================================================
            //
            if (EXPECTING_LENGTH == pChannel->pRxQue->ucStateProtocol) 
            {
                pChannel->ucCalculatedCheckSum += c;

                //
                // Protect against buffer overruns...
                //
                // TBD: Compare payload length using packet type.
                //

                if ( c >= sizeof(pChannel->ucPayload))
                {
                    //
                    // Consume a single byte and resynchronize...
                    //
                    
                    SerQueRemove (pChannel->pRxQue, &c);
                    pChannel->pRxQue->ucStateProtocol = EXPECTING_STX;
                    ++(pChannel->pRxQue->Stats.uiErrorChecksumOrLength);
                    continue;
                }

                pChannel->ucPayloadLength = c;
                pChannel->ucPayloadIdx = 0;
                pChannel->pRxQue->ucStateProtocol = EXPECTING_SRC_ADDR;
                continue;
            }
            
            //
            // =====================================================
            // When expecting SRC address, store source address
            // and advance to next state (expecting destination
            // address).
            // =====================================================
            //
            
            if (EXPECTING_SRC_ADDR == pChannel->pRxQue->ucStateProtocol) 
            {
                pChannel->ucCalculatedCheckSum += c;
                pChannel->ucSourceAddress = c;
                pChannel->pRxQue->ucStateProtocol = EXPECTING_DEST_ADDR;
                continue;
            }
            
            //
            // =====================================================
            // When expecting destination address, store destionation
            // address and advance to expecting packet type.
            // =====================================================
            //
            
            if (EXPECTING_DEST_ADDR == pChannel->pRxQue->ucStateProtocol) 
            {
                pChannel->ucCalculatedCheckSum += c;
                pChannel->ucDestinationAddress = c;
                pChannel->pRxQue->ucStateProtocol = EXPECTING_PAYLOAD;
                
                if (pChannel->ucPayloadLength == 0) 
                {
                    pChannel->pRxQue->ucStateProtocol = EXPECTING_CHECKSUM;
                }
                
                continue;
            }
            
            //
            // =====================================================
            // When expecting payload, store each byte of payload
            // until ucPayloadIdx == ucPayloadLength,
            // then transition to expecting checksum state.
            // =====================================================
            //
            if (EXPECTING_PAYLOAD      == pChannel->pRxQue->ucStateProtocol) 
            {
                pChannel->ucCalculatedCheckSum += c;
                pChannel->ucPayload[pChannel->ucPayloadIdx++] = c;
                if (pChannel->ucPayloadIdx >= pChannel->ucPayloadLength)
                {
                    pChannel->pRxQue->ucStateProtocol = EXPECTING_CHECKSUM;
                    continue;
                }
            }
            
            //
            // =====================================================
            // When expecting checksup, store and validate the 
            // check sum received against the calculated checksum.
            // If the checksum checks out, then transition to expecting
            // the ETX.  Otherwise (checksum does not checkout), then
            // record the error, and transtion to expecting STX.
            // =====================================================
            // 
            
            if (EXPECTING_CHECKSUM     == pChannel->pRxQue->ucStateProtocol) 
            {
                //
                // Do checksum's NOT match?
                //
                if (pChannel->ucCalculatedCheckSum != c)
                {
                    //
                    // Checksum do not match, consume a single byte and resynchronize...
                    //
                    
                    SerQueRemove (pChannel->pRxQue, &c);
                    pChannel->pRxQue->ucStateProtocol = EXPECTING_STX;
                    ++(pChannel->pRxQue->Stats.uiErrorChecksumOrLength);
                    continue;
                }
                
                pChannel->pRxQue->ucStateProtocol = EXPECTING_ETX;
                continue;
            }
            
            if (EXPECTING_ETX  == pChannel->pRxQue->ucStateProtocol) 
            {
                //
                // Is ETX missing?
                //
                
                if (c != ETX)
                {
                    //
                    // ETX is missing, consume a single byte and resynchronize...
                    //
                    
                    SerQueRemove (pChannel->pRxQue, &c);
                    pChannel->pRxQue->ucStateProtocol = EXPECTING_STX;
                    ++(pChannel->pRxQue->Stats.uiErrorEtx);

                    
                    if (pChannel->pRxQue->Stats.uiErrorEtx > 100)   // BUGBUG: Debug logic 8/22/08 PRB issue.
                    {                                               // BUGBUG: Debug logic 8/22/08 PRB issue.
                        pChannel->pRxQue->Stats.uiErrorEtx = 100;   // BUGBUG: Debug logic 8/22/08 PRB issue.
                    }                                               // BUGBUG: Debug logic 8/22/08 PRB issue.
                    continue;
                } 
                
                //
                // Synchronize RX que to inspection point
                //
                
                SerQueSyncToInspectionPoint (pChannel->pRxQue);

                //
                // When an RX packet is seen on the main channel, zeroize that
                // device's entry in the polling table (as indexed by protocol index).
                // Note that g_PollTable is protected against data structure overrun.
                //
    
                if ( (pChannel->pRxQue == &g_SerRxQueMain) &&
                     (pChannel->ucSourceAddress < hbound1(g_PollTable)) )
                {
                    g_PollTable[pChannel->ucSourceAddress] = 0;
                    // StdPrintf(&g_SerTxQueSerPort, "<R%02x>", pChannel->ucSourceAddress);
                }
                ++(pChannel->pRxQue->Stats.uiNumPacketsOk);
                
                //
                // When an RX packet is seen on SerPort, setup Src Address 
                // to reflect SerPort.
                //
                
                if (pChannel->pRxQue == &g_SerRxQueSerPort)
                {
                    pChannel->ucSourceAddress = g_BoardNumber | SLC_SERPORT_PROTOCOL_OFFSET;
                }
                
                //
                // When an RX packet is seen on EthPort (CME port), setup Src Address 
                // to reflect EthPort (CME).
                //
                
                if (pChannel->pRxQue == &g_SerRxQueEthPort)
                {
                    pChannel->ucSourceAddress = g_BoardNumber | SLC_ETHPORT_PROTOCOL_OFFSET;
                }
              
                UtilProcessPacket ( pChannel );
                
                pChannel->pRxQue->ucStateProtocol = EXPECTING_STX;
                
            }
         
	 		if (PELCO_COMMAND == pChannel->pRxQue->ucStateProtocol)
			{
				ucPelcoByteCount++;
				
				if(ucPelcoByteCount == 2)
				{
					pChannel->ucDestinationAddress = c;		// store Pelco destination address
					pChannel->ucCalculatedCheckSum += c;
					pChannel->ucPayloadIdx = 0;				// set up for loading payload buffer                
				}
				else if(ucPelcoByteCount > 2 && ucPelcoByteCount < 7)
				{
					pChannel->ucPayload[pChannel->ucPayloadIdx++] = c;
					pChannel->ucCalculatedCheckSum += c;
                }
				else if(ucPelcoByteCount == 7)	// final byte is checksum
				{
					if(pChannel->ucCalculatedCheckSum == c)	// only process command if calculated checksum matches received checksum
					{
						PelcoProcessCommand( pChannel );
					}
					pChannel->pRxQue->ucStateProtocol = EXPECTING_STX;             
				}
				continue;
			}
        } // while ( !SerQueEmpty(pChannel->pRxQue ))
       
        ++pChannel;
    
    } // for (i = 0; i < StdHbound1(sProtocolChannels); i++)

} // UtilCheckProtocolChannels ()

//=====================================================================
// Function   : UtilCheckPotReaderBoard ()                                           
// Purpose    : This function checks the state of the Pot Reader Board.
// Parameters : Nothing is passed.
// Returns    : Nothing is returned.
//=====================================================================

VOID UtilCheckPotReaderBoard ( VOID )
{
    //
    // Perform task depending on the current mode of the PRB.
    //
    
    switch ( g_PotReader.OperationStatus )
    {
        
        // POT READER BOARD CONFIG MODE
        case NORMAL_MODE:

            break;
        
        case CAL_MODE:

            break;

    } // switch ( g_PotReader.Mode )

} // UtilCheckPotReaderBoard ( )

//=====================================================================
// Function   : UtilPollPrb ()                                           
// Purpose    : This function sends a slave speak packet to the PRB.
// Parameters : Nothing is passed.
// Returns    : Nothing is returned.
//=====================================================================

_reentrant VOID UtilPollPrb ( VOID )
{
    tSER_PACKET TxPacket;
    UCHAR ucByte10In;
    UCHAR ucByte10Out;

    StdMemClr(&TxPacket, sizeof(TxPacket));
    TxPacket.Preamble.SrcAddr  = g_BoardNumber + SLC_PROTOCOL_OFFSET;  
    TxPacket.Preamble.DestAddr = g_BoardNumber + PRB_PROTOCOL_OFFSET;
    TxPacket.Preamble.PacketType = PKT_TYPE_PRB_SLAVE_SPEAK;
    TxPacket.Preamble.PayloadLength = 1;

    TxPacket.pPayload[0] = g_PrbSlaveSpeakPayload;
    
    SerTransmitPacket ( &g_SerTxQuePrb , &TxPacket);
    
    //
    // If not a Smartview 1 light, Drive or clear a PRB COMM fault as appropriate...
    //
    
    if (!g_SmartView1)
    {
        ucByte10In = SerSlcExtStatusPayload.ucByte10 & SLC_EXT_STAT_B10_PRB_COMM_FAULT;
        if (!g_TimeMs.PrbCommFault)		// Set PRB_COMM_FAULT flag if timer reaches 0 next time the PRB slave speak packet is sent (timer is reset to 2 seconds when packet is received from PRB)
        {
            SerSlcExtStatusPayload.ucByte10 |= SLC_EXT_STAT_B10_PRB_COMM_FAULT;
        } 
        else
        {
            SerSlcExtStatusPayload.ucByte10 &= ~SLC_EXT_STAT_B10_PRB_COMM_FAULT;
        }
    }
    
    //
    // If comm status has changed, then set flag
    // to send extended status packet.
    //
    
    ucByte10Out = SerSlcExtStatusPayload.ucByte10 & SLC_EXT_STAT_B10_PRB_COMM_FAULT;
    if (ucByte10In ^ ucByte10Out)
    {
        ucUtlJcsStatByte3 |= JCS_STAT_B3_REQ_SLC_EXT_STAT_PKT;
    }
    
} // UtilPollPrb ( )

//=====================================================================
// Function   : UtlThisSlcSpeaksOnMain ( )
// Purpose    : This function sends the most important packet from 
//              this SLC.
// Parameters : Nothing is passed.
// Returns    : Nothing is returned.
//=====================================================================

static VOID _reentrant UtlThisSlcSpeaksOnMain ( VOID )
{
    tSER_PACKET      TxPacket;
    
    ++g_TxPacketCount[g_BoardNumber + SLC_PROTOCOL_OFFSET];
    
    //
    // Is there a routed SerPort packet to transmit?
    //

    if (g_RoutedRxSerPortPacket.Preamble.SrcAddr != INVALID_PROTOCOL_ADDRESS)            
    {
        SerTransmitPacket ( &g_SerTxQueMain , &g_RoutedRxSerPortPacket);
        g_RoutedRxSerPortPacket.Preamble.SrcAddr = INVALID_PROTOCOL_ADDRESS;           // Clear SrcAddr which acts as flag
        return;
    }

    //
    // Is there a routed EthPort packet to transmit?
    //

    if (g_RoutedRxEthPortPacket.Preamble.SrcAddr != INVALID_PROTOCOL_ADDRESS)            
    {
        SerTransmitPacket ( &g_SerTxQueMain , &g_RoutedRxEthPortPacket);
        g_RoutedRxEthPortPacket.Preamble.SrcAddr = INVALID_PROTOCOL_ADDRESS;           // Clear SrcAddr which acts as flag
        return;
    }

    //
    // There is no routed packet, respond
    // with either:
    //          1) extended SLC status broadcast to all device(s), or
    //          2) SLC_SETUP packet, or
    //          3) PKT_TYPE_SLC_BASIC_STATUS, or
    // 
    //
    
    TxPacket.Preamble.SrcAddr        = g_BoardNumber + SLC_PROTOCOL_OFFSET;
    TxPacket.Preamble.DestAddr       = JCS_BROADCAST;
    
    // ====================================================
    // Respond with SLC EXTENDED STATUS packet if requested.
    // ====================================================
    
    if (ucUtlJcsStatByte3 & JCS_STAT_B3_REQ_SLC_EXT_STAT_PKT )
    {
        //
        // Populate LampTime fields...
        // 
        
        FlashRetrieveLampTime   ( &SerSlcExtStatusPayload.usLampHours,
                                  &SerSlcExtStatusPayload.ucLampMinutes,
                                  &SerSlcExtStatusPayload.ucLampSeconds);
                                  
        //
        // Populate total SL time fields...
        //
        
        FlashRetrieveSlTotalTime( &SerSlcExtStatusPayload.usRoTotalSlHours,
                                  &SerSlcExtStatusPayload.ucRoTotalSlMinutes,
                                  &SerSlcExtStatusPayload.ucRoTotalSlSeconds);
                                      
        //
        // Accumulate and adjust total SL time with current lamp time
        //
        
        SerSlcExtStatusPayload.ucRoTotalSlSeconds += SerSlcExtStatusPayload.ucLampSeconds;
        if (SerSlcExtStatusPayload.ucRoTotalSlSeconds >= 60)
        {
            ++SerSlcExtStatusPayload.ucRoTotalSlMinutes;
            SerSlcExtStatusPayload.ucRoTotalSlSeconds -= 60;
        }
        SerSlcExtStatusPayload.ucRoTotalSlMinutes += SerSlcExtStatusPayload.ucLampMinutes;
        if (SerSlcExtStatusPayload.ucRoTotalSlMinutes >= 60)
        {
            ++SerSlcExtStatusPayload.usRoTotalSlHours;
            SerSlcExtStatusPayload.ucRoTotalSlMinutes -= 60;
        }
        SerSlcExtStatusPayload.usRoTotalSlHours += SerSlcExtStatusPayload.usLampHours;

        //
        // Move the Payload to the TX packet...
        //
            
        StdMemCopy((UCHAR *) TxPacket.pPayload, (UCHAR *) &SerSlcExtStatusPayload, sizeof(SerSlcExtStatusPayload) );
        TxPacket.Preamble.PacketType     = PKT_TYPE_SLC_EXTENDED_STATUS;
        TxPacket.Preamble.PayloadLength  = sizeof(SerSlcExtStatusPayload);
        
        if  ( ucUtlJcsStatDestAddress != INVALID_PROTOCOL_ADDRESS )
        {
            //
			// changed by ACM!! per MAS!! in v1.68
			// The goal is to modify the protocol to allow and AUX device to inhibit response SLC BASIC STATUS 
			// packet to allow higher update rate of joystick control while only sending position feedback at 
			// a fraction of that rate. MAS was seeing problems with reading out of TCP/IP RX buffer at a high
			// enough rate with a single threaded application, while also providing responsive joystick feel 
			// through a PC-based GUI.
	        //

			TxPacket.Preamble.DestAddr = ucUtlJcsStatDestAddress;
            ucUtlJcsStatDestAddress = INVALID_PROTOCOL_ADDRESS;

            if (ucJcsInhibitSlcResponse)   // if the device connected to an AUX port does not want a status packet, don't send one
			{
			      	return; //MAS!!	 #1
			}
        }
        
        //
        // A variety of status indications are cleared
        // upon sending extend status.
        //
        
        ucUtlJcsStatByte3               &= ~JCS_STAT_B3_REQ_SLC_EXT_STAT_PKT;
        SerSlcExtStatusPayload.ucByte10 &= ~( SLC_EXT_STAT_B10_SLC_RESET |
                                              SLC_EXT_STAT_B10_PRB_RESET );
                                              
    } // if (ucUtlJcsStatByte3 & JCS_STAT_B3_REQ_SLC_EXT_STAT_PKT )
    
    // ======================================================
    // Respond with PKT_TYPE_SLC_DEVICE_NAME_ETC if requested
    // ======================================================
    
    else if ( ( ucUtlJcsStatByte3       & JCS_STAT_B3_REQ_SLC_DEVICE_NAME_ETC_PKT) ||
              ( SerRequestSlcDevNameEtc != INVALID_PROTOCOL_ADDRESS              ) )
    {
        //
        // Note: If the setup packet was requested from an SerPort channel,
        //       or CME device, then route this packet specifically to
        //       that device.  Otherwise, broadcast the response
        //
        
        if ( SerRequestSlcDevNameEtc != INVALID_PROTOCOL_ADDRESS)
        {
            TxPacket.Preamble.DestAddr = SerRequestSlcDevNameEtc;
            SerRequestSlcDevNameEtc = INVALID_PROTOCOL_ADDRESS;
            
        } else if ( ucUtlJcsStatDestAddress != INVALID_PROTOCOL_ADDRESS)
        {                                                                            
            TxPacket.Preamble.DestAddr = ucUtlJcsStatDestAddress;
            ucUtlJcsStatDestAddress = INVALID_PROTOCOL_ADDRESS;
        }
        
        StdMemCopy((UCHAR *) TxPacket.pPayload, (UCHAR *) &SerSlcDeviceNameEtcPayload, sizeof(SerSlcDeviceNameEtcPayload) );
        TxPacket.Preamble.PacketType     = PKT_TYPE_SLC_DEVICE_NAME_ETC; 
        TxPacket.Preamble.PayloadLength  = sizeof(SerSlcDeviceNameEtcPayload);
        
        //
        // Clear indications of request for SLC SETUP packet
        //
        
        ucUtlJcsStatByte3 &= ~JCS_STAT_B3_REQ_SLC_DEVICE_NAME_ETC_PKT;
        SerRequestSlcDevNameEtc = INVALID_PROTOCOL_ADDRESS;
    }
    
    // ===================================================
    // Respond with SLC_MISC_SETUP packet if requested
    // ===================================================
    
    else if (   (ucUtlJcsStatByte3       & JCS_STAT_B3_REQ_SLC_MISC_SETUP_PKT) ||
                ( SerRequestSlcMiscSetup != INVALID_PROTOCOL_ADDRESS             ) )
    {
        //
        // Note: If the setup packet was requested from an SerPort channel,
        //       or CME device, then route this packet specifically to
        //       that device.  Otherwise, broadcast the response
        //
        
        if ( SerRequestSlcMiscSetup != INVALID_PROTOCOL_ADDRESS)
           
        {
            TxPacket.Preamble.DestAddr = SerRequestSlcMiscSetup;
        }
                                                                            
        if (    (ucUtlJcsStatByte3       & JCS_STAT_B3_REQ_SLC_MISC_SETUP_PKT               ) &&
                (ucUtlJcsStatDestAddress & (SLC_SERPORT_PROTOCOL_OFFSET | SLC_ETHPORT_PROTOCOL_OFFSET )) 
           )
        {
            TxPacket.Preamble.DestAddr = ucUtlJcsStatDestAddress;
        }
        
        StdMemCopy((UCHAR *) TxPacket.pPayload, (UCHAR *) &SerSlcMiscSetupPayload, sizeof(SerSlcMiscSetupPayload) );
        TxPacket.Preamble.PacketType     = PKT_TYPE_SLC_MISC_SETUP;
        TxPacket.Preamble.PayloadLength  = sizeof(SerSlcMiscSetupPayload);
        
        //
        // Clear indications of request for SLC SETUP packet
        //
        
        ucUtlJcsStatByte3     &= ~JCS_STAT_B3_REQ_SLC_MISC_SETUP_PKT;
        SerRequestSlcMiscSetup = INVALID_PROTOCOL_ADDRESS  ;
    }
    
    // ===================================================
    // Respond with PKT_TYPE_TOUR_POINT packet if requested
    // ===================================================
    
    else if (SerTourPointDestAddr != INVALID_PROTOCOL_ADDRESS)
    {
        TxPacket.Preamble.DestAddr = SerTourPointDestAddr;
                                                                            
        StdMemCopy((UCHAR *) TxPacket.pPayload, (UCHAR *) &SerTourPointPayload, sizeof(SerTourPointPayload) );
        TxPacket.Preamble.PacketType     = PKT_TYPE_TOUR_POINT;
        TxPacket.Preamble.PayloadLength  = sizeof(SerTourPointPayload);
        
        //
        // Clear indication of request
        //
        
        SerTourPointDestAddr = INVALID_PROTOCOL_ADDRESS;
    }
    
    // ===================================================
    // Added for version 1.71
    // Send one PKT_TYPE_SLC_DEVICE_NAME_ETC on start up.
    // ===================================================
    else if (sendNameOnce == 1)

    {
         TxPacket.Preamble.DestAddr = ucUtlJcsStatDestAddress;
         ucUtlJcsStatDestAddress = INVALID_PROTOCOL_ADDRESS;
                 
        StdMemCopy((UCHAR *) TxPacket.pPayload, (UCHAR *) &SerSlcDeviceNameEtcPayload, sizeof(SerSlcDeviceNameEtcPayload) );
        TxPacket.Preamble.PacketType     = PKT_TYPE_SLC_DEVICE_NAME_ETC; 
        TxPacket.Preamble.PayloadLength  = sizeof(SerSlcDeviceNameEtcPayload);

          sendNameOnce = 0;
    }

    // ===============================================
    // Otherwise, respond with SLC BASIC STATUS packet 
    // ===============================================
    else 
    {
        //
        // Default response to a SLAVE_SPEAK at the SLC, is to transmit
        // a basic SLC status packet.
        //
        
        StdMemCopy((UCHAR *) TxPacket.pPayload, (UCHAR * ) &SerSlcExtStatusPayload.BasicStatus, sizeof(SerSlcExtStatusPayload.BasicStatus) );
        TxPacket.Preamble.PacketType     = PKT_TYPE_SLC_BASIC_STATUS;
        TxPacket.Preamble.PayloadLength  = sizeof(SerSlcExtStatusPayload.BasicStatus);
        
        //
        // if an AUX channel sourced the request for the SLC basic
        // status packet, then respond specifically to that device.
        //
        
        if ( ucUtlJcsStatDestAddress != INVALID_PROTOCOL_ADDRESS )      
        {
			//
			// changed by ACM!! per MAS!! in v1.68
			// The goal is to modify the protocol to allow and AUX device to inhibit response SLC BASIC STATUS 
			// packet to allow higher update rate of joystick control while only sending position feedback at 
			// a fraction of that rate. MAS was seeing problems with reading out of TCP/IP RX buffer at a high
			// enough rate with a single threaded application, while also providing responsive joystick feel 
			// through a PC-based GUI.
	        //

			TxPacket.Preamble.DestAddr = ucUtlJcsStatDestAddress;
            ucUtlJcsStatDestAddress = INVALID_PROTOCOL_ADDRESS;
			
			if (ucJcsInhibitSlcResponse)   // if the device connected to an AUX port does not want a status packet, don't send one
			{
		   	   return; //MAS!!   #2
			}
        }
        
        //
        // The SENT_HOME indication only needs to arrive at the JCS
        // once, so unconditionally, clear this bit from the basic 
        // status packet.
        //
        // SerSlcExtStatusPayload.BasicStatus.ucByte7 &= ~SLC_STAT_B7_AT_HOME;   // commented out by ACM!! in v1.68, clear based on distance from "absolute Home"
    }
    
    SerTransmitPacket ( &g_SerTxQueMain , &TxPacket);
    
} // UtlThisSlcSpeaksOnMain ( )
            
//=====================================================================
// Function   : UtilSendSlaveSpeakOnMain ()                                           
// Purpose    : This function selects the next device from the poll
//              table and sends a slave speak packet to that device.
// Note       : This function increments the polling table entry
//              which corresponds to the protocol address of the
//              addressed device.
// Parameters : Nothing is passed.
// Returns    : Nothing is returned.
//=====================================================================

static _reentrant VOID UtilSendSlaveSpeakOnMain (UCHAR ucProtocolAddress )
{
    
      
    tSER_PACKET TxPacket;
    
    // 
    // Assemble and transmit a SLAVE_SPEAK packet to the
    // addressed device.
    //

    // StdPrintf(&g_SerTxQueSerPort, "<%c%02x>", ucFrom, ucProtocolAddress);

    TxPacket.Preamble.SrcAddr  = g_BoardNumber + SLC_PROTOCOL_OFFSET;
    TxPacket.Preamble.DestAddr = ucProtocolAddress; 
    TxPacket.Preamble.PacketType = PKT_TYPE_SLAVE_SPEAK;
    TxPacket.Preamble.PayloadLength = 0;

    //
    // TxPacket built, now send it...
    //
    
    SerTransmitPacket ( &g_SerTxQueMain , &TxPacket);
    
    //
    // Do some housekeeping ...
    //
    
    //
    // Subject to the low poll threshold, increment the
    // polling table entry which corresponds to the
    // addressed device.
    //

    if (g_PollTable[ucProtocolAddress] < g_PollLowThreshold) 
    {
        ++g_PollTable[ucProtocolAddress];
    }
    
    //
    // In case the technician has been dynamically 
    // fiddling with the "low poll threshold", which
    // might mix up the consistency of the polling
    // table entry values...clamp the max value of
    // of the polling table values.
    //
    
    if (g_PollTable[ucProtocolAddress] > g_PollLowThreshold) 
    {
        g_PollTable[ucProtocolAddress] = g_PollLowThreshold;
    }
    
} // UtilSendSlaveSpeakOnMain ( )

//=====================================================================
// Function   : UtilPollMain ()                                           
// Purpose    : This function selects the next device from the poll
//              table and sends a slave speak packet to that device.
// Parameters : Nothing is passed.
// Returns    : Nothing is returned.
//=====================================================================

VOID UtilPollMain ( VOID )
{
    static UCHAR ucPollTableWrapped;
    static UCHAR ucNextDeviceToPoll;
    
    UCHAR ucStartPollIdx;
    
    //
    // Time to poll?
    //
    
    if (g_TimeMs.PollMain)
    {
        return;
    }
     

    //
    // Retrieve the board number, this allows dynamic changes
    // to DIP switches.
    //
    
    SFRPAGE = CONFIG_PAGE;
    g_BoardNumber = ( ( P2 & 0xF0 ) >> 4 );

    //
    // If no slave speak packets have been received recently,
    // then this board may assume the master role.
    //
    
    if (0 == g_TimeMs.HoldoffMastering)
    {
        g_MasterProtocolAddress = g_BoardNumber;
    }
    
    //
    // If not the master, then we're not authorized to speak
    //
        
    if (g_BoardNumber != g_MasterProtocolAddress)
    {
        g_TimeMs.PollMain = 100;
        return;
    }
    
    // ==========================================================
    // CRITICAL CODE POINT: The firmware has determined that the
    // current host board is the master, and that it is time to
    // either transmit a slave speak, or have the host board report
    // its SLC status.
    // ==========================================================
    
    
    // g_TimeMs.PollMain = POLL_RESPONSE_WAIT_TIME_MS;
    g_TimeMs.PollMain = uiPollResponseWaitTimeMs;
    if ( ucNextDeviceToPoll == g_BoardNumber + SLC_PROTOCOL_OFFSET) 
    {
        UtlThisSlcSpeaksOnMain ( );
    } else
    {
        UtilSendSlaveSpeakOnMain ( ucNextDeviceToPoll );
    }
    
    // ===========================================================
    // The remainder of this function deals with setup of the
    // "ucNextDeviceToPoll" variable, which is used on the next
    // iteration of this function which reaches the "CRITICAL
    // CODE POINT"
    // ===========================================================
    
    //
    // Every 10 poll table wraps, look at 1 off line device, and
    // select it for a "Low Poll" operation.
    //
    
    if (ucPollTableWrapped > 10)
    {
        ucPollTableWrapped = 0;
        
        //
        // Select next offline board for polling ...
        //
        
        ucStartPollIdx = g_PollTableLowPollIdx;       // Don't get stuck in following loop in fully populated system
        do
        {
            ++g_PollTableLowPollIdx;
            if (g_PollTableLowPollIdx >= hbound1(g_PollTable))
            {
                g_PollTableLowPollIdx = 0;
            }
            
            //   
            // The host SLC current board is not 
            // a candidate for a poll operation.
            //
            
            if (g_PollTableLowPollIdx == g_BoardNumber + SLC_PROTOCOL_OFFSET)
                continue;
        
            //
            // is this board offline?  If so,
            // set it up for the polling operation.
            //
            
            if (g_PollTable[g_PollTableLowPollIdx] >= g_PollLowThreshold)
            {
                // g_TimeMs.PollMain = POLL_RESPONSE_WAIT_TIME_MS;
                // UtilSendSlaveSpeakOnMain ( g_PollTableLowPollIdx);
                
                //
                // Setup for next iteration to critical code point.
                //
                
                ucNextDeviceToPoll = g_PollTableLowPollIdx;
                return;
            }
        
            //    
            // Don't get stuck in following loop in fully populated system
            //
            
        } while (ucStartPollIdx != g_PollTableLowPollIdx);
        
    } // if (ucPollTableWrapped > 10)
    
    //
    // Select next active device, which may
    // be either a connected device or the
    // SLC board hosting this firmware.
    //
    
    ucStartPollIdx = g_PollTableIdx; 
    do 
    {
        ++g_PollTableIdx;
        if (g_PollTableIdx >= hbound1(g_PollTable))
        {
            g_PollTableIdx = 0;
            ++ucPollTableWrapped;
        } 
        
        //
        // Is it time for this SLC board (which is mastering),
        // to be selected to speak?  If so, it speaks without
        // being told to "slave speak".
        //
        if ( ( g_BoardNumber + SLC_PROTOCOL_OFFSET) == g_PollTableIdx)
        {
            // g_TimeMs.PollMain = POLL_RESPONSE_QUICK_MS;
            // UtlThisSlcSpeaksOnMain ( );
            
            //
            // Setup for next iteration to critical code point.
            //
            
            ucNextDeviceToPoll = g_PollTableIdx;
            return;
        }

        //
        // Select next active device or the
        // SLC board hosting this firmware.
        //
        
        if ( g_PollTable[g_PollTableIdx] < g_PollLowThreshold )
        {
            // g_TimeMs.PollMain = POLL_RESPONSE_WAIT_TIME_MS;
            // UtilSendSlaveSpeakOnMain ( g_PollTableIdx );
            //
            // Setup for next iteration to critical code point.
            //
            
            ucNextDeviceToPoll = g_PollTableIdx;
            return;
        }
        
    }  while (ucStartPollIdx != g_PollTableIdx);
    
    return;
    
} // UtilPollMain ( )


//=====================================================================
// Function   : UtilAbsToRel ()                                           
// Purpose    : 
// Parameters : 
// Returns    : 
//=====================================================================

VOID  UtilAbsToRel ( )
{
    g_TargetRelPosPan   = ( INT ) g_TargetAbsPosPan   - ( INT ) SerSlcMiscSetupPayload.usPanHomeOffset;
    g_TargetRelPosTilt  = ( INT ) g_TargetAbsPosTilt  - ( INT ) SerSlcMiscSetupPayload.usTiltHomeOffset;
    g_TargetRelPosFocus = ( INT ) g_TargetAbsPosFocus - ( INT ) g_FocusHomeOffset;
    
}   // UtilAbsToRel ( )

//=====================================================================
// Function   : UtilRelToAbs ()                                           
// Purpose    : 
// Parameters : 
// Returns    : 
//=====================================================================

VOID  UtilRelToAbs ( )
{
    g_TargetAbsPosPan   = ( UINT ) ( g_TargetRelPosPan   + SerSlcMiscSetupPayload.usPanHomeOffset   );
    g_TargetAbsPosTilt  = ( UINT ) ( g_TargetRelPosTilt  + SerSlcMiscSetupPayload.usTiltHomeOffset  );
    g_TargetAbsPosFocus = ( UINT ) ( g_TargetRelPosFocus + g_FocusHomeOffset );
    
}   // UtilRelToAbs ( )

//=====================================================================
// Function   : UtilSqrt ()                                           
// Purpose    : This function calculates the square root of an integer
//              using the Newton-Raphson method.
// Parameters : The integer to take the square root of is passed.
// Returns    : The integer result is returned.
//=====================================================================

UINT UtilSqrt ( LONG n )
{
    LONG root = 50;
    UCHAR   i;
    LONG temp; 

    for ( i = 0; i < 7; i++ )
    {
        temp = root * root;
        temp -= n;
        temp /= 2 * root;
        root -= temp;
    }
    return ( UINT ) root;

} // UtilSqrt ( )

	enum pelco_handling			  // mbdpelco
	{
		extended,
		jcs_packet,
		position_rel_packet
	};

	UINT		PanPosAbs,TiltPosAbs;		// mbdpelco

//=============================================================================
// Function   : PelcoProcessCommand( )
// Purpose    : This function decodes a Pelco D command and issues SV command.
// Parameters : Nothing is passed. 
// Returns    : Nothing is returned.
//=============================================================================

VOID _reentrant PelcoProcessCommand ( tProtocolChannels *pChannel )
{
	tPAYLOAD_JCS_STATUS	JcsStatusPayload;
	tPAYLOAD_DO_POSITION_SL_REL DoPositionSLRelativePayload;
	static UCHAR ucZoomSpeed = 3;	// set to max zoom speed
//	tPresetPolarPos  *pTempPresetPolarPos;  // mbd
	UINT		temp1,temp2;
	SHORT		temp3,temp4,sPreset_Tilt_Rel;
	LONG		lPreset_Pan_Rel;
	UCHAR		PanPos_MSB, PanPos_LSB,TiltPos_MSB, TiltPos_LSB;	 // mbdpelco
	tSER_PELCO_PACKET TxPelcoPacket;								 // mbdpelco
	enum		pelco_handling   pelco_route = jcs_packet ;  // // mbdpelco signals "extended" Pelco commands, not to be treated as a JCS packet !
//	UINT		PanPosAbs,TiltPosAbs;		// mbdpelco
   	UINT		temp5,temp6;

	StdMemClr(&JcsStatusPayload, sizeof(JcsStatusPayload));			   // mbdtest

	
	// Initialize X, Y, Z values to "center"
	JcsStatusPayload.ucX = 127;
	JcsStatusPayload.ucY = 127;
	JcsStatusPayload.ucZ = 127;
	JcsStatusPayload.ucByte4  = JCS_STAT_B4_INHIBIT_SLC_RESPONSE;

	// Treat incoming Pelco D command as a Joystick Status packet from SerPort that needs to be routed
	StdMemClr(&g_RoutedRxSerPortPacket, sizeof(g_RoutedRxSerPortPacket));
	StdMemClr(&g_RoutedRxEthPortPacket, sizeof(g_RoutedRxEthPortPacket)); // mbdpelco

	if ( ((pChannel->ucSourceAddress) & 0xF0)  == SLC_SERPORT_PROTOCOL_OFFSET )
	{	 
		g_RoutedRxSerPortPacket.Preamble.SrcAddr       = g_BoardNumber + SLC_SERPORT_PROTOCOL_OFFSET;
		if(pChannel->ucDestinationAddress <= 1)
    	{
    		g_RoutedRxSerPortPacket.Preamble.DestAddr      = g_BoardNumber + SLC_PROTOCOL_OFFSET;		// if address is set to 0 or 1, route to SLC it is connected to
    	} 
		else
		{
		g_RoutedRxSerPortPacket.Preamble.DestAddr      = pChannel->ucDestinationAddress;   // Needs to be in the range of 0x10 to 0x1F	
		}
		g_RoutedRxSerPortPacket.Preamble.PacketType    = PKT_TYPE_JCS_STATUS;
		g_RoutedRxSerPortPacket.Preamble.PayloadLength = 8;	  // extended JCS packet. Was 5


	}
	else if	( ((pChannel->ucSourceAddress) & 0xF0)  == SLC_ETHPORT_PROTOCOL_OFFSET )
	{
		g_RoutedRxEthPortPacket.Preamble.SrcAddr       = g_BoardNumber + SLC_ETHPORT_PROTOCOL_OFFSET ;
		if(pChannel->ucDestinationAddress <= 1)
    	{
    		g_RoutedRxEthPortPacket.Preamble.DestAddr      = g_BoardNumber + SLC_PROTOCOL_OFFSET;		// if address is set to 0 or 1, route to SLC it is connected to
    	} 
		else
		{
			g_RoutedRxEthPortPacket.Preamble.DestAddr      = pChannel->ucDestinationAddress;   // Needs to be in the range of 0x10 to 0x1F	
		}
		g_RoutedRxEthPortPacket.Preamble.PacketType    = PKT_TYPE_JCS_STATUS;
		g_RoutedRxEthPortPacket.Preamble.PayloadLength = 8;	  // extended JCS packet. Was 5
			   		
	}
    
    

    
        	
	if(!CHECK_BIT(pChannel->ucPayload[Command2],ExtendedCommand) )	// check to see if "Extended Command" bit (LSB of second command byte) is set
	{
		// Standard Command handler
		DEBUG_PIN_4 = 1;			
		if(CHECK_BIT(pChannel->ucPayload[Command2],ZoomOut) )
		{
			// Handle "Zoom Wide/Out" bit
			JcsStatusPayload.ucZ = 127 + ((ucZoomSpeed + 1) * 31);	// scale speed based on 0-3 speed multiplier
		} 
		else if(CHECK_BIT(pChannel->ucPayload[Command2],ZoomIn) )
		{
			// Handle "Zoom Tele/In" bit
			JcsStatusPayload.ucZ = 127 - ((ucZoomSpeed + 1) * 31);	// scale speed based on 0-3 speed multiplier
		}

		if(CHECK_BIT(pChannel->ucPayload[Command2],Down) )
		{
			// Handle "Down" bit
			JcsStatusPayload.ucY = 127 - ( (pChannel->ucPayload[Data2] & 0x3F) * 2);	  // scale speed based on 0-63 (0x3F) proportional range, mask so 0xFF ("Turbo" speed) looks like 0x3F
		}
		else if(CHECK_BIT(pChannel->ucPayload[Command2],Up) )
		{
			// Handle "Up" bit
			JcsStatusPayload.ucY = 127 + ( (pChannel->ucPayload[Data2] & 0x3F) * 2);	  // scale speed based on 0-63 (0x3F) proportional range, mask so 0xFF ("Turbo" speed) looks like 0x3F
		}

		if(CHECK_BIT(pChannel->ucPayload[Command2],Left) )
		{
			// Handle "Left" bit
			JcsStatusPayload.ucX = 127 - ( (pChannel->ucPayload[Data1] & 0x3F) * 2);	  // scale speed based on 0-63 (0x3F) proportional range, mask so 0xFF ("Turbo" speed) looks like 0x3F
		}
		else if(CHECK_BIT(pChannel->ucPayload[Command2],Right) )
		{
			// Handle "Right" bit
			JcsStatusPayload.ucX = 127 + ( (pChannel->ucPayload[Data1] & 0x3F) * 2);	  // scale speed based on 0-63 (0x3F) proportional range, mask so 0xFF ("Turbo" speed) looks like 0x3F
		}

		if (  (pChannel->ucPayload[Command1] == 0x00) &&   (pChannel->ucPayload[Command2] == 0x00) )  // full stop
		{
			JcsStatusPayload.ucX = 127;
			JcsStatusPayload.ucY = 127;
			JcsStatusPayload.ucZ = 127;
		}


	}
	else
	{
		// Extended Command handler
		switch(pChannel->ucPayload[Command2])
		{
			case SetPreset:
				pelco_route = extended;
				ucFlashActivePresetNum = pChannel -> ucPayload[Data2];  // grab the preset number from PELCO stream,
				if (ucFlashActivePresetNum > 0x20)  // user trying to set HOME via PELCO-D ( which is preset 0x22 ), or any preset greater than 0x20
				{
				// prob do nothing
				}
 				else if	(ucFlashActivePresetNum <=4) // reserve first 4 preset for AUX functions
				{
				// probably do nothing
				}
				else
				{
//				pTempPresetPolarPos->Preset_Pan = g_PanPos;
//				pTempPresetPolarPos->Preset_Tilt = g_TiltPos;
//				FlashStorePresetPoint ( ucFlashActivePresetNum, pTempPresetPolarPos  );
				FlashStorePresetPoint_XX ( ucFlashActivePresetNum );
				}  
				break;

			case ClearPreset:
				break;

			case GoToPreset:
				 pelco_route = extended;
				 ucFlashActivePresetNum = pChannel -> ucPayload[Data2];
				 if ( ucFlashActivePresetNum == 0x22 )	  // this is the case if the PelcoD "HOME" preset is set.  Always use the SLC home preset.
				{
					UtilGotoHomePosition();
				}
				else if ( ucFlashActivePresetNum <=4)  // use first 4 presets to do relay/AUX functions
				{
					switch (ucFlashActivePresetNum)
					{
						case PELCO_BEAM_ON:
							UtlLampOn();
							break;
						case PELCO_BEAM_OFF:
							UtlLampOff();
							break;
						case PELCO_AUX1_ON:
						    UtlRelayOn (RELAY_AUX1);
							break;
						case PELCO_AUX1_OFF:
						    UtlRelayOff (RELAY_AUX1);
							break;
					}
				}
				else if ( 0x04 <= ucFlashActivePresetNum <= 0x20 ) 
				{
				 FlashRetrievePresetPoint ( ucFlashActivePresetNum );
				 temp1 = g_ActivePresetPanPos;
				 temp2 = g_ActivePresetTiltPos;

				 //temp1 = CurrentPresetPolarPos->Preset_Pan	;
 				 //temp2 = CurrentPresetPolarPos->Preset_Tilt	 ;
				 temp3 = SerSlcMiscSetupPayload.usPanHomeOffset;
				 temp4 = SerSlcMiscSetupPayload.usTiltHomeOffset;
			 // need to convert absolute unsigned values to signed values that contain the home offset prior to call the GotoPolarPosition() routine
				 lPreset_Pan_Rel = (LONG)temp1 - (LONG)temp3;
				 sPreset_Tilt_Rel = (LONG)temp2 - (LONG)temp4;
				 UtilGotoPolarPosition(lPreset_Pan_Rel, sPreset_Tilt_Rel, 0xFFFF, 0);
				 }
				 else
				 {
				 // don't do anything
				 }
				break;

			case SetAuxiliary:
				 //g_DynTrackingMode = 0;
				pelco_route = jcs_packet;
				switch (pChannel->ucPayload[Data2])
				{
					case 0x01:
						JcsStatusPayload.ucByte4 = JCS_STAT_B4_AUX1_ON;
						break;

					case 0x02:
						JcsStatusPayload.ucByte3 = JCS_STAT_B3_AUX2_ON;
						break;

					case 0x03:
						JcsStatusPayload.ucByte3 = JCS_STAT_B3_AUX3_ON;
						break;

					case 0x04:
						JcsStatusPayload.ucByte5 = JCS_STAT_B5_AUX4_ON;
						break;

					case 0x05:
						JcsStatusPayload.ucByte5 = JCS_STAT_B5_AUX5_ON;
						break;

					case 0x06:
						JcsStatusPayload.ucByte5 = JCS_STAT_B5_AUX6_ON;
						break;

					case 0x07:
						JcsStatusPayload.ucByte5 = JCS_STAT_B5_AUX7_ON;
						break;

					case 0x08:
						JcsStatusPayload.ucByte4 = JCS_STAT_B4_LAMP_ON_START;
						break;
				}
				break;

			case ClearAuxiliary:
				 //g_DynTrackingMode = 0;
				pelco_route = jcs_packet;
				switch (pChannel->ucPayload[Data2])
				{

					case 0x01:
						JcsStatusPayload.ucByte4 = JCS_STAT_B4_AUX1_OFF;
						break;

					case 0x02:
						JcsStatusPayload.ucByte3 = JCS_STAT_B3_AUX2_OFF;
						break;

					case 0x03:
						JcsStatusPayload.ucByte3 = JCS_STAT_B3_AUX3_OFF;
						break;

					case 0x04:
						JcsStatusPayload.ucByte5 = JCS_STAT_B5_AUX4_OFF;
						break;

					case 0x05:
						JcsStatusPayload.ucByte5 = JCS_STAT_B5_AUX5_OFF;
						break;

					case 0x06:
						JcsStatusPayload.ucByte5 = JCS_STAT_B5_AUX6_OFF;
						break;

					case 0x07:
						JcsStatusPayload.ucByte5 = JCS_STAT_B5_AUX7_OFF;
						break;

					case 0x08:
						JcsStatusPayload.ucByte4 = JCS_STAT_B4_LAMP_OFF;
						break;
								
				}
				break;

			case SetZoomSpeed:	// used for beam width speed
				pelco_route = extended;
				ucZoomSpeed = pChannel->ucPayload[Data2];
				break;

			case SetFocusSpeed:	// not currently used
				pelco_route = extended;
				break;

			case SetPanPosAbs:			// mbdpelco
				pelco_route = extended;
				PanPos_MSB = pChannel->ucPayload[Data1];
				PanPos_LSB = pChannel->ucPayload[Data2];
				PanPosAbs =    (UINT)(PanPos_MSB <<8) + (UINT)PanPos_LSB;
				PanPosAbs = PanPosAbs/10;  // tenths of a degree for use in SL relative packet
 				break;
				
			case SetTiltPosAbs:		  // mbdpelco
				pelco_route = extended;
				TiltPos_MSB = pChannel->ucPayload[Data1];
				TiltPos_LSB = pChannel->ucPayload[Data2];
				//TiltPosAbs =    (UINT)(TiltPos_MSB <<8) + (UINT)TiltPos_LSB;
				temp6 = (UINT)(TiltPos_MSB <<8) + (UINT)TiltPos_LSB;	  // temp6 is in pelco coords
				if ( (temp6<=35999) && (temp6>(35999-g_PotReader.TiltRange/2) ) )		// upper quadrant
				{
					 TiltPosAbs = 35999 + g_PotReader.TiltRange/2 -temp6 ;
				}
				else if ( (temp6 >=0) && (temp6<= g_PotReader.TiltRange/2) )	   // lower quadrant
				{
					 TiltPosAbs =  (g_PotReader.TiltRange/2) - temp6;
				}
				TiltPosAbs = TiltPosAbs/10;  // tenths of a degree for use in SL relative packet
				break;

			case SetZoomPosAbs:	// not currently used
				pelco_route = position_rel_packet; //
				DoPositionSLRelativePayload.sAzimuth = PanPosAbs;  // just use current value of Az
				DoPositionSLRelativePayload.sElevation = TiltPosAbs;  // 
				DoPositionSLRelativePayload.usGrowth0 = 0x0001;		// dyn tkg
				DoPositionSLRelativePayload.usGrowth1 = 0x0000;
				DoPositionSLRelativePayload.usRate = 0xFFFF;	 // max speed

			
				if ( ((pChannel->ucSourceAddress) & 0xF0)  == SLC_SERPORT_PROTOCOL_OFFSET )
				{
					StdMemClr(&g_RoutedRxSerPortPacket, sizeof(g_RoutedRxSerPortPacket));
    				g_RoutedRxSerPortPacket.Preamble.SrcAddr       = g_BoardNumber + SLC_SERPORT_PROTOCOL_OFFSET;
    
    				if(pChannel->ucDestinationAddress <= 1)
    				{
    					g_RoutedRxSerPortPacket.Preamble.DestAddr      = g_BoardNumber + SLC_PROTOCOL_OFFSET;		// if address is set to 0 or 1, route to SLC it is connected to
    				} 
    				g_RoutedRxSerPortPacket.Preamble.PacketType    = PKT_TYPE_DO_POSITION_SL_REL;
    				g_RoutedRxSerPortPacket.Preamble.PayloadLength = 10;	  //
    			} 
				else if ( ((pChannel->ucSourceAddress) & 0xF0)  == SLC_ETHPORT_PROTOCOL_OFFSET )
				{
					StdMemClr(&g_RoutedRxEthPortPacket, sizeof(g_RoutedRxEthPortPacket));
    				g_RoutedRxEthPortPacket.Preamble.SrcAddr       = g_BoardNumber + SLC_ETHPORT_PROTOCOL_OFFSET;
    
    				if(pChannel->ucDestinationAddress <= 1)
    				{
    					g_RoutedRxEthPortPacket.Preamble.DestAddr      = g_BoardNumber + SLC_PROTOCOL_OFFSET;		// if address is set to 0 or 1, route to SLC it is connected to
    				} 
    				g_RoutedRxEthPortPacket.Preamble.PacketType    = PKT_TYPE_DO_POSITION_SL_REL;
    				g_RoutedRxEthPortPacket.Preamble.PayloadLength = 10;	  //

				}

								   
				break;

			case QueryPanPos:	  // mbdpelco
				// reply with Abs pan position: ResponsePanPosition
				pelco_route = extended ;//extended_Pelco = TRUE;
				StdMemClr(&TxPelcoPacket, sizeof(TxPelcoPacket));
				// determine which SLC. "Pelco" addresses range from 1 to 16

				//PanPos_MSB = (UCHAR)((SerSlcExtStatusPayload.BasicStatus.sAzimuth & 0xFF00) >>8) ;
				//PanPos_LSB = (UCHAR)(SerSlcExtStatusPayload.BasicStatus.sAzimuth & 0x00FF );

			   // below code to make compatible with PELCO-D statndard.  Need to RETURN hundredths of a degree, always POSITIVE...1 to 35999.  Need to ignore any offset
				PanPos_MSB = (UCHAR)( (g_PotReader.PanFb & 0xFF00) >> 8);
				PanPos_LSB = (UCHAR)(g_PotReader.PanFb & 0x00FF);

			    TxPelcoPacket.addr = pChannel->ucDestinationAddress;
 				TxPelcoPacket.cmd1 = 0x00;                         
 				TxPelcoPacket.cmd2 = 0x59;	
 				TxPelcoPacket.data1 = PanPos_MSB;                  
 				TxPelcoPacket.data2 = PanPos_LSB;                  
				if ( ((pChannel->ucSourceAddress) & 0xF0)  == SLC_SERPORT_PROTOCOL_OFFSET )
				{	 
			   		SerTransmitPelcoPacket(&g_SerTxQueSerPort,&TxPelcoPacket);
			   	}
			   	else if	( ((pChannel->ucSourceAddress) & 0xF0)  == SLC_ETHPORT_PROTOCOL_OFFSET )
			   	{
			   		SerTransmitPelcoPacket(&g_SerTxQueEthPort,&TxPelcoPacket);
				}
				else
				{
				}
				 	
				break;

			case QueryTiltPos:		// mbdpelco
				// reply with Abs tilt position: ResponseTiltPosition
				pelco_route = extended; // extended_Pelco = TRUE;
				StdMemClr(&TxPelcoPacket, sizeof(TxPelcoPacket));
 //				TiltPos_MSB = (UCHAR)((SerSlcExtStatusPayload.BasicStatus.sElevation & 0xFF00) >>8) ;
 //				TiltPos_LSB = (UCHAR)(SerSlcExtStatusPayload.BasicStatus.sElevation & 0x00FF );

			   // below code to make compatible with PELCO-D statndard.  Need to RETURN hundredths of a degree, always POSITIVE...1 to 35999.  Need to ignore any offset
				if (g_PotReader.TiltFb >= 0 && g_PotReader.TiltFb <= (g_PotReader.TiltRange/2) )
				{
					temp5 = (UINT)g_PotReader.TiltRange/2 - g_PotReader.TiltFb;
				}
				else if ( g_PotReader.TiltFb > (g_PotReader.TiltRange/2) && g_PotReader.TiltFb <= g_PotReader.TiltRange )
				{
					temp5 = (UINT)35999 + (g_PotReader.TiltRange/2) - g_PotReader.TiltFb;
				}

				TiltPos_MSB = (UCHAR)( (temp5 & 0xFF00) >> 8);
				TiltPos_LSB = (UCHAR)(temp5 & 0x00FF);

				TxPelcoPacket.addr = pChannel->ucDestinationAddress;
				TxPelcoPacket.cmd1 = 0x00;                 
 				TxPelcoPacket.cmd2 = 0x5B;		
 				TxPelcoPacket.data1 = TiltPos_MSB;         
 				TxPelcoPacket.data2 = TiltPos_LSB;         
				if ( ((pChannel->ucSourceAddress) & 0xF0)  == SLC_SERPORT_PROTOCOL_OFFSET )
				{	 
			   		SerTransmitPelcoPacket(&g_SerTxQueSerPort,&TxPelcoPacket);
			   	}
			   	else if	( ((pChannel->ucSourceAddress) & 0xF0)  == SLC_ETHPORT_PROTOCOL_OFFSET )
			   	{
			   		SerTransmitPelcoPacket(&g_SerTxQueEthPort,&TxPelcoPacket);
				}
				else
				{
				} 	
							
				break;

			case QueryZoomPos:
				// reply with Abs zoom position: ResponseZoomPosition - for now just reply with zero since no position pot
				pelco_route = extended; // extended_Pelco = TRUE;
				StdMemClr(&TxPelcoPacket, sizeof(TxPelcoPacket));
 
				TxPelcoPacket.addr = pChannel->ucDestinationAddress;
				TxPelcoPacket.cmd1 = 0x00;                 
 				TxPelcoPacket.cmd2 = 0x5D;		
 				TxPelcoPacket.data1 = 0x00;          
 				TxPelcoPacket.data2 = 0x00;          
				if ( ((pChannel->ucSourceAddress) & 0xF0)  == SLC_SERPORT_PROTOCOL_OFFSET )
				{	 
			   		SerTransmitPelcoPacket(&g_SerTxQueSerPort,&TxPelcoPacket);
			   	}
			   	else if	( ((pChannel->ucSourceAddress) & 0xF0)  == SLC_ETHPORT_PROTOCOL_OFFSET )
			   	{
			   		SerTransmitPelcoPacket(&g_SerTxQueEthPort,&TxPelcoPacket);
				}
				else
				{
				} 	
							
				break;
							
						 
		}	// switch/case
	} // 	if(!CHECK_BIT(pChannel->ucPayload[Command2],ExtendedCommand) )
																													
						 // mbdpelco
	if ( pelco_route == jcs_packet )
	{
		g_DynTrackingMode = FALSE;
		
		if ( ((pChannel->ucSourceAddress) & 0xF0)  == SLC_SERPORT_PROTOCOL_OFFSET )
		{
			StdMemCopy(g_RoutedRxSerPortPacket.pPayload, (UCHAR*) &JcsStatusPayload, g_RoutedRxSerPortPacket.Preamble.PayloadLength); 

			if (g_RoutedRxSerPortPacket.Preamble.DestAddr == g_BoardNumber + SLC_PROTOCOL_OFFSET)
			{
				pChannel->ucDestinationAddress = g_RoutedRxSerPortPacket.Preamble.DestAddr;
				pChannel->ucSourceAddress = g_RoutedRxSerPortPacket.Preamble.SrcAddr;
				pChannel->ucPayloadLength = 8;			// extended JCS packet, was 5
				pChannel->ucPacketType = PKT_TYPE_JCS_STATUS;//g_RoutedRxSerPortPacket.Preamble.PacketType;
				StdMemCopy(pChannel->ucPayload, g_RoutedRxSerPortPacket.pPayload, g_RoutedRxSerPortPacket.Preamble.PayloadLength); 
				UtilProcessPacket ( pChannel );
			}
		}
		else if (  ((pChannel->ucSourceAddress) & 0xF0)  == SLC_ETHPORT_PROTOCOL_OFFSET )
		{
			StdMemCopy(g_RoutedRxEthPortPacket.pPayload, (UCHAR*) &JcsStatusPayload, g_RoutedRxEthPortPacket.Preamble.PayloadLength);

			if (g_RoutedRxEthPortPacket.Preamble.DestAddr == g_BoardNumber + SLC_PROTOCOL_OFFSET)
			{
				pChannel->ucDestinationAddress = g_RoutedRxEthPortPacket.Preamble.DestAddr;
				pChannel->ucSourceAddress = g_RoutedRxEthPortPacket.Preamble.SrcAddr;
				pChannel->ucPayloadLength = 8;			// extended JCS packet, was 5
				pChannel->ucPacketType = PKT_TYPE_JCS_STATUS;//g_RoutedRxEthPortPacket.Preamble.PacketType;
				StdMemCopy(pChannel->ucPayload, g_RoutedRxEthPortPacket.pPayload, g_RoutedRxEthPortPacket.Preamble.PayloadLength); 
				UtilProcessPacket ( pChannel );
			}

		}
	}
	else if (pelco_route ==  position_rel_packet)																  
	{
				if ( ((pChannel->ucSourceAddress) & 0xF0)  == SLC_SERPORT_PROTOCOL_OFFSET )
				{
					StdMemCopy(g_RoutedRxSerPortPacket.pPayload, (UCHAR*) &DoPositionSLRelativePayload, g_RoutedRxSerPortPacket.Preamble.PayloadLength);
					StdMemCopy(pChannel->ucPayload, g_RoutedRxSerPortPacket.pPayload, g_RoutedRxSerPortPacket.Preamble.PayloadLength); 
					pChannel->ucDestinationAddress = g_RoutedRxSerPortPacket.Preamble.DestAddr;
					pChannel->ucSourceAddress = g_RoutedRxSerPortPacket.Preamble.SrcAddr;
					pChannel->ucPayloadLength = g_RoutedRxSerPortPacket.Preamble.PayloadLength;			
					pChannel->ucPacketType = g_RoutedRxSerPortPacket.Preamble.PacketType;
					UtilProcessPacket ( pChannel );
				}
				else if (  ((pChannel->ucSourceAddress) & 0xF0)  == SLC_ETHPORT_PROTOCOL_OFFSET )
				{
					StdMemCopy(g_RoutedRxEthPortPacket.pPayload, (UCHAR*) &DoPositionSLRelativePayload, g_RoutedRxEthPortPacket.Preamble.PayloadLength);
					StdMemCopy(pChannel->ucPayload, g_RoutedRxEthPortPacket.pPayload, g_RoutedRxEthPortPacket.Preamble.PayloadLength); 
					pChannel->ucDestinationAddress = g_RoutedRxEthPortPacket.Preamble.DestAddr;
					pChannel->ucSourceAddress = g_RoutedRxEthPortPacket.Preamble.SrcAddr;
					pChannel->ucPayloadLength = g_RoutedRxEthPortPacket.Preamble.PayloadLength;			
					pChannel->ucPacketType = g_RoutedRxEthPortPacket.Preamble.PacketType;
					UtilProcessPacket ( pChannel );

				}

	}
	else	// extended, do nothing
	{
		StdMemClr(&TxPelcoPacket, sizeof(TxPelcoPacket));
		StdMemClr(&g_RoutedRxSerPortPacket, sizeof(g_RoutedRxSerPortPacket));
		StdMemClr(&g_RoutedRxEthPortPacket, sizeof(g_RoutedRxEthPortPacket));
	}
			DEBUG_PIN_4 = 0;
} // PelcoProcessCommand ()

 

