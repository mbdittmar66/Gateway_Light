//**********************************************************************
// Filename:      utils.h
// Rights:        Copyright (c) 2006 The Carlisle and Finch Company
//                All Rights Reserved
// Creation Date: 9/11/06
// Compiler:      Tasking cc51
// Version:       v7.1r1
// Purpose:       This module contains additional JSC interface constants,
//                definitions, and function prototypes.
// Modification History:
// Date         Author      Description
// ----         ------      -----------
// 9-11-06		JRN			Created
//**********************************************************************

#ifndef _UTILS_INC_
#define _UTILS_INC_

#include "serial.h"

// ===========================================
// Constants
// ===========================================

#define DEBUG_JCS_ANALOG		0x01
#define DEBUG_PRB_PRINT         0x02
#define DEBUG_POLAR_POINT       0x04
#define DEBUG_TOUR_EXCEPTION    0x08

#define ASCII_CR                0x0d
#define ASCII_LF                0x0a
#define ASCII_BELL              0x07
#define ASCII_BS                0x08
#define ASCII_DEL               0x7F

#define ADC_GAIN_HALF			7
#define ADC_GAIN_ONE			0
#define ADC_GAIN_TWO			1
#define ADC_GAIN_FOUR			2
#define ADC_GAIN_EIGHT			3
#define ADC_GAIN_SIXTEEN		4

#define ADC0_0			        0
#define ADC0_1			        1
#define ADC0_2			        2
#define ADC0_3			        3

#define ADC_ITER_MAX			100

#define PRB_POLLS_PER_SEC       20

#define FOCUS_RELAY_THRESHOLD   20

#define MODEL_STR               "Searchlight Control Board"

#define VERSION_DBG             "version"
#define MODEL_DBG               "model"
#define BOARD_ID_DBG            "id"
#define BOARD_NAME_DBG          "name"
#define MOTOR_DBG               "motor"
#define RELAY_DBG               "relay"
#define CAL_POT_DBG             "calpot"
#define POLL_TABLE_DBG          "polltable"
#define TOUR_TABLE_DBG          "tourtable"

#define PAN_MOTOR               1
#define TILT_MOTOR              2
#define FOCUS_MOTOR             3
 
#define MOTOR_1                 1
#define MOTOR_2                 2
#define MOTOR_3                 3

#define MOTOR_START_MOVE        1
#define MOTOR_MOVING            2        

#define RELAY_ENABLE            1
#define RELAY_DISABLE           0

//
// Used to control a tour (etc).
//

#define TOUR_INVALID            0xFF
#define TOUR_PAUSED_BIT         0x80    // Applicable to ucUtlActiveTourNumber

//#define NUM_PRESETS				20 // mbd determined by PELCO-D protocol
//
// g_OperationStatus
//

#define NORMAL_MODE             0
#define NEEDS_CAL_MODE          1
#define CAL_MODE                2

//
// g_PrbSlaveSpeakPayload flags
//

#define IN_CAL_MODE_BIT         1
#define SLC_POWER_FAILED_BIT    2
// mbdtest	experimental tracking mode masks
#define DYN_TRACKING_MODE		0x01
// mbdtest

#define PELCO_BEAM_ON	1	   // uses preset 1
#define PELCO_BEAM_OFF	2	   // uses preset 2
#define PELCO_AUX1_ON	3	   // uses preset 3
#define PELCO_AUX1_OFF	4	   // uses preset 4

// ===========================================
// Data structures
// ===========================================

typedef struct
{
	UCHAR	ucPanPNorm;
	UCHAR	ucPanINorm;
	UCHAR   ucPanDNorm;
	UCHAR	ucTiltPNorm;
	UCHAR	ucTiltINorm;
	UCHAR	ucTiltDNorm;
	UCHAR	ucPanTarget;
	UCHAR	ucTiltTarget;
	UCHAR	ucPanDeadband;
	UCHAR	ucTiltDeadband;
	UCHAR	ucPanEndDeadband;
	UCHAR	ucTiltEndDeadband;
	UCHAR	ucStartRampCycles;
	UCHAR	ucEndRampCycles;
	UINT	uiSlowAccumDump;
	INT		ucMaxPanRateOffset;
	INT		ucMaxTiltRateOffset;
	UCHAR   ucJcsAccelLimit;
} tBaseConfig;


typedef struct
{
    INT P;          // Proportionate
    INT I;          // Integral
    INT D;          // Differential

} tPID;

typedef struct
{
    UCHAR   ID;                 // Identification
    UCHAR   Speed;              // Speed
    tPID    Gain;               // PID Gains
    tPID    PidTerms;           // PID Terms
    INT     IntegralAccum;      // I Accumulator
    INT     AccumLimit;         // Maximum value (+/-) for accumulator value 
    INT     DegreeError;        // Error between setpoint and actual location
    INT     DiffError;          // Used to calculate D term
    UCHAR   LastSpeed;          // Used to control acceleration
    UCHAR   FinalPosStartSpeed; // Used to control speed during final positioning
	UCHAR	LoadedSpeed;		// v1.73 - speed that is getting loaded into PWM registers
    UCHAR   Deadband;           // Motor deadband value
} tMotor; 

typedef struct
{
    UCHAR Gen_1;    // General Purpose Relay 1
    UCHAR Gen_2;    // General Purpose Relay 2
    UCHAR Gen_3;    // General Purpose Relay 3
    UCHAR Gen_4;    // General Purpose Relay 4
    UCHAR Gen_5;    // General Purpose Relay 5
    UCHAR AC_On;    // AC Motor On / Off Relay
    UCHAR AC_Dir;   // AC Motor Directional Relay
    
} tRelays;

typedef struct
{
    UCHAR OperationStatus;
	UCHAR CalStatus;
    UCHAR Name[17];
    UINT  PanFb;
    UINT  TiltFb;
    UINT  FocusFb;
	UINT  PanRange;
	UINT  TiltRange;
	UINT  FocusRange;
    UINT  PanMaxRate;
    UINT  TiltMaxRate;
    UINT  FocusMaxRate;
	UINT  PanRawData;
	UINT  TiltRawData;
	    
} tPotReader;


typedef struct
{
    UINT X;
    UINT Y;
    UINT Z;
    UINT HoldTime;

} tPolarPos;


typedef struct
{
    UCHAR     TourId;
    tPolarPos Pos[256];

} tTourTable;
    
// ===========================================
// Function prototypes
// ===========================================

// VOID  UtilCheckMessage          ( VOID );
// VOID  UtilCheckCommand          ( VOID );
UCHAR    UtilGetBoardId            ( VOID );
VOID     UtilCheckPanMotor         ( VOID );
VOID     UtilCheckTiltMotor        ( VOID );
VOID     UtilSetMotorSpeed         ( UCHAR MotorId, INT Speed );
VOID     UtilCheckRelays           ( VOID );
VOID     UtilCalibrateMotor        ( UCHAR MotorId, UINT Range );
VOID     UtilCheckPotReaderBoard   ( VOID );
VOID     UtilSynchronizeExecution  ( VOID );
VOID     UtilPotDisplay            ( VOID );
VOID     UtilCheckSerPort          ( VOID );
VOID     UtilCheckEthPort          ( VOID );
VOID     UtilPollMain              ( VOID );
VOID     UtilControlSlc            ( VOID );
VOID     UtlRelayOn                ( UCHAR ucRelay );
VOID     UtlRelayOff               ( UCHAR ucRelay );
VOID     UtlLampOn                 ( VOID );
VOID     UtlLampOff                ( VOID );
VOID     UtilSetAllMotorsWithLimits( UCHAR PanSpeed, UCHAR TiltSpeed, UCHAR FocusSpeed );
VOID     UtilCheckMotorAccel       ( VOID );
VOID     UtilAbsToRel			   ( VOID );
VOID     UtilRelToAbs			   ( VOID );
VOID     UtilNextTourPoint         ( VOID );
VOID     UtilGotoHomePosition      ( VOID );
BOOLEAN  UtilGotoPolarPosition     ( LONG lPan, SHORT sTilt, USHORT usRate, UINT uiFocus);
BOOLEAN  UtilGotoDynTrackingPosition     ( LONG lPan, SHORT sTilt, USHORT usRate);	  // mbdtest experimental
_reentrant VOID  UtilCheckProtocolChannels ( VOID );
_reentrant VOID  UtilPollPrb               ( VOID );


// ===========================================
// Global variables
// ===========================================

//extern tMotor  g_PanMotor;
//extern tMotor  g_TiltMotor;
extern UCHAR   ucUtlJcsStatByte3;
extern tRelays g_Relays;

extern LONG  g_TargetPosX;
extern LONG  g_TargetPosY;
extern LONG  g_TargetPosZ;

// extern UINT  g_PanHomeOffset;
// extern UINT  g_TiltHomeOffset;

extern UCHAR g_BoardNumber;
extern UCHAR g_BaseType;
extern UCHAR g_LampState;

extern UCHAR g_PrbSlaveSpeakPayload;

extern BOOLEAN g_SmartView1;		// v1.59
extern BOOLEAN sendNameOnce;        // V1.71

extern UCHAR    g_MotorDeadband;

extern UCHAR   g_PanSpeedDebug;
extern UCHAR   g_PanLastSpeedDebug;
extern UCHAR   g_iMotorDebug;

extern tMotor  g_PanMotor;
extern tMotor  g_TiltMotor;
extern tMotor  g_FocusMotor;
extern tBaseConfig g_BaseConfig[3];

extern tSER_PACKET g_RoutedRxSerPortPacket;
extern tSER_PACKET g_RoutedRxEthPortPacket;

extern USHORT      usUtlLampHours                 ;
extern UCHAR       ucUtlLampMinutes               ;
extern UCHAR       ucUtlLampSeconds               ;

extern UCHAR       ucUtlActiveTourNext    ;
extern tTourPt     UtlActivePoint         ;

extern UCHAR       szUtlLampModelNumber [DEVICE_NAME_LENGTH];
extern UCHAR       szUtlLampSerialNumber[DEVICE_NAME_LENGTH];
extern USHORT      usUtlAutoReturnCountdown;
extern UCHAR       ucUtlDebugMask;       // Operates with UtlQueDebugPrint

extern UCHAR       recePacket;       // The will be set to one while receiving a packet. This varible will be used to lockout transmitting anything while receiving. Added for ver 1.71 6/18/2013. 
extern UINT g_ActivePresetPanPos;
extern UINT g_ActivePresetTiltPos;

extern UINT g_PanPos;
extern UINT g_TiltPos;
                   
#endif // _UTILS_INC_