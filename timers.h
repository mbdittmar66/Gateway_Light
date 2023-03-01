//**********************************************************************
// Filename:      timers.h
// Rights:        Copyright (c) 2006 The Carlisle and Finch Company
//                All Rights Reserved
// Creation Date: 6/13/06
// Compiler:      Tasking cc51
// Version:       v7.1r1
// Purpose:       This module contains timer constants and function prototypes.
// Modification History:
// Date         Author      Description
// ----         ------      -----------
// 6-13-06      JRN         Created
//**********************************************************************

#ifndef _TIMERS_INC_
#define _TIMERS_INC_
       
// ===========================================
// Constants
// ===========================================

#define TIME_LOCKSTEP_MS            25
#define	SIXTY_SECONDS				60000
#define TIME_TO_RESET_SLC_SECONDS	12*60*60		// 12 hours

// ===========================================
// Data structures
// ===========================================

#define TIMER_STATUS_ACTIVE     0x01
#define TIMER_STATUS_SIGNALED   0x02

typedef struct
{
    UINT  DebugLamp;
    UINT  LampStart;
    UINT  FiveSeconds;
    UINT  I2cTimeoutTimer;
    UINT  TimeWaitMs;
    UINT  ActiveJcs;
    UINT  HoldoffMastering;
    UINT  PollMain;
    UINT  CheckPrb;
    UINT  PollPrb;
    UINT  PrbCommFault;
    UINT  LampOnSecond;
    UINT  PrbBlanked;
    UINT  RxTimeout[SER_NUMBER_UARTS];
	UINT  CheckMotorAccel;
	UINT  OverTempInput;	// Added v1.72, don't indicate overtemp bit until 3 seconds have elapsed (also indicates over/under current, which activates on 1000W startup)
	UINT  MotorTimeout;		// v1.73
                        
} tTimeMilliSecondTimers;

#define TIMER_DECREMENT(_a_) { if (g_TimeMs._a_) \
                                   --g_TimeMs._a_; \
						     }



// ===========================================
// uiValue of TimerMsSetupCallback...
// ===========================================

#define TIMER_ZAP       0xFFFF

// ===========================================
// Function prototypes
// ===========================================

VOID InitializeTimers     ( VOID );
VOID TimeSyncExecution    ( VOID );
VOID TimeResetCop         ( VOID );
VOID TimeWaitTicks        ( UINT WaitTimeTicks );
VOID TimerMsSetupCallback ( UINT uiValue, UINT pFunction);
VOID TimerServiceCallbacks( VOID );
VOID TimerService1MsPops  ( VOID );


// ===========================================
// Global variables
// ===========================================

extern tTimeMilliSecondTimers   g_TimeMs;
extern UINT                     guiTime250uSTicks;
extern UINT                     g_uiMotorTimeout;
#endif // _TIMERS_INC_