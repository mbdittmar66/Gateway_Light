//**********************************************************************
// Filename:      timers.c
// Rights:        Copyright (c) 2006 The Carlisle and Finch Company
//                All Rights Reserved
// Creation Date: 6/13/06
// Compiler:      Tasking cc51
// Version:       v7.1r1
// Purpose:       This module contains a timer interface.
// Modification History:
// Date         Author      Description
// ----         ------      -----------
// 6-13-06      JRN         Created
//**********************************************************************
        
#include "c8051f040.h"            
#include "SlProtocol.h"
#include "serial.h"
#include "timers.h"
#include "main.h"
#include "utils.h"
#include "SLC_Init.h"
#include "stdlib.h"

// -----------------
// Misc constants...
// -----------------

//
// The following values are: 10000h - (11,059,200 / 1000) = 10000h - 2b33h = D4CD
// + / -  a little tweak (from numerical measurements).
//

// 1mS
#define TIM_TH0_1_PER_MS     0x9E
#define TIM_TL0_1_PER_MS     0x58

// 2mS
//#define TIM_TH0_1_PER_MS     0xA9
//#define TIM_TL0_1_PER_MS     0x9A

//
// The following values are: 10000h - (11,059,200 / 10000) = 10000h - 2b33h = 452
// + / -  a little tweak (from numerical measurements).
//

// #define TIM_TH0_10_PER_MS   0xFB // 0xF6
// #define TIM_TL0_10_PER_MS   0xCD // 0xAE

tTimeMilliSecondTimers  g_TimeMs;

//
// Local variables / data structures...
//

//=============================================================================
// Function   : Timers ( ) 
// Purpose    : This function initializes hardware and software timers.
// Parameters : Nothing is passed. 
// Returns    : Nothing is returned.
//=============================================================================

VOID InitializeTimers ( VOID )
{
    SFRPAGE   = TIMER01_PAGE;
    TCON      = 0x50;           // Timer 0 and Timer 1 is enabled.
    TMOD      = 0x21;           // Timer 0 is 16 bit, and Timer 1 operates in Mode 2 : 8-bit timer w/auto reload.
    
    //
    // Setup Timer 0 such that interrupts are generated every 
    // millisecond.
    //
    
    TH0       = TIM_TH0_1_PER_MS;      
    TL0       = TIM_TL0_1_PER_MS;      
    
    //                            
    // CKCON: Clock Control
    //                            
    // Bits7–5: UNUSED. Read = 000b, Write = don’t care.
    //                            
    // Bit4: T1M: Timer 1 Clock Select.
    //      This select the clock source supplied to Timer 1. T1M is ignored when C/T1 is set to logic 1.
    //      0: Timer 1 uses the clock defined by the prescale bits, SCA1–SCA0.
    //      1: Timer 1 uses the system clock.
    //                            
    // Bit3: T0M: Timer 0 Clock Select.
    //      This bit selects the clock source supplied to Timer 0. T0M is ignored when C/T0 is set to
    //      logic 1.
    //      0: Counter/Timer 0 uses the clock defined by the prescale bits, SCA1-SCA0.
    //      1: Counter/Timer 0 uses the system clock.
    //                            
    // Bit2: UNUSED. Read = 0b, Write = don’t care.
    //                            
    // Bits1–0: SCA1–SCA0: Timer 0/1 Prescale Bits
    //      These bits control the division of the clock supplied to Timer 0 and/or Timer 1 if configured
    //      to use prescaled clock inputs.
    //
    //      SCA1 SCA0 Prescaled Clock
    //      ==== ==== ===============
    //      0    0    System clock divided by 12
    //      0    1    System clock divided by 4
    //      0    1    System clock divided by 4
    //      1    0    System clock divided by 48
    //      1    1    External clock divided by 8*
    //
    //       *Note: External clock divided by 8 is synchronized with the system
    //       clock, and external clock must be less than or equal to the
    //       system clock frequency to operate the timer in this mode.    
    //     
   
    
    CKCON     = 0x18;   // Timer 1 uses the system clock - 115200bps
//    CKCON     = 0x08;  // 9600 bps
//    TH1       = 0xD0;   // Timer 1 provides UART1 a baud rate of 115200bps when using "system clock"  0xD0
    TH1       = 0x27;   // Timer 1 provides UART1 a baud rate of 57600 bps with "system clock"  of 25 Mhz
    
    SFRPAGE   = TMR2_PAGE;
    TMR2CN    = 0x04;           // Enable Timer 2
    TMR2CF    = 0x08;           // Timer 2 uses SYSCLK
    RCAP2L    = 0x5D;
    RCAP2H    = 0xFF;           // Timer 2 provides UART0 a baud rate of 9600 bps.
    SFRPAGE   = TMR3_PAGE;
    
    // TMR3CN    = 0x04;           // Enable Timer 3
    TMR3CN    = 0x00;           // Disable Timer 3
    // TMR3CF    = 0x18;           // Timer 3 uses SYSCLK / 2
    // RCAP3L    = 0x00;
    // RCAP3H    = 0xFA;           // Timer 3 interrupts occur once every 250us.

    g_TimeMs.FiveSeconds = 5000;


} // InitializeTimers ()
                   
//=====================================================================
// Function   : TimeResetCop ()
// Purpose    : Kicks the watchdog to prevent a reset.
// Parameters : Nothing is passed.
// Returns    : Nothing is returned.
//=====================================================================

VOID TimeResetCop ( VOID ) 
{
//    KICK_WDT;                // Kick the watchdog

} // TimeResetCop ()

//=====================================================================
// Function   : TimeWaitTicks ()
// Purpose    : Delays program execution for a specified amount of 
//              SYSCLK cycles.
// Parameters : Number of SYSCLK cycles to delay.
// Returns    : Nothing is returned.
//=====================================================================

VOID TimeWaitTicks ( UINT WaitTimeTicks )
{
    UINT i;
    
    for ( i = 0; i < WaitTimeTicks; i++ )
    {    
#pragma asm
        NOP
#pragma endasm
    }

} // TimeWaitTicks ()

//=====================================================================
// Function   : Timer0_ISR ()                                           
// Purpose    : This interrupt is driven off of TIMER0 overflow.
//              Executed approx. once every 125us. 
//              Used for time keeping purposes.
// Parameters : Nothing is passed.
// Returns    : Nothing is returned.
//=====================================================================
                  
_interrupt ( 1 ) _large
VOID Timer0_ISR ( VOID )
{
    static UINT  *puiTimer;
    static UCHAR toggle;

    //
    // Clear the TIMER0 interrupt flag.
    //

    TF0 = 0x00;
    TH0 = TIM_TH0_1_PER_MS; // Reload
    TL0 = TIM_TL0_1_PER_MS; //       

    //
    // Toggle PWM enable line
    // 
    
    SFRPAGE = CONFIG_PAGE;
    
    if (toggle)
    {
        EN_PWM = 0;
    }
    else
    {
        EN_PWM = 1;
    }
    toggle = ~toggle;
 
    //
    // Adjust all milli-second timers...
    // Set pointer to first data item in structure, 
    // which is reserved for internal timing usage.
    //
    
    puiTimer = ( UINT * ) &g_TimeMs;
    
    // 
    // Decrement all specific milli-second timers.
    //
       
    TIMER_DECREMENT(DebugLamp           );
    TIMER_DECREMENT(LampStart           );
    TIMER_DECREMENT(FiveSeconds         );
    TIMER_DECREMENT(I2cTimeoutTimer     );
    TIMER_DECREMENT(TimeWaitMs          );
    TIMER_DECREMENT(ActiveJcs           );
    TIMER_DECREMENT(HoldoffMastering    );
    TIMER_DECREMENT(PollMain            );
    TIMER_DECREMENT(CheckPrb            );
    TIMER_DECREMENT(PollPrb             );
    TIMER_DECREMENT(PrbCommFault        );
    TIMER_DECREMENT(LampOnSecond        );
    TIMER_DECREMENT(PrbBlanked          );
    TIMER_DECREMENT(RxTimeout[0]        );
    TIMER_DECREMENT(RxTimeout[1]        );
    TIMER_DECREMENT(RxTimeout[2]        );
    TIMER_DECREMENT(RxTimeout[3]        );
	TIMER_DECREMENT(CheckMotorAccel     );
	TIMER_DECREMENT(OverTempInput       );		// Added v1.72
	TIMER_DECREMENT(MotorTimeout		);		// Added v1.73
                        
    //
    // Kick WDT timer once per ms, only here.
    //

    WDTCN=0xA5;
    
    //
    // Once every 5 seconds, consider driving a reset operation if the
    // main loop is stuck.
    //

    if (g_TimeMs.FiveSeconds == 0 )
    {
        g_TimeMs.FiveSeconds = 5000;
        while (0 == ucMainCycleCount);
        ucMainCycleCount = 0;
    }

} // TimerService1MsPops ( )

