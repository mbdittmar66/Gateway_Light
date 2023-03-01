//**********************************************************************
// Project:       Searchlight Controller Board
// Filename:      main.c
// Rights:        Copyright (c) 2006 The Carlisle & Finch Co.
//                All Rights Reserved
// Creation Date: 9/11/06
// Compiler:      Tasking cc51
// Version:       v7.1r1
// Purpose:       This module contains main program interface.
// Modification History:
// Date         Author      Description
// ----         ------      -----------
// 9-11-06		JRN			Created
//**********************************************************************

#include "C8051F040.h"
#include "SlProtocol.h"
#include "SLC_Init.h"
#include "utils.h"
#include "main.h"
#include "spi.h"
#include "serial.h"
#include "timers.h"
#include "stdlib.h"
#include "version.h"
#include "flash.h"

//
//  Recognized globals
//

tMainStats MainStats;
UCHAR      ucMainCycleCount = 1;
UINT		uiOneSecondCounts = 0;  // mbdtest
static UCHAR		toggle;
//=============================================================================
// Function   : main ()
// Purpose    : This function must never return.
// Parameters : Nothing is passed. 
// Returns    : Nothing is returned.
//=============================================================================

VOID main ( VOID )
{
    InitializeSLC ();
    
    //
    // Setup various recurring timer dependent functions.
    //
    
    StdPrintf     ( &g_SerTxQueSerPort,
                STR_NEW_LINE     "Search Light Controller" 
                STR_NEW_LINE      VERSION_STR 
                STR_NEW_LINE      "(c) 2006-2020 The Carlisle & Finch Company"
                STR_NEW_LINE      "ALL RIGHTS RESERVED");

    if (ucSlcRstsrc & 0x40)
    {
        StdPrintf     ( &g_SerTxQueSerPort, STR_NEW_LINE "CNVSTR0 Reset"  );
    }
    if (ucSlcRstsrc & 0x20)
    {
        StdPrintf     ( &g_SerTxQueSerPort, STR_NEW_LINE "Comparator 0 Reset"  );
    }
    if (ucSlcRstsrc & 0x10)
    {
        StdPrintf     ( &g_SerTxQueSerPort, STR_NEW_LINE "Software Reset"  );
    }
    if (ucSlcRstsrc & 0x08)
    {
        StdPrintf     ( &g_SerTxQueSerPort, STR_NEW_LINE "WDT Reset"  );
    }
    if (ucSlcRstsrc & 0x04)
    {
        StdPrintf     ( &g_SerTxQueSerPort, STR_NEW_LINE "Clock Detector Reset"  );
    }
    if (ucSlcRstsrc & 0x02)
    {
        StdPrintf     ( &g_SerTxQueSerPort, STR_NEW_LINE "Power Reset"  );
    }
    if (ucSlcRstsrc & 0x01)
    {
        StdPrintf     ( &g_SerTxQueSerPort, STR_NEW_LINE "Hardware Pin Reset"  );
    }                                                               

    StdPrintf     ( &g_SerTxQueSerPort, STR_NEW_LINE STR_PROMPT, g_BoardNumber+1);

    //
    // Spin forever, do not exit.
    //

    while ( TRUE )
    {
        SFRPAGE = CONFIG_PAGE;
    
	    if (toggle)
    	{
//        	TestBit0 = 0;	   mbd need test points with Rev H SLC
    	}
    	else
    	{
//        	TestBit0 = 1;
    	}
    	toggle = ~toggle;
        
        //
        // Count the cycles though main loop, don't
        // reach zero by design.
        //

        if (0xFF != ucMainCycleCount)
        {
            ++ucMainCycleCount;
        }

        // =================== CRITICAL CODE POINT ==============
        // Kick the watchdog in the main loop.  This is the only
        // place in the code paths through the main loop where
        // the dog is kicked.  If the WDT reset fires then that
        // suggests the rule of never block the CPU has been violated.
        // If that rule is voilated then the protocol timing and the
        // entire system flow is FOOBAR.  So, Mr. Developer, do not
        // break the rule of never block the CPU!
        // ======================================================
        KICK_WDT;                // Kick the watchdog

        SpiCheckUart2  ( );
        SpiCheckUart3  ( );
        UtilCheckProtocolChannels ( );                      // Check for incoming commands.
        UtilPollMain ( );

        //
        // Check Pot reader board as required.
        //


        
        if (!g_TimeMs.CheckPrb)
        {
            g_TimeMs.CheckPrb = 10;
            UtilCheckPotReaderBoard ( );
        }

        //
        // Poll PRB channel as required.
        //
        
        if (!g_TimeMs.PollPrb)
        {
            g_TimeMs.PollPrb = 50;
            UtilPollPrb ( );
        }

		if (!g_TimeMs.CheckMotorAccel )
		{
			g_TimeMs.CheckMotorAccel = 25;
			UtilCheckMotorAccel ( );
		}

        //
        // v1.65 -  If no longer being controlled, then show no active device..
        // 
        
        if (0 == g_TimeMs.ActiveJcs)
        {
            SerSlcExtStatusPayload.BasicStatus.ucControllingDevice = INVALID_PROTOCOL_ADDRESS;
        }
        
        //
        // v1.65 - If lamp start relay is being held ON, automatically 
        //         turn it OFF when timer is below 100ms.
        //

        if ((g_TimeMs.LampStart) && (g_TimeMs.LampStart < 100))
        {
            g_TimeMs.LampStart = 0;
            UtlRelayOff (RELAY_LAMP_START);
        }
        
        //
        // Do once per second housekeeping
        //
        
        if (!g_TimeMs.LampOnSecond )
        {
            g_TimeMs.LampOnSecond = 1000;
			
			uiOneSecondCounts ++ ;
			if ( uiOneSecondCounts ==  TIME_TO_RESET_SLC_SECONDS )
			{
			uiOneSecondCounts = 0;  // mbd
 
			// need to check if BEAM is on....skip the reset if so
 				if ( !(SerSlcExtStatusPayload.BasicStatus.ucByte7 & SLC_STAT_B7_BEAM_ON) )
 				{
 				SFRPAGE = LEGACY_PAGE;
 			    RSTSRC = 0x01;          // Hard Reset
 				}
			}

			            
            ucUtlJcsStatByte3 |= JCS_STAT_B3_REQ_SLC_EXT_STAT_PKT;          // Added v1.72: Send extended status packet every second to keep JCS's updated (for NMEA output)   
            
            //
            // Once per second check lamp life remaining.  Assume
            // zeroized Lamp life status bits.
            //
            
            
            SerSlcExtStatusPayload.BasicStatus.ucByte7 &= ~(SLC_STAT_B7_10_PERCENT_LAMP_LEFT |
                                                            SLC_STAT_B7_END_LAMP_LIFE        );
            
            if (SerSlcMiscSetupPayload.ucByte1Details & SLC_SETUP_B1_ENABLE_END_OF_LIFE_MSG)
            {
                if (SerSlcExtStatusPayload.usLampHours > SerSlcMiscSetupPayload.usLampLifeHours)
                {
                    SerSlcExtStatusPayload.BasicStatus.ucByte7 |= SLC_STAT_B7_END_LAMP_LIFE;
                } 
                else if (SerSlcExtStatusPayload.usLampHours > usFlash90PercentLampLifeHours)
                {
                    SerSlcExtStatusPayload.BasicStatus.ucByte7 |= SLC_STAT_B7_10_PERCENT_LAMP_LEFT;
                    SerSlcExtStatusPayload.ucPercentLampUsed = ((100 * SerSlcExtStatusPayload.usLampHours) / 
                                                           SerSlcMiscSetupPayload.usLampLifeHours);
                }
            }
            
            //
            // Adjust SL On time as required.
            //
        
            if (SerSlcExtStatusPayload.BasicStatus.ucByte7 & SLC_STAT_B7_BEAM_ON)
            {
                ++ucUtlLampSeconds;
            
                if (ucUtlLampSeconds >= 60)
                {
                    ++ucUtlLampMinutes;
                    ucUtlLampSeconds = 0;
                }
                if (ucUtlLampMinutes >= 60)
                {
                    ++usUtlLampHours;
                    ucUtlLampMinutes = 0;
                }
            }
            
            //
            // When Touring, and not paused, figure end of dwelling...
            //
            
            if (    ( TOUR_INVALID         != SerSlcExtStatusPayload.BasicStatus.ucActiveTour ) &&
                    ( 0 == (TOUR_PAUSED_BIT & SerSlcExtStatusPayload.BasicStatus.ucActiveTour)) &&
                    ( SerSlcExtStatusPayload.usTourDwellCountdown                             ) )
            {
                --SerSlcExtStatusPayload.usTourDwellCountdown;
                if (!SerSlcExtStatusPayload.usTourDwellCountdown)
                {
                    UtilNextTourPoint ( );
                }
            }
            
            //
            // When not touring, figure time for auto home
            //
            
            else if (usUtlAutoReturnCountdown)
            {
                if (SerSlcExtStatusPayload.BasicStatus.ucByte7 & SLC_STAT_B7_BEAM_ON)		   // Added v1.72: If beam is on, reset the auto-home timer (previously auto-homed when beam was on)
            	{
					usUtlAutoReturnCountdown = SerSlcMiscSetupPayload.usAutoReturnSeconds;
				}
                --usUtlAutoReturnCountdown;
                if (!usUtlAutoReturnCountdown)
                {
                    UtilGotoHomePosition ( );
                }
            }


            
            //
            // Once per second gander at the overtemp input
            //
            
            SFRPAGE    = CONFIG_PAGE;
            if ( LAMP_OVERTEMP )
            {
                if ( !g_TimeMs.OverTempInput )	 // Added v1.72 (code within braces was present prior...): Don't immediately set bit (also indicates over/under current, which activates on 1000W startup)
                {                                
	                //
	                // Make sure overtemp is communicated...
	                //
	                
	                ucUtlJcsStatByte3 |= JCS_STAT_B3_REQ_SLC_EXT_STAT_PKT;
	                SerSlcExtStatusPayload.ucByte9 |= SLC_EXT_STAT_B9_TEMP_FAIL;
				}
            } 
            else
            { 
                g_TimeMs.OverTempInput = 2000;	// Added v1.72: Reset timer if pin is clear

                //
                // When transitioning from ON to OFF, make sure indication is communicated.
                //
                
                if (  SerSlcExtStatusPayload.ucByte9 & SLC_EXT_STAT_B9_TEMP_FAIL )
                {
                    ucUtlJcsStatByte3 |= JCS_STAT_B3_REQ_SLC_EXT_STAT_PKT;
                }
                
                // 
                // Clear overtemp indication.
                //
                
                SerSlcExtStatusPayload.ucByte9 &= ~SLC_EXT_STAT_B9_TEMP_FAIL;
            }
            
        } // if (!g_TimeMs.LampOnSecond )
        
 
    
        ++MainStats.uiMainCycles;
        
	} // while ( TRUE )

} // main ()

        