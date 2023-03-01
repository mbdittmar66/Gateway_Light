//**********************************************************************
// Filename:      pelco.c
// Rights:        Copyright (c) 2017 The Carlisle and Finch Company
//                Added to SLC firmware in v1.73
// Creation Date: 11/21/17
// Compiler:      Tasking cc51
// Version:       v7.2r6
// Purpose:       This module contains processing of Pelco D commands interface.
// Modification History:
// Date         Author      Description
// ----         ------      -----------
// 11-21-17      ACM         Created
//**********************************************************************
        
#include "c8051f040.h"            
#include "utils.h"
#include "pelco.h"
#include "SlProtocol.h"
#include "stdlib.h"
#include "flash.h"  // mbd

// -----------------
// Misc constants...
// -----------------

//
// Local variables / data structures...
//

//=============================================================================
// Function   : PelcoProcessCommand( )
// Purpose    : This function decodes a Pelco D command and issues SV command.
// Parameters : Nothing is passed. 
// Returns    : Nothing is returned.
//=============================================================================

VOID _reentrant PelcoProcessCommand ( tProtocolChannels *pChannel )
{
	tPAYLOAD_JCS_STATUS	JcsStatusPayload;
	static UCHAR ucZoomSpeed = 3;	// set to max zoom speed
	tPresetPolarPos  *CurrentPresetPolarPos;  // mbd
	UINT		temp1,temp2;
	SHORT		temp3,temp4,sPreset_Tilt_Rel;
	LONG		lPreset_Pan_Rel;
	// Initialize X, Y, Z values to "center"
	JcsStatusPayload.ucX = 127;
	JcsStatusPayload.ucY = 127;
	JcsStatusPayload.ucZ = 127;

	// Treat incoming Pelco D command as a Joystick Status packet from SerPort that needs to be routed
	StdMemClr(&g_RoutedRxSerPortPacket, sizeof(g_RoutedRxSerPortPacket));
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
    g_RoutedRxSerPortPacket.Preamble.PayloadLength = 5;
    	
	if(!CHECK_BIT(pChannel->ucPayload[Command2],ExtendedCommand) )	// check to see if "Extended Command" bit (LSB of second command byte) is set
	{
		// Standard Command handler
		
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
	}
	else
	{
		// Extended Command handler
		switch(pChannel->ucPayload[Command2])
		{
			case SetPreset:
				ucFlashActivePresetNum = pChannel -> ucPayload[Data2];  // grab the preset number from PELCO stream,
				if (ucFlashActivePresetNum > 0x20)  // user trying to set HOME via PELCO-D ( which is preset 0x22 ), or any preset greater than 0x20
				{
				// prob do nothing
				}
				else
				{
				CurrentPresetPolarPos->Preset_Pan = g_PanPos;
				CurrentPresetPolarPos->Preset_Tilt = g_TiltPos;

				FlashStorePresetPoint ( ucFlashActivePresetNum, CurrentPresetPolarPos  );
				}  
				break;

			case ClearPreset:
				break;

			case GoToPreset:
				 ucFlashActivePresetNum = pChannel -> ucPayload[Data2];
				 if ( ucFlashActivePresetNum == 0x22 )	  // this is the case if the PelcoD "HOME" preset is set.  Always use the SLC home preset.
				{
					UtilGotoHomePosition();
				}
				else if ( 0x01 <= ucFlashActivePresetNum <= 0x20 ) 
				{
				 FlashRetrievePresetPoint ( ucFlashActivePresetNum, CurrentPresetPolarPos );
				 temp1 = CurrentPresetPolarPos->Preset_Pan	;
 				 temp2 = CurrentPresetPolarPos->Preset_Tilt	 ;
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
						JcsStatusPayload.ucByte4 = JCS_STAT_B4_LAMP_ON_START;
						break;
				}
				break;

			case ClearAuxiliary:
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
						JcsStatusPayload.ucByte4 = JCS_STAT_B4_LAMP_OFF;
						break;				
				}
				break;

			case SetZoomSpeed:	// used for beam width speed
				ucZoomSpeed = pChannel->ucPayload[Data2];
				break;

			case SetFocusSpeed:	// not currently used
				break;				 
		}
	}

	StdMemCopy(g_RoutedRxSerPortPacket.pPayload, (UCHAR*) &JcsStatusPayload, g_RoutedRxSerPortPacket.Preamble.PayloadLength); 

	if (g_RoutedRxSerPortPacket.Preamble.DestAddr == g_BoardNumber + SLC_PROTOCOL_OFFSET)
	{
		pChannel->ucDestinationAddress = g_RoutedRxSerPortPacket.Preamble.DestAddr;
		pChannel->ucSourceAddress = g_RoutedRxSerPortPacket.Preamble.SrcAddr;
		pChannel->ucPayloadLength = 5;
		pChannel->ucPacketType = g_RoutedRxSerPortPacket.Preamble.PacketType;
		StdMemCopy(pChannel->ucPayload, g_RoutedRxSerPortPacket.pPayload, g_RoutedRxSerPortPacket.Preamble.PayloadLength); 
		UtilProcessPacket ( pChannel );
	}

} // PelcoProcessCommand ()
                   	