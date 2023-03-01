//**********************************************************************
// Filename:      pelco.h
// Rights:        Copyright (c) 2017 The Carlisle and Finch Company
//                All Rights Reserved
// Creation Date: 11/21/17
// Compiler:      Tasking cc51
// Version:       v7.2r6
// Purpose:       This module contains Pelco D protocol constants and function prototypes.
// Modification History:
// Date         Author      Description
// ----         ------      -----------
// 11-21-17      ACM         Created
//**********************************************************************

#ifndef _PELCO_INC_
#define _PELCO_INC_
       
#include "utils.h"

// ===========================================
// Constants
// ===========================================

#define CHECK_BIT(variable, bit)   (variable & (1 << bit))

// ===========================================
// Data structures
// ===========================================

enum tPelcoPayload
{
	Command1,
	Command2,
	Data1,
	Data2
};

enum tStandardCommand1    // define each bit in "Standard Command" Command1 byte
{
  	FocusNear,
  	IrisOpen,
  	IrisClose,
  	CameraOnOff,            // turn on camera if this bit and "Sense" both set, turn off camera if this bit is set and "Sense" is 0
  	AutoManualScan,         // turn on Auto Scan if this bit and "Sense" both set, turn off Auto Scan if this bit is set and "Sense" is 
  	Reserved1,              // Set to 0
  	Reserved2,              // Set to 0
  	Sense
};

enum tStandardCommand2    // define each bit in "Standard Command" Command2 byte
{
  	ExtendedCommand,               // Always 0 for standard command, 1 for extended command
  	Right,
  	Left,
  	Up,
  	Down,
  	ZoomIn,
  	ZoomOut,
  	FocusFar
};

enum tExtendedCommands    // Command2 value, Command1 always 0
{
  	SetPreset       = 0x03,   // Data2: 01-20
  	ClearPreset     = 0x05,   // Data2: 01-20
  	GoToPreset      = 0x07,   // Data2: 01-20, 21 = Flip (180deg), 22 = Zero Pan (home)
  	SetAuxiliary    = 0x09,   // 01 to 08 (turn on aux function)
  	ClearAuxiliary  = 0x0B,   // 01 to 08 (turn off aux function)
  	SetZoomSpeed    = 0x25,   // 00 to 03
  	SetFocusSpeed   = 0x27    // 00 to 03
};

// ===========================================
// Function prototypes
// ===========================================

#endif // _PELCO_INC_