//**********************************************************************
// Filename:      main.h
// Rights:	      Copyright (c) 2006 The Carlisle and Finch Company
// 			      All Rights Reserved
// Creation Date: 01/13/07
// Compiler:      Tasking cc51
// Version:       v7.1r1
// Purpose: 	  This module contains IIC constants and function prototypes.
// Modification History:
// Date			Author		Description
// ----     	------      -----------
// 01/13/07     sc			Created
//**********************************************************************

#ifndef _MAIN_H
#define _MAIN_H

// ===========================================
// Constants
// ===========================================


//
// Status bits:
//

#define MAIN_STATUS_TIMER_SLOTS		1
#define MAIN_STATUS_POWER_ON_RESET	2

// ===========================================
// Recognized globals
// ===========================================

typedef struct 
{
    UINT    uiMainStatus;
    UINT    uiMainCycles;
    UINT    uiUart0Isr;
    UINT    uiUart1Isr;
    UINT    uiI2cIsr;
    
} tMainStats;

extern tMainStats   MainStats;
extern UCHAR        ucMainCycleCount;


// ===========================================
// Function prototypes
// ===========================================

              
              
#endif // _MAIN_H
