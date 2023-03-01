//**********************************************************************
// Filename:      memory.h
// Rights:	      Copyright (c) 2006 The Carlisle and Finch Company
// 			      All Rights Reserved
// Creation Date: 7/05/06
// Compiler:      Tasking cc51
// Version:       v7.1r1
// Purpose: 	  This module contains AT24C04A Serial EEPROM 
//                constants and function prototypes.
// Modification History:
// Date			Author		Description
// ----     	------      -----------
// 10-12-06		JRN			Created
//**********************************************************************

#ifndef _MEM_INC_	   
#define _MEM_INC_

// ===========================================
// Constants
// ===========================================

#define SER_EEPROM_ADDR         0xA0    // I2C address of Serial EEPROM

// ===========================================
// Function prototypes
// ===========================================

VOID MemReadData  ( UINT Addr, UCHAR *pData, UINT Size );
VOID MemWriteData ( UINT Addr, UCHAR *pData, UINT Size );


#endif // _MEM_INC_
