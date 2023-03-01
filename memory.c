//**********************************************************************
// Filename:      memory.c
// Rights:        Copyright (c) 2006 The Carlisle and Finch Company
//                All Rights Reserved
// Creation Date: 7/05/06
// Compiler:      Tasking cc51
// Version:       v7.1r1
// Purpose:       This module contains AT24C04A Serial EEPROM interface.
// Modification History:
// Date         Author      Description
// ----         ------      -----------
// 10-12-06		JRN			Created
//**********************************************************************

#include "c8051f040.h"
#include "memory.h"
#include "i2c.h"


//=============================================================================
// Function   : MemReadData ()
// Purpose    : Retrieves data from the AT24C04A 512-byte EEPROM.
// Parameters : 16-bit address, pointer to buffer, number of bytes to read.
// Returns    : Nothing is returned.
//=============================================================================

VOID  MemReadData  ( UINT Addr, UCHAR *pData, UINT Size )
{
    UINT DeviceAddr;
    UINT i;
    
    //
    // Do not overflow memory address range.
    //
    
    if ( ( Addr + Size ) > 0x200 )
        return;
    
    //
    // Read specified memory from EEPROM.
    //
         
    for ( i = 0; i < Size; i++, pData++ )
    {
        //
        // Determine which memory block of EEPROM is being accessed.
        //
    
        DeviceAddr = SER_EEPROM_ADDR;       // Memory block 0
        if ( Addr > 0xFF )
            DeviceAddr |= 0x01;             // Memory block 1
            
        *pData = I2cReadByte ( DeviceAddr, Addr + i );
        
    } // for ( i = 0; i < Size; i++ )

} // MemReadData ()



//=============================================================================
// Function   : MemWriteData ()
// Purpose    : Retrieves data from the AT24C04A 512-byte EEPROM.
// Parameters : 16-bit address, pointer to buffer, number of bytes to write.
// Returns    : Nothing is returned.
//=============================================================================

VOID MemWriteData ( UINT Addr, UCHAR *pData, UINT Size )
{
    UINT DeviceAddr;
    UINT i;
    
    //
    // Do not overflow memory address range.
    //
    
    if ( ( Addr + Size ) > 0x200 )
        return;
    
    //
    // Read specified memory from EEPROM.
    //
         
    for ( i = 0; i < Size; i++, pData++ )
    {
        //
        // Determine which memory block of EEPROM is being accessed.
        //
    
        DeviceAddr = SER_EEPROM_ADDR;       // Memory block 0
        if ( Addr > 0xFF )
            DeviceAddr |= 0x01;             // Memory block 1
            
        I2cWriteByte ( DeviceAddr, ( Addr + i ), *pData );
        
    } // for ( i = 0; i < Size; i++ )

} // MemWriteData ()
