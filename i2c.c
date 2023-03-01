//**********************************************************************
// Filename:      i2c.c
// Rights:        Copyright (c) 2006 The Carlisle and Finch Company
//                All Rights Reserved
// Creation Date: 7/05/06
// Compiler:      Tasking cc51
// Version:       v7.1r1
// Purpose:       This module contains IIC interface.
// Modification History:
// Date         Author      Description
// ----         ------      -----------
// 7-05-06		JRN			Created
//**********************************************************************

#include "c8051f040.h"
#include "main.h"
#include "i2c.h"
#include "timers.h"

static INT   g_Status;        		 // holds the current status
static UCHAR g_DeviceAddr;           // holds the address of the target device
static UCHAR g_StorageAddr;          // holds the address to read from or write to
static UCHAR g_IsrDataByte;          // holds the value read or the value to write
static UCHAR g_State;                // holds the current transmit state - LOWER_ADDRESS, DATA BYTE, etc.
static UCHAR g_Operation;            // holds the operation - READ or WRITE




//=============================================================================
// Function   : I2cInitializeSMBus ()
// Purpose    : This function initializes the Serial Management Bus.
// Parameters : Nothing is passed. 
// Returns    : Nothing is returned.
//=============================================================================

VOID I2cInitializeSMBus ( VOID )
{
    SFRPAGE   = SMB0_PAGE;
    SMB0CN    = 0x40;
    SMB0CR    = 0x8D// 0xCD;

} // I2cInitializeSMBus ()



//=============================================================================
// Function   : I2cReadByte ()
// Purpose    : Reads a byte at a specific address.
// Parameters : 8-bit device address, 8-bit internal source address.
// Returns    : 8-bit data value.
//=============================================================================

UCHAR I2cReadByte ( UCHAR device, UCHAR address )
{
    UINT LoopIter = 0;
    
    //
    // Set the interface status to busy.
    //
    
    g_Status = I2C_BUSY;
    
    //
    // Store the device address and target address for
    // for use in the ISR.
    //
    
    g_DeviceAddr  = device;
    g_StorageAddr = address;
    
    //
    // Setup for a read operation , which requires a dummy write
    // to configure the address of the slave.
    // 
    
    g_Operation = READ;
    g_State     = DUMMY_WRITE;
    
    //
    // Request to transmit a start condition.
    // This places the peripheral in master transmitter mode.
    //
    
    SFRPAGE = SMB0_PAGE;
    STA = 1;

	//
	// Wait for read operation to complete.
    // Return the byte read from slave device.
	//

    while ( ( g_Status  == I2C_BUSY ) &&
            ( ++LoopIter < 1000     ) );
	
    return g_IsrDataByte;
    
} // I2cReadByte ()


//=============================================================================
// Function  : I2cWriteByte ()
// Purpose   : Stores a byte at a specific address.
// Parameters: 8-bit device address, 8-bit source address, and byte to store.
// Returns   : Nothing is returned.
//=============================================================================

VOID I2cWriteByte (UCHAR device, UCHAR address, UCHAR byte)
{   
    UINT LoopIter = 0;
                           
    //
    // Store the device address, target address, and data byte
    // for use in the ISR.
    //
    
    g_DeviceAddr  = device;
    g_StorageAddr = address;
    g_IsrDataByte = byte;
    
    //
    // Setup for a write operation.
    // Set the interface status to busy.
    //
    
    g_Status    = I2C_BUSY;
    g_Operation = WRITE;
    
    //
    // Request to transmit a start condition.
    // Places peripheral in master transmitter mode.
    //
    
    SFRPAGE = SMB0_PAGE;
    STA = 1;

	//
	// Wait for write operation to complete.
	//

    while ( ( g_Status  == I2C_BUSY ) &&
            ( ++LoopIter < 1000     ) );
    
} // I2cWriteByte ()


//=============================================================================
// Function   : I2C_ISR ()
// Purpose    : I2C interrupt service routine. An interrupt is requested 
//              whenever an event occurs on the I2C bus causing the I2C 
//              peripheral to change state. The state is held in the SMB0STA 
//              SFR register.
// Parameters : Nothing is passed. 
// Returns    : Nothing is returned.
// Note       : If debugging is inserted into this function the execution
//              will be slowed down to the point where the timeout occurs.
//=============================================================================

_interrupt ( 7 ) _large
VOID I2C_ISR ( VOID )
{
    //
    // Accumulate statistics...
    //
    
    ++MainStats.uiI2cIsr;
    
	switch ( SMB0STA )
	{
    	// START CONDITION TRANSMITTED
    	case 0x08:
      		//
      		// Clear start condition flag so another start condition is not requested.
      		// Transmit device address + write command (SLA + W).
			//

      		STA = 0;
	  		SMB0DAT = g_DeviceAddr & 0xFE;    
            
            //
            // Start I2C timeout timer. Allow 60ms timeout period.
            //
            
            g_TimeMs.I2cTimeoutTimer = I2C_TIMEOUT;
      		
            break;
		// REPEATED START CONDITION TRANSMITTED
    	case 0x10:
      		//
      		// Clear start condition flag so another start condition is not requested.
      		//

      		STA = 0;  
      		
      		//
			// If performing a read operation and dummy write is completed...
			//

      		if ( ( g_Operation == READ ) && ( g_State != DUMMY_WRITE ) ) 
            {
	    		  //
	    		  // Transmit device address + read command (SLA + R)
				  //

	    		  SMB0DAT = g_DeviceAddr | 0x01;
  	  		} else {
			  	//
			  	// Transmit device address + write command (SLA + W)
                // This situation will arise if no response was received from the device
   	            // and another attempt is being made.
				//

				SMB0DAT = g_DeviceAddr & 0xFE;
			}
			break;
		// SLAVE ADDRESS + WRITE TRANSMITTED - ACK RECEIVED
    	case 0x18:
	  		//
			// Transmit 8 high bits of address.
			// Next time around transmit lower part of address.
			//

	  		//SMB0DAT = (i2c_storage_address >> 8) & 0x0F;
			//i2c_state = LOWER_ADDRESS;
      		
            
	  		SMB0DAT = g_StorageAddr;
			g_State = DATA_BYTE;
            
            break;
		// SLAVE ADDRESS + WRITE TRANSMITTED - NO ACK RECEIVED
    	case 0x20:
            //
            // Check first to see if a timeout situation has occured.
            //
            
            if ( g_TimeMs.I2cTimeoutTimer == 0 )
            {
                //
                // Request to transmit a stop condition,
                // and return error status.
                //
                
                STO = 1;
                g_Status = I2C_ERROR;
            }
            else
            {    
	            //
			    // Request to transmit a repeated start - try again to communicate.
			    //

	            STA = 1;                      
            }
		    break;
		// DATA BYTE OR UPPER ADDRESS OR LOWER ADDRESS TRANSMITTED - ACK RECEIVED
    	case 0x28:
	  		switch( g_State ) 
            {
				// Next byte to transmit is lower part of address.
				case LOWER_ADDRESS:
					//
					// Transmit lower 8 bits of address.
					// Next time around transmit the data byte.
					//

			    	SMB0DAT = g_StorageAddr & 0xFF;
	    		  	g_State = DATA_BYTE;              
	      			break;
	    		// Next byte to transmit is the data byte.
	    		case DATA_BYTE:
          			//
					// If performing a read, dummy write cycle ends here.
					// No data is transmitted.
					//
					if ( g_Operation == READ ) 
                    {
					    //
						// Request to transmit repeated start.
						//

					    STA = 1;
		  			} 
                    else 
                    {
						//
						// Transmit the data byte.
						// Next time around the write cycle is finished.     
						//

			            SMB0DAT = g_IsrDataByte;
		   			    g_State = FINISHED;
          			}
		          	break;
				// Finished writing byte
	    		case FINISHED:
					//
					// Request to trasmit a stop condition.
					// Status of operation is OK.
					//

					STO = 1;
		  			g_Status = I2C_OK;
		  			break;
      		}
	  		break;
		// DATA BYTE OR UPPER ADDRESS OR LOWER ADDRESS TRANSMITTED - NO ACK RECEIVED
		case 0x30:
	  		//
			// No response. Request to transmit a stop condition.
			// Status of operation is ERROR.
			//

	  		STO = 1;               
	  		g_Status = I2C_ERROR;
      		break;
		// SLAVE ADDRESS + READ TRANSMITTED - ACK RECEIVED
		case 0x40:
	  		//
			// No action is performed. Next time an interrupt is generated,
			// the data byte will have been received from the device.
			//

	  		break;
		// SLAVE ADDRESS + READ TRANSMITTED - NO ACK RECEIVED
		case 0x48:
      		//
			// Request to transmit a stop condition.
			// Status of operation is ERROR.
			//

      		STO = 1; 
	  		g_Status = I2C_ERROR;
      		break;
		// DATA BYTE RECEIVED - NO ACK TRANSMITTED
    	case 0x58:
      		//
			// Data byte has been received. Read the data byte. 
			// Status of operation is OK. Request to transmit stop condition.			
			//

      		g_IsrDataByte = SMB0DAT;
	  	    g_Status = I2C_OK;
      		STO = 1;            
      		break;
		// UNKNOWN STATE
    	default:
	  		//
			// Request to transmit stop condition.
			// Status of operation is ERROR.
	  		//
	  		
	  		STO = 1;                        
	  		g_Status = I2C_ERROR;         

			//
			// Disable the I2C interrupt.
			//

	  		EIE1 &= ~0x02;                  
	  		break;
  	}
	
	//
	// Clear the I2C interrupt flag. This causes the next event on the
    // I2C bus to be performed, possibly resulting in the request of
	// another I2C interrupt.
	//

	SI = 0;

} // I2C_ISR ()

