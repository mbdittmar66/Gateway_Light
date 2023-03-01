//**********************************************************************
// Filename:      i2c.h
// Rights:	      Copyright (c) 2006 The Carlisle and Finch Company
// 			      All Rights Reserved
// Creation Date: 7/05/06
// Compiler:      Tasking cc51
// Version:       v7.1r1
// Purpose: 	  This module contains IIC constants and function prototypes.
// Modification History:
// Date			Author		Description
// ----     	------      -----------
// 7-05-06		JRN			Created
//**********************************************************************

#ifndef _I2C_INC_	   
#define _I2C_INC_

// ===========================================
// Constants
// ===========================================

#define I2C_BUSY  	    0				// i2c_state holds I2C_BUSY during a read or write operation
#define I2C_ERROR	    1       		// i2c_state holds I2C_ERROR or I2C_OK after reading
#define I2C_OK    	    2       		// or writing a byte to indicate the result of the operation

#define LOWER_ADDRESS	1			    // States during an I2C operation.
#define DATA_BYTE		2
#define FINISHED		4
#define DUMMY_WRITE		8

#define I2C_TIMEOUT     60;             // I2C default timeout value

#define READ			1			    // I2C operations
#define WRITE			2

// ===========================================
// Function prototypes
// ===========================================

VOID  I2cInitializeSMBus( VOID );
UCHAR I2cReadByte       ( UCHAR device, UCHAR address );
VOID  I2cWriteByte      ( UCHAR device, UCHAR address, UCHAR  byte );
              
#endif // _I2C_INC_
