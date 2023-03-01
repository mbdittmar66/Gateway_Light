//**********************************************************************
// Filename:      stdlib.c
// Rights:        Copyright (c) 2006 The Carlisle and Finch Company
//                All Rights Reserved
// Creation Date: 6/13/06
// Compiler:      Tasking cc51
// Version:       v7.1r1
// Purpose:       This module contains a standard library function interface.
// Modification History:
// Date         Author      Description
// ----         ------      -----------
// 6-13-06      JRN         Created
//**********************************************************************

#include "c8051f040.h"
#include "SlProtocol.h"
#include "serial.h"
#include "stdlib.h"
#include "spi.h"
#include "flash.h"

//
// Local function prototypes.
//

//
// Module local ("static") globals...
//

//
// Support which allows "StdPutc" to either target a
// TX / RX que, or otherwise target a RAM buffer
// for sprintf operations.
//

#define PUTC_TYPE_QUE       0
#define PUTC_TYPE_SPRINTF   1
#define PUTC_TYPE_FLASH     2

typedef struct
{ 
    UCHAR   ucQueOrSprintfRam;  // Must be 1st in list ( see SER_UART_QUE in serial.h )
    UCHAR   *pC;                // Must be 2nd in list ( see SER_UART_QUE in serial.h )
} tSprintfSupport;

tSprintfSupport SprintfSupport = {1};

//
// ===========================================================================
//
// Function:    StdPutc()
//
// Purpose:     Puts a byte to the indicated UART's TX que
//
// ===========================================================================
//

void _reentrant StdPutc (SER_UART_QUE *pQue, UCHAR c)
{
    UCHAR *pC;
    //
    // The "pQue" pointer can either be a SET_UART_QUE, or
    // in another mode, it identifes a string as the target
    // of the operation...
    //
    
    if (PUTC_TYPE_SPRINTF == pQue->ucQueOrSprintfRam)
    {
        
        //
        // "pIsr" field really contains (in sprintf mode)
        // the address of the buffer (destination).
        //
        
        pC = (UCHAR *) pQue-> pIsr;
        
        //
        // Store character byte at destination
        //
        
        *pC = c;
        pC++;
        
        //
        // Null terminate string on the fly.
        //
        // *pC = 0;
    
        //
        // Advance to next buffer position in anticipation
        // of next iteration.
        //
        
        pQue -> pIsr = (UINT) pC;
        return;
    }

    if (PUTC_TYPE_FLASH == pQue->ucQueOrSprintfRam)
    {
        //
        // "pIsr" field really contains (in sprintf mode)
        // the address of the buffer (destination).
        //
        
        pC = (UCHAR *) pQue-> pIsr;
        
        //
        // Store character byte at destination
        //
        
        FlashWrite ( pC, &c, 1);
        
        //
        // Null terminate string on the fly.
        // Advance to next buffer position in anticipation
        // of next iteration.
        //
        
        // *pC = 0;
        pC++;
        pQue -> pIsr = (UINT) pC;
        return;
    }
    
    //
    // If the buffer is full, then spin until there is space available. 
    // 
  
    while (0 == SerQueSpaceAvailable(pQue) )
    {
        if (pQue->pIsr)
        {
            void (*function) ( void ); 
            UINT *pF;

            pF = (UINT *) &function;
            *pF = pQue->pIsr;                
            function ( );
        }   
    }
    
        
    //
    // Buffer byte for TX
    //
    
    SerQueInsert (pQue, c);
    
    //
    // When UART is already in TX mode, leave.
    //
    
    if ( pQue->LineStatus == LINE_TRANSMIT ) 
    {
        return;
    }
     
   
    //
    // The UART is TX inactive and a byte is available for
    // transmission, load up that byte, and tag the UART as 
    // being now "busy".
    //
    
    SerQueRemove (pQue, &c);
    
    //
    // Directly send TX byte "c" to UART 
    //
    
    if (pQue->ucUart == SER_UART_0)
    {
        SFRPAGE = CONFIG_PAGE;
        MAIN_485EN = 0;
        SFRPAGE   = UART0_PAGE;
        SBUF0 = c;
    }
        
    if (pQue->ucUart == SER_UART_1)
    {
        SFRPAGE = CONFIG_PAGE;
        PRB_485EN = 0;
        SFRPAGE   = UART1_PAGE;
        SBUF1 = c;
    }
        
    if (pQue->ucUart == SER_UART_2)
    {
        SpiWriteByteUart2 ( SPI_UART_W_THR, c);
    }
    
    if (pQue->ucUart == SER_UART_3)
    {
        SpiWriteByteUart3 ( SPI_UART_W_THR, c );
    }
    
    pQue->LineStatus = LINE_TRANSMIT;
    
} // StdPutc()

//
// ===========================================================================
//
// Function:    StdNibble2Ascii()
//
// Purpose:     Converts a nibble to an ASCII character.
//
// ===========================================================================
//

UCHAR StdNibble2Ascii (UCHAR x)
{
    x &= 0x0F; // Practice safe coding.
    if (x <= 9)
        x += '0';
    else
    {
        x -= 10;
        x += 'a';
    }

    return (UCHAR) x;

} // StdNibble2Ascii()





//
// ===========================================================================
//
// Function:    StdPutHex()
//
// Purpose:     
//
// ===========================================================================
//

VOID StdPutHex (SER_UART_QUE *pQue, ULONG iVal, UINT Width)
{
    UINT i;

    for (i = Width; i > 0; i--)
    {
        UCHAR Byte, c;
        Byte = (UCHAR) (iVal >> (((i-1)/2) * 8));
            
        //
        // Get even or odd nibble?
        //

        if (i & 1)
            c = StdNibble2Ascii(Byte & 0x0f);
        else
            c = StdNibble2Ascii(Byte >> 4);

        StdPutc( pQue, c);
    }
} // StdPutHex ()

//
// ===========================================================================
//
// Function:    StdDoPrintf ()
//
// Purpose:     This function "lays under" SerPrintfUart0 ( ) and 
//              SerPrintfUart1 ( ).
//
// ===========================================================================
//


#define IN_FORMAT 0
#define IN_STRING 1

#define FIELD_TYPE_16_BITS 1 
#define FIELD_TYPE_32_BITS 2

VOID _reentrant StdDoPrintf (SER_UART_QUE *pQue, _rom UCHAR *fmt, va_list ap )
{
    _rom UCHAR *p;
    BOOLEAN State;
    BOOLEAN LeadingZero;
    BOOLEAN bIsNegative;
    UCHAR   FieldType;
    ULONG   ulVal;
    UINT    uiVal;
    static UINT    FieldWidth;
    
    //
    // Kick watch dog.
    //
    
    KICK_WDT;                // Kick the watchdog

    State = IN_STRING;

    //
    // Load print line.
    //

    // va_start(ap, fmt);
    for (p = fmt; *p; p++)
    {
        if (IN_STRING == State)
        {
            if (*p != '%')
            {
                StdPutc (pQue, *p);
                continue;
            }
            else
            {
                FieldWidth  = 0;
                FieldType   = FIELD_TYPE_16_BITS;
                State       = IN_FORMAT;
                LeadingZero = FALSE;
                continue;
            }
        }
        else if (IN_FORMAT == State)
        {
            if (*p == 'c')
            {
                unsigned int c;
                c = va_arg(ap, int);
                StdPutc( pQue, c);
                State = IN_STRING;
            }
            
            else if (*p ==  's')
            {
                UCHAR *c;
                UINT        Width;

                c = (UCHAR *) va_arg(ap, char *);

                Width = 0;
                while ( *c ) 
                {
                    StdPutc( pQue, (UCHAR) *c);
                    c++;
                    Width++;
                }
                
                State = IN_STRING;
                while (Width++ < FieldWidth)
                {
                    StdPutc( pQue, ' ');
                }
            }
            
            //
            // Use upper case 'S' as a type specifier for
            // ROM strings.
            //
            
            else if (*p ==  'S')
            {
                _rom UCHAR *c;
                UINT        Width;

                c = (_rom UCHAR *) va_arg(ap, char *);

                Width = 0;
                while ( *c ) 
                {
                    StdPutc( pQue, (UCHAR) *c);
                    c++;
                    Width++;
                }

                State = IN_STRING;
                while (Width++ < FieldWidth)
                {
                    StdPutc( pQue, ' ');
                }
           }
            
            // NO FLOAT: case 'f':
            else if ((*p ==  'd') || (*p ==  'u'))
            {
                UCHAR   c;
                int     i, j;
                UCHAR   Temp[20];
                // NO FLOAT: float   fVal;
                // NO FLOAT: 
                // NO FLOAT: if ('f' == *p)
                // NO FLOAT: {
                // NO FLOAT:     fVal = (float) va_arg(ap, double);
                // NO FLOAT: 
                // NO FLOAT:     //
                // NO FLOAT:     // Handle case where floating pt value is 
                // NO FLOAT:     // less than zero.
                // NO FLOAT:     //
                // NO FLOAT:     
                // NO FLOAT:     if (fVal < 0.0f)
                // NO FLOAT:     {
                // NO FLOAT:         StdPutc( pQue, '-');
                // NO FLOAT:         fVal = -fVal;
                // NO FLOAT:     }
                // NO FLOAT: 
                // NO FLOAT:     //
                // NO FLOAT:     // Place integer portion in "uVal" and
                // NO FLOAT:     // place fractional (float) portion in "fVal".
                // NO FLOAT:     //
                // NO FLOAT:     
                // NO FLOAT:     ulVal = fVal;
                // NO FLOAT:     fVal  -= (float) ulVal;
                // NO FLOAT: } else ...
                
                bIsNegative = FALSE;
                if (FIELD_TYPE_16_BITS == FieldType)
                {
                    // ulVal = (ULONG) va_arg(ap, unsigned int);
                    
                    uiVal =  va_arg(ap, unsigned int);
                    
                    ulVal = (ULONG) uiVal;
                    
                    //
                    // If a signed value, and negative, handle appropriately
                    //
                
                
                    if ( ( 'd'   ==  *p   ) && 
                         (0x8000 &  ulVal ) )
                    {
                        bIsNegative = TRUE;
                        ulVal |= 0xFFFF0000;
                        ulVal = -ulVal;
                    }
                }
                else    
                {
                    ulVal = (ULONG) va_arg(ap, long);
                
                    //
                    // If a signed value, and negative, handle appropriately
                    //
                
                    if ( (    'd'    ==  *p   ) && 
                         (0x80000000 &  ulVal ) )
                    {
                        bIsNegative = TRUE;
                        ulVal = -ulVal;
                    }
                }   
                
                //
                // Store ASCII value into "temp" in "reverse"
                // order...
                //    

                i = 0;
                do {
                    Temp[i++] = (UCHAR) ('0' + ulVal % 10);
                    ulVal /= 10;
                } while (ulVal);
                
                j = i;

                if (bIsNegative)
                {
                    ++j;
                }                   

                while (j++ < FieldWidth)
                {
                    c = ' ';
                    if (LeadingZero)
                        c = '0';
                    StdPutc( pQue, c);
                }
                
                //
                // TBD: Do something with field width.
                //
                
                if (bIsNegative)
                {
                    StdPutc( pQue, '-');
                }

                do 
                {
                    StdPutc( pQue, Temp[--i]);
                } while (i>0);

                // NO FLOAT: if ('f' == *p)
                // NO FLOAT: {
                // NO FLOAT:     UCHAR n;
                // NO FLOAT:     UCHAR c;
                // NO FLOAT:     StdPutc( pQue, '.');
                // NO FLOAT:     for (n = 0; n < 5; n++)
                // NO FLOAT:     {
                // NO FLOAT:         fVal *= 10.0;
                // NO FLOAT:         c    = (UCHAR) fVal;
                // NO FLOAT:         fVal -= (float) c;
                // NO FLOAT:         
                // NO FLOAT:         StdPutc( pQue, c + '0');
                // NO FLOAT:     }
                // NO FLOAT: }
                
                State = IN_STRING;
            }

            else if ( (*p ==  'X') || (*p ==  'x') )
            {
                if (FIELD_TYPE_16_BITS == FieldType)
                    ulVal = (ULONG) va_arg(ap, int);
                else    
                    ulVal = (ULONG) va_arg(ap, long);
                
                if (FieldWidth == 0) 
                {
                    if (FIELD_TYPE_16_BITS == FieldType)
                        FieldWidth = 2 * sizeof(UINT);
                    else    
                        FieldWidth = 2 * sizeof(ULONG);
                }    

                StdPutHex (pQue, ulVal, FieldWidth);
            
                State = IN_STRING;
            }

            else if (*p ==  '0')
            {
                LeadingZero = TRUE;
            }

            else if (*p ==  'l')
            {
                FieldType = FIELD_TYPE_32_BITS;
            }

            else if ( (*p >=  '1') && (*p <= '9') )
            {
                FieldWidth *= 10;
                FieldWidth += *p - '0';
            }

            else 
            {
                State = IN_STRING;
                break;
            }

        }   // else if (IN_FORMAT == State) ...
        
    } // for (p = fmt; *p; p++)
    
    //
    // Flash / RAM sprintf operations must be terminated with a NULL
    //
    
    if (    (PUTC_TYPE_SPRINTF == pQue->ucQueOrSprintfRam) ||
            (PUTC_TYPE_FLASH   == pQue->ucQueOrSprintfRam) ) 
    {
        StdPutc( pQue, 0);
    }
    
} // StdDoPrintf ()


//
//=====================================================================
//
// Function: StdSprintf ()                                        
//
// Purpose:  Uses "printf" technology on RAM buffer.
//
//=====================================================================
//

void _reentrant StdSprintf ( char *pString, _rom char *format, ... )
{
    va_list ap;
    va_start( ap, format );
    
    SprintfSupport.ucQueOrSprintfRam = PUTC_TYPE_SPRINTF;
    SprintfSupport.pC = (UCHAR *) pString;
    
    StdDoPrintf ( (SER_UART_QUE *) &SprintfSupport, (_rom UCHAR *) format, ap);
    va_end( ap );
    
} // StdSprintf ( )


//
//=====================================================================
//
// Function: StdSprintfFlash ()                                        
//
// Purpose:  Uses "printf" technology on RAM buffer stored in flash.
//
//=====================================================================
//

void _reentrant StdSprintfFlash ( char *pString, _rom char *format, ... )
{
    va_list ap;
    va_start( ap, format );
    
    SprintfSupport.ucQueOrSprintfRam = PUTC_TYPE_FLASH;
    SprintfSupport.pC = (UCHAR *) pString;
    
    StdDoPrintf ( (SER_UART_QUE *) &SprintfSupport, (_rom UCHAR *) format, ap);
    va_end( ap );
    
} // StdSprintf ( )


//
//=====================================================================
//
// Function: StdPrintf ()                                        
//
// Purpose:  Uses "printf" technology on passed que.
//
//=====================================================================
//

void _reentrant StdPrintf ( SER_UART_QUE *pQue, _rom char *format, ... )
{
    va_list ap;
    va_start( ap, format );
    StdDoPrintf ( pQue, (_rom UCHAR *) format, ap);
    va_end( ap );
    
} // StdPrintf ( )


//
//=====================================================================
//
// Function: StdPrintfPrb ()                                        
//
// Purpose:  Uses "printf" technology on Prb.
//
//=====================================================================
//

void _reentrant StdPrintfPrb ( _rom char *format, ... )
{
    va_list ap;
    va_start( ap, format );
    StdDoPrintf ( &g_SerTxQuePrb, (_rom UCHAR *) format, ap);
    va_end( ap );
    
} // StdPrintfPrb ( )


//
//=====================================================================
//
// Function: StdPrintfMain ()                                        
//
// Purpose:  Uses "printf" technology on Main.
//
//=====================================================================
//

void _reentrant StdPrintfMain ( _rom char *format, ... )
{
    va_list ap;
    va_start( ap, format );
    StdDoPrintf ( &g_SerTxQueMain, (_rom UCHAR *) format, ap);
    va_end( ap );
    
} // StdPrintfMain ( )


//
//=====================================================================
//
// Function: StdPrintfSerPort ()                                        
//
// Purpose:  Uses "printf" technology on SerPort.
//
//=====================================================================
//

void _reentrant StdPrintfSerPort ( _rom char *format, ... )
{
    va_list ap;
    va_start( ap, format );
    StdDoPrintf ( &g_SerTxQueSerPort, (_rom UCHAR *) format, ap);
    va_end( ap );
    
} // StdPrintfSerPort ( )


//
//=====================================================================
//
// Function: StdPrintfEthPort ()                                        
//
// Purpose:  Uses "printf" technology on EthPort.
//
//=====================================================================
//

void _reentrant StdPrintfEthPort ( _rom char *format, ... )
{
    va_list ap;
    va_start( ap, format );
    StdDoPrintf ( &g_SerTxQueEthPort, (_rom UCHAR *) format, ap);
    va_end( ap );
    
} // StdPrintfEthPort ( )


//=====================================================================
//
// Function Name: StdStrFromRomStr ()                                           
//
// Description:   This function copies one string to another.         
//
//=====================================================================

void StdStrFromRomStrCopy ( UCHAR *str1, _rom UCHAR *str2)
{
    while (*str2)
        *str1++ = *str2++;

    *str1 = 0x00;

} // StdStrFromRomStrCopy ()


//=====================================================================
// Function   : StdUintToAsciiHex ()
// Purpose    : Converts a value to a ASCII hexadecimal string.
// Parameters : ULONG value, and precision value are passed in.
// Returns    : ASCII string representing ULONG value is returned.
//=====================================================================

VOID StdUintToAsciiHex ( UCHAR *AsciiStr, UINT uiVal, UCHAR FieldWidth )
{
    UCHAR i;

    for (i = FieldWidth; i > 0; i--)
    {
        UCHAR Byte, c;
        Byte = (UCHAR) (uiVal >> (((i - 1) / 2) * 8));
            
        //
        // Get even or odd nibble?
        //

        if (i & 1)
            c = StdNibbleToAscii(Byte & 0x0f);
        else
            c = StdNibbleToAscii(Byte >> 4);

        *AsciiStr = c;
        ++AsciiStr;
    }
    AsciiStr = 0;

} // StdPutHex ()




//=====================================================================
// Function   : StdNibbleToAscii ()
// Purpose    : Converts a nibble to an ASCII character.
// Parameters : Nibble is passed in.
// Returns    : ASCII character is returned.
//=====================================================================

UCHAR StdNibbleToAscii ( UCHAR c )
{
    c &= 0x0F;
    if (c < 10)
        return (c + '0');
    return (c + ('a' - 10));

} // StdNibbleToAscii()




//=====================================================================
// Function   : StdStrCopy ()
// Purpose    : This function copies one string to another.
// Parameters : Destination string, source string.
// Returns    : Nothing is returned.
//=====================================================================

VOID StdStrCopy ( UCHAR *str1, UCHAR *str2 )
{
    while (*str2)
        *str1++ = *str2++;

    *str1 = 0x00;

} // StdStrCopy ()



//=====================================================================
// Function   : StdAsciiToUlong ()
// Purpose    : Converts an ASCII-HEX string to an ULONG value. Stops
//              processing at first unrecognizable character.
// Parameters : Pointer to string, base format (DECIMAL or HEX).
// Returns    : ULONG representation of string is returned.
//=====================================================================

ULONG StdAsciiToUlong ( UCHAR *AsciiStr, UCHAR Base )
{
    UCHAR byte;
    UCHAR iter;
    UCHAR length;
    ULONG multiplier;
    ULONG ret_val;
    
    multiplier = 1;
    ret_val    = 0;

    //
    // Start at the back of the string, and convert from
    // ascii encoded hex, working your way to the front of the string.
    //
    // Assumption: What is passed in may be in ascii encoded hex in
    //             either upper or in lower case.
    //
    
    length = StdStrLen(AsciiStr) - 1;

    AsciiStr += length;      // Start at the back of "hex" (passed in);
    
    for (iter = 0; iter <= length; iter++) 
    {
        //
        // Get current byte to convert from hex ascii to integer.
        //

        byte = *AsciiStr;
        
        //
        // Convert to upper case
        //
        if ( (byte >= 'a') && (byte <= 'z') )
        {
            byte = byte - 'a' + 'A';
        }
        

        //
        // Convert from ASCII ('0'..'9', and 'a' ..'f')...
        //

        if ((byte >= '0') && (byte <= '9')) 
            byte -= '0';
        else if ((byte >= 'A') && (byte <= 'F')) 
            byte -= 'A' + 10; 
        else
            return ret_val;

        //
        // Prepare multiplier for next iteration...
        //
        
        ret_val += byte * multiplier;
        multiplier *= Base;
        AsciiStr--;
    }

    return ret_val;

} // StdAsciiToUlong ()



//=====================================================================
// Function   : StdStrLen ()
// Purpose    : Finds the length of the string.
// Parameters : Pointer to string.
// Returns    : Size of the string.
//=====================================================================

UINT StdStrLen ( UCHAR *str )
{
    UINT iter;

    for (iter = 0; *str != 0; str++, iter++);

    return iter;

} // StdStrLen ()




//=====================================================================
// Function   : StdToLower ()
// Purpose    : Converts uppercase characters to lowercase.
// Parameters : Uppercase/lowercase character.
// Returns    : Lowercase character.
//=====================================================================

UCHAR StdToLower ( UCHAR c )
{
    if ((c >= 'A') && (c <= 'Z'))
        c += 0x20;            // 'a' - 'A' = ' ' (0x20)
    
    return c;

} // StdToLower ()




//=====================================================================
// Function   : StdStrComp ()
// Purpose    : This function compares two strings.
// Parameters : First string, second string.
// Returns    : 0    - Strings are identical
//              +/-  - Strings mismatch
//=====================================================================

CHAR StdStrComp ( UCHAR *str1, UCHAR *str2 )
{
    while ((*str1 == *str2) && (*str1 != 0)) {
        ++str1;
        ++str2;
    }
        
    return(*str1 - *str2);

} // StdStrComp ()




//=====================================================================
// Function   : StdIntToAscii ()
// Purpose    : This function converts an integer to an ASCII HEX string.
// Parameters : Pointer to string, value to convert, precision value.
// Returns    : Nothing is returned.
//=====================================================================

VOID _reentrant StdIntToAscii ( UCHAR *AsciiStr, ULONG Value, UCHAR Precision )
{
    UCHAR temp[20];
    UCHAR length;
    UCHAR iter;

    iter = 0;
    do {
        temp[iter++] = ((UCHAR)Value % 0x100);
        Value /= 0x100;
    } while (Value);

    length = iter;
    for (iter = 0; iter < ((Precision / 2) - length); iter++)
        *AsciiStr++ = 0x00;

    do {
        *AsciiStr++ = temp[--length];
    } while (length > 0);

    *AsciiStr = 0;

} // StdIntToAscii ()



//=====================================================================
// Function   : StdBinaryToAscii ()
// Purpose    : This function converts a value to an ASCII HEX string.
// Parameters : Pointer to string, values to convert, number of values.
// Returns    : Nothing is returned.
//=====================================================================

VOID StdBinaryToAscii (UCHAR *AsciiStr, UCHAR *BinaryStr, UCHAR Size)
{
    UCHAR iter;
    UCHAR MSB;
    UCHAR LSB;
    UCHAR c;

    for (iter = 0; iter < Size; iter++) 
    {
        c = *BinaryStr++;
        
        LSB = StdNibbleToAscii(c & 0x0F);
        MSB = StdNibbleToAscii(c >>   4);
        
        *AsciiStr++ = MSB;
        *AsciiStr++ = LSB;
    }
    
    *AsciiStr = 0x00;
    
} // StdBinaryToAscii ()




//=====================================================================
// Function   : StdAsciiToBinary ()
// Purpose    : This function converts an ASCII HEX string to a binary hex string.
// Parameters : Pointer to string, values to convert, number of values.
// Returns    : Nothing is returned.
//=====================================================================

VOID StdAsciiToBinary (UCHAR *BinaryStr, UCHAR *AsciiStr, UCHAR Size)
{
    UCHAR iter;
    UCHAR MSB;
    
    for (iter = 0; iter < Size; iter++) 
    {
        MSB  = *AsciiStr++;
        *BinaryStr++ = MSB;
    }
    
} // StdAsciiToBinary ()





//=====================================================================
// Function   : StdRomStrCopy ()
// Purpose    : This function copies a string from ROM to another in RAM.
// Parameters : Destination RAM string, source ROM string.
// Returns    : Nothing is returned.
//=====================================================================

VOID StdRomStrCopy ( UCHAR *str1, _rom UCHAR *str2 )
{
    while (*str2)
        *str1++ = *str2++;

    *str1 = 0x00;

} // StdRomStrCopy ()



//=====================================================================
// Function   : StdRomStrLen ()
// Purpose    : Finds the length of the ROM string.
// Parameters : Pointer to ROM string.
// Returns    : Size of the string.
//=====================================================================

UINT StdRomStrLen ( _rom UCHAR *str )
{
    UINT iter;

    for ( iter = 0; *str != 0; str++, iter++ );

    return iter;

} // StdRomStrLen ()




//=====================================================================
// Function   : StdRomStrComp ()
// Purpose    : This function compares a string in ROM with a string in RAM.
// Parameters : First string located in RAM, second string located in ROM.
// Returns    : 0    - Strings are identical
//              +/-  - Strings mismatch
//=====================================================================

CHAR StdRomStrComp ( UCHAR *str1, _rom UCHAR *str2 )
{
    while ((*str1 == *str2) && (*str1 != 0)) {
        ++str1;
        ++str2;
    }
        
    return(*str1 - *str2);

} // StdRomStrComp ()



//=====================================================================
// Function   : StdPow ()
// Purpose    : This function return base ^ power.
// Parameters : Base value, to the nth power.
// Returns    : base ^ power
//=====================================================================

ULONG StdPow ( UINT base, UINT power )
{
    UCHAR i;
    ULONG ret_val = base;
    
    if ( power == 0 )
        return 1;
    
    for ( i = 1; i < power; i++ )
        ret_val *= base;
        
    return ret_val;
    
} // StdPow ()    




//
//=====================================================================
//
// Function Name: StdAsciiToUlongAutoBase ()                                             
//
// Description:   This function converts an ASCII string to a    
//                long integer value. Values which end in 'h'
//                or 'H' are processed as base 16, otherwise
//                decimal is assumed.
//
//=====================================================================
//

ULONG StdAsciiToUlongAutoBase (UCHAR *str)
{
    UCHAR byte;
    UCHAR length;
    ULONG ulValue;
    UCHAR *pC;
    
    if (NULL == str)
        return 0;
    
    length = 0;
    pC = str;
    while ( ( *pC ) && (*pC != ' ') )
    {
        ++length;
        ++pC;
    }
    
    if (0 == length)
        return 0;
    
    //
    // Use base 16 for strings which end in
    // 'h' or 'H'.
    //
    
    byte = str[length-1];
    if (( byte == 'h') || (byte == 'H') )
    {
        str[length-1] = 0;  // Zap the 'h' at the end of the value
        ulValue = StdAsciiToUlong (str, 16);
        str[length-1] = byte;  // re-insert 'h' at end of value
    }
    else
    {
        //
        // If the leading byte is a minus sign, 
        // handle negative values appropriately.
        //
        
        if ('-' == *str)
        {
            ulValue = StdAsciiToUlong (str+1, 10);
            ulValue = -ulValue;
        }
        else
        {
            ulValue = StdAsciiToUlong (str, 10);
        }
    }
    
    return ulValue;
    
} // StdAsciiToUlongAutoBase ( ) 



//
//=====================================================================
//
// Function Name: StdMemSet ()                                             
//
// Description:   This function sets a block of RAM to a indicated byte
//                value.
//
//=====================================================================
//

void StdMemSet (void *p, UCHAR ucValue, UINT uiLength)
{
    UCHAR *pC;
    
    pC = p;
    while (uiLength)
    {
        *pC = ucValue;
        ++pC;
        --uiLength;
    }
    
} // StdMemSet ( )


//
//=====================================================================
//
// Function Name: StdMemClr ()                                             
//
// Description:   This function zeroizes a block of RAM.
//
//=====================================================================
//

void StdMemClr (void *p, UINT uiLength)
{
    StdMemSet(p, 0, uiLength);
    
} // StdMemClr ( )


//
//=====================================================================
//
// Function Name: StdMemCopy ()                                             
//
// Description:   This function copyies a block of RAM.
//
//=====================================================================
//

void StdMemCopy (UCHAR *d, UCHAR *s, UINT uiLength)
{
    while (uiLength)
    {
        *d = *s;
        ++d;
        ++s;
        --uiLength;
    }
    
} // StdMemCopy ( )
