//**********************************************************************
// Filename:      stdlib.h
// Rights:        Copyright (c) 2006 The Carlisle and Finch Company
//                All Rights Reserved
// Creation Date: 6/13/06
// Compiler:      Tasking cc51
// Version:       v7.1r1
// Purpose:       This module contains standard library constants and function prototypes.
// Modification History:
// Date         Author      Description
// ----         ------      -----------
// 6-13-06      JRN         Created
//**********************************************************************

#ifndef _STDLIB_INC_
#define _STDLIB_INC_

// ===========================================
// Variable argument support
// ===========================================
typedef void * va_list[1];

#define va_start( _ap,_lastarg )	_ap[0] = ( (char *) &_lastarg ) + sizeof(_lastarg)
#define va_arg( _ap,_type )	( * (*(_type **)_ap) ++ )
#define va_end( _ap)		_ap[0] = NULL

// ===========================================
// Generic Std type macros
// ===========================================

#define StdHbound1(_a_) (sizeof(_a_)/sizeof(_a_[0]))
#define hbound1(_a_)    (sizeof(_a_)/sizeof(_a_[0]))

// ===========================================
// Function prototypes
// ===========================================

VOID  _reentrant StdPrintfMain             ( _rom char *format, ... );
VOID  _reentrant StdPrintfPrb              ( _rom char *format, ... );
VOID  _reentrant StdSprintf                ( char *pcBuffer, _rom char *format, ... );
VOID  _reentrant StdSprintfFlash           ( char *pcBuffer, _rom char *format, ... );
VOID  _reentrant StdPrintf                 ( SER_UART_QUE *pQue, _rom char *format, ... );

UCHAR            StdNibbleToAscii          ( UCHAR c);
VOID             StdStrCopy                ( UCHAR *str1, UCHAR *str2);
ULONG            StdAsciiToUlong           ( UCHAR *AsciiStr, UCHAR Base );
UINT             StdStrLen                 ( UCHAR *str);
UCHAR            StdToLower                ( UCHAR c);
CHAR             StdStrComp                ( UCHAR *str1, UCHAR *str2);
VOID _reentrant  StdIntToAscii             ( UCHAR *AsciiStr, ULONG Value, UCHAR Precision);
VOID             StdAsciiToBinary          ( UCHAR *BinaryStr, UCHAR *AsciiStr, UCHAR Size);
VOID             StdBinaryToAscii          ( UCHAR *AsciiStr, UCHAR *BinaryStr, UCHAR Size);
VOID             StdRomStrCopy             ( UCHAR *str1, _rom UCHAR *str2 );
UINT             StdRomStrLen              ( _rom UCHAR *str );
CHAR             StdRomStrComp             ( UCHAR *str1, _rom UCHAR *str2 );
VOID             StdUintToAsciiHex         ( UCHAR *AsciiStr, UINT uiVal, UCHAR FieldWidth );
ULONG            StdAsciiToUlongAutoBase   ( UCHAR *str );
void             StdMemClr                 (void *p, UINT uiLength);
void             StdMemCopy                (UCHAR *d, UCHAR *s, UINT uiLength);
void             StdMemSet                 (void *p, UCHAR ucValue, UINT uiLength);
void _reentrant  StdPutc                   (SER_UART_QUE *pQue, UCHAR c);


#endif // _STDLIB_INC_