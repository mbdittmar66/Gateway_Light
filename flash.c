//*********************************************************************
// Project:       Control System for Searchlight 
// Author:        Jason R. Niklas
// Filename:      flash.c
// Rights:	      Copyright (c) 2006 The Carlisle and Finch Company
// 			      All Rights Reserved
// Creation Date: 2/09/05
// Compiler:      Tasking cc51
// Version:       v7.1r1
// Purpose:       This module contains flash variables
//                and flash programming methods.
// Modification History:
// Date        Author        Description
// ----        ------        -----------
// 2/15/06     JRN           Added code to handle new serial flash part
//           
//**********************************************************************
        
#include "C8051F040.h"
#include "SlProtocol.h"
#include "SLC_Init.h"
#include "utils.h"
#include "serial.h"
#include "timers.h"
#include "main.h"
#include "spi.h"
#include "stdlib.h"
#include "flash.h"
#include "version.h"

//
//  Recognized globals
//

tFlashMiscPage   FlashMisc;
UCHAR            ucFlashActiveTourNum;
UCHAR            ucFlashActiveTourPt;
USHORT           usFlash90PercentLampLifeHours;
UCHAR            ucFlashActivePresetNum;	 // mbd
//tPresetPolarPos  *pTempPresetPolarPos;  // mbd

//=============================================================================
// Function   : FlashInit ()
// Purpose    : This function initializes flash system.
// Parameters : Nothing is passed. 
// Returns    : Nothing is returned.
//=============================================================================

VOID _reentrant FlashInit ( VOID )
{

    UCHAR n;
    tFlashMiscPage     *pFlashMisc;
    tFlashLampTimePage *pFlashLampTime;

    SFRPAGE = CONFIG_PAGE;
    FLACL = 0;               // Flash Access Limit
    
    // ============================================
    // Verify or Initialize Flash Misc structure...
    // ============================================
    
    pFlashLampTime = FLASH_LAMP_TIME_PTR;
    if ( ( FLASH_LAMP_TIME_SIGNATURE_TOP    != FlashReadUShort(&pFlashLampTime->usSignatureTop    ) ) ||     // If flash is not valid, then...
         ( FLASH_LAMP_TIME_SIGNATURE_BOTTOM != FlashReadUShort(&pFlashLampTime->usSignatureBottom ) ) )
    {
        tFlashLampTimePage FlashLampTime;
        
        //
        // Setup Initial Flash Misc structure and write to flash.
        //
        
        FlashLampTime.usLampHours        = 0;   // v1.61 - Now start at zero hours
        FlashLampTime.ucLampMinutes      = 0;
        FlashLampTime.ucLampSeconds      = 0;
        
        FlashLampTime.usSignatureTop     = FLASH_LAMP_TIME_SIGNATURE_TOP ; 
        FlashLampTime.usSignatureBottom  = FLASH_LAMP_TIME_SIGNATURE_BOTTOM;
        
        FlashErasePage  ( FLASH_LAMP_TIME_PAGE );
        FlashWrite      ( FLASH_LAMP_TIME_PTR, &FlashLampTime, sizeof (FlashLampTime));
    
        StdPrintf ( &g_SerTxQueSerPort, STR_NEW_LINE "RESET FLASH LAMP TIME");
    }
    
    pFlashMisc = FLASH_MISC_PTR;
    if ( ( FLASH_MISC_SIGNATURE_TOP    != FlashReadUShort(&pFlashMisc->usSignatureTop    ) ) ||     // If flash is not valid, then...
         ( FLASH_MISC_SIGNATURE_BOTTOM != FlashReadUShort(&pFlashMisc->usSignatureBottom ) ) )
    {
        //
        // Setup Initial Flash Misc structure and write to flash.
        //
        FlashErasePage( FLASH_MISC_PAGE );
        
        FlashWriteUShort (&pFlashMisc->usSignatureTop   , FLASH_MISC_SIGNATURE_TOP   ); 
        FlashWriteUShort (&pFlashMisc->usSignatureBottom, FLASH_MISC_SIGNATURE_BOTTOM);

        StdSprintfFlash  ( (char *) pFlashMisc->szMainDeviceName , "Srch Lt %02u  ", g_BoardNumber + 1);        // v1.72: Add two spaces to end of string for cleaner display
        StdSprintfFlash  ( (char *) pFlashMisc->szSerPortDeviceName , "Ser SL %02u  ", g_BoardNumber + 1);      // Note 2/2/17: SLC does NOT handle names/initialization correctly, JCS does much better now
        StdSprintfFlash  ( (char *) pFlashMisc->szEthPortDeviceName , "Eth SL %02u  ", g_BoardNumber + 1);      // This method does not change when DIP switches are changed! Names must be set during production.
        FlashWriteUShort (         &pFlashMisc->usTotalSlHours                       , 0);      // v1.61 - Now start at zero hours
        FlashWriteUChar  (         &pFlashMisc->ucTotalSlMinutes                     , 0);
        FlashWriteUChar  (         &pFlashMisc->ucTotalSlSeconds                     , 0);
        
        FlashWriteUShort (         &pFlashMisc->SlcMiscSetupPayload.usPanHomeOffset     ,     0);
        FlashWriteUShort (         &pFlashMisc->SlcMiscSetupPayload.usTiltHomeOffset    ,     0);
        StdSprintfFlash  ( (char *) pFlashMisc->SlcMiscSetupPayload.szLampModelNumber   ,"?LMN");
        StdSprintfFlash  ( (char *) pFlashMisc->SlcMiscSetupPayload.szLampSerialNumber  ,"?LSN");
        FlashWriteUShort (         &pFlashMisc->SlcMiscSetupPayload.usLampLifeHours     ,  1500);        // v1.72: Increased (from 100) to 1500
        FlashWriteUShort (         &pFlashMisc->SlcMiscSetupPayload.usAutoReturnSeconds ,  1800);		 // v2.01 Increased from 20 mins to 30 mins
        FlashWriteUShort (         &pFlashMisc->SlcMiscSetupPayload.usPanHomeOffset     ,     0);
        FlashWriteUShort (         &pFlashMisc->SlcMiscSetupPayload.usTiltHomeOffset    ,     0);
        FlashWriteUShort (         &pFlashMisc->SlcMiscSetupPayload.usRoFirmwareVersion , VERSION_DECIMAL);
        FlashWriteUChar  (         &pFlashMisc->SlcMiscSetupPayload.ucByte1Details      , SLC_SETUP_B1_LAMP_TYPE_NOT_SETUP );
                                                                                       //  | SLC_SETUP_B1_ENABLE_END_OF_LIFE_MSG);      // v1.72: Commented out to default to EOL message off

        StdPrintf ( &g_SerTxQueSerPort, STR_NEW_LINE "RESET FLASH MISC");
    }
    
    //
    // Set Misc values to RAM from flash.
    //
    
    FlashRead ( SerSlcDeviceNameEtcPayload.szMainDeviceName, pFlashMisc->szMainDeviceName , DEVICE_NAME_LENGTH ); 
    FlashRead ( SerSlcDeviceNameEtcPayload.szSerPortDeviceName, pFlashMisc->szSerPortDeviceName , DEVICE_NAME_LENGTH );
    FlashRead ( SerSlcDeviceNameEtcPayload.szEthPortDeviceName, pFlashMisc->szEthPortDeviceName , DEVICE_NAME_LENGTH );
    FlashRead ( &SerSlcMiscSetupPayload, &pFlashMisc->SlcMiscSetupPayload, sizeof(SerSlcMiscSetupPayload));
    
    //
    // Setup misc globals from values read from flash 
    //
                                                                 
    usFlash90PercentLampLifeHours = (90 * SerSlcMiscSetupPayload.usLampLifeHours) / 100;
    
    //
    // Setup "Read only" values in SerSlcDeviceNameEtcPayload
    //
    SerSlcMiscSetupPayload.usRoFirmwareVersion = VERSION_DECIMAL;
    
    
    // ===============================================
    // Verify or Initialize Flash Tour Structure(s)...
    // ===============================================
    
    for (n = 0; n < FLASH_NUM_TOUR_PAGES; n ++)
    {
        tFlashTourPage  *pTourPage;
        
        pTourPage = (tFlashTourPage *) ((UINT) (FLASH_TOUR_PAGE0 + (n<<1)) << 8);
        
        if ( ( (n + FLASH_TOUR_SIGNATURE_TOP   ) != FlashReadUShort(&pTourPage->uiSignatureTop    ) ) ||     // If flash is not valid, then...
             ( (n + FLASH_TOUR_SIGNATURE_BOTTOM) != FlashReadUShort(&pTourPage->uiSignatureBottom ) ) )
        {
            FlashErasePage( FLASH_TOUR_PAGE0 + (n<<1));
            
            //
            // Setup Initial values in Flash tour structure and write to flash.
            //
            
            FlashWriteUShort (&(pTourPage->uiSignatureTop   ), n + FLASH_TOUR_SIGNATURE_TOP   );    
            FlashWriteUShort (&(pTourPage->uiSignatureBottom), n + FLASH_TOUR_SIGNATURE_BOTTOM);    
            // pTourPage->uiSignatureTop    = n + FLASH_TOUR_SIGNATURE_TOP   ;    
            // pTourPage->uiSignatureBottom = n + FLASH_TOUR_SIGNATURE_BOTTOM;

            // FlashWrite    ( FLASH_TOUR_PAGE0 + n, &FlashTourWorking, sizeof(FlashTourWorking));
        
            StdPrintf ( &g_SerTxQueSerPort, STR_NEW_LINE "RESET FLASH TOUR # %d", n);
        
        } // if ( ( FLASH_TOUR_SIGNATURE_TOP    != FlashTourWorking.SignatureTop    ) || ... 
    
    } // for (n = 0; n < FLASH_TOUR_NUM_OF_PAGES; n ++)
    
    // FlashErasePage( FLASH_TOUR_PAGE1);
    
    ucFlashActiveTourNum = ucFlashActiveTourPt = FLASH_TOUR_NUM_INVALID;
    
} // Flash_Init ( )



//=======================================================================
// Function:    FlashErasePage ()                               
// Description: This function erases a single page of flash.
//=======================================================================

// #define FLASH_ADDR 0x0000
#define SFLE_BIT   0 // 0x04
#define SENSE_AMPS 0x80

static UCHAR cFlash;
static UCHAR *p;
// static UINT  Dptr;

void _reentrant FlashErasePage ( UCHAR ucPage )
{
    UCHAR SfrPageSave;
    
    //
    // Set pointer to beginning of targeted flash page
    //

    p = (UCHAR *) ucPage;
    p = (UCHAR *) ((UINT) p << 8);
    
    //
    // Disable interrupts.
    //

    EA = 0;      // IE = 0x00;

    //
    // Store the current SFR page.
    //

    SfrPageSave = SFRPAGE;

    //
    // Set up flash register for erase operation.
    //

    SFRPAGE = LEGACY_PAGE;

    FLSCL = SENSE_AMPS | 0x81;           // Enable flash write/erase.
    PSCTL = SFLE_BIT | 0x03;             // enable erasing.

    RSTSRC = 0x02;          // Enable VDDMON as reset source

    //
    // Write anything to the pointer, which will actually command an erase since PSCTL.1 is set.
    //

    //
    // Setup to use DPTR to access flash for erase operation.
    //

    #pragma asm 
    MOV     DPTR,#_p
    MOVX    A,@DPTR
    MOV     R3,A
    INC     DPTR
    MOVX    A,@DPTR
    MOV     DPL,A
    MOV     DPH,R3
    mov     a,0
    MOVX    @DPTR,A
    #pragma endasm


    // *p = 0;

    //
    // Erasure complete, reset flash registers.
    //

    PSCTL = 0;
    FLSCL = SENSE_AMPS;
    
    //
    // Restore SFRPAGE.
    //

    SFRPAGE = SfrPageSave;
    
    //
    // Restore interrupts.
    //
    
    EA = 1;      // IE = 0x92;
  

} // FlashErasePage ()

//=======================================================================
// Function:    FlashWrite ()                               
// Description: This function writes data to scratchpad Flash. 
//=======================================================================

void _reentrant FlashWrite ( void *pDest, void *pSrc, UINT numbytes )
{
    UCHAR SfrPageSave;
    UINT i;

    //
    // Set pointer to beginning of targeted flash page
    //

    p = (UCHAR *) pDest;
    // p = (UCHAR *) ((UINT) p << 8);


    //
    // Disable interrupts.
    //

    EA = 0;      // IE = 0x12;

    //
    // Store the current SFR page.
    //

    SfrPageSave = SFRPAGE;

    //
    // Set up flash register for write operation.
    //


      
    SFRPAGE = LEGACY_PAGE;

 
    //
    // Ebable VDDMON as reset source.
    //

    RSTSRC = 0x02;

    //
    // Write "numbytes" to FLASH address "dest" starting with byte at "src".
    //
    
    for ( i = 0; i < numbytes; i++ )
    {
        //
        // Store byte value to write to cFlash
        //

        cFlash = *(UCHAR *) pSrc;

        //
        // Place "cFlash" byte value in R2
        //

        #pragma asm
	    MOV	    DPTR,#_cFlash
	    MOVX	A,@DPTR
        MOV     R2,A
        #pragma endasm
        
        //
        // Setup to use DPTR to access flash for write operation.
        //

        #pragma asm 
        MOV     DPTR,#_p
        MOVX    A,@DPTR
        MOV     R3,A
        INC     DPTR
        MOVX    A,@DPTR
        MOV     DPL,A
        MOV     DPH,R3
        #pragma endasm


        SFRPAGE = LEGACY_PAGE;
        FLSCL = SENSE_AMPS | 0x01;           // Enable flash write/erase.
        PSCTL = SFLE_BIT | 0x01;           // Use FLASH, enable writes.

        //
        // Store "src" byte value (R2) to flash (@DPTR)
        //

        #pragma asm
        MOV     A,R2
        MOVX    @DPTR,A
        #pragma endasm


        PSCTL = 0x00;
        FLSCL = SENSE_AMPS;    
        p++;
        (UCHAR *) pSrc += 1;
    } 
    
    //
    // Restore SFRPAGE.
    //

    SFRPAGE = SfrPageSave;

    //
    // Restore interrupts.
    //
    
    EA = 1;      // IE = 0x92;

} // FlashWrite ( )

//=======================================================================
// Function:    FlashWriteUShort ()                               
// Description: This function writes data to scratchpad Flash. 
//=======================================================================

void _reentrant FlashWriteUShort ( USHORT *pusFlash, USHORT usValue)
{
    UCHAR SfrPageSave;
    UINT i;
    UCHAR *pSrc;
    
    //
    // Set pointer to beginning of targeted flash page
    //

    p = (UCHAR *) pusFlash;

    //
    // Disable interrupts.
    //

    EA = 0;      // IE = 0x12;

    //
    // Store the current SFR page.
    //

    SfrPageSave = SFRPAGE;

    //
    // Set up flash register for write operation.
    //
      
    SFRPAGE = LEGACY_PAGE;
   
    //
    // Ebable VDDMON as reset source.
    //

    RSTSRC = 0x02;

    //
    // Write "numbytes" to FLASH address "dest" starting with byte at "src".
    //
    
    pSrc = (UCHAR *) &usValue;

    for ( i = 0; i < sizeof (usValue); i++ )
    {
        //
        // Store byte value to write to cFlash
        //

        cFlash = *pSrc;

        //
        // Place "cFlash" byte value in R2
        //

        #pragma asm
	    MOV	    DPTR,#_cFlash
	    MOVX	A,@DPTR
        MOV     R2,A
        #pragma endasm
        
        //
        // Setup to use DPTR to access flash for write operation.
        //

        #pragma asm 
        MOV     DPTR,#_p
        MOVX    A,@DPTR
        MOV     R3,A
        INC     DPTR
        MOVX    A,@DPTR
        MOV     DPL,A
        MOV     DPH,R3
        #pragma endasm


        SFRPAGE = LEGACY_PAGE;
        FLSCL = SENSE_AMPS | 0x01;           // Enable flash write/erase.
        PSCTL = SFLE_BIT | 0x01;           // Use FLASH, enable writes.

        //
        // Store "src" byte value (R2) to flash (@DPTR)
        //

        #pragma asm
        MOV     A,R2
        MOVX    @DPTR,A
        #pragma endasm


        PSCTL = 0x00;
        FLSCL = SENSE_AMPS;    
        p++;
        pSrc++;
    } 
    

    //
    // Restore SFRPAGE.
    //

    SFRPAGE = SfrPageSave;

    //
    // Restore interrupts.
    //
    
    EA = 1;      // IE = 0x92;

} // FlashWriteUshort ( )

//=======================================================================
// Function:    FlashWriteUChar ()                               
// Description: This function writes data to scratchpad Flash. 
//=======================================================================

void _reentrant FlashWriteUChar ( UCHAR *pucFlash, UCHAR ucValue)
{
    UCHAR SfrPageSave;
    UINT i;
    UCHAR *pSrc;
    
    //
    // Set pointer to beginning of targeted flash page
    //

    p = (UCHAR *) pucFlash;

    //
    // Disable interrupts.
    //

    EA = 0;      // IE = 0x12;

    //
    // Store the current SFR page.
    //

    SfrPageSave = SFRPAGE;

    //
    // Set up flash register for write operation.
    //
      
    SFRPAGE = LEGACY_PAGE;
   
    //
    // Ebable VDDMON as reset source.
    //

    RSTSRC = 0x02;

    //
    // Write "numbytes" to FLASH address "dest" starting with byte at "src".
    //
    
    pSrc = (UCHAR *) &ucValue;

    for ( i = 0; i < sizeof (ucValue); i++ )
    {
        //
        // Store byte value to write to cFlash
        //

        cFlash = *pSrc;

        //
        // Place "cFlash" byte value in R2
        //

        #pragma asm
	    MOV	    DPTR,#_cFlash
	    MOVX	A,@DPTR
        MOV     R2,A
        #pragma endasm
        
        //
        // Setup to use DPTR to access flash for write operation.
        //

        #pragma asm 
        MOV     DPTR,#_p
        MOVX    A,@DPTR
        MOV     R3,A
        INC     DPTR
        MOVX    A,@DPTR
        MOV     DPL,A
        MOV     DPH,R3
        #pragma endasm


        SFRPAGE = LEGACY_PAGE;
        FLSCL = SENSE_AMPS | 0x01;           // Enable flash write/erase.
        PSCTL = SFLE_BIT | 0x01;           // Use FLASH, enable writes.

        //
        // Store "src" byte value (R2) to flash (@DPTR)
        //

        #pragma asm
        MOV     A,R2
        MOVX    @DPTR,A
        #pragma endasm


        PSCTL = 0x00;
        FLSCL = SENSE_AMPS;    
        p++;
        pSrc++;
    } 
    

    //
    // Restore SFRPAGE.
    //

    SFRPAGE = SfrPageSave;

    //
    // Restore interrupts.
    //
    
    EA = 1;      // IE = 0x92;

} // FlashWriteUChar ( )

//=======================================================================
// Function:    FlashReadUShort ()                               
// Description: This function reads data from scratchpad Flash. 
//=======================================================================

USHORT _reentrant FlashReadUShort ( USHORT *pusFlash) 
{
    UCHAR SfrPageSave;
    UCHAR i;
    USHORT usValue;
    
    //
    // Disable interrupts.
    //
    EA = 0;      // IE = 0x00;

    //
    // Set pointer to beginning of targeted flash page
    //

    p = (UCHAR *) pusFlash;
    // p = (UCHAR *) ((UINT) p << 8);

    //
    // Store the current SFR page.
    //

    SfrPageSave = SFRPAGE;


    //
    // Set up flash registers for read operation.
    //

    SFRPAGE = LEGACY_PAGE;
    
    //
    // Read "numbytes" from FLASH address "src" and write bytes beginning at "dest".
    //

    usValue = 0;
    for ( i = 0; i < sizeof(*pusFlash); i++)
    {
        //
        // Setup to use DPTR to access flash for read operation.
        //

#pragma asm 
        MOV     DPTR,#_p
        MOVX    A,@DPTR
        MOV     R3,A
        INC     DPTR
        MOVX    A,@DPTR
        MOV     DPL,A
        MOV     DPH,R3
#pragma endasm


        FLSCL = SENSE_AMPS;           // Enable flash reads.
        PSCTL = SFLE_BIT;           // Flash (XDATA) 

        //
        // Read the target byte into A
        //

#pragma asm
        MOV      A,#0
        MOVC     A,@A+DPTR
#pragma endasm

        PSCTL = 0x00;           // 
        FLSCL = SENSE_AMPS;           // Disable flash write/erase.

        //
        // Store target byte to cFlash
        //          

        #pragma asm
	    MOV	    DPTR,#_cFlash
	    MOVX	@DPTR,A
        #pragma endasm

        //
        // Move from cFlash to destination pointer
        //

        usValue = usValue << 8;
        usValue |= cFlash;

        ++p;
    } 
    
    //
    // Restore interrupts.
    //
    
    EA = 1;      // IE = 0x92;

    //
    // Restore SFRPAGE.
    //

    SFRPAGE = SfrPageSave;
    
    return usValue;

} // FlashReadUShort ( )


//=======================================================================
// Function:    FlashRead ()                               
// Description: This function reads data from Flash to RAM.
//=======================================================================

void _reentrant FlashRead ( void *dest, void *src, UINT numbytes )
{
    UCHAR SfrPageSave;
    UINT i;

    //
    // Disable interrupts.
    //
    EA = 0;      // IE = 0x00;

    //
    // Set pointer to beginning of targeted flash page
    //

    p = (UCHAR *) src;
    // p = (UCHAR *) ((UINT) p << 8);

    //
    // Store the current SFR page.
    //

    SfrPageSave = SFRPAGE;


    //
    // Set up flash registers for read operation.
    //

    SFRPAGE = LEGACY_PAGE;
    
    //
    // Read "numbytes" from FLASH address "src" and write bytes beginning at "dest".
    //

    for ( i = 0; i < numbytes; i++)
    {
        //
        // Setup to use DPTR to access flash for read operation.
        //

#pragma asm 
        MOV     DPTR,#_p
        MOVX    A,@DPTR
        MOV     R3,A
        INC     DPTR
        MOVX    A,@DPTR
        MOV     DPL,A
        MOV     DPH,R3
#pragma endasm


        FLSCL = SENSE_AMPS;           // Enable flash reads.
        PSCTL = SFLE_BIT;           // Flash (XDATA) 

        //
        // Read the target byte into A
        //

#pragma asm
        MOV      A,#0
        MOVC     A,@A+DPTR
#pragma endasm

        PSCTL = 0x00;           // 
        FLSCL = SENSE_AMPS;           // Disable flash write/erase.

        //
        // Store target byte to cFlash
        //          

        #pragma asm
	    MOV	    DPTR,#_cFlash
	    MOVX	@DPTR,A
        #pragma endasm

        //
        // Move from cFlash to destination pointer
        //

        *(UCHAR *) dest = cFlash;

        ++p;
        (UCHAR *) dest += 1;
    } 
    
    //
    // Restore interrupts.
    //
    
    EA = 1;      // IE = 0x92;

    //
    // Restore SFRPAGE.
    //

    SFRPAGE = SfrPageSave;

} // FlashRead ( )


//=======================================================================
// Function:    FlashStoreMisc ( )
// Description: This function writes misc data from RAM
//              to flash
// Caution:     This function is rather heavy in stack usage so be
//              careful to manage the stack usage in the system.
//=======================================================================

void _reentrant FlashStoreMisc ( UCHAR bMergeLampTimes )

{
    tFlashMiscPage   FlashMiscPage;
    
    FlashRead (&FlashMiscPage, FLASH_MISC_PTR, sizeof (FlashMiscPage));
    
    FlashMiscPage.usSignatureTop        = FLASH_MISC_SIGNATURE_TOP   ;
    FlashMiscPage.usSignatureBottom     = FLASH_MISC_SIGNATURE_BOTTOM;
    
    //
    // Move from utility RAM to all target fields
    //
    
    StdMemCopy(FlashMiscPage.szMainDeviceName  , SerSlcDeviceNameEtcPayload.szMainDeviceName, sizeof(FlashMiscPage.szMainDeviceName  ));
    StdMemCopy(FlashMiscPage.szSerPortDeviceName  , SerSlcDeviceNameEtcPayload.szSerPortDeviceName, sizeof(FlashMiscPage.szSerPortDeviceName  ));
    StdMemCopy(FlashMiscPage.szEthPortDeviceName  , SerSlcDeviceNameEtcPayload.szEthPortDeviceName, sizeof(FlashMiscPage.szEthPortDeviceName  ));
    
    if (bMergeLampTimes)
    {
        //
        // Populate LampTime fields...
        // 
    
        FlashRetrieveLampTime   ( &SerSlcExtStatusPayload.usLampHours,
                                  &SerSlcExtStatusPayload.ucLampMinutes,
                                  &SerSlcExtStatusPayload.ucLampSeconds);
                              
        //
        // Populate total SL time fields...
        //
    
        FlashRetrieveSlTotalTime( &SerSlcExtStatusPayload.usRoTotalSlHours,
                                  &SerSlcExtStatusPayload.ucRoTotalSlMinutes,
                                  &SerSlcExtStatusPayload.ucRoTotalSlSeconds);
                                  
        //
        // Accumulate and adjust total SL time with current lamp time
        //
    
        SerSlcExtStatusPayload.ucRoTotalSlSeconds += SerSlcExtStatusPayload.ucLampSeconds;
        if (SerSlcExtStatusPayload.ucRoTotalSlSeconds >= 60)
        {
            ++SerSlcExtStatusPayload.ucRoTotalSlMinutes;
            SerSlcExtStatusPayload.ucRoTotalSlSeconds -= 60;
        }
        SerSlcExtStatusPayload.ucRoTotalSlMinutes += SerSlcExtStatusPayload.ucLampMinutes;
        if (SerSlcExtStatusPayload.ucRoTotalSlMinutes >= 60)
        {
            ++SerSlcExtStatusPayload.usRoTotalSlHours;
            SerSlcExtStatusPayload.ucRoTotalSlMinutes -= 60;
        }
        SerSlcExtStatusPayload.usRoTotalSlHours += SerSlcExtStatusPayload.usLampHours;
        
        //
        // Store Total SL time field in Flash MISC page (from SerSlcExtStatusPayload fields).
        //
        
        FlashMiscPage.usTotalSlHours   = SerSlcExtStatusPayload.usRoTotalSlHours;
        FlashMiscPage.ucTotalSlMinutes = SerSlcExtStatusPayload.ucRoTotalSlMinutes;
        FlashMiscPage.ucTotalSlSeconds = SerSlcExtStatusPayload.ucRoTotalSlSeconds;
    }
            
    //
    // move from passed misc setup payload to target fields.
    //
    
    FlashMiscPage.SlcMiscSetupPayload = SerSlcMiscSetupPayload ;
        
    FlashErasePage  ( FLASH_MISC_PAGE );
    FlashWrite      ( FLASH_MISC_PTR, &FlashMiscPage, sizeof (FlashMiscPage));
    
    if (bMergeLampTimes)
    {
        FlashZeroizeLampTime ( );
    }
    
    //
    // Setup 90% lamp life hours field.
    //
    
    usFlash90PercentLampLifeHours = (90 * SerSlcMiscSetupPayload.usLampLifeHours) / 100;
    
    
} // FlashStoreMisc (  )


//=======================================================================
// Function:    FlashStoreTourPoint ( )
// Description: This function writes / re-writes a tour point from RAM
//              to flash
// Caution:     This function is rather heavy in stack usage so be
//              careful to manage the stack usage in the system.
//=======================================================================

void _reentrant FlashStoreTourPoint ( UCHAR ucTourNumber, UCHAR ucPointNumber, tTourPt *pPoint,
                                      UCHAR ucNextTourNumber)
{
    tFlashTourPage   FlashTourPage;
    UCHAR            ucTourPage;
    UCHAR            ucTourIndex;
    
    //
    // Protect against flash / buffer overruns.
    //
    if ( (ucTourNumber  >= TOUR_MAX_NUMBER_OF_TOURS ) ||
         (ucPointNumber >= TOUR_MAX_NUMBER_OF_POINTS) )
    {
        return;
    }

    ucTourPage  = ucTourNumber / FLASH_TOURS_PER_PAGE;
    ucTourIndex = ucTourNumber % FLASH_TOURS_PER_PAGE;
    
    //
    // Read / Modify / Write...
    //
    
    FlashRead       (&FlashTourPage, FLASH_TOUR_PTR(ucTourPage), sizeof (FlashTourPage));
    
    FlashTourPage.Tour[ucTourIndex].Point[ucPointNumber] = *pPoint;
    FlashTourPage.Tour[ucTourIndex].ucNextTourNumber     = ucNextTourNumber;
    
    FlashErasePage  ( FLASH_TOUR_PAGE0 + (ucTourPage<<1) );
    FlashWrite      ( FLASH_TOUR_PTR(ucTourPage), &FlashTourPage, sizeof (FlashTourPage));
    
} // FlashStoreTourPoint (  )


//=======================================================================
// Function:    FlashRetrieveTourPoint ( )
// Description: This function a reads tour point from flash to RAM
//=======================================================================

void _reentrant FlashRetrieveTourPoint  ( UCHAR ucTourNumber, UCHAR ucPointNumber, tTourPt *pPoint,
                                          UCHAR *pucNextTourNumber)
{
    tFlashTourPage   *pFlashTourPage;
    UCHAR            ucTourPage;
    UCHAR            ucTourIndex;
    
    //
    // Protect against flash / buffer overruns.
    //
    
    if ( (ucTourNumber  >= TOUR_MAX_NUMBER_OF_TOURS ) ||
         (ucPointNumber >= TOUR_MAX_NUMBER_OF_POINTS) )
    {
        return;
    }

    ucTourPage = ucTourNumber / FLASH_TOURS_PER_PAGE;
    ucTourIndex = ucTourNumber % FLASH_TOURS_PER_PAGE;
    
    pFlashTourPage = FLASH_TOUR_PTR(ucTourPage);
    
    //
    // Read point
    //
    
    FlashRead (pPoint, &pFlashTourPage->Tour[ucTourIndex].Point[ucPointNumber], sizeof(*pPoint));
    
    //
    // Read next tour number 
    //
    
    FlashRead (pucNextTourNumber, &pFlashTourPage->Tour[ucTourIndex].ucNextTourNumber, sizeof(*pucNextTourNumber));
    
} // FlashRetrieveTourPoint  ( )


//=======================================================================
// Function:    FlashDeleteTourPoint ( )
// Description: This function deletes a tour point in flash by setting
//              all fields in the target tour point to ONE's (FFh).
// Caution:     This function calls FlashStorePoint(), which is 
//              rather heavy in its use of stack, so be careful.
//=======================================================================

void _reentrant FlashDeleteTourPoint ( UCHAR ucTourNumber, UCHAR ucPointNumber )
{
    tTourPt Point;
    
    //
    // Protect against flash / buffer overruns.
    //
    
    if ( (ucTourNumber  >= TOUR_MAX_NUMBER_OF_TOURS ) ||
         (ucPointNumber >= TOUR_MAX_NUMBER_OF_POINTS) )
    {
        return;
    }

    StdMemSet           (&Point, 0xFF, sizeof(Point));
    FlashStoreTourPoint ( ucTourNumber, ucPointNumber, &Point, 0xFF);   // FF means do not change "next tour" number in flash
    
} // FlashDeleteTourPoint ( )




//=======================================================================
// Function:    FlashUpdateLampTime ( )
// Description: This function updates lamp times in flash from
//              global value passed.
//=======================================================================

void _reentrant FlashUpdateLampTime ( void )
{
    tFlashLampTimePage  FlashLampTimePage;
    
    FlashRead (&FlashLampTimePage, FLASH_LAMP_TIME_PTR, sizeof (FlashLampTimePage));
    
    FlashLampTimePage.ucLampSeconds += ucUtlLampSeconds;
    while (FlashLampTimePage.ucLampSeconds >= 60)
    {
        ++FlashLampTimePage.ucLampMinutes;
        FlashLampTimePage.ucLampSeconds -= 60;
    }
    
    FlashLampTimePage.ucLampMinutes += ucUtlLampMinutes;
    while (FlashLampTimePage.ucLampMinutes >= 60)
    {
        ++FlashLampTimePage.usLampHours;
        FlashLampTimePage.ucLampMinutes -= 60;
    }
    
    FlashLampTimePage.usLampHours += usUtlLampHours;    // v1.61 - Fix bug hours not accumulated 
    
    FlashErasePage  ( FLASH_LAMP_TIME_PAGE );
    FlashWrite      ( FLASH_LAMP_TIME_PTR, &FlashLampTimePage, sizeof (FlashLampTimePage));
    
    //
    // Now that the Utl Lamp time values have been reflected
    // in RAM, clear the Utl time values.
    //
    usUtlLampHours = 0;
    ucUtlLampMinutes = 0;
    ucUtlLampSeconds = 0;
    
} // FlashUpdateLampTime ( )


//=======================================================================
// Function:    FlashZeroizeLampTime ( )
// Description: This function zeroizes lamp times in flash 
//=======================================================================

void _reentrant FlashZeroizeLampTime ( void )
{
    tFlashLampTimePage  FlashLampTimePage;
    
    FlashRead (&FlashLampTimePage, FLASH_LAMP_TIME_PTR, sizeof (FlashLampTimePage));
    
    FlashLampTimePage.usLampHours   = 0;
    FlashLampTimePage.ucLampMinutes = 0;
    FlashLampTimePage.ucLampSeconds = 0;
    
    FlashErasePage  ( FLASH_LAMP_TIME_PAGE );
    FlashWrite      ( FLASH_LAMP_TIME_PTR, &FlashLampTimePage, sizeof (FlashLampTimePage));
    
} // FlashZeroizeLampTime ( )

//=======================================================================
// Function:    FlashRetrieveLampTime ( )
// Description: This function a reads lamp time from flash, and 
//              adds in any bias presently in RAM.
//=======================================================================

void _reentrant FlashRetrieveLampTime  ( USHORT *pusHours, UCHAR *pucMinutes, UCHAR *pucSeconds)
{
    tFlashLampTimePage  FlashLampTimePage;
    
    //
    // Read stored lamp time from flash.
    //
    
    FlashRead       (&FlashLampTimePage, FLASH_LAMP_TIME_PTR, sizeof (FlashLampTimePage));
    
    //
    // Update lamp time as need with any value(s) which
    // may be incrementing in RAM.
    //
    
    FlashLampTimePage.ucLampSeconds += ucUtlLampSeconds;
    
    while (FlashLampTimePage.ucLampSeconds >= 60)
    {
        ++FlashLampTimePage.ucLampMinutes;
        FlashLampTimePage.ucLampSeconds -= 60;
    }
    
    FlashLampTimePage.ucLampMinutes += ucUtlLampMinutes;
    while (FlashLampTimePage.ucLampMinutes >= 60)
    {
        ++FlashLampTimePage.usLampHours;
        FlashLampTimePage.ucLampMinutes -= 60;
    }
    FlashLampTimePage.usLampHours += usUtlLampHours;    // v1.61 - Fix bug hours not accumulated
    
    //
    // Setup return values.
    //
    
    *pusHours = FlashLampTimePage.usLampHours;
    *pucMinutes = FlashLampTimePage.ucLampMinutes;
    *pucSeconds = FlashLampTimePage.ucLampSeconds;
    
} // FlashRetrieveLampTime  ( )



//=======================================================================
// Function:    FlashRetrieveSlTime ( )
// Description: This function a reads total SL time from flash, adds in
//              the current lamp time and the flash lamp time, and returns
//              those values.
//=======================================================================

void _reentrant FlashRetrieveSlTotalTime ( USHORT *pusHours, UCHAR *pucMinutes, UCHAR *pucSeconds)
{
    tFlashMiscPage      FlashMiscPage;
    
    //
    // Read stored lamp time from flash.
    //
    
    FlashRead       (&FlashMiscPage, FLASH_MISC_PTR, sizeof (FlashMiscPage));
    
    //
    // Setup return values.
    //
    
    *pusHours   = FlashMiscPage.usTotalSlHours   ;
    *pucMinutes = FlashMiscPage.ucTotalSlMinutes ;
    *pucSeconds = FlashMiscPage.ucTotalSlSeconds ;
    
} // FlashRetrieveSlTotalTime  ( )
//=======================================================================
// Function:    FlashStorePresetPoint ( )
// Description: This function writes / re-writes a Pelco-D/ONVIF preset point from RAM
//              to flash
// Caution:     This function is rather heavy in stack usage so be
//              careful to manage the stack usage in the system.
//=======================================================================

void _reentrant FlashStorePresetPoint ( UCHAR ucPresetNumber, tPresetPolarPos *pPresetPolarPos )
{
    tFlashSLPresetsPage   FlashSLPresetsPage;
    //
    // Protect against flash / buffer overruns.
    //
    if (ucPresetNumber  > 0x20 ) 
    {
        return;
    }

    //
    // Read / Modify / Write...
    //
    FlashRead       (&FlashSLPresetsPage, FLASH_SL_PRESETS_PTR, sizeof (FlashSLPresetsPage));
    
    FlashSLPresetsPage.Presets[ucPresetNumber - 1] = *pPresetPolarPos;
    
    FlashErasePage  ( FLASH_SL_PRESETS ) ;
    FlashWrite      ( FLASH_SL_PRESETS_PTR, &FlashSLPresetsPage, sizeof (FlashSLPresetsPage));
    
} // FlashStorePresetPoint (  )


void _reentrant FlashStorePresetPoint_XX ( UCHAR ucPresetNumber )
{
    tFlashSLPresetsPage   FlashSLPresetsPage;
    //
    // Protect against flash / buffer overruns.
    //
    if (ucPresetNumber  > 0x20 ) 
    {
        return;
    }

    //
    // Read / Modify / Write...
    //
    FlashRead       (&FlashSLPresetsPage, FLASH_SL_PRESETS_PTR, sizeof (FlashSLPresetsPage));
    
    FlashSLPresetsPage.Presets[ucPresetNumber - 1].Preset_Pan = g_PanPos;
	FlashSLPresetsPage.Presets[ucPresetNumber - 1].Preset_Tilt = g_TiltPos;

    
    FlashErasePage  ( FLASH_SL_PRESETS ) ;
    FlashWrite      ( FLASH_SL_PRESETS_PTR, &FlashSLPresetsPage, sizeof (FlashSLPresetsPage));
    
} // FlashStorePresetPoint_XX (  )



VOID _reentrant     FlashRetrievePresetPoint    ( UCHAR ucPresetNumber )

//=======================================================================
// Function:    FlashRetrievePresetPoint ( )
// Description: This function a reads preset point from flash to RAM
//=======================================================================

{
   // tFlashSLPresetsPage   *pFlashSLPresetsPage;
	tFlashSLPresetsPage    FlashSLPresetsPage;
	tPresetPolarPos temp ;
    //
    // Protect against flash / buffer overruns.
    //

    if (ucPresetNumber  > 0x20 ) 
    {
        return;
    }

	//pFlashSLPresetsPage = FLASH_SL_PRESETS_PTR;    
    
    //
    // Read point
    //
	FlashRead       (&FlashSLPresetsPage, FLASH_SL_PRESETS_PTR, sizeof (FlashSLPresetsPage));
//	temp = FlashSLPresetsPage.Presets[ ucPresetNumber-1 ];
	temp = 	FlashSLPresetsPage.Presets[ ucPresetNumber-1 ];
	g_ActivePresetPanPos = temp.Preset_Pan;
	g_ActivePresetTiltPos = temp.Preset_Tilt;

    //FlashRead (pPresetPolarPos, &pFlashSLPresetsPage->Presets[ucPresetNumber], sizeof(*pPresetPolarPos));
//    FlashRead (pPresetPolarPos, &pFlashSLPresetsPage->Presets[ucPresetsIndex], sizeof(CurrentPresetPolarPos));
    
} // FlashRetrievePresetPoint  ( )

