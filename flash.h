//**********************************************************************
// Author:        Jason R. Niklas
// Filename:      flash.h
// Rights:	      Copyright (c) 2006 The Carlisle and Finch Company
// 			      All Rights Reserved
// Creation Date: 2/09/05
// Compiler:      Tasking cc51
// Version:       v7.1r1
// Purpose:       This module contains flash definitions
//                and flash function prototypes.
// Modification History:
// Date        Author        Description
// ----        ------        -----------
//
//**********************************************************************

//
// Flash structures, types, and constants.
//

//
// Page constants...
//

#define FLASH_MISC_PAGE                 0xF0
#define FLASH_TOUR_PAGE0                0xF2    // ==================================================
#define FLASH_TOUR_PAGE1                0xF4    // !! Important: Flash Tour pages MUST be consecutive
#define FLASH_TOUR_PAGE2                0xF8    // ==================================================
#define FLASH_TOUR_PAGE3                0xFA
#define FLASH_TOUR_PAGE4                0xFC
#define FLASH_LAMP_TIME_PAGE            0xEE
#define FLASH_SL_PRESETS				0xEC  // mbd-> Pelco-D allows 32 presets ( 0x01 to 0x20 ). 32*4 = 128 bytes required to store all presets, so will fit in a single flash "page"

#define FLASH_NUM_TOUR_PAGES            5

#define FLASH_MISC_PTR                  ((tFlashMiscPage     *)(FLASH_MISC_PAGE      << 8))
#define FLASH_LAMP_TIME_PTR             ((tFlashLampTimePage *)(FLASH_LAMP_TIME_PAGE << 8))
#define FLASH_TOUR_PTR(_page_)          ((tFlashTourPage *) ((UINT) (FLASH_TOUR_PAGE0 + ((_page_)<<1)) << 8))
#define FLASH_SL_PRESETS_PTR            ((tFlashSLPresetsPage     *)(FLASH_SL_PRESETS      << 8))

// ==================================
// Note: There is 1 "misc" flash page
// ==================================

#define FLASH_MISC_SIGNATURE_TOP    0x1207
#define FLASH_MISC_SIGNATURE_BOTTOM 0x1941

typedef struct
{
    USHORT                    usSignatureTop                             ;
    UCHAR                     szMainDeviceName     [DEVICE_NAME_LENGTH]  ;
    UCHAR                     szSerPortDeviceName  [DEVICE_NAME_LENGTH]  ;
    UCHAR                     szEthPortDeviceName  [DEVICE_NAME_LENGTH]  ; 
    USHORT                    usTotalSlHours                             ;               
    UCHAR                     ucTotalSlMinutes                           ;             
    UCHAR                     ucTotalSlSeconds                           ;             
    tPAYLOAD_SLC_MISC_SETUP   SlcMiscSetupPayload                        ;
    
    USHORT                    usSignatureBottom                          ;
    
} tFlashMiscPage;

// ======================================
// Note: Mangaged lamp time page.  
// ======================================

typedef struct
{
    USHORT      usSignatureTop                      ;
    USHORT      usLampHours                         ;
    UCHAR       ucLampMinutes                       ;
    UCHAR       ucLampSeconds                       ;  
    USHORT      usSignatureBottom                   ;
    
} tFlashLampTimePage;

#define FLASH_LAMP_TIME_SIGNATURE_TOP      0x0507
#define FLASH_LAMP_TIME_SIGNATURE_BOTTOM   0x1945

// ======================================
// Note: There 10 tours flash pages.  Each
//       page contains 2 tours (20 tours in
//       all), and each tour contains 
//       20 pts.
// ======================================

typedef struct
{
    tTourPt         Point[TOUR_MAX_NUMBER_OF_POINTS];
    UCHAR           ucNextTourNumber;
} tTour;

#define FLASH_TOUR_NUM_INVALID          0xFF
#define FLASH_POINT_NUM_INVALID         0xFF

#define FLASH_TOUR_SIGNATURE_TOP        0x0902
#define FLASH_TOUR_SIGNATURE_BOTTOM     0x1945

#define FLASH_TOURS_PER_PAGE            3
#define FLASH_TOTAL_NUM_TOURS           (FLASH_TOURS_PER_PAGE*FLASH_NUM_TOUR_PAGES)
typedef struct
{
    UINT            uiSignatureTop              ;
    tTour           Tour[FLASH_TOURS_PER_PAGE]  ;
    UINT            uiSignatureBottom           ;
    
} tFlashTourPage;

#define NUM_FLASH_PRESETS		32

typedef struct
{
    UINT Preset_Pan;
    UINT Preset_Tilt;
} tPresetPolarPos;

typedef struct
{
	tPresetPolarPos	Presets[NUM_FLASH_PRESETS] 		;	 // Each preset is 4 bytes ( 2 unsigned ints ) form is simply pan / tilt ABSOLUTE value. 
   
} tFlashSLPresetsPage;



//==========================================
// Global variables
//==========================================

extern UCHAR            ucFlashActiveTourNum;
extern UCHAR            ucFlashActiveTourPt;
extern USHORT           usFlash90PercentLampLifeHours;

extern UCHAR            ucFlashActivePresetNum;	 // mbd
//extern tPresetPolarPos 	*pTempPresetPolarPos;  // mbd

//
// Function prototypes.
//

VOID _reentrant     FlashInit                ( VOID );
void _reentrant     FlashWriteUShort         ( USHORT *pusFlash, USHORT usValue );
void _reentrant     FlashWriteUChar          ( UCHAR  *pucFlash, UCHAR  ucValue );
void _reentrant     FlashWrite               ( void *pDest, void *pSrc, UINT  uiNumBytes );
void _reentrant     FlashRead                ( void *pDest, void *pSrc, UINT  uiNumBytes);
USHORT _reentrant   FlashReadUShort          ( USHORT *pusFlash );
void _reentrant     FlashErasePage           ( UCHAR ucPage );
void _reentrant     FlashStoreTourPoint      ( UCHAR ucTourNumber, UCHAR ucPointNumber, tTourPt *pPoint, UCHAR ucNextTour);
void _reentrant     FlashRetrieveTourPoint   ( UCHAR ucTourNumber, UCHAR ucPointNumber, tTourPt *pPoint, UCHAR *ucNextTour);
void _reentrant     FlashDeleteTourPoint     ( UCHAR ucTourNumber, UCHAR ucPointNumber );
void _reentrant     FlashUpdateLampTime      ( void );
void _reentrant     FlashRetrieveLampTime    ( USHORT *pusHours, UCHAR *pucMinutes, UCHAR *pucSeconds);
void _reentrant     FlashRetrieveSlTotalTime ( USHORT *pusHours, UCHAR *pucMinutes, UCHAR *pucSeconds);
void _reentrant     FlashStoreMisc           ( UCHAR bMergeLampTimes );
void _reentrant     FlashZeroizeLampTime     ( void );

void _reentrant     FlashStorePresetPoint    ( UCHAR ucPresetNumber, tPresetPolarPos *pPresetPolarPos );		  // mbd
void _reentrant     FlashStorePresetPoint_XX ( UCHAR ucPresetNumber );
void _reentrant     FlashRetrievePresetPoint    ( UCHAR ucPresetNumber );
