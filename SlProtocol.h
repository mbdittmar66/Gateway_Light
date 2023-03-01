//
// ============================================================================
//
// Module:      SlProtocol.h
//
// Contains:    Shared Protocol Between all Firmware and API components
//
// Copyright:   (c) 2007-2011 Carlisle and Finch Company
//
// Modification History (most recent at top)
//
// Date         Initials  Version      Description
// ====         ========  =======      ===========
//
// 26 Oct 11	ACM		   1.01	       * Harmonized with JCS: v1.33
//														 SLC: v1.68
//                                     * Added JCS_STAT_B4_INHIBIT_SLC_RESPONSE bit
//									   * Added SLC_SETUP_B0_SET_HOME_CURRENT_POS bit
//                                     * Removed references to Aux/Cam/Zoom JCS and SLC bits,
//                                          replaced with AUX1/AUX2/AUX3
//                                     * Removed references to Aux1/Aux2 Ports, replaced with
//                                          SerPort and EthPort
//
// 13 Mar 08    src                    * Haromized with SDK: v1.12
//                                                      SLC: v1.39
//                                                      JCS: v1.22
//
// 12 Mar 08    src                    * Harmonized with SDK: v1.11
//
// 11 Mar 08    src                    * Harmonized with SLC: v1.37, API: v1.10
//
// 07 Mar 08    src                    * Harmonized with SLC v1.36
//
// 25 Feb 08    src                    * Harmonized with API: v1.08
//
// 24 Feb 08    src                    * Harmonized with API: v1.07
//
// 22 Feb 08    src                    * Harmonized with JCS v1.20, PRB v1.09, SLC v1.34, API:TBD
//                                                        
//
// 18 Feb 08    src                    * Original.
//                                     * Harmonized with SLC v1.32, PRB v1.09, API v1.06
//
//
// ============================================================================

#ifndef _SLPROTOCOL_H_
#define _SLPROTOCOL_H_

// ===========================================
// Error Codes
// ===========================================

#define ERROR_NO_STX                2
#define ERROR_NO_SRC_ADDR           3
#define ERROR_NO_DEST_ADDR          4
#define ERROR_NO_TYPE               5
#define ERROR_NO_LENGTH             6
#define ERROR_NO_CHECKSUM           8
#define ERROR_INVALID_CHECKSUM      9
#define ERROR_NO_ETX               10
#define ERROR_INVALID_ETX          11
#define ERROR_TILT                 12



// ===========================================
// Address Definitions
// ===========================================

#define BROADCAST_BIT                   0x80
#define JCS_PROTOCOL_OFFSET             0
#define SLC_PROTOCOL_OFFSET             0x10
#define SLC_SERPORT_PROTOCOL_OFFSET     0x20
#define SLC_ETHPORT_PROTOCOL_OFFSET     0x30
#define PRB_PROTOCOL_OFFSET             0x40
#define MASK_ADDRESS                    0x0F
#define MASK_DEVICE_TYPE                0xF0


#define INVALID_PROTOCOL_ADDRESS        0xFF

#define SPECIFIC_ADDR_RESPONSE          (SLC_SERPORT_PROTOCOL_OFFSET|SLC_ETHPORT_PROTOCOL_OFFSET)
#define JCS_BROADCAST                   (BROADCAST_BIT + JCS_PROTOCOL_OFFSET)
#define SLC_BROADCAST                   (BROADCAST_BIT + SLC_PROTOCOL_OFFSET)
#define SLC_SERPORT_BROADCAST           (BROADCAST_BIT + SLC_SERPORT_PROTOCOL_OFFSET)
#define SLC_ETHPORT_BROADCAST           (BROADCAST_BIT + SLC_ETHPORT_PROTOCOL_OFFSET)

#define JCS_MAX_NUMBER              16
#define SLC_MAX_NUMBER              16
#define JCS_LAST_ADDRESS            0x0F
#define SLC_LAST_ADDRESS            0x1F

// =================================
// Packet type constants
// =================================

#define PKT_TYPE_JCS_STATUS              3
#define PKT_TYPE_SLAVE_SPEAK             4
#define PKT_TYPE_SLC_BASIC_STATUS        5
#define PKT_TYPE_SLC_MISC_SETUP          6
#define PKT_TYPE_TOUR_POINT              8
#define PKT_TYPE_TOUR_CONTROL            9
#define PKT_TYPE_DO_CALIBRATION          0x0a
#define PKT_TYPE_SLC_DEVICE_NAME_ETC     0x0b
#define PKT_TYPE_DO_POSITION_SL_REL      0x0c
#define PKT_TYPE_PRB_SLAVE_SPEAK         0x0e
#define PKT_TYPE_PRB_FEEDBACK_INFO       0x0f
#define PKT_TYPE_PRB_CAL_FEEDBACK        0x10
#define PKT_TYPE_PRB_CONFIG              0x11
#define PKT_TYPE_ZEROIZE_LAMP_HOURS      0x12
#define PKT_TYPE_REQUEST_SLC_MISC_SETUP  0x14
#define PKT_TYPE_SLC_EXTENDED_STATUS     0x18
#define PKT_TYPE_JCS_SETUP               0x19
#define PKT_TYPE_REQUEST_JCS_SETUP       0x1a
#define PKT_TYPE_REQUEST_SLC_DEVICE_NAME 0x1b

#define FIRST_PKT_TYPE                   3
#define LAST_PKT_TYPE                    0x1c

// ===========================================
// Misc Protocol Constants
// ===========================================

#define DEVICE_NAME_LENGTH  12      // MAX: 11 Bytes of text + a NULL
#define SLP_MAX_PAYLOAD     80

// ===========================================
// Payload Types (for different packet types)
// ===========================================

// 
// Applicable to PRB_SLAVE_SPEAK packet type
//

typedef struct
{
    UCHAR   ucMode;

} tPAYLOAD_PRB_SLAVE_SPEAK;

typedef struct
{
    UINT    Pan1Adc;
    UINT    Pan2Adc;
    UINT    TiltAdc;
    UINT    FocusAdc;
    UCHAR   PanSpeed;
    UCHAR   TiltSpeed;
    UCHAR   FocusSpeed;
    UCHAR   CalProgress;

} tPAYLOAD_PRB_CAL_FEEDBACK;

typedef struct
{
    UINT    PanPosition;
    UINT    TiltPosition;
    UINT    FocusPosition;
	UINT	PanAdc;
	UINT	TiltAdc;
    UCHAR   OperationStatus;

} tPAYLOAD_PRB_FB_INFO;

#define PRB_FB_OP_STAT_NEEDS_CAL_MODE   0x01 
#define PRB_FB_OP_STAT_CAL_MODE         0x02
#define PRB_FB_OP_STAT_PRB_RESET        0x04
#define PRB_FB_OP_STAT_PRB_CONT_FAULT   0x08
#define PRB_FB_OP_STAT_POT1_FAULT       0x10
#define PRB_FB_OP_STAT_POT2_FAULT       0x20
#define PRB_FB_OP_STAT_POT3_FAULT       0x40
#define PRB_FB_OP_STAT_POT4_FAULT       0x80

typedef struct
{
    UCHAR   SLBaseType;
    UCHAR   AdcAvgs;
    UINT    PanRange;
    UINT    TiltRange;
    UINT    FocusRange;
    UINT    PanMaxRate;
    UINT    TiltMaxRate;
    UINT    FocusMaxRate;
    UCHAR   BeginCal;

} tPAYLOAD_PRB_CONFIG;   

//
// =======================================
// DO_CALIBRATION Payload
// =======================================
//
typedef struct
{

    USHORT  usPanRange;
    USHORT  usTiltRange;
    USHORT  usFocusRange;
    UCHAR   ucAdcAvgs;
    
} tPAYLOAD_DO_CALIBRATION;   
		  
//
// =======================================
// DO_POSITION_SL_RELATIVE Payload
// =======================================
//

typedef struct
{
    SHORT   sAzimuth;
    SHORT   sElevation;
    USHORT  usRate;
    USHORT  usGrowth0;
    USHORT  usGrowth1;
    
} tPAYLOAD_DO_POSITION_SL_REL;   
  
//
// =======================================
// JCS_STATUS Payload
// =======================================
//
typedef struct
{
    UCHAR   ucX;
    UCHAR   ucY;
    UCHAR   ucZ;
    UCHAR   ucByte3;
    UCHAR   ucByte4;
	UCHAR	ucByte5;  // mbd addl AUXes
	UCHAR	ucByte6;  // mbd addl AUXes
	UCHAR	ucByte7;  // mbd addl AUXes

    
} tPAYLOAD_JCS_STATUS;   

#define JCS_STAT_B3_REQ_SLC_EXT_STAT_PKT        0x01
#define JCS_STAT_B3_REQ_SLC_DEVICE_NAME_ETC_PKT 0x02
#define JCS_STAT_B3_REQ_SLC_MISC_SETUP_PKT      0x04
#define JCS_STAT_B3_IGNORE_CONTROLS             0x08        // SDK v1.17

#define JCS_STAT_B3_AUX2_ON                     0x10        // ACM!! v1.01 Changed from CAMERA to AUX2
#define JCS_STAT_B3_AUX2_OFF                    0x20        // ACM!! v1.01 Changed from CAMERA to AUX2
#define JCS_STAT_B3_AUX3_ON                     0x40        // ACM!! v1.01 Changed from ZOOM to AUX3
#define JCS_STAT_B3_AUX3_OFF                    0x80        // ACM!! v1.01 Changed from ZOOM to AUX3
                                                
#define JCS_STAT_B3_CLEAR_BUTTON_MASK           0x0F

#define JCS_STAT_B4_AUX1_ON                     0x01        // ACM!! v1.01 Changed from MISC to AUX1
#define JCS_STAT_B4_AUX1_OFF                    0x02        // ACM!! v1.01 Changed from MISC to AUX1
#define JCS_STAT_B4_HOME_ON                     0x04
#define JCS_STAT_B4_LAMP_ON_START               0x08
#define JCS_STAT_B4_LAMP_OFF                    0x10
#define JCS_STAT_B4_ACK_SLC_EXT_STAT_PKT        0x20
#define	JCS_STAT_B4_INHIBIT_SLC_RESPONSE		0x40		// ACM!! per MAS!! Added in v1.01 for communications on Serial/Ethernet ports

#define JCS_STAT_B4_CLEAR_BUTTON_MASK           0xE0

#define JCS_STAT_B5_AUX4_ON                     0x01        // mbd
#define JCS_STAT_B5_AUX4_OFF                    0x02        // mbd
#define JCS_STAT_B5_AUX5_ON                    	0x04        // mbd
#define JCS_STAT_B5_AUX5_OFF                    0x08        // mbd
#define JCS_STAT_B5_AUX6_ON                   	0x10        // mbd
#define JCS_STAT_B5_AUX6_OFF                    0x20        // mbd
#define JCS_STAT_B5_AUX7_ON                    	0x40        // mbd
#define JCS_STAT_B5_AUX7_OFF                    0x80        // mbd

#define JCS_STAT_B6_AUX8_ON                     0x01        // mbd
#define JCS_STAT_B6_AUX8_OFF                    0x02        // mbd
#define JCS_STAT_B6_AUX9_UP_ON                  0x04        // mbd
#define JCS_STAT_B6_AUX9_UP_OFF                 0x08        // mbd
#define JCS_STAT_B6_AUX9_DOWN_ON                0x10        // mbd
#define JCS_STAT_B6_AUX9_DOWN_OFF               0x20        // mbd
#define JCS_STAT_B6_AUX10_UP_ON                 0x40        // mbd
#define JCS_STAT_B6_AUX10_UP_OFF                0x80        // mbd

#define JCS_STAT_B7_AUX10_DOWN_ON               0x01        // mbd
#define JCS_STAT_B7_AUX10_DOWN_OFF              0x02        // mbd
#define JCS_STAT_B7_DIG_XEN_STR_ON              0x04        // mbd	not supported 6/1/2020
#define JCS_STAT_B7_DIG_XEN_STR_OFF             0x08        // mbd	not supported 6/1/2020

//
// =======================================
// JCS_SETUP Payload
// =======================================
//
typedef struct
{
    UCHAR   ucByte0CommandAndControl;
    UCHAR   szDeviceName [DEVICE_NAME_LENGTH];       // +1 Byte for NULL terminator
    USHORT  uiRoFirmwareVersion;                    // Ro means Read Only
    UCHAR   ucRoGrowth[20];
    
} tPAYLOAD_JCS_SETUP;   
#define JCS_SETUP_B0_STORE_DEVICE_NAME_ONLY 0x01

//
// =======================================
// SLC_BASIC_STATUS Payload
// =======================================
//
typedef struct
{
    SHORT   sAzimuth  ;
    SHORT   sElevation;
    UCHAR   ucActiveTour;
    UCHAR   ucControllingDevice;
    UCHAR   ucByte7;
    UCHAR   ucActiveTourPoint;
    
} tPAYLOAD_SLC_BASIC_STATUS;   

#define SLC_STAT_B7_AUX2_ON              0x01               // ACM!! v1.01 Changed from CAMERA to AUX2
#define SLC_STAT_B7_BEAM_ON              0x02
#define SLC_STAT_B7_AT_HOME              0x04
#define SLC_STAT_B7_AUX3_ON              0x08               // ACM!! v1.01 Changed from CAMERA_ZOOM to AUX3
#define SLC_STAT_B7_AUX1_ON              0x10               // ACM!! v1.01 Changed from MISC to AUX1
#define SLC_STAT_B7_END_LAMP_LIFE        0x20
#define SLC_STAT_B7_10_PERCENT_LAMP_LEFT 0x40

//
// =======================================
// SLC_EXTENDED_STATUS Payload
// =======================================
//
typedef struct
{
    tPAYLOAD_SLC_BASIC_STATUS   BasicStatus;
    UCHAR   ucByte8;
    UCHAR   ucByte9;
    UCHAR   ucByte10;
    USHORT  usLampHours;
    UCHAR   ucLampMinutes;
    UCHAR   ucLampSeconds;
    USHORT  usRoTotalSlHours;               
    UCHAR   ucRoTotalSlMinutes;             
    UCHAR   ucRoTotalSlSeconds;             
    UCHAR   ucPercentLampUsed;
    USHORT  usTourDwellCountdown;
    UCHAR   ucByte22;	// mbd ext protocol	byte contains Aux statuses
    UCHAR   ucByte23;	// mbd ext protocol, contains mainly digi Xenon I/F statuses
	CHAR	scDigXeMasterBoardTemp ;  // digi Xe Master board Temp +/- 127 temp in C
	USHORT	usDigiXePwrSupOutputVoltage ; 
	USHORT	usDigiXeLampVoltage ;	   // byte 27-28
    UCHAR   ucPwrSupBoard_0_Status;	// byte 29
    UCHAR   ucPwrSupBoard_1_Status;	// byte 30
    UCHAR   ucPwrSupBoard_2_Status;	// byte 31
    UCHAR   ucPwrSupBoard_3_Status;	// byte 32
    UCHAR   ucPwrSupBoard_4_Status;	// byte 33
    UCHAR   ucPwrSupBoard_5_Status;	// byte 34
    UCHAR   ucPwrSupBoard_6_Status;	// byte 35
    UCHAR   ucPwrSupBoard_7_Status;	// byte 36
    
} tPAYLOAD_SLC_EXT_STATUS;   

#define SLC_EXT_STAT_B8_SMARTVIEW1          0x01

#define SLC_EXT_STAT_B9_PRB_CAL_IN_PROCESS  0x01
#define SLC_EXT_STAT_B9_PRB_NOT_CALIBRATED  0x02
#define SLC_EXT_STAT_B9_TEMP_FAIL           0x04
#define SLC_EXT_STAT_B9_HORIZ_STOP_1        0x08
#define SLC_EXT_STAT_B9_HORIZ_STOP_2        0x10
#define SLC_EXT_STAT_B9_VERT_STOP_1         0x20
#define SLC_EXT_STAT_B9_VERT_STOP_2         0x40
#define SLC_EXT_STAT_B9_SLC_CONT_FAULT      0x80

#define SLC_EXT_STAT_B10_SLC_RESET          0x01
#define SLC_EXT_STAT_B10_PRB_COMM_FAULT     0x02     
#define SLC_EXT_STAT_B10_PRB_CONT_FAULT     0x04                           
#define SLC_EXT_STAT_B10_PRB_RESET          0x08
#define SLC_EXT_STAT_B10_PRB_POT1_FAULT     0x10
#define SLC_EXT_STAT_B10_PRB_POT2_FAULT     0x20
#define SLC_EXT_STAT_B10_PRB_POT3_FAULT     0x40
#define SLC_EXT_STAT_B10_PRB_POT4_FAULT     0x80

#define SLC_EXT_STAT_B22_AUX4_ON			0x01
#define SLC_EXT_STAT_B22_AUX5_ON			0x02
#define SLC_EXT_STAT_B22_AUX6_ON			0x04
#define SLC_EXT_STAT_B22_AUX7_ON			0x08
#define SLC_EXT_STAT_B22_AUX8_ON			0x10
#define SLC_EXT_STAT_B22_AUX9_UP_ON			0x20
#define SLC_EXT_STAT_B22_AUX9_DOWN_ON		0x40
#define SLC_EXT_STAT_B22_AUX10_UP_ON		0x80

#define SLC_EXT_STAT_B23_AUX10_DOWN_ON			0x01
#define SLC_EXT_STAT_B23_DIGIXE_CONNECTED		0x02
#define SLC_EXT_STAT_B23_DIGIXE_STROBE_ON		0x04
#define SLC_EXT_STAT_B23_DIGIXE_PS_OVERTEMP		0x08
#define SLC_EXT_STAT_B23_DIGIXE_PS_OK			0x10
#define SLC_EXT_STAT_B23_DIGIXE_SNS_VOLT_ON		0x20
#define SLC_EXT_STAT_B23_DIGIXE_LAMP_WIRING_OK	0x40

//
// =======================================
// SLC_SETUP payload
// =======================================
//
typedef struct
{
    UCHAR   ucByte0CommandAndControl;
    UCHAR   ucByte1Details;
    UCHAR   szLampModelNumber [DEVICE_NAME_LENGTH];
    USHORT  usLampLifeHours;
    UCHAR   szLampSerialNumber[DEVICE_NAME_LENGTH];
//    UCHAR   ucGrowth;
    UCHAR   ucHeightAboveSurface;     // mbdtest                    
    USHORT  usAutoReturnSeconds;
    USHORT  usRoFirmwareVersion;            // "Ro" means read only          
    USHORT  usPanHomeOffset                         ; // feeds global: g_PanHomeOffset
    USHORT  usTiltHomeOffset                        ; // feeds global: g_TiltHomeOffset

} tPAYLOAD_SLC_MISC_SETUP;   

#define SLC_SETUP_B0_WRITE_TO_FLASH         0x01
#define SLC_SETUP_B0_SET_HOME_CURRENT_POS	0x02	// ACM!! v1.01

#define SLC_SETUP_B1_LAMP_TYPE_MASK         0x03
#define SLC_SETUP_B1_LAMP_TYPE_NOT_SETUP    0x00
#define SLC_SETUP_B1_LAMP_TYPE_METAL_HALIDE 0x01 
#define SLC_SETUP_B1_LAMP_TYPE_XENON        0x02 
#define SLC_SETUP_B1_LAMP_TYPE_HALOGEN      0x03 
#define SLC_SETUP_B1_ENABLE_END_OF_LIFE_MSG 0x04 
	
//
// =======================================
// SLC_DEVICE_NAME_ETC Packet
// =======================================
//

typedef struct
{
    UCHAR   ucByte0CommandAndControl;
    UCHAR   szMainDeviceName [DEVICE_NAME_LENGTH];
    UCHAR   szSerPortDeviceName [DEVICE_NAME_LENGTH];
    UCHAR   szEthPortDeviceName [DEVICE_NAME_LENGTH];
    ULONG   ulDatumPtX;
    ULONG   ulDatumPtY;
    ULONG   ulDatumPtZ;
    
} tPAYLOAD_SLC_DEVICE_NAME_ETC;   

#define SLC_DEVICE_NAME_ETC_B0_STORE_FLASH      0x01
 
//
// =======================================
// PKT_TYPE_TOUR_CONTROL Payload
// =======================================
//
typedef struct
{
    UCHAR   ucControl;
    UCHAR   ucTour;
    UCHAR   ucPoint;
    
} tPAYLOAD_TOUR_CONTROL; 

#define TOUR_CONTROL_RUN            0x01    // Applicable to ucControl
#define TOUR_CONTROL_STOP           0x02    // Applicable to ucControl
#define TOUR_CONTROL_PAUSE          0x04    // Applicable to ucControl
#define TOUR_CONTROL_RESUME         0x08    // Applicable to ucControl

#define TOUR_INVALID                0xFF    // Applicable to ucTour
#define TOUR_PAUSED_BIT             0x80    // Applicable to ucTour

#define TOUR_POINT_INVALID          0xFF    // Applicable to ucPoint

//
// =======================================
// PKT_TYPE_ZEROIZE_LAMP_HOURS Payload
// =======================================
//


typedef struct 
{
    USHORT          usSignature;
    
} tPAYLOAD_ZEROIZE_LAMP_HOURS;

#define SIGNATURE_ZEROIZE_LAMP_HOURS 0x43F9

//
// =======================================
//  PKT_TYPE_TOUR_POINT Payload
// =======================================
//

#define TOUR_OPT_TO_PT_AUX1_ON      0x01        // Applicable to ucOptions of tTourPt		// ACM!! v1.01 Added bit to allow using AUX1 in tours
#define TOUR_OPT_DWELL_PT_AUX1_ON   0x02        // Applicable to ucOptions of tTourPt		// ACM!! v1.01 Added bit to allow using AUX1 in tours
#define TOUR_OPT_TO_PT_AUX2_ON      0x04        // Applicable to ucOptions of tTourPt		// ACM!! v1.01 Changed from CAM to AUX2
#define TOUR_OPT_TO_PT_BEAM_ON      0x08        // Applicable to ucOptions of tTourPt
#define TOUR_OPT_TO_PT_AUX3_ON      0x10        // Applicable to ucOptions of tTourPt       // ACM!! v1.01 Changed from ZOOM to AUX3
#define TOUR_OPT_DWELL_PT_AUX2_ON   0x20        // Applicable to ucOptions of tTourPt       // ACM!! v1.01 Changed from CAM to AUX2
#define TOUR_OPT_DWELL_PT_BEAM_ON   0x40        // Applicable to ucOptions of tTourPt
#define TOUR_OPT_DWELL_PT_AUX3_ON   0x80        // Applicable to ucOptions of tTourPt		// ACM!! v1.01 Changed from ZOOM to AUX3

#define TOUR_MAX_NUMBER_OF_TOURS            15          // Max number of tours in an SLC
#define TOUR_MAX_NUMBER_OF_POINTS           20          // Max number of points in an SLC tour

#define TOUR_RATE_MASK                      0x0FFF      // Applicable to usRate of tTourPt
#define TOUR_RATE_FASTEST_RATE              0x0FFF      // Applicable to usRate of tTourPt
#define TOUR_MAX_DWELL_TIME                 0x0FFF      // Applicable to ucDwellSec + MS Nibble of usRate
#define TOUR_DWELL_SEC_MSN_FROM_RATE_MASK   0xF000      // Applicable to usRate of tTourPt MSN = Most Significant Nibble
#define TOUR_DWELL_SEC_MSN_SHL_FOR_BYTE     4           // Applicable to usRate of tTourPt
#define TOUR_POINT_NOT_SET(_p_) (0xFFFF==(_p_).usRate)  // Applicable to usRate of tTourPt
                                                         
typedef struct                                           
{
    SHORT   sAzimuth    ;
    SHORT   sElevation  ;     
    USHORT  usRate      ;
    UCHAR   ucDwellSec  ;
    UCHAR   ucOptions   ;
        
} tTourPt;

typedef struct 
{
    UCHAR           ucCommandAndControl;
    UCHAR           ucTourNumber;
    UCHAR           ucPointNumber;
    tTourPt         Point;
    UCHAR           ucNextTour; // Note: Applies to entire tour, not just single point
    UCHAR           ucGrowth[9];
} tPAYLOAD_TOUR_POINT;

#define SLC_TOUR_PT_COMMAND_STORE       0x01
#define SLC_TOUR_PT_COMMAND_DELETE      0x02
#define SLC_TOUR_PT_COMMAND_RETRIEVE    0x04
#define SLC_TOUR_PT_COMMAND_RETURNED    0x08

//
// Number of bytes in a SER_PACKET before the payload...
//

#define SER_PACKET_LENGTH_B4_PAYLOAD    5


//==========================================
// Data structures
//==========================================

//
// Applicable to ucStateProtocol...
//

#define EXPECTING_STX             0
#define EXPECTING_PACKET_TYPE     1
#define EXPECTING_LENGTH          2
#define EXPECTING_SRC_ADDR        3
#define EXPECTING_DEST_ADDR       4
#define EXPECTING_PAYLOAD         5
#define EXPECTING_CHECKSUM        6
#define EXPECTING_ETX             7
#define PELCO_COMMAND			  8
                                  
typedef struct 
{
    UCHAR Stx;
    UCHAR PacketType;
    UCHAR PayloadLength;
    UCHAR SrcAddr;
    UCHAR DestAddr;

} tSER_PACKET_PREAMBLE;

typedef struct SER_PACKET
{
    tSER_PACKET_PREAMBLE  Preamble;
    UCHAR                 pPayload[SLP_MAX_PAYLOAD];
    UCHAR                 Checksum;
    UCHAR                 Etx;
    
} tSER_PACKET;

// typedef SER_PACKET *pSER_PACKET;

typedef struct
{
    tSER_PACKET_PREAMBLE    Preamble;
    tPAYLOAD_SLC_EXT_STATUS Payload;
    UCHAR                   Checksum;
    UCHAR                   Etx;
    
} SER_SLC_EXT_STATUS_PACKET;

typedef struct
{
    tSER_PACKET_PREAMBLE        Preamble;
    UCHAR                       Checksum;
    UCHAR                       Etx;
    
} SER_REQUEST_JCS_SETUP_PACKET;

typedef struct
{
    tSER_PACKET_PREAMBLE        Preamble;
    UCHAR                       Checksum;
    UCHAR                       Etx;
    
} SER_REQUEST_SLC_MISC_SETUP_PACKET;

typedef struct
{
    tSER_PACKET_PREAMBLE        Preamble;
    UCHAR                       Checksum;
    UCHAR                       Etx;
    
} SER_REQUEST_SLC_DEV_NAME_ETC_PACKET;

typedef struct
{
    tSER_PACKET_PREAMBLE        Preamble;
    tPAYLOAD_SLC_MISC_SETUP     Payload;
    UCHAR                       Checksum;
    UCHAR                       Etx;
    
} SER_SLC_MISC_SETUP_PACKET;

typedef struct
{
    tSER_PACKET_PREAMBLE            Preamble;
    tPAYLOAD_SLC_DEVICE_NAME_ETC    Payload;
    UCHAR                           Checksum;
    UCHAR                           Etx;
    
} SER_SLC_DEVICE_NAME_ETC_PACKET;

typedef struct
{
    tSER_PACKET_PREAMBLE        Preamble;
    tPAYLOAD_JCS_STATUS         Payload;
    UCHAR                       Checksum;
    UCHAR                       Etx;
    
} SER_JCS_STATUS_PACKET;

typedef struct
{
    tSER_PACKET_PREAMBLE        Preamble;
    tPAYLOAD_JCS_SETUP          Payload;
    UCHAR                       Checksum;
    UCHAR                       Etx;
    
} SER_JCS_SETUP_PACKET;


typedef struct
{
    tSER_PACKET_PREAMBLE        Preamble;
    tPAYLOAD_DO_POSITION_SL_REL Payload;
    UCHAR                       Checksum;
    UCHAR                       Etx;

} SER_DO_POSITION_SL_REL_PACKET;


typedef struct
{
    tSER_PACKET_PREAMBLE        Preamble;
    tPAYLOAD_DO_CALIBRATION     Payload;
    UCHAR                       Checksum;
    UCHAR                       Etx;
    
} SER_DO_CALIBRATION_PACKET;

typedef struct
{
    tSER_PACKET_PREAMBLE        Preamble;
    tPAYLOAD_PRB_CONFIG         Payload;
    UCHAR                       Checksum;
    UCHAR                       Etx;
    
} SER_PRB_CONFIG_PACKET;

typedef struct
{
    tSER_PACKET_PREAMBLE        Preamble;
    tPAYLOAD_TOUR_POINT         Payload;
    UCHAR                       Checksum;
    UCHAR                       Etx;

} SER_TOUR_POINT_PACKET;

typedef struct
{
    tSER_PACKET_PREAMBLE        Preamble;
    tPAYLOAD_SLC_BASIC_STATUS   Payload;
    UCHAR                       Checksum;
    UCHAR                       Etx;

} SER_SLC_BASIC_STATUS_PACKET;

typedef struct
{
    tSER_PACKET_PREAMBLE        Preamble;
    UCHAR                       Checksum;
    UCHAR                       Etx;

} SER_SLC_REQUEST_MISC_SETUP_PACKET;

typedef struct
{
    tSER_PACKET_PREAMBLE        Preamble;
    tPAYLOAD_TOUR_CONTROL       Payload;
    UCHAR                       Checksum;
    UCHAR                       Etx;

} SER_TOUR_CONTROL_PACKET;

typedef struct
{
    tSER_PACKET_PREAMBLE            Preamble;
    tPAYLOAD_ZEROIZE_LAMP_HOURS     Payload;
    UCHAR                           Checksum;
    UCHAR                           Etx;

} SER_ZEROIZE_LAMP_HOURS_PACKET;

//
// For use with SrcAddress / Destination Address / Protocol Addresses
//

#define SER_ADDRESS_OFFLINE             0xff
                                  
//
// Number of bytes in a SER_PACKET before the payload...
//

#define SER_PACKET_LENGTH_B4_PAYLOAD    5

#endif // #ifndef _SLPROTOCOL_H_