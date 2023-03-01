//**********************************************************************
// Filename:      SLC_Init.h
// Project:       Searchlight Controller Board
// Rights:        Copyright (c) 2006 The Carlisle and Finch Company
//                All Rights Reserved
// Creation Date: 9/11/06
// Compiler:      Tasking cc51
// Version:       v7.1r1
// Purpose:       This module contains project specific register/bit definitions.
// Modification History:
// Date         Author      Description
// ----         ------      -----------
// 9-11-06		JRN			Created
//**********************************************************************

#ifndef   _SLC_INC_
#define   _SLC_INC_


_sfrbit HBRIDGE_FAULT  _atbit(P3, 2);    // P3.2  -  HBRIDGE_FAULT, Open-Drain, Digital
_sfrbit ETH_RST        _atbit(P3, 7);    // P3.7  -  ETH_RST,       Push-Pull , Digital
_sfrbit RLY_AUX4       _atbit(P3, 4);    // P3.4  -  RLY_8,       Push-Pull,  Digital       // mbd Rev H SLC new AUX4

// _sfrbit TestBit0       _atbit(P4, 3);    // P4.3  -  TestBit,     Push-Pull,  Digital // Used for outputing a test signal to a scope for testing ver 1.71, Added by MAS on 6/20/2013
// _sfrbit TestBit1       _atbit(P4, 4);    // P4.4  -  TestBit,     Push-Pull,  Digital // Used for outputing a test signal to a scope for testing ver 1.71, Added by MAS on 6/20/2013
// _sfrbit TestBit2       _atbit(P4, 5);    // P4.5  -  TestBit,     Push-Pull,  Digital // Used for outputing a test signal to a scope for testing ver 1.71, Added by MAS on 6/25/2013
// _sfrbit MTR_CS1        _atbit(P4, 6);    // P4.6  -  MTR_CS1,     Push-Pull,  Digital
// _sfrbit MTR_CS2        _atbit(P4, 7);    // P4.7  -  MTR_CS2,     Push-Pull,  Digital

_sfrbit HSS_OUT_8       _atbit(P4, 0);    // P4.0  -  HSS_OUT_8,     Push-Pull,  Digital
_sfrbit HSS_OUT_7       _atbit(P4, 1);    // P4.1  -  HSS_OUT_7,     Push-Pull,  Digital
_sfrbit HSS_OUT_6       _atbit(P4, 2);    // P4.2  -  HSS_OUT_6,     Push-Pull,  Digital
_sfrbit HSS_OUT_5       _atbit(P4, 3);    // P4.3  -  HSS_OUT_5,     Push-Pull,  Digital
_sfrbit HSS_OUT_4       _atbit(P4, 4);    // P4.4  -  HSS_OUT_4,     Push-Pull,  Digital
_sfrbit HSS_OUT_3       _atbit(P4, 5);    // P4.5  -  HSS_OUT_3,     Push-Pull,  Digital
_sfrbit HSS_OUT_2       _atbit(P4, 6);    // P4.6  -  HSS_OUT_2,     Push-Pull,  Digital
_sfrbit HSS_OUT_1       _atbit(P4, 7);    // P4.7  -  HSS_OUT_1,     Push-Pull,  Digital
                                         
_sfrbit MTR3_DIS       _atbit(P5, 0);    // P5.0  -  MTR1_SLEEP,   Push-Pull,  Digital
_sfrbit TestBit_0      _atbit(P5, 1);    // P5.1  -  TestBit_0,   Push-Pull,  Digital
_sfrbit TestBit_1      _atbit(P5, 2);    // P5.2  -  TestBit_1,    Push-Pull,  Digital
_sfrbit TestBit_2      _atbit(P5, 3);    // P5.3  -  TestBit_2,   Push-Pull, Digital
_sfrbit TestBit_3      _atbit(P5, 4);    // P5.4  -  TestBit_3,  Push-Pull,  Digital
_sfrbit TestBit_4      _atbit(P5, 5);    // P5.5  -  TestBit_4,   Push-Pull,  Digital

//_sfrbit MTR1_STEP      _atbit(P5, 1);    // P5.1  -  MTR1_STEP,   Push-Pull,  Digital
//_sfrbit MTR1_DIR       _atbit(P5, 2);    // P5.2  -  MTR1_DIR,    Push-Pull,  Digital
//_sfrbit MTR1_HOME      _atbit(P5, 3);    // P5.3  -  MTR1_HOME,   Open-Drain, Digital
//_sfrbit MTR2_SLEEP     _atbit(P5, 4);    // P5.4  -  MTR2_SLEEP,  Push-Pull,  Digital
//_sfrbit MTR2_STEP      _atbit(P5, 5);    // P5.5  -  MTR2_STEP,   Push-Pull,  Digital
//_sfrbit MTR2_DIR       _atbit(P5, 6);    // P5.6  -  MTR2_DIR,    Push-Pull,  Digital
//_sfrbit MTR2_HOME      _atbit(P5, 7);    // P5.7  -  MTR2_HOME,   Open-Drain, Digital

_sfrbit MTR1_EN1       _atbit(P6, 0);    // P6.0  -  MTR1_EN1,    Push-Pull,  Digital
_sfrbit MTR1_EN2       _atbit(P6, 1);    // P6.1  -  MTR1_EN2,    Push-Pull,  Digital
_sfrbit MTR2_EN1       _atbit(P6, 2);    // P6.2  -  MTR2_EN1,    Push-Pull,  Digital
_sfrbit MTR2_EN2       _atbit(P6, 3);    // P6.3  -  MTR2_EN2,    Push-Pull,  Digital
_sfrbit MTR3_EN1       _atbit(P6, 4);    // P6.4  -  MTR3_EN1,    Push-Pull,  Digital
_sfrbit MTR3_EN2       _atbit(P6, 5);    // P6.5  -  MTR3_EN2,    Push-Pull,  Digital
_sfrbit MTR1_DIS       _atbit(P6, 6);    // P6.6  -  MTR1_DIS,    Push-Pull,  Digital
_sfrbit MTR2_DIS       _atbit(P6, 7);    // P6.7  -  MTR2_DIS,    Push-Pull,  Digital

_sfrbit RLY_LAMP       _atbit(P7, 0);    // P7.0  -  RLY_1,       Push-Pull,  Digital
_sfrbit RLY_LAMP_START _atbit(P7, 1);    // P7.1  -  RLY_2,       Push-Pull,  Digital
_sfrbit RLY_FOCUS_OUT  _atbit(P7, 2);    // P7.2  -  RLY_3,       Push-Pull,  Digital
_sfrbit RLY_FOCUS_IN   _atbit(P7, 3);    // P7.3  -  RLY_4,       Push-Pull,  Digital
_sfrbit RLY_AUX2       _atbit(P7, 4);    // P7.4  -  RLY_5,       Push-Pull,  Digital       // ACM!! v1.68 Changed from CAMERA to AUX2
_sfrbit RLY_AUX3       _atbit(P7, 5);    // P7.5  -  RLY_6,       Push-Pull,  Digital       // ACM!! v1.33 Changed from ZOOM to AUX3
_sfrbit RLY_AUX1       _atbit(P7, 6);    // P7.6  -  RLY_7,       Push-Pull,  Digital       // ACM!! v1.33 Changed from MISC to AUX1
_sfrbit EN_PWM         _atbit(P7, 7);    // P7.7  -  EN_PWM,      Push-Pull,  Digital

#define RELAY_LAMP          0
#define RELAY_LAMP_START    1
#define RELAY_FOCUS_OUT     2
#define RELAY_FOCUS_IN      3
#define RELAY_AUX2          4           // ACM!! v1.33 Changed from CAMERA to AUX2
#define RELAY_AUX3          5           // ACM!! v1.33 Changed from ZOOM to AUX3
#define RELAY_AUX1          6           // ACM!! v1.33 Changed from MISC to AUX1
#define RELAY_AUX4			7			// mbd for Rev. H new SLC
#define AUX5				8			// not necessarily relays - goes to high side driver chip - mbd rev H
#define AUX6				9
#define AUX7				10
#define AUX8				11
#define AUX9_UP				12
#define AUX9_DOWN				13
#define AUX10_UP				14
#define AUX10_DOWN				15

// _sfrbit DEBUG          _atbit(P4, 2);  // mbd need test points with Rev H SLC

extern UCHAR ucSlcRstsrc;	// v1.64
			   
void InitializeSLC ( void );


#endif // _SLC_INC_