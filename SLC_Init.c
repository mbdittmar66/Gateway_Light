//**********************************************************************
// Project:       Searchlight Controller Board
// Filename:      SLC_Init.c
// Rights:        Copyright (c) 2006 The Carlisle & Finch Co.
//                All Rights Reserved
// Creation Date: 9/11/06
// Compiler:      Tasking cc51
// Version:       v7.1r1
// Purpose:       This module contains hardware and software initialization interface.
// Modification History:
// Date         Author      Description
// ----         ------      -----------
// 9-11-06		JRN			Created
//**********************************************************************

#include "C8051F040.h"
#include "SlProtocol.h"
#include "SLC_Init.h"
#include "utils.h"
#include "serial.h"
#include "timers.h"
// #include "i2c.h"
#include "stdlib.h"
#include "spi.h"
#include "version.h"
#include "flash.h"

//
// Recognized globals
//

UCHAR ucSlcRstsrc;	// v1.64

//
// Forward function prototypes.
//

VOID Voltage_Ref_Init   ( VOID );
VOID PCA_Init           ( VOID );
VOID Reset_Sources_Init ( VOID );
VOID ADC_Init           ( VOID );
VOID DAC_Init           ( VOID );
VOID Port_IO_Init       ( VOID );
VOID Oscillator_Init    ( VOID );
VOID Interrupts_Init    ( VOID );
VOID SpiUartInitialize  ( VOID );
VOID InitializePid      ( VOID );

//=============================================================================
// Function   : Reset_Sources_Init ()
// Purpose    : This function initializes reset sources of the CPU.
// Parameters : Nothing is passed. 
// Returns    : Nothing is returned.
//=============================================================================

VOID Reset_Sources_Init ( VOID )
{
    //
    // v1.64 - Store initial value of of RSTSRC in RAM so that we
    // can print it out later with the copyright (etc) info.
    // 

    SFRPAGE   = LEGACY_PAGE;
    ucSlcRstsrc = RSTSRC;

    // 
    // v1.64 - Clear RSTSRC
    //

    RSTSRC = 0;

#if 0  // mbd temp turn off
    //
    // Disable WDT for debugging tour stops.
#pragma asm
    CLR     EA
    MOV     WDTCN,#0DEh
    MOV     WDTCN,#0ADh
    SETB    EA
#pragma endasm
#endif

} // Reset_Sources_Init () 

//=============================================================================
// Function   : ADC_Init ()
// Purpose    : This function initializes the analog to digital convertor.
// Parameters : Nothing is passed. 
// Returns    : Nothing is returned.
//=============================================================================

VOID ADC_Init ( VOID )
{
    SFRPAGE = ADC0_PAGE;
    ADC0CN  = 0xC4;      // Tracking starts with Timer3 overflow.
    ADC0CF  = 0xF8;//0xA0; 

} // ADC_Init ()

//=============================================================================
// Function   : Voltage_Ref_Init ()
// Purpose    : This function initializes the voltage reference.
// Parameters : Nothing is passed. 
// Returns    : Nothing is returned.
//=============================================================================

VOID Voltage_Ref_Init ( VOID )
{
    SFRPAGE   = ADC0_PAGE;
    REF0CN    = 0x03;

} // Voltage_Ref_Init ()



//=============================================================================
// Function   : DAC_Init ()
// Purpose    : This function initializes the digital to analog converter.
// Parameters : Nothing is passed. 
// Returns    : Nothing is returned.
//=============================================================================

VOID DAC_Init ( VOID )
{
    SFRPAGE   = DAC0_PAGE;
    DAC0CN    = 0x00;
    SFRPAGE   = DAC1_PAGE;
    DAC1CN    = 0x00;

} // DAC_Init ()




//=============================================================================
// Function   : PCA_Init ()
// Purpose    : This function initializes the Programmable Counter Array ( PCA ).
// Parameters : Nothing is passed. 
// Returns    : Nothing is returned.
//=============================================================================

VOID PCA_Init ( VOID )
{
    SFRPAGE  = PCA0_PAGE;
    PCA0CN   = 0x40;        // Enable PCA0
    PCA0MD   = 0x80;        // Uses SYSCLK / 12
//    PCA0CPM0 = 0x00;        // PCA0 module 0 disable ( Motor 1 )
//    PCA0CPM1 = 0x00;        // PCA0 module 1 disable ( Motor 2 )
//    PCA0CPM2 = 0x00;        // PCA0 module 2 disable ( Motor 3 )
//    PCA0CPH0 = 0x00;
//    PCA0CPL0 = 0x00;

} // PCA_Init ()




//=============================================================================
// Function   : Port_IO_Init ()
// Purpose    : This function initializes port I/O.
// Parameters : Nothing is passed. 
// Returns    : Nothing is returned.
//=============================================================================

VOID Port_IO_Init ( VOID )
{
    SFRPAGE   = CONFIG_PAGE;
    
    EMI0CF = 0x00;  // Make sure External Mem Intf is disabled;

    // P0.0  -  TX0 (UART0), Push-Pull,  Digital
    // P0.1  -  RX0 (UART0), Open-Drain, Digital
    // P0.2  -  SCK  (SPI0), Push-Pull ,  Digital
    // P0.3  -  MISO (SPI0), Open-Drain, Digital
    // P0.4  -  MOSI (SPI0), Push-Pull,  Digital
    // P0.5  -  NSS  (SPI0), Push-Pull,  Digital
    // P0.6  -  SDA (SMBus), Open-Drain, Digital
    // P0.7  -  SCL (SMBus), Push-Pull,  Digital

    P0MDOUT = 0xB5;

    // P1.0  -  MTR1_FB (AIN2.0), Open-Drain, Analog
    // P1.1  -  MTR2_FB (AIN2.1), Open-Drain, Analog
    // P1.2  -  MTR3_FB (AIN2.2), Open-Drain, Analog
    // P1.3  -  TX1 (UART1),      Push-Pull,  Digital
    // P1.4  -  RX1 (UART1),      Open-Drain, Digital
    // P1.5  -  CEX0 (PCA),       Push-Pull,  Digital
    // P1.6  -  CEX1 (PCA),       Push-Pull,  Digital
    // P1.7  -  CEX2 (PCA),       Push-Pull,  Digital

    P1MDIN  = 0xF8;
    P1MDOUT = 0xE8;

    // P2.0  -  CEX3 (PCA),        Open-Drain, Digital
    // P2.1  -  CEX4 (PCA),        Open-Drain, Digital
    // P2.2  -  CEX5 (PCA),        Open-Drain, Digital
    // P2.3  -  UART2_IRQ (/INT0), Open-Drain, Digital
    // P2.4  -  DIPSW1,            Open-Drain, Digital
    // P2.5  -  DIPSW2,            Open-Drain, Digital
    // P2.6  -  DIPSW3,            Open-Drain, Digital
    // P2.7  -  DIPSW4,            Open-Drain, Digital

//    P2MDIN  = 0x0F;
    P2MDOUT = 0x00;
    P2      = 0xFF;     // Turn off output driver.		
    // P3.0  -  UART3_IRQ (/INT1), Open-Drain, Digital
    // P3.1  -  Unassigned,        Open-Drain, Digital
    // P3.2  -  HBRIDGE_FAULT,     Open-Drain, Digital
    // P3.3  -  LCD_RS,            Push-Pull , Digital
    // P3.4  -  AUX4 ( RLY8) in rev H new SLC formerly LCD_CS,            Push-Pull , Digital
    // P3.5  -  UART2_CS,          Push-Pull , Digital
    // P3.6  -  UART3_CS,          Push-Pull , Digital
    // P3.7  -  ETH_RST,           Push-Pull , Digital

    P3MDOUT = 0xF8;
	P3 = 0xFF;  // make sure AUX4 defaults to ON.
	//P3 = 0xEF;  // make sure AUX4 defaults to OFF.  

    // P4.0  -  LAMP_OVERTEMP
    // P4.1  -  AUX_DIGITAL_INPUT
    // P4.2  -  DEBUG_PIN,  Open-Drain, Digital
    // P4.3  -  Test_PIN0, Push-Pull,   Digital
    // P4.4  -  Test_PIN1, Push-Pull,   Digital
    // P4.5  -  Test_PIN2, Push-Pull,   Digital
    // P4.6  -  MTR_CS1,    Push-Pull,  Digital
    // P4.7  -  LSC_RS_SLC_B / MTR_CS2, Push-Pull,  Digital

// P4.0  -  HSS_OUT_8,     Push-Pull,  Digital
// P4.1  -  HSS_OUT_7,     Push-Pull,  Digital
// P4.2  -  HSS_OUT_6,     Push-Pull,  Digital
// P4.3  -  HSS_OUT_5,     Push-Pull,  Digital
// P4.4  -  HSS_OUT_4,     Push-Pull,  Digital
// P4.5  -  HSS_OUT_3,     Push-Pull,  Digital
// P4.6  -  HSS_OUT_2,     Push-Pull,  Digital
// P4.7  -  HSS_OUT_1,     Push-Pull,  Digital    
    P4MDOUT = 0xFF; // Rev H SLC all p/p digi out
    P4 = 0x00;		// all off

    // P5.0  -  MTR3_DIS,  Push-Pull,  Digital
	// P5.1  -  TestBit_0,   Push-Pull,  Digital
	// P5.2  -  TestBit_1,    Push-Pull,  Digital
	// P5.3  -  TestBit_2,   Push-Pull, Digital
	// P5.4  -  TestBit_3,  Push-Pull,  Digital
	// P5.5  -  TestBit_4,   Push-Pull,  Digital
    // P5.6  -  LAMP_OVERTEMP rev H SLC board
    // P5.7  -  AUX_DIGITAL_INPUT rev H SLC board

    P5MDOUT = 0x3F;	// rev H SLC 
    
    // P6.0  -  MTR1_EN1,    Push-Pull,  Digital
    // P6.1  -  MTR1_EN2,    Push-Pull,  Digital
    // P6.2  -  MTR2_EN1,    Push-Pull,  Digital
    // P6.3  -  MTR2_EN2,    Push-Pull,  Digital
    // P6.4  -  MTR3_EN1,    Push-Pull,  Digital
    // P6.5  -  MTR3_EN2,    Push-Pull,  Digital
    // P6.6  -  MTR1_DIS,    Push-Pull,  Digital
    // P6.7  -  MTR2_DIS,    Push-Pull,  Digital
    
    P6MDOUT = 0xFF;
           
    // P7.0  -  RLY_1,       Push-Pull,  Digital
    // P7.1  -  RLY_2,       Push-Pull,  Digital
    // P7.2  -  RLY_3,       Push-Pull,  Digital
    // P7.3  -  RLY_4,       Push-Pull,  Digital
    // P7.4  -  RLY_5,       Push-Pull,  Digital
    // P7.5  -  RLY_6,       Push-Pull,  Digital
    // P7.6  -  RLY_7,       Push-Pull,  Digital
    // P7.7  -  EN_PWM,      Push-Pull,  Digital

    P7MDOUT = 0xFF;
    P7 = 0;
    
//    XBR0      = 0x06;               // UART0 / PCA0 modules / SPI / SMBus enable
    XBR0      = 0x1F;               // UART0 / PCA0 modules / SPI / SMBus enable
    XBR1      = 0x00;               // 
    XBR2      = 0x44;               // Xbar enable / UART1 enable
    LAMP_OVERTEMP       = 1;
    AUX_DIGITAL_INPUT   = 1;
    
//    XBR3      = 0x80;             // CAN enable

    //
    // Initialize UART chip selects.
    //
    
    SFRPAGE    = CONFIG_PAGE;
    UART2_CS   = 1;
    UART3_CS   = 1;

} // Port_IO_Init ()



//=============================================================================
// Function   : Oscillator_Init ()
// Purpose    : This function initializes the oscillator.
// Parameters : Nothing is passed. 
// Returns    : Nothing is returned.
//=============================================================================

VOID Oscillator_Init ( VOID )
{
    SFRPAGE = CONFIG_PAGE;

    //
    // External CMOS Clock Mode.
    //
    
    OSCXCN = 0x20;
    
    //
    // SYSCLK derived from the External Oscillator Circuit.
    //
    
    CLKSEL = 0x01;
    
    //
    // Internal Oscillator disabled.
    //
    
    OSCICN = 0x00;

} // Oscillator_Init ()



//=============================================================================
// Function   : Interrupts_Init ()
// Purpose    : This function initializes interrupts.
// Parameters : Nothing is passed. 
// Returns    : Nothing is returned.
//=============================================================================

VOID Interrupts_Init ( VOID )
{
    //
    // IE: Interrupt Enable
    //  Bit7 - EA: Enable All Interrupts.
    //  Bit6 - IEGF0: General Purpose Flag 0.
    //  Bit5 - ET2: Enabler Timer 2 Interrupt.
    //  Bit4 - ES0: Enable UART0 Interrupt.
    //  Bit3 - ET1: Enable Timer 1 Interrupt. 
    //  Bit2 - EX1: Enable External Interrupt 1. 
    //  Bit1 - ET0: Enable Timer 0 Interrupt. 
    //  Bit0 - EX0: Enable External Interrupt 0.
    //
    
    IE = 0x92;
    IP = 0x12;
    
    //
    // EIE1: Extended Interrupt Enable 1
    //  Bit7 - Reserved.
    //  Bit6 - CP2IE: Enable Comparator (CP2) Interrupt.
    //  Bit5 - CP1IE: Enable Comparator (CP1) Interrupt.
    //  Bit4 - CP0IE: Enable Comparator (CP0) Interrupt.
    //  Bit3 - EPCA0: Enable Programmable Counter Array (PCA0) Interrupt.
    //  Bit2 - EWADC0: Enable Window Comparison ADC0 Interrupt.
    //  Bit1 - ESMB0: Enable System Management Bus (SMBus0) Interrupt.
    //  Bit0 - ESPI0: Enable Serial Peripheral Interface (SPI0) Interrupt. 
    //
    
    EIE1 = 0x02;

    //
    // EIE2: Extended Interrupt Enable 2
    //  Bit7 - Reserved
    //  Bit6 - ES1: Enable UART1 Interrupt.
    //  Bit5 - ECAN0: Enable CAN Controller Interrupt.
    //  Bit4 - EADC2: Enable ADC2 End Of Conversion Interrupt (C8051F040/1/2/3 only).
    //  Bit3 - EWADC2: Enable Window Comparison ADC2 Interrupt (C8051F040/1/2/3 only).
    //  Bit2 - ET4: Enable Timer 4 Interrupt
    //  Bit1 - EADC0: Enable ADC0 End of Conversion Interrupt.
    //  Bit0 - ET3: Enable Timer 3 Interrupt.
    //  
    
    EIE2 = 0x40;
    EIP2 = 0x41;

} // Interrupts_Init ()

//=============================================================================
// Function   : InitializeMotors ()
// Purpose    : This function initializes the PID coefficients.
// Parameters : Nothing is passed. 
// Returns    : Nothing is returned.
//=============================================================================

VOID InitializeMotors ( VOID )
{
    //
    // Default PID gain values for pan motor.
    //
    
    g_PanMotor.ID = PAN_MOTOR;
    g_PanMotor.Gain.P = g_BaseConfig[g_BaseType].ucPanPNorm;
    g_PanMotor.Gain.I = g_BaseConfig[g_BaseType].ucPanINorm;
    g_PanMotor.Gain.D = g_BaseConfig[g_BaseType].ucPanDNorm;
    g_PanMotor.Speed = 128;
    g_PanMotor.LastSpeed = 128;
    g_PanMotor.Deadband = g_BaseConfig[g_BaseType].ucPanDeadband;

    //
    // Default PID gain values for tilt motor.
    //
    
    g_TiltMotor.ID = TILT_MOTOR;
    g_TiltMotor.Gain.P = g_BaseConfig[g_BaseType].ucTiltPNorm;
    g_TiltMotor.Gain.I = g_BaseConfig[g_BaseType].ucTiltINorm;
    g_TiltMotor.Gain.D = g_BaseConfig[g_BaseType].ucTiltDNorm;
    g_TiltMotor.Speed = 128;
    g_TiltMotor.LastSpeed = 128;
    g_TiltMotor.Deadband = g_BaseConfig[g_BaseType].ucTiltDeadband;

    //
    // Default motor deadband value.
    //
    
    g_FocusMotor.Gain.P = 10;
    g_FocusMotor.Gain.I = 10;
    g_FocusMotor.Gain.D = 0;
    g_FocusMotor.Speed = 128;    
    g_FocusMotor.LastSpeed = 128;
    g_FocusMotor.Deadband = 5;

} // InitializeMotors ( VOID )




//=============================================================================
// Function   : InitializeSLC ()
// Purpose    : This function initializes the SearchLight Control Board hardware and software.
// Parameters : Nothing is passed. 
// Returns    : Nothing is returned.
//=============================================================================

VOID InitializeSLC ( VOID )
{
    UCHAR i;

    Reset_Sources_Init  ( );
    KICK_WDT;                // Kick the watchdog
    Port_IO_Init        ( );
    KICK_WDT;                // Kick the watchdog
    Oscillator_Init     ( );
    KICK_WDT;                // Kick the watchdog
    InitializeTimers    ( );
    KICK_WDT;                // Kick the watchdog
    PCA_Init            ( );
    KICK_WDT;                // Kick the watchdog
    SerInitializeUARTs  ( );
    KICK_WDT;                // Kick the watchdog
    // I2cInitializeSMBus  ( );
    SpiInitialize       ( );
    KICK_WDT;                // Kick the watchdog
    ADC_Init            ( );
    KICK_WDT;                // Kick the watchdog
    DAC_Init            ( );
    KICK_WDT;                // Kick the watchdog
    Voltage_Ref_Init    ( );
    KICK_WDT;                // Kick the watchdog
    Interrupts_Init     ( );
    KICK_WDT;                // Kick the watchdog
    InitializeMotors    ( );
    KICK_WDT;                // Kick the watchdog
    SpiInitializeUart2  ( );                     
    KICK_WDT;                // Kick the watchdog
    SpiInitializeUart3  ( );                                 
    
    //
    // Initialize the board number.  May
    // be dynamically changed by the servicing
    // technician.
    //
                        
    SFRPAGE = CONFIG_PAGE;
    g_BoardNumber = ( ( P2 & 0xF0 ) >> 4 );
    
    SerSlcMiscSetupPayload.usRoFirmwareVersion          = VERSION_DECIMAL;
    
    SerRequestSlcDevNameEtc  = 
    SerRequestSlcMiscSetup   = 
    SerTourPointDestAddr     = INVALID_PROTOCOL_ADDRESS;
    
    //
    // Initialize the board type using DIP switches.
    //

    SFRPAGE = CONFIG_PAGE;
    g_BaseType = P2;
    g_BaseType = P2 & 0x07;

    //
    // Detect SmartView 1 DIP SW settings (v1.59)
    //

    if ( g_BaseType == 7 )
    {
        g_SmartView1 = TRUE;
    }

    if ( g_BaseType > 2 )
        g_BaseType = 0;
	
    //
    // Initialize all relays off.
    //

    SFRPAGE = CONFIG_PAGE;
    P7 = 0;
    
    //
    // Initialize all motors off.
    //
    
//    UtilSetMotorSpeed ( MOTOR_1, 128 );			  mbd_gateway_lite
//    UtilSetMotorSpeed ( MOTOR_2, 128 );
//    UtilSetMotorSpeed ( MOTOR_3, 128 );

	g_TimeMs.MotorTimeout = SIXTY_SECONDS;		// v1.73

    //
    // Tell the PRB to send up config data on next PRB_SLAVE_SPEAK packet.
    //

//    g_PrbSlaveSpeakPayload |= SLC_POWER_FAILED_BIT;		mbd_gateway_lite
    
    //
    // Initialize the polling table...
    //
    
    for ( i = 0; i < hbound1(g_PollTable); i++)
    {
        g_PollTable[i] = g_PollLowThreshold;
    }

    //
    // Do not assume the master role for 
    // at at least 1 second following POR. 
    // To mitigate 2 boards with the exact
    // same power on dynamic, note that the 
    // holdoff is offset based
    // upon the host board number.
    //
    
    g_TimeMs.HoldoffMastering = 1000 + 100*g_BoardNumber;

    //
    // Indicate SmartView 1 SLC if so configured.
    //

    if (g_SmartView1)
    {
        SerSlcExtStatusPayload.ucByte8 |= SLC_EXT_STAT_B8_SMARTVIEW1;
    }
    
    //
    // Indicate to connected devices that this SLC has reset.
    //
    
    SerSlcExtStatusPayload.ucByte10 |= SLC_EXT_STAT_B10_SLC_RESET;
    ucUtlJcsStatByte3               |= JCS_STAT_B3_REQ_SLC_EXT_STAT_PKT;
    
    g_RoutedRxSerPortPacket.Preamble.SrcAddr = INVALID_PROTOCOL_ADDRESS;
    g_RoutedRxEthPortPacket.Preamble.SrcAddr = INVALID_PROTOCOL_ADDRESS;
    
    SerSlcExtStatusPayload.BasicStatus.ucActiveTour = TOUR_INVALID;
    
    KICK_WDT;                // Kick the watchdog
    FlashInit ( );
    KICK_WDT;                // Kick the watchdog
    
    sendNameOnce = 1; // Used to send the SLC Device Name packet once during startup.

} // InitializeSLC ()
