//###########################################################################
//
// FILE:   DSP2803x_EPwm.c
//
// TITLE:  DSP2803x EPwm Initialization & Support Functions.
//
//###########################################################################
// $TI Release: F2803x Support Library v2.02.00.00 $
// $Release Date: Sun Oct  4 16:06:22 IST 2020 $
// $Copyright:
// Copyright (C) 2009-2020 Texas Instruments Incorporated - http://www.ti.com/
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions 
// are met:
// 
//   Redistributions of source code must retain the above copyright 
//   notice, this list of conditions and the following disclaimer.
// 
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the 
//   documentation and/or other materials provided with the   
//   distribution.
// 
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// $
//###########################################################################

//
// Included Files
//
#include "DSP2803x_Device.h"     // DSP2803x Headerfile Include File
#include "DSP2803x_Examples.h"   // DSP2803x Examples Include File

typedef struct
{
    volatile struct EPWM_REGS *EPwmRegHandle;
    Uint16 EPwm_CMPA_Direction;
    Uint16 EPwm_CMPB_Direction;
    Uint16 EPwmTimerIntCount;
    Uint16 EPwmMaxCMPA;
    Uint16 EPwmMinCMPA;
    Uint16 EPwmMaxCMPB;
    Uint16 EPwmMinCMPB;
}EPWM_INFO;



EPWM_INFO epwm1_info;
EPWM_INFO epwm2_info;
EPWM_INFO epwm4_info;
EPWM_INFO epwm5_info;

// pwm defines
//
// Defines for the period for each timer
//
#define EPWM1_TIMER_TBPRD  7500  // Period register
#define EPWM1_MAX_CMPA     (int)(EPWM1_TIMER_TBPRD *.8)
#define EPWM1_MIN_CMPA       50
#define EPWM1_MAX_CMPB     1950
#define EPWM1_MIN_CMPB       50

#define EPWM2_TIMER_TBPRD  7500  // Period register
#define EPWM2_MAX_CMPA     1950
#define EPWM2_MIN_CMPA       50
#define EPWM2_MAX_CMPB     1950
#define EPWM2_MIN_CMPB       50

#define EPWM3_TIMER_TBPRD  7500  // Period register
#define EPWM3_MAX_CMPA     1950
#define EPWM3_MIN_CMPA       50
#define EPWM3_MAX_CMPB     1950
#define EPWM3_MIN_CMPB     1050

#define EPWM4_TIMER_TBPRD  7500  // Period register
#define EPWM4_MAX_CMPA     1950
#define EPWM4_MIN_CMPA       50
#define EPWM4_MAX_CMPB     1950
#define EPWM4_MIN_CMPB       50

#define EPWM5_TIMER_TBPRD  7500  // Period register
#define EPWM5_MAX_CMPA     1950
#define EPWM5_MIN_CMPA       50
#define EPWM5_MAX_CMPB     1950
#define EPWM5_MIN_CMPB       50



//
// Maximum Dead Band Defines
//
#define EPWM1_MAX_DB   0x03FF
#define EPWM2_MAX_DB   0x03FF
#define EPWM3_MAX_DB   0x03FF

#define EPWM1_MIN_DB   20
#define EPWM2_MIN_DB   20
#define EPWM3_MIN_DB   0

#define EPWM4_MIN_DB   20
#define EPWM5_MIN_DB   20
#define EPWM6_MIN_DB   0

//
// Defines that keep track of which way the Dead Band is moving
//
#define DB_UP   1
#define DB_DOWN 0


//
// Defines that keep track of which way the compare value is moving
//
#define EPWM_CMP_UP   1
#define EPWM_CMP_DOWN 0


void
InitEPwm1Example()
{
    //
    // Setup TBCLK
    //
    EPwm1Regs.TBPRD = EPWM1_TIMER_TBPRD;   // Set timer period 801 TBCLKs 2000 = 15.02 KHZ
    EPwm1Regs.TBPHS.half.TBPHS = 0x0000;   // Phase is 0
    EPwm1Regs.TBCTR = 0x0000;              // Clear counter
    EPwm1Regs.TBCTL.bit.SYNCOSEL = 01;     // select synch output on counter = 0 This will be
                                           //   used to sync EPwm2 180 degrees out of phase.

    //
    // Set Compare values
    //
    EPwm1Regs.CMPA.half.CMPA = (EPWM1_TIMER_TBPRD/2);     // Set compare A value
    EPwm1Regs.CMPB = 0x8000;               // Set Compare B value

    //
    // Setup counter mode
    //
    EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up
    EPwm1Regs.TBCTL.bit.PHSEN = TB_ENABLE;        // Disable phase loading
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;       // Clock ratio to SYSCLKOUT
    EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV1;

    //
    // Setup shadowing
    //
    EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;  // Load on Zero
    EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

    //
    // Set actions
    //
    EPwm1Regs.AQCTLA.bit.CAU = AQ_SET;    // Set PWM1A on event A, up count
    EPwm1Regs.AQCTLA.bit.CAD = AQ_CLEAR;  // Clear PWM1A on event A, down count

    EPwm1Regs.AQCTLB.bit.CBU = AQ_SET;   // Set PWM1B on event B, up count
    EPwm1Regs.AQCTLB.bit.CBD = AQ_CLEAR; // Clear PWM1B on event B, down count

    //
    // Active Low PWMs - Setup Deadband
    //
    EPwm1Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;   // Use rising edge and falling edges
    EPwm1Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;        // A is active High complementary
    EPwm1Regs.DBCTL.bit.IN_MODE = DBA_ALL;      // PWM A is used for dead-band
    EPwm1Regs.DBRED = EPWM1_MIN_DB;             // these settings correspond to mode 2
    EPwm1Regs.DBFED = EPWM1_MIN_DB;             // in SPRU10 table 3-14
    //EPwm1_DB_Direction = DB_UP;  from example code for changing deadband example
 //
    // Interrupt where we will change the Compare Values
    //
    EPwm1Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;      // Select INT on Zero event
    EPwm1Regs.ETSEL.bit.INTEN = 1;                 // Enable INT

    // Setup ADC SOC trigger
    EPwm1Regs.ETSEL.bit.SOCASEL = ET_CTR_ZERO;   // Start Conversion on CTR = 0 (valley)
    EPwm1Regs.ETSEL.bit.SOCAEN = 1;               // enable start of conversion

    EPwm1Regs.ETPS.bit.INTPRD = ET_1ST;            // Generate INT on 1st event
    EPwm1Regs.ETPS.bit.SOCAPRD = 0b01;
    //
    // Information this example uses to keep track of the direction the
    // CMPA/CMPB values are moving, the min and max allowed values and
    // a pointer to the correct ePWM registers
    //

    //
    // Start by increasing CMPA & decreasing CMPB
    //
  /*  epwm1_info.EPwm_CMPA_Direction = EPWM_CMP_UP;
    epwm1_info.EPwm_CMPB_Direction = EPWM_CMP_DOWN;

    epwm1_info.EPwmTimerIntCount = 0;      //Zero the interrupt counter
    epwm1_info.EPwmRegHandle = &EPwm1Regs; //Set the pointer to the ePWM module
    epwm1_info.EPwmMaxCMPA = EPWM1_MAX_CMPA;  // Setup min/max CMPA/CMPB values
    epwm1_info.EPwmMinCMPA = EPWM1_MIN_CMPA;
    epwm1_info.EPwmMaxCMPB = EPWM1_MAX_CMPB;
    epwm1_info.EPwmMinCMPB = EPWM1_MIN_CMPB; */
}
// The proper procedure for enabling the ePWM clocks is as follows:
// 1. Enable the individual ePWM module clocks. This is described in the specific device version of the
//    System Control and Interrupts Reference Guide listed in Section 1.
// 2. Set TBCLKSYNC = 0. This will stop the time-base clock within any enabled ePWM module.
// 3. Configure the prescaler values and desired ePWM modes.
// 4. Set TBCLKSYNC = 1.
//
// InitEPwm2Example - EPwm2 example
//
void
InitEPwm2Example()
{
    //
    // Setup TBCLK
    //
    EPwm2Regs.TBPRD = EPWM2_TIMER_TBPRD;        // Set timer period 801 TBCLKs
    EPwm2Regs.TBPHS.half.TBPHS = EPWM2_TIMER_TBPRD;        // Phase is 180
    EPwm2Regs.TBCTR = 0;                     // Default value - not important because
                                             //    pwm2 is synced to EPwm1
    //
    // Set Compare values
    //
    EPwm2Regs.CMPA.half.CMPA = (EPWM1_TIMER_TBPRD/2);     // Set compare A value
    EPwm2Regs.CMPB = EPWM2_MAX_CMPB;               // Set Compare B value

    //
    // Setup counter mode
    //
    EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up
    EPwm2Regs.TBCTL.bit.PHSEN = TB_ENABLE;        // Disable phase loading
    EPwm2Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;       // Clock ratio to SYSCLKOUT
    EPwm2Regs.TBCTL.bit.CLKDIV = TB_DIV1;
    EPwm2Regs.TBCTL.bit.SWFSYNC = 0;

    //
    // Setup shadowing
    //
    EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm2Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm2Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;  // Load on Zero
    EPwm2Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

    //
    // Set actions
    //
    EPwm2Regs.AQCTLA.bit.CAU = AQ_CLEAR;    // Set PWM2A on event A, up count
    EPwm2Regs.AQCTLA.bit.CAD = AQ_SET;  // Clear PWM2A on event B, down count

    EPwm2Regs.AQCTLB.bit.CBU = AQ_CLEAR;        // Clear PWM2B on zero
    EPwm2Regs.AQCTLB.bit.CBD = AQ_SET;       // Set PWM2B on period

    //
    // Active Low PWMs - Setup Deadband
    //
    EPwm2Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm2Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
    EPwm2Regs.DBCTL.bit.IN_MODE = DBA_ALL;
    EPwm2Regs.DBRED = EPWM2_MIN_DB;
    EPwm2Regs.DBFED = EPWM2_MIN_DB;
//
    // Interrupt where we will change the Compare Values
    //
    EPwm2Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;     // Select INT on Zero event -valley
                                                  // EPwm2 is 180 phase shift from EPwm1
    EPwm2Regs.ETSEL.bit.INTEN = 1;                // Enable INT
    EPwm2Regs.ETPS.bit.INTPRD = ET_1ST;           // Generate INT on 3rd event


    // Setup ADC SOC trigger
    EPwm2Regs.ETSEL.bit.SOCASEL = ET_CTR_ZERO;  // use this for SOC 8 to start a new group of
                                                // conversions.
    // Information this example uses to keep track of the direction the
    // CMPA/CMPB values are moving, the min and max allowed values and
    // a pointer to the correct ePWM registers
    //

    //
    // Start by increasing CMPA & increasing CMPB
    //
    /*epwm2_info.EPwm_CMPA_Direction = EPWM_CMP_UP;
    epwm2_info.EPwm_CMPB_Direction = EPWM_CMP_UP;

    epwm2_info.EPwmTimerIntCount = 0;      //Zero the interrupt counter
    epwm2_info.EPwmRegHandle = &EPwm2Regs; //Set the pointer to the ePWM module
    epwm2_info.EPwmMaxCMPA = EPWM2_MAX_CMPA;  // Setup min/max CMPA/CMPB values
    epwm2_info.EPwmMinCMPA = EPWM2_MIN_CMPA;
    epwm2_info.EPwmMaxCMPB = EPWM2_MAX_CMPB;
    epwm2_info.EPwmMinCMPB = EPWM2_MIN_CMPB; */
}

//
// InitEPwm3Example - EPwm4 example
//
void
InitEPwm4Example(void)
{
    //
    // Setup TBCLK
    //
    EPwm4Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;// Count up/down
    EPwm4Regs.TBPRD = EPWM4_TIMER_TBPRD;          // Set timer period
    EPwm4Regs.TBCTL.bit.PHSEN = TB_DISABLE;       // Disable phase loading
    EPwm4Regs.TBPHS.half.TBPHS = 0x0000;          // Phase is 0
    EPwm4Regs.TBCTR = 0x0000;                     // Clear counter
    EPwm4Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;      // Clock ratio to SYSCLKOUT
    EPwm4Regs.TBCTL.bit.CLKDIV = TB_DIV1;

    //
    // Setup shadow register load on ZERO
    //
    EPwm4Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm4Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm4Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    EPwm4Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

    //
    // Set Compare values
    //
    EPwm4Regs.CMPA.half.CMPA = EPWM4_MIN_CMPA;    // Set compare A value
    EPwm4Regs.CMPB = EPWM4_MAX_CMPB;              // Set Compare B value
    //
    // Set actions
    //
    EPwm4Regs.AQCTLA.bit.CAU = AQ_SET;    // Set PWM1A on event A, up count
    EPwm4Regs.AQCTLA.bit.CAD = AQ_CLEAR;  // Clear PWM1A on event A, down count

    EPwm4Regs.AQCTLB.bit.CBU = AQ_SET;   // Set PWM1B on event B, up count
    EPwm4Regs.AQCTLB.bit.CBD = AQ_CLEAR; // Clear PWM1B on event B, down count

    //
    // Active Low PWMs - Setup Deadband
    //
    EPwm4Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm4Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;        // A is active High complementary
    EPwm4Regs.DBCTL.bit.IN_MODE = DBA_ALL;
    EPwm4Regs.DBRED = EPWM1_MIN_DB;
    EPwm4Regs.DBFED = EPWM1_MIN_DB;

    //
    // Interrupt where we will change the Compare Values
    //
    EPwm4Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;     // Select INT on Zero event
    EPwm4Regs.ETSEL.bit.INTEN = 1;                // Enable INT
    EPwm4Regs.ETPS.bit.INTPRD = ET_3RD;           // Generate INT on 3rd event

    //
    // Information this example uses to keep track of the direction the
    // CMPA/CMPB values are moving, the min and max allowed values and
    // a pointer to the correct ePWM registers
    //

    //
    // Start by increasing CMPA & decreasing CMPB
    //
    epwm4_info.EPwm_CMPA_Direction = EPWM_CMP_UP;
    epwm4_info.EPwm_CMPB_Direction = EPWM_CMP_DOWN;

    epwm4_info.EPwmTimerIntCount = 0;     // Zero the interrupt counter
    epwm4_info.EPwmRegHandle = &EPwm4Regs; //Set the pointer to the ePWM module
    epwm4_info.EPwmMaxCMPA = EPWM4_MAX_CMPA;  // Setup min/max CMPA/CMPB values
    epwm4_info.EPwmMinCMPA = EPWM4_MIN_CMPA;
    epwm4_info.EPwmMaxCMPB = EPWM4_MAX_CMPB;
    epwm4_info.EPwmMinCMPB = EPWM4_MIN_CMPB;
}
void
InitEPwm5Example(void)
{
    //
    // Setup TBCLK
    //
    EPwm5Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;// Count up/down
    EPwm5Regs.TBPRD = EPWM5_TIMER_TBPRD;          // Set timer period
    EPwm5Regs.TBCTL.bit.PHSEN = TB_DISABLE;       // Disable phase loading
    EPwm5Regs.TBPHS.half.TBPHS = 0x0000;          // Phase is 0
    EPwm5Regs.TBCTR = 0x0000;                     // Clear counter
    EPwm5Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;      // Clock ratio to SYSCLKOUT
    EPwm5Regs.TBCTL.bit.CLKDIV = TB_DIV1;

    //
    // Setup shadow register load on ZERO
    //
    EPwm5Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm5Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm5Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    EPwm5Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

    //
    // Set Compare values
    //
    EPwm5Regs.CMPA.half.CMPA = EPWM5_MIN_CMPA;    // Set compare A value
    EPwm5Regs.CMPB = EPWM3_MAX_CMPB;              // Set Compare B value

    //
    // Set Actions
    //
    EPwm5Regs.AQCTLA.bit.CAU = AQ_SET;    // Set PWM2A on event A, up count
    EPwm5Regs.AQCTLA.bit.CAD = AQ_CLEAR;  // Clear PWM2A on event B, down count

    EPwm5Regs.AQCTLB.bit.CBU = AQ_SET;        // Clear PWM2B on zero
    EPwm5Regs.AQCTLB.bit.CBD = AQ_CLEAR;       // Set PWM2B on period

    //
    // Interrupt where we will change the Compare Values
    //
    EPwm5Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;     // Select INT on Zero event
    EPwm5Regs.ETSEL.bit.INTEN = 1;                // Enable INT
    EPwm5Regs.ETPS.bit.INTPRD = ET_3RD;           // Generate INT on 3rd event

    //
    // Information this example uses to keep track of the direction the
    // CMPA/CMPB values are moving, the min and max allowed values and
    // a pointer to the correct ePWM registers
    //

    //
    // Start by increasing CMPA & decreasing CMPB
    //
    epwm5_info.EPwm_CMPA_Direction = EPWM_CMP_UP;
    epwm5_info.EPwm_CMPB_Direction = EPWM_CMP_DOWN;

    epwm5_info.EPwmTimerIntCount = 0;     // Zero the interrupt counter
    epwm5_info.EPwmRegHandle = &EPwm5Regs; //Set the pointer to the ePWM module
    epwm5_info.EPwmMaxCMPA = EPWM5_MAX_CMPA;  // Setup min/max CMPA/CMPB values
    epwm5_info.EPwmMinCMPA = EPWM5_MIN_CMPA;
    epwm5_info.EPwmMaxCMPB = EPWM5_MAX_CMPB;
    epwm5_info.EPwmMinCMPB = EPWM5_MIN_CMPB;
}


//
// InitEPwm - This function initializes the EPwm(s) to a known state.
//
void 
InitEPwm(void)
{
    //
    // Initialize EPwm1/2/3/4/5/6/7
    //
}

//
// InitEPwmGpio - This function initializes GPIO pins to function as EPwm pins
//
// Each GPIO pin can be configured as a GPIO pin or up to 3 different
// peripheral functional pins. By default all pins come up as GPIO
// inputs after reset.
//
void 
InitEPwmGpio(void)
{
    InitEPwm1Gpio();    
    InitEPwm2Gpio();
    InitEPwm3Gpio();
#if DSP28_EPWM4
    InitEPwm4Gpio();
#endif
#if DSP28_EPWM5
    InitEPwm5Gpio();
#endif
#if DSP28_EPWM6
    InitEPwm6Gpio();
#endif
#if DSP28_EPWM7
    InitEPwm7Gpio();
#endif
}

//
// InitEPwm1Gpio - This function initializes GPIO pins to function as EPwm1
//
void 
InitEPwm1Gpio(void)
{
    EALLOW;

    //
    // Disable internal pull-up for the selected output pins
    // for reduced power consumption
    // Pull-ups can be enabled or disabled by the user.
    // Comment out other unwanted lines.
    //
    GpioCtrlRegs.GPAPUD.bit.GPIO0 = 1;    // Disable pull-up on GPIO0 (EPWM1A)
    GpioCtrlRegs.GPAPUD.bit.GPIO1 = 1;    // Disable pull-up on GPIO1 (EPWM1B)

    //
    // Configure EPWM-1 pins using GPIO regs
    // This specifies which of the possible GPIO pins will be
    // EPWM1 functional pins.
    // Comment out other unwanted lines.
    //
    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;   // Configure GPIO0 as EPWM1A
    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1;   // Configure GPIO1 as EPWM1B

    EDIS;
}

//
// InitEPwm2Gpio - This function initializes GPIO pins to function as EPwm2
//
void 
InitEPwm2Gpio(void)
{
    EALLOW;

    //
    // Disable internal pull-up for the selected output pins
    // for reduced power consumption
    // Pull-ups can be enabled or disabled by the user.
    // This will enable the pullups for the specified pins.
    // Comment out other unwanted lines.
    //
    GpioCtrlRegs.GPAPUD.bit.GPIO2 = 1;    // Disable pull-up on GPIO2 (EPWM2A)
    GpioCtrlRegs.GPAPUD.bit.GPIO3 = 1;    // Disable pull-up on GPIO3 (EPWM2B)

    //
    // Configure EPwm-2 pins using GPIO regs
    // This specifies which of the possible GPIO pins will be
    // EPWM2 functional pins.
    // Comment out other unwanted lines.
    //
    GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1;   // Configure GPIO2 as EPWM2A
    GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1;   // Configure GPIO3 as EPWM2B

    EDIS;
}

//
// InitEPwm3Gpio - This function initializes GPIO pins to function as EPwm3
//
void 
InitEPwm3Gpio(void)
{
    EALLOW;

    // Disable internal pull-up for the selected output pins
    // for reduced power consumption 
    // Pull-ups can be enabled or disabled by the user.
    // This will enable the pullups for the specified pins.
    // Comment out other unwanted lines.
    //
    GpioCtrlRegs.GPAPUD.bit.GPIO4 = 1;    // Disable pull-up on GPIO4 (EPWM3A)
    GpioCtrlRegs.GPAPUD.bit.GPIO5 = 1;    // Disable pull-up on GPIO5 (EPWM3B)

    //
    // Configure EPwm-3 pins using GPIO regs
    // This specifies which of the possible GPIO pins will be
    // EPWM3 functional pins.
    // Comment out other unwanted lines.
    //
    GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 1;   // Configure GPIO4 as EPWM3A
    GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 1;   // Configure GPIO5 as EPWM3B

    EDIS;
}

#if DSP28_EPWM4
//
// InitEPwm4Gpio - This function initializes GPIO pins to function as EPwm4
//
void 
InitEPwm4Gpio(void)
{
    EALLOW;
    
    //
    // Disable internal pull-up for the selected output pins
    // for reduced power consumption 
    // Pull-ups can be enabled or disabled by the user.
    // This will enable the pullups for the specified pins.
    // Comment out other unwanted lines.
    //
    GpioCtrlRegs.GPAPUD.bit.GPIO6 = 1;    // Disable pull-up on GPIO6 (EPWM4A)
    GpioCtrlRegs.GPAPUD.bit.GPIO7 = 1;    // Disable pull-up on GPIO7 (EPWM4B)

    //
    // Configure EPWM-4 pins using GPIO regs
    // This specifies which of the possible GPIO pins will be
    // EPWM4 functional pins.
    // Comment out other unwanted lines.
    //
    GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 1;   // Configure GPIO6 as EPWM4A
    GpioCtrlRegs.GPAMUX1.bit.GPIO7 = 1;   // Configure GPIO7 as EPWM4B

    EDIS;
}
#endif

#if DSP28_EPWM5
//
// InitEPwm5Gpio - This function initializes GPIO pins to function as EPwm5
//
void 
InitEPwm5Gpio(void)
{
    EALLOW;
    
    //
    // Disable internal pull-up for the selected output pins
    // for reduced power consumption
    // Pull-ups can be enabled or disabled by the user.
    // This will enable the pullups for the specified pins.
    // Comment out other unwanted lines.
    //
    GpioCtrlRegs.GPAPUD.bit.GPIO8 = 1;    // Disable pull-up on GPIO8 (EPWM5A)
    GpioCtrlRegs.GPAPUD.bit.GPIO9 = 1;    // Disable pull-up on GPIO9 (EPWM5B)

    //
    // Configure EPWM-5 pins using GPIO regs
    // This specifies which of the possible GPIO pins will be 
    // EPWM5 functional pins.
    // Comment out other unwanted lines.
    //
    GpioCtrlRegs.GPAMUX1.bit.GPIO8 = 1;   // Configure GPIO8 as EPWM5A
    GpioCtrlRegs.GPAMUX1.bit.GPIO9 = 1;   // Configure GPIO9 as EPWM5B

    EDIS;
}
#endif

#if DSP28_EPWM6
//
// InitEPwm6Gpio - This function initializes GPIO pins to function as EPwm6
//
void 
InitEPwm6Gpio(void)
{
    EALLOW;
    
    //
    // Disable internal pull-up for the selected output pins
    // for reduced power consumption
    // Pull-ups can be enabled or disabled by the user.
    // This will enable the pullups for the specified pins.
    // Comment out other unwanted lines.
    //
    GpioCtrlRegs.GPAPUD.bit.GPIO10 = 1;   // Disable pull-up on GPIO10 (EPWM6A)
    GpioCtrlRegs.GPAPUD.bit.GPIO11 = 1;   // Disable pull-up on GPIO11 (EPWM6B)

    //
    // Configure EPWM-6 pins using GPIO regs
    // This specifies which of the possible GPIO pins will be 
    // EPWM6 functional pins.
    // Comment out other unwanted lines.
    //
    GpioCtrlRegs.GPAMUX1.bit.GPIO10 = 1;   // Configure GPIO10 as EPWM6A
    GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 1;   // Configure GPIO11 as EPWM6B

    EDIS;
}
#endif

#if DSP28_EPWM7
//
// InitEPwm7Gpio - This function initializes GPIO pins to function as EPwm7
//
void 
InitEPwm7Gpio(void)
{
    EALLOW;
    
    //
    // Disable internal pull-up for the selected output pins
    // for reduced power consumption
    // Pull-ups can be enabled or disabled by the user.
    // This will enable the pullups for the specified pins.
    // Comment out other unwanted lines.
    //
    GpioCtrlRegs.GPBPUD.bit.GPIO40 = 1;  // Disable pull-up on GPIO40 (EPWM7A)
    GpioCtrlRegs.GPBPUD.bit.GPIO41 = 1;  // Disable pull-up on GPIO41 (EPWM7B)

    //
    // Configure EPWM-7 pins using GPIO regs
    // This specifies which of the possible GPIO pins will be
    // EPWM7 functional pins.
    // Comment out other unwanted lines.
    //
    GpioCtrlRegs.GPBMUX1.bit.GPIO40 = 1;   // Configure GPIO40 as EPWM7A
    GpioCtrlRegs.GPBMUX1.bit.GPIO41 = 1;   // Configure GPIO41 as EPWM7B

    EDIS;
}
#endif

//
// InitEPwmSyncGpio - This function initializes GPIO pins to function as 
// EPwm Synch pins
//
void 
InitEPwmSyncGpio(void)
{
    //
    // EALLOW;
    //

    //
    // Configure EPWMSYNCI
    //

    //
    // Enable internal pull-up for the selected pins
    // Pull-ups can be enabled or disabled by the user.
    // This will enable the pullups for the specified pins.
    // Comment out other unwanted lines.
    //
    //GpioCtrlRegs.GPAPUD.bit.GPIO6 = 0; //Enable pull-up on GPIO6 (EPWMSYNCI)
    GpioCtrlRegs.GPBPUD.bit.GPIO32 = 0;  //Enable pull-up on GPIO32 (EPWMSYNCI)

    //
    // Set qualification for selected pins to asynch only
    // This will select synch to SYSCLKOUT for the selected pins.
    // Comment out other unwanted lines.
    //
    //GpioCtrlRegs.GPAQSEL1.bit.GPIO6 = 0; //Synch SYSCLKOUT GPIO6(EPWMSYNCI)
    GpioCtrlRegs.GPBQSEL1.bit.GPIO32 = 0;  //Synch SYSCLKOUT GPIO32(EPWMSYNCI)

    //
    // Configure EPwmSync pins using GPIO regs
    // This specifies which of the possible GPIO pins will be 
    // EPwmSync functional pins.
    // Comment out other unwanted lines.
    //
    //GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 2;  //Configures GPIO6 for EPWMSYNCI
    GpioCtrlRegs.GPBMUX1.bit.GPIO32 = 2;   //Configures GPIO32 for EPWMSYNCI

    //
    // Configure EPWMSYNC0
    //

    //
    // Disable internal pull-up for the selected output pins
    // for reduced power consumption
    // Pull-ups can be enabled or disabled by the user.
    // Comment out other unwanted lines.
    //
    //GpioCtrlRegs.GPAPUD.bit.GPIO6 = 1; //Disable pull-up on GPIO6(EPWMSYNCO)
    GpioCtrlRegs.GPBPUD.bit.GPIO33 = 1;  //Disable pull-up on GPIO33(EPWMSYNCO)

    //GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 3; //Configures GPIO6 for EPWMSYNCO
    GpioCtrlRegs.GPBMUX1.bit.GPIO33 = 2;  //Configures GPIO33 for EPWMSYNCO
}

//
// InitTzGpio - This function initializes GPIO pins to function as 
// Trip Zone (TZ) pins
//
// Each GPIO pin can be configured as a GPIO pin or up to 3 different
// peripheral functional pins. By default all pins come up as GPIO
// inputs after reset.
//
void 
InitTzGpio(void)
{
    EALLOW;

    //
    // Enable internal pull-up for the selected pins
    // Pull-ups can be enabled or disabled by the user.
    // This will enable the pullups for the specified pins.
    // Comment out other unwanted lines.
    //
    GpioCtrlRegs.GPAPUD.bit.GPIO12 = 0;    // Enable pull-up on GPIO12 (TZ1)
    //GpioCtrlRegs.GPAPUD.bit.GPIO15 = 0;    // Enable pull-up on GPIO15 (TZ1)
    GpioCtrlRegs.GPAPUD.bit.GPIO13 = 0;    // Enable pull-up on GPIO13 (TZ2)
    //GpioCtrlRegs.GPAPUD.bit.GPIO16 = 0;    // Enable pull-up on GPIO16 (TZ2)
    //GpioCtrlRegs.GPAPUD.bit.GPIO28 = 0;    // Enable pull-up on GPIO28 (TZ2)
    GpioCtrlRegs.GPAPUD.bit.GPIO14 = 0;    // Enable pull-up on GPIO14 (TZ3)
    //GpioCtrlRegs.GPAPUD.bit.GPIO17 = 0;    // Enable pull-up on GPIO17 (TZ3)
    //GpioCtrlRegs.GPAPUD.bit.GPIO29 = 0;    // Enable pull-up on GPIO29 (TZ3)

    //
    // Set qualification for selected pins to asynch only
    // Inputs are synchronized to SYSCLKOUT by default.
    // This will select asynch (no qualification) for the selected pins.
    // Comment out other unwanted lines.
    //
    GpioCtrlRegs.GPAQSEL1.bit.GPIO12 = 3;  // Asynch input GPIO12 (TZ1)
    //GpioCtrlRegs.GPAQSEL1.bit.GPIO15 = 3;  // Asynch input GPIO15 (TZ1)
    GpioCtrlRegs.GPAQSEL1.bit.GPIO13 = 3;  // Asynch input GPIO13 (TZ2)
    //GpioCtrlRegs.GPAQSEL2.bit.GPIO16 = 3;  // Asynch input GPIO16 (TZ2)
    //GpioCtrlRegs.GPAQSEL2.bit.GPIO28 = 3;  // Asynch input GPIO28 (TZ2)
    GpioCtrlRegs.GPAQSEL1.bit.GPIO14 = 3;  // Asynch input GPIO14 (TZ3)
    //GpioCtrlRegs.GPAQSEL2.bit.GPIO17 = 3;  // Asynch input GPIO17 (TZ3)
    //GpioCtrlRegs.GPAQSEL2.bit.GPIO29 = 3;  // Asynch input GPIO29 (TZ3)

    //
    // Configure TZ pins using GPIO regs
    // This specifies which of the possible GPIO pins will be TZ 
    // functional pins.
    // Comment out other unwanted lines.
    //
    GpioCtrlRegs.GPAMUX1.bit.GPIO12 = 1;  // Configure GPIO12 as TZ1
    //GpioCtrlRegs.GPAMUX1.bit.GPIO15 = 1;  // Configure GPIO15 as TZ1
    GpioCtrlRegs.GPAMUX1.bit.GPIO13 = 1;  // Configure GPIO13 as TZ2
    //GpioCtrlRegs.GPAMUX2.bit.GPIO16 = 3;  // Configure GPIO16 as TZ2
    //GpioCtrlRegs.GPAMUX2.bit.GPIO28 = 3;  // Configure GPIO28 as TZ2
    GpioCtrlRegs.GPAMUX1.bit.GPIO14 = 1;  // Configure GPIO14 as TZ3
    //GpioCtrlRegs.GPAMUX2.bit.GPIO17 = 3;  // Configure GPIO17 as TZ3
    //GpioCtrlRegs.GPAMUX2.bit.GPIO29 = 3;  // Configure GPIO29 as TZ3

    EDIS;
}

//
// End of file
//

