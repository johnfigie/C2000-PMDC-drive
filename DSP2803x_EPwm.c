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
// Modifications by John Figie 2021/10/10
//
// Included Files
//
#include "DSP2803x_Device.h"     // DSP2803x Headerfile Include File
#include "DSP2803x_Examples.h"   // DSP2803x Examples Include File
#include "PMDC_drive.h"

// pwm defines
//
// Defines for the period for each timer
//
#define EPWM1_TIMER_TBPRD  EPWM_TIMER_PRD  // Period register
#define EPWM2_TIMER_TBPRD  EPWM_TIMER_PRD  // Period register
#define EPWM3_TIMER_TBPRD  EPWM_TIMER_PRD  // Period register
#define EPWM4_TIMER_TBPRD  EPWM_TIMER_PRD  // Period register
#define EPWM5_TIMER_TBPRD  EPWM_TIMER_PRD  // Period register
#define EPWM7_CMPA          7425     // SOC start for PWM 1-3
#define EPWM7_CMPB          1825     // Optional SOC start for PWM 4-6


//
// Maximum Dead Band Defines
//

#define EPWM1_MIN_DB   20
#define EPWM2_MIN_DB   20
#define EPWM3_MIN_DB   20

#define EPWM4_MIN_DB   20
#define EPWM5_MIN_DB   20
#define EPWM6_MIN_DB   20


void
InitEPwm1()
{
    //
    // Setup TBCLK
    //
    EPwm1Regs.TBPRD = EPWM1_TIMER_TBPRD;   // Set timer period
    EPwm1Regs.TBPHS.half.TBPHS = 0x0000;   // Phase is 0
    EPwm1Regs.TBCTR = 0x0000;              // Clear counter
    EPwm1Regs.TBCTL.bit.SYNCOSEL = 01;     // select synch output on counter = 0 This will be
                                           //   used to sync EPwm2.

    //
    // Set Compare values
    //
    EPwm1Regs.CMPA.half.CMPA = (EPWM1_TIMER_TBPRD/2);     // Set compare A value
    EPwm1Regs.CMPB = 0x8000;               // Set Compare B value

    //
    // Setup counter mode
    //
    EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up
    EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;        // Disable phase loading
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;       // Clock ratio to SYSCLKOUT
    EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV1;

    //
    // Setup shadowing
    //
    EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;        // B not used
    EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO_PRD;  // Load on Zero or period
    EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO_PRD;      // B not used

    //
    // Set actions
    //
    EPwm1Regs.AQCTLA.bit.CAU = AQ_SET;    // Set PWM1A on event A, up count
    EPwm1Regs.AQCTLA.bit.CAD = AQ_CLEAR;  // Clear PWM1A on event A, down count

    EPwm1Regs.AQCTLB.bit.CBU = AQ_SET;   // Set PWM1B on event B, up count -B not used
    EPwm1Regs.AQCTLB.bit.CBD = AQ_CLEAR; // Clear PWM1B on event B, down count B not used

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
    EPwm1Regs.ETSEL.bit.SOCASEL = ET_CTR_PRDZERO;   // Start Conversion peak and valley
    EPwm1Regs.ETSEL.bit.SOCAEN = 1;               // enable start of conversion

    EPwm1Regs.ETPS.bit.INTPRD = ET_1ST;            // Generate INT on 1st event
    EPwm1Regs.ETPS.bit.SOCAPRD = 0b01;             // Sets the SOC to occur on first event
    //
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
InitEPwm2()
{
    //
    // Setup TBCLK
    //
    EPwm2Regs.TBPRD = EPWM2_TIMER_TBPRD;        // Set timer period
    EPwm2Regs.TBPHS.half.TBPHS = 0;        // Phase is 0
    EPwm2Regs.TBCTR = 0;                     // Default value - not important because
                                             //    pwm2 is synced to EPwm1
    EPwm2Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN ;  // select synch flow through
                                           //   used to sync EPwm3.
    EPwm2Regs.TBCTL.bit.PHSEN = TB_ENABLE;    // Slave module
    // Set Compare values
    //
    EPwm2Regs.CMPA.half.CMPA = (EPWM1_TIMER_TBPRD/2);     // Set compare A value

    //
    // Setup counter mode
    //
    EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up
    EPwm2Regs.TBCTL.bit.PHSEN = TB_ENABLE;        // Load the time base counter with Syncin
    EPwm2Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;       // Clock ratio to SYSCLKOUT
    EPwm2Regs.TBCTL.bit.CLKDIV = TB_DIV1;
    EPwm2Regs.TBCTL.bit.SWFSYNC = 0;

    //
    // Setup shadowing
    //
    EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm2Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm2Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO_PRD;  // Load on Zero or period
    EPwm2Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO_PRD;

    //
    // Set actions
    //
    EPwm2Regs.AQCTLA.bit.CAU = AQ_SET;    // Set PWM2A on event A, up count
    EPwm2Regs.AQCTLA.bit.CAD = AQ_CLEAR;  // Clear PWM2A on event B, down count

    EPwm2Regs.AQCTLB.bit.CBU = AQ_SET;        // Clear PWM2B on zero
    EPwm2Regs.AQCTLB.bit.CBD = AQ_CLEAR;       // Set PWM2B on period

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
    EPwm2Regs.ETSEL.bit.INTEN = 0;                // Disable INT
    EPwm2Regs.ETPS.bit.INTPRD = ET_1ST;           // Generate INT on 3rd event


    // Setup ADC SOC trigger
    EPwm2Regs.ETSEL.bit.SOCASEL = ET_CTR_ZERO;  // use this for SOC 8 to start a new group of
                                                // conversions.
}
//
// InitEPwm3Example - EPwm4 example
//
void
InitEPwm4(void)
{
    //
    // Setup TBCLK
    //
    EPwm4Regs.TBPRD = EPWM4_TIMER_TBPRD;          // Set timer period
    EPwm4Regs.TBCTL.bit.PHSEN = TB_ENABLE;       // Disable phase loading
    EPwm4Regs.TBPHS.half.TBPHS = 0x0000;          // Phase is 0
    //EPwm4Regs.TBCTR = 0x0000;                     // Clear counter
    //
    // Set Compare values
    //
    EPwm4Regs.CMPA.half.CMPA = (EPWM4_TIMER_TBPRD/2);    // Set compare A value
    //
    // Setup counter mode
    //
    EPwm4Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up
    EPwm4Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;       // Clock ratio to SYSCLKOUT
    EPwm4Regs.TBCTL.bit.CLKDIV = TB_DIV1;
    EPwm4Regs.TBCTL.bit.SWFSYNC = 0;

    //
    // Setup shadow register load on ZERO
    //
    EPwm4Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm4Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm4Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO_PRD; // load on zero or period
    EPwm4Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO_PRD;

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

    //
}
void
InitEPwm5(void)
{
    //
    // Setup TBCLK
    //
    EPwm5Regs.TBPRD = EPWM5_TIMER_TBPRD;          // Set timer period
    EPwm5Regs.TBCTL.bit.PHSEN = TB_ENABLE;       // Disable phase loading
    EPwm5Regs.TBPHS.half.TBPHS = 0x0000;          // Phase is 0
    //EPwm5Regs.TBCTR = 0x0000;                     // Clear counter
    //
    // Set Compare values
    //
    EPwm5Regs.CMPA.half.CMPA = (EPWM1_TIMER_TBPRD/2);     // Set compare A value
    //
    // Setup counter mode
    //
    EPwm5Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up
    EPwm5Regs.TBCTL.bit.PHSEN = TB_ENABLE;        // Load the time base counter with Syncin
    EPwm5Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;       // Clock ratio to SYSCLKOUT
    EPwm5Regs.TBCTL.bit.CLKDIV = TB_DIV1;
    EPwm5Regs.TBCTL.bit.SWFSYNC = 0;

    //
    // Setup shadow register load on ZERO
    //
    EPwm5Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm5Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm5Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO_PRD;
    EPwm5Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO_PRD;

    //
    // Set Actions
    //
    EPwm5Regs.AQCTLA.bit.CAU = AQ_SET;    // Set PWM2A on event A, up count
    EPwm5Regs.AQCTLA.bit.CAD = AQ_CLEAR;  // Clear PWM2A on event B, down count

    EPwm5Regs.AQCTLB.bit.CBU = AQ_SET;        // Clear PWM2B on zero
    EPwm5Regs.AQCTLB.bit.CBD = AQ_CLEAR;       // Set PWM2B on period
    //
    // Active Low PWMs - Setup Deadband
    //
    EPwm5Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm5Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
    EPwm5Regs.DBCTL.bit.IN_MODE = DBA_ALL;
    EPwm5Regs.DBRED = EPWM2_MIN_DB;
    EPwm5Regs.DBFED = EPWM2_MIN_DB;

    //
    // Interrupt where we will change the Compare Values
    //
    EPwm5Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;     // Select INT on Zero event
    EPwm5Regs.ETSEL.bit.INTEN = 0;                // Enable INT
    EPwm5Regs.ETPS.bit.INTPRD = ET_1ST;           // Generate INT on 3rd event

}

void InitEPwm7(void)
{
    //
    // Setup TBCLK
    //
    EPwm7Regs.TBPRD = EPWM2_TIMER_TBPRD;        // Set timer period TBCLKs
    EPwm7Regs.TBPHS.half.TBPHS = 0;        // Phase is 0
    //EPwm7Regs.TBCTR = 0;                     // Default value - not important because
                                             //    pwm2 is synced to EPwm1
    //
    // Set Compare values
    //
    EPwm7Regs.CMPA.half.CMPA = EPWM7_CMPA;     // Set compare A value
    EPwm7Regs.CMPB = EPWM7_CMPB;                    // Set Compare B value

    //
    // Setup counter mode
    //
    EPwm7Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP*2; // Count up * 2 will make samples only at peak 4 Khz update
    EPwm7Regs.TBCTL.bit.PHSEN = TB_ENABLE;        // Load the time base counter with Syncin
    EPwm7Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;       // Clock ratio to SYSCLKOUT
    EPwm7Regs.TBCTL.bit.CLKDIV = TB_DIV1;
    EPwm7Regs.TBCTL.bit.SWFSYNC = 0;

    //
    // Setup shadowing
    //
    EPwm7Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm7Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm7Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;  // Load on Zero
    EPwm7Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

    //
    // Set actions
    //
    EPwm7Regs.AQCTLA.bit.CAU = AQ_SET;    // Set PWM2A on event A, up count
    EPwm7Regs.AQCTLA.bit.CAD = AQ_CLEAR;  // Clear PWM2A on down
    EPwm7Regs.AQCTLA.bit.PRD = AQ_CLEAR;  // Clear on period
    EPwm7Regs.AQCTLA.bit.ZRO = AQ_CLEAR;  // Clear on period

    EPwm7Regs.AQCTLB.bit.CBU = AQ_SET;        // Clear PWM2B on B up count
    EPwm7Regs.AQCTLB.bit.CBD = AQ_CLEAR;       // Clear PWM2A on down
    EPwm7Regs.AQCTLB.bit.PRD = AQ_CLEAR;  // Clear on period
    EPwm7Regs.AQCTLB.bit.ZRO = AQ_CLEAR;  // Clear on period

    //
    // Active Low PWMs - Setup Deadband
    //
    EPwm7Regs.DBCTL.bit.OUT_MODE = DB_DISABLE;
    EPwm7Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
    EPwm7Regs.DBCTL.bit.IN_MODE = DBA_ALL;
    EPwm7Regs.DBRED = EPWM2_MIN_DB;
    EPwm7Regs.DBFED = EPWM2_MIN_DB;
//
    // Interrupt where we will change the Compare Values
    //
    EPwm7Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;     // Select INT on Zero event -valley
                                                  //
    EPwm7Regs.ETSEL.bit.INTEN = 0;                // Disable INT
    EPwm7Regs.ETPS.bit.INTPRD = ET_1ST;           // Generate INT on 1st event


    // Setup ADC SOC trigger
    EPwm7Regs.ETSEL.bit.SOCASEL = ET_CTRU_CMPA;  // use this for SOC 0 to start a new group of
                                                // conversions.
    EPwm7Regs.ETSEL.bit.SOCBSEL = ET_CTRU_CMPA;  // use this for SOC 8 to start a new group of
                                                 // conversions with Axis2 pwm phase shifted (option)

    EPwm7Regs.ETPS.bit.SOCAPRD = 0b01;             // Sets the SOC to occur on first event
    EPwm7Regs.ETSEL.bit.SOCAEN = 1;               // enable start of conversion

}


//
// End of file
//

