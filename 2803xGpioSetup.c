//###########################################################################
//
// FILE:    Example_2803xGpioSetup.c
//
// TITLE:   GPIO Setup example
//
//! \addtogroup f2803x_example_list
//! <h1>GPIO Setup (gpio_setup)</h1>
//!
//! This example Configures the 2803x GPIO into two different configurations
//! This code is verbose to illustrate how the GPIO could be setup.In a real
//! application, lines of code can be combined for improved code size and
//! efficiency.
//!
//! This example only sets-up the GPIO.. nothing is actually done with
//! the pins after setup.
//!
//! In general:
//!  - All pullup resistors are enabled.  For ePWMs this may not be desired.
//!  - Input qual for communication ports (eCAN, SPI, SCI, I2C) is asynchronous
//!  - Input qual for Trip pins (TZ) is asynchronous
//!  - Input qual for eCAP and eQEP signals is synch to SYSCLKOUT
//!  - Input qual for some I/O's and interrupts may have a sampling window
//
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
#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File

//
// Prototype statements
//
//
// Gpio_setup
//
void
Gpio_setup(void)
{
    //
    // These can be combined into single statements for improved
    // code efficiency.
    //

    EALLOW;
    //GpioCtrlRegs.GPAPUD.bit.GPIO0 = 0;   // Enable pullup on GPIO0 example
                                           // can also add pullup to non GPIO pins

    //
    // Enable an GPIO output on GPIO6, set it high
    //
    GpioCtrlRegs.GPAPUD.bit.GPIO24 = 0;   // Enable pullup on GPIO6
    GpioDataRegs.GPACLEAR.bit.GPIO24 = 1;   // Clear output latch
    GpioCtrlRegs.GPADIR.bit.GPIO24 = 1;   // GPIO6 = output

    //
    // Enable GPIO outputs on GPIO8 - GPIO11, set it high
    //
    GpioCtrlRegs.GPBPUD.bit.GPIO43 = 0;   // Enable pullup on GPIO8
    GpioDataRegs.GPBSET.bit.GPIO43 = 1;   // Load output latch
    GpioCtrlRegs.GPBDIR.bit.GPIO43 = 1;   // GPIO8 = output

    GpioCtrlRegs.GPAPUD.bit.GPIO27 = 0;   // Enable pullup on GPIO9
    GpioDataRegs.GPASET.bit.GPIO27 = 1;   // Load output latch
    GpioCtrlRegs.GPADIR.bit.GPIO27 = 1;   // GPIO9 = output

    GpioCtrlRegs.GPBPUD.bit.GPIO34 = 0;  // Enable pullup on GPIO10
    GpioDataRegs.GPBSET.bit.GPIO34 = 1;  // Load output latch
    GpioCtrlRegs.GPBDIR.bit.GPIO34 = 1;  // GPIO10 = output


    //
    // Enable SPI-A on GPIO16 - GPIO19
    //
    /*
    GpioCtrlRegs.GPAPUD.bit.GPIO16 = 0;   // Enable pullup on GPIO16
    GpioCtrlRegs.GPAPUD.bit.GPIO17 = 0;   // Enable pullup on GPIO17
    GpioCtrlRegs.GPAPUD.bit.GPIO18 = 0;   // Enable pullup on GPIO18
    GpioCtrlRegs.GPAPUD.bit.GPIO19 = 0;   // Enable pullup on GPIO19
    GpioCtrlRegs.GPAQSEL2.bit.GPIO16 = 3; // asynch input
    GpioCtrlRegs.GPAQSEL2.bit.GPIO17 = 3; // asynch input
    GpioCtrlRegs.GPAQSEL2.bit.GPIO18 = 3; // asynch input
    GpioCtrlRegs.GPAQSEL2.bit.GPIO19 = 3; // asynch input
    GpioCtrlRegs.GPAMUX2.bit.GPIO16 = 1;  // GPIO16 = SPISIMOA
    GpioCtrlRegs.GPAMUX2.bit.GPIO17 = 1;  // GPIO17 = SPIS0MIA
    GpioCtrlRegs.GPAMUX2.bit.GPIO18 = 1;  // GPIO18 = SPICLKA
    GpioCtrlRegs.GPAMUX2.bit.GPIO19 = 1;  // GPIO19 = SPISTEA
    */
    EDIS;
}


//
// End of File
//

