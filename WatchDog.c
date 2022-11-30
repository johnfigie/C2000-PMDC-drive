//###########################################################################
//
//    Copyright (C) 2021 John Figie
//
//    This program is free software; you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation; either version 2 of the License, or
//    (at your option) any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERinstTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with this program; if not, write to the Free Software
//    Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301 USA
//


//
// Included Files
//
#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include "PMDC_drive.h"
#include <stdio.h>
#include <string.h>

// external functions

// Watchdog Configuration
void watchdoginit(void)
{
        // Connect the watchdog to the WAKEINT interrupt of the PIE
        // Write to the whole SCSR register to avoid clearing WDOVERRIDE bit
        //
        EALLOW;
        SysCtrlRegs.SCSR = BIT1;
        EDIS;

        //
        // Enable WAKEINT in the PIE: Group 1 interrupt 8
        // Enable INT1 which is connected to WAKEINT:
        //
        PieCtrlRegs.PIECTRL.bit.ENPIE = 1;   // Enable the PIE block
        PieCtrlRegs.PIEIER1.bit.INTx8 = 1;   // Enable PIE Group 1 INT8
        IER |= M_INT1;                       // Enable CPU INT1
        EINT;                                // Enable Global Interrupts

        //
        // Reset the watchdog counter
        //
        ServiceDog();

        //
        // Enable the watchdog
        //
        EALLOW;
        SysCtrlRegs.WDCR = 0x0028;
        EDIS;
}
