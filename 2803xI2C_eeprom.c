//###########################################################################
//
//  FILE:  Example_2803xI2C_eeprom.c
//
//  TITLE: I2C EEPROM example
//
//!  \addtogroup f2803x_example_list
//!  <h1>I2C EEPROM(i2c_eeprom)</h1>
//!
//!  This program requires an external I2C EEPROM connected to
//!  the I2C bus at address 0x50.
//!  This program will write 1-14 words to EEPROM and read them back.
//!  The data written and the EEPROM address written to are contained
//!  in the message structure, \b I2cMsgOut1. The data read back will be
//!  contained in the message structure \b I2cMsgIn1.
//!
//!  \note This program will only work on kits that have an on-board I2C EEPROM.
//!  (e.g. F2803x eZdsp)
//!
//!  \b Watch \b Variables \n
//!  - I2cMsgIn1
//!  - I2cMsgOut1
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
#include "PMDC_drive.h"
#include <stdio.h>
#include <string.h>

//
// Note: I2C Macros used in this example can be found in the
// DSP2803x_I2C_defines.h file
//

//
// Prototype statements
//
void   I2CA_Init(void);
Uint16 I2CA_WriteData(struct I2CMSG *msg);
Uint16 I2CA_ReadData(struct I2CMSG *msg);
void eewrite(Uint16 eeaddress, Uint16 eedata);
Uint16 eeread(Uint16 eeaddress);
__interrupt void i2c_int1a_isr(void);
void pass(void);
void fail(void);
extern void scia_msg(char *msg);

//
// Defined in main Globals
//
extern struct I2CMSG I2cMsgOut1;
extern struct I2CMSG I2cMsgIn1;
extern struct I2CMSG *CurrentMsgPtr;
extern Uint16 PassCount;
extern Uint16 FailCount;

Uint16 Error;
//
// Two bytes will be used for the outgoing address,
// thus only setup 14 bytes maximum
//


//
// I2CA_Init - Initialize I2C
//
void
I2CA_Init(void)
{
    I2caRegs.I2CSAR = 0x0050;       // Slave address - EEPROM control code

    I2caRegs.I2CPSC.all = 6;        // Prescaler - need 7-12 Mhz on module clk
    I2caRegs.I2CCLKL = 10;          // NOTE: must be non zero
    I2caRegs.I2CCLKH = 5;           // NOTE: must be non zero
    I2caRegs.I2CIER.all = 0x24;     // Enable SCD & ARDY interrupts

    I2caRegs.I2CMDR.all = 0x0020;   // Take I2C out of reset
                                    // Stop I2C when suspended

    I2caRegs.I2CFFTX.all = 0x6000;  // Enable FIFO mode and TXFIFO
    I2caRegs.I2CFFRX.all = 0x2040;  // Enable RXFIFO, clear RXFFINT,

    return;
}

//
// I2CA_WriteData -
//
Uint16
I2CA_WriteData(struct I2CMSG *msg)
{
    Uint16 i;

    //
    // Wait until STP bit is cleared from any previous master communication.
    // Clearing of this bit by the module is delayed until after the SCD bit is
    // set. If this bit is not checked prior to initiating a new message, the
    // I2C could get confused.
    //
    if (I2caRegs.I2CMDR.bit.STP == 1)
    {
        return I2C_STP_NOT_READY_ERROR;
    }
//    while (I2caRegs.I2CSTR.bit.SCD == 0 ); // wait for stop condition detected

    // add polling sequence to ensure write is complete before other operations can proceed.
//    do
//    {
//       I2caRegs.I2CMDR.all = 0x6C20;   // send start with r/w = 0 and wait for ack
       //while (I2caRegs.I2CSTR.bit.ARDY == 0 ); // wait for transmit S/R to become empty
//    } while (I2caRegs.I2CSTR.bit.NACK == 1);
    //
    // Setup slave address
    //
    I2caRegs.I2CSAR = msg->SlaveAddress;

    //
    // Check if bus busy
    //
    if (I2caRegs.I2CSTR.bit.BB == 1)
    {
        return I2C_BUS_BUSY_ERROR;
    }

    //
    // Setup number of bytes to send
    // MsgBuffer + Address
    //
    I2caRegs.I2CCNT = msg->NumOfBytes+1; // add one for address

    //
    // Setup data to send
    //
    I2caRegs.I2CDXR = msg->MemoryLowAddr;

    //
    // for (i=0; i<msg->NumOfBytes-2; i++)
    //
    for (i=0; i<msg->NumOfBytes; i++)
    {
        I2caRegs.I2CDXR = *(msg->MsgBuffer+i);
    }

    //
    // Send start as master transmitter
    //
    I2caRegs.I2CMDR.all = 0x6E20;



    return I2C_SUCCESS;
}

//
// I2CA_ReadData - Reads I2CA data
//
Uint16
I2CA_ReadData(struct I2CMSG *msg)
{
    int i;
    //
    // Wait until STP bit is cleared from any previous master communication.
    // Clearing of this bit by the module is delayed until after the SCD bit is
    // set. If this bit is not checked prior to initiating a new message, the
    // I2C could get confused.
    //
    if (I2caRegs.I2CMDR.bit.STP == 1)
    {
      return I2C_STP_NOT_READY_ERROR;
    }

    I2caRegs.I2CSAR = msg->SlaveAddress;
    //
    // Check if bus busy
    //
    if (I2caRegs.I2CSTR.bit.BB == 1)
    {
    return I2C_BUS_BUSY_ERROR;
    }

    I2caRegs.I2CCNT = 1;
    I2caRegs.I2CDXR = msg->MemoryLowAddr;
    I2caRegs.I2CMDR.all = 0x6620;   // Send data to setup EEPROM address
    while (I2caRegs.I2CSTR.bit.ARDY == 0 ); // wait for transmit S/R to become empty
    I2caRegs.I2CSTR.bit.NACK = 1;   // set NACK bit and

    I2caRegs.I2CCNT = msg->NumOfBytes;  // Setup how many bytes to expect
    I2caRegs.I2CMDR.all = 0x6C20;       // Send restart as master receiver
    while (I2caRegs.I2CSTR.bit.SCD == 0 ); // wait for stop condition detected
    for(i=0; i < I2C_NUMBYTES; i++)
        {
            CurrentMsgPtr->MsgBuffer[i] = I2caRegs.I2CDRR;
        }
        if(I2caRegs.I2CSTR.bit.NACK == 1)  // after all of the data is read then
        {
            I2caRegs.I2CMDR.bit.STP = 1;
            I2caRegs.I2CSTR.all = I2C_CLR_NACK_BIT;

        }



    return I2C_SUCCESS;
}
__interrupt void
i2c_int1a_isr(void)
{
    //
    // Enable future I2C (PIE Group 8) interrupts
    //
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP8;
}
void eewrite(Uint16 eeaddress, Uint16 eedata)
{
    I2cMsgOut1.MemoryLowAddr = eeaddress & 0x00ff;
    I2cMsgOut1.MemoryHighAddr = eeaddress >> 8;
    I2cMsgOut1.MsgBuffer[0] = eedata & 0x00ff;
    I2cMsgOut1.MsgBuffer[1] = eedata >> 8;
    I2cMsgOut1.NumOfBytes = 2;
    Error = I2CA_WriteData(&I2cMsgOut1);

    //
    // If communication is correctly initiated, set msg status to busy
    // and update CurrentMsgPtr for the interrupt service routine.
    // Otherwise, do nothing and try again next loop. Once message is
    // initiated, the I2C interrupts will handle the rest. Search for
    // i2c_int1a_isr in the i2c_eeprom_isr.c file.
    //
    if (Error == I2C_SUCCESS)
    {
        CurrentMsgPtr = &I2cMsgOut1;
    }
    if (Error != I2C_SUCCESS) scia_msg("\r\neeWrite Failed");

}
Uint16 eeread(Uint16 eeaddress)
{
    I2cMsgIn1.MemoryLowAddr = eeaddress & 0x00ff;
    I2cMsgIn1.MemoryHighAddr = eeaddress >> 8;
    CurrentMsgPtr = &I2cMsgIn1;
    //
    // Read data from EEPROM section
    //

    I2CA_ReadData(&I2cMsgIn1);
            // Update current message pointer and message status

    return (CurrentMsgPtr->MsgBuffer[0] + (CurrentMsgPtr->MsgBuffer[1] << 8));
}
void
pass()
{
    __asm("   ESTOP0");
    for(;;);
}


void
fail()
{
    __asm("   ESTOP0");
    for(;;);
}
// end of file
