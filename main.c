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

extern void GPIO_setPinMuxConfig(void);
extern int ExecuteCommand(char *);
extern void Gpio_setup(void);
extern void i2c_int1a_isr(void);
extern void I2CA_Init(void);
extern Uint16 eeread(Uint16 eeaddress);
extern int16 ioread(Uint16 io_number);
extern void iowrite(Uint16 io_number, Uint16 io_value);
extern char* itoa( char * , int32);

//
// Functions Prototypes
//
void SetupSCI(void);
void error(void);
void scia_xmit(char Char);
void scia_msg(char *msg);
void set_pwm(Uint16 axis, Uint16 pwm_value);
void pulse_pwm(Uint16 axis, Uint16 pwm_value, Uint16 npulses);
void pulse_cref(Uint16 axis, int16 cref_value,Uint16 npulses);
void pwm_on(Uint16 axis);
void pwm_off(Uint16 axis);
void pwm_off_message(Uint16 axis);
Uint16 bus_state(int16 bus_measurement);


extern void watchdoginit(void);
// pwm functions
extern void InitEPwm1(void);
extern void InitEPwm2(void);
extern void InitEPwm4(void);
extern void InitEPwm5(void);
extern void InitEPwm7(void);
void pid(struct PID *axis);
//
// Defines
//
#define I2C_SLAVE_ADDR        0x50
#define I2C_NUMBYTES          2
#define I2C_EEPROM_HIGH_ADDR  0x00
#define I2C_EEPROM_LOW_ADDR   0x30


//
// Globals
//
Uint16 LoopCount;
Uint16 ErrorCount;
int16 ADCch[10];
Uint16 ADCch_offset[10];
Uint16 npulses1,npulses2,npulses1_last,npulses2_last;
Uint16 bus_up_threshold,bus_down_threshold;


struct I2CMSG I2cMsgOut1=
{
    I2C_MSGSTAT_SEND_WITHSTOP,
    I2C_SLAVE_ADDR,
    I2C_NUMBYTES,
    I2C_EEPROM_HIGH_ADDR,
    I2C_EEPROM_LOW_ADDR,
    0x12,     // Msg Byte 1
    0x34      // Msg Byte 2
};

struct I2CMSG I2cMsgIn1=
{
    I2C_MSGSTAT_SEND_NOSTOP,
    I2C_SLAVE_ADDR,
    I2C_NUMBYTES,
    I2C_EEPROM_HIGH_ADDR,
    I2C_EEPROM_LOW_ADDR
};

struct I2CMSG *CurrentMsgPtr;       // Used in interrupts



struct PID axis1, axis2;


//
// Exeternal defines d by the linker
//
extern Uint16 RamfuncsLoadStart;
extern Uint16 RamfuncsLoadSize;
extern Uint16 RamfuncsRunStart;

//
// Main
//
void main(void)
{
    Uint16 ReceivedChar,i,bus_up;
    char ReceivedLine[80], output_buffer[80];
    int j,jl;
    jl = 0;
    int escape_flag = 0;
    npulses1 = 0;  // init global vaiables
    npulses2 = 0;
    npulses1_last = 0;
    npulses2_last = 0;
    bus_up = 0;
    CurrentMsgPtr = &I2cMsgOut1;
    Uint32 fault_temp;
    memcpy((Uint16 *)&RamfuncsRunStart,(Uint16 *)&RamfuncsLoadStart,
            (unsigned long)&RamfuncsLoadSize);

    InitSysCtrl();
    GPIO_setPinMuxConfig();
    DINT;
    InitPieCtrl();
    IER = 0x0000;
    IFR = 0x0000;
    InitPieVectTable();
//    memcpy((Uint16 *)&RamfuncsRunStart,(Uint16 *)&RamfuncsLoadStart,
//            (unsigned long)&RamfuncsLoadSize);
    SetupSCI();
    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;
    PieVectTable.I2CINT1A = &i2c_int1a_isr;
    EDIS;
    I2CA_Init();
    pwm_off( (Uint16) 0);   // start with PWMs Off
    pwm_off_message( (Uint16) 0);
    InitEPwm1();
    InitEPwm2();
    InitEPwm4();
    InitEPwm5();
    InitEPwm7();
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;   // sync all pwms
    InitAdc();  // For this example, init the ADC
    InitAdcAio();

    //
    // Clear incoming message buffer for I2C
    //
    for (i = 0; i < I2C_MAX_BUFFER_SIZE; i++)
    {
        I2cMsgIn1.MsgBuffer[i] = 0x0000;
    }
    InitFlash();
    Gpio_setup();
    // Initialize axis control variable structures
    axis1.cref = eeread(EE_CV_CREF1);              //  0 amps due to 12 bit ADC input.
    axis1.pgain = eeread(EE_CV_P1);
    axis1.igain = eeread(EE_CV_I1);
    axis1.ff = eeread(EE_CV_FF1);
    axis1.iterm = 0;
    axis1.pwm = 3750;
    axis1.loop_mode = eeread(EE_CV_LM1);
    axis1.input_mode = eeread(EE_CV_IM1);
    axis1.cref_limit = eeread(EE_CV_CREFLIMIT1);
    axis1.current_limit = eeread(EE_CV_CLIM1);

    axis2.cref = eeread(EE_CV_CREF2);
    axis2.pgain = eeread(EE_CV_P2);    // address for PI gain2
    axis2.igain = eeread(EE_CV_I2);
    axis2.ff = eeread(EE_CV_FF2);
    axis2.iterm = 0;
    axis2.pwm = 3750;
    axis2.cref_limit = eeread(EE_CV_CREFLIMIT2);
    axis2.current_limit = eeread(EE_CV_CLIM2);
    //  load ADC calibration values
    ADCch_offset[0] = eeread(100);  //Axis 1 Uphase amps
    ADCch_offset[1] = eeread(102);  //Axis 2 Uphase amps
    ADCch_offset[2] = eeread(104);  // Bus Volts
    ADCch_offset[3] = eeread(106);  //Axis 1 Control in
    ADCch_offset[4] = eeread(108);  //Axis 2 Control in
    ADCch_offset[5] = eeread(110);  // Axis 1 IGBT temp
    ADCch_offset[6] = eeread(112);  // Axis 2 IGBT temp
    //
    bus_up_threshold = eeread(EE_CV_BUS_UP);
    bus_down_threshold = eeread(EE_CV_BUS_DOWN);

    //
    // Enable I2C interrupt 1 in the PIE: Group 8 interrupt 1
    //
    PieCtrlRegs.PIEIER8.bit.INTx1 = 1;

    //
    // Enable CPU INT8 which is connected to PIE group 8
    //

    IER |= M_INT8;



    //
    // Enable ADC Interrupts in the PIEIER1 group register
    //
    // now enable the interrupts for ADC with limit checks

    PieCtrlRegs.PIEIER1.bit.INTx1 = 1; // Enable INT 1.1 in the PIE
    PieCtrlRegs.PIEIER1.bit.INTx2 = 1; // Enable INT 1.2 in the PIE

    IER |= M_INT3;
    IER |= M_INT1;  // ADC interrupt

    //
    // Disable EPWM INTn in the PIE: Group 3 interrupt 1-3
    //
    PieCtrlRegs.PIEIER3.bit.INTx1 = 0;
    EINT;       // Enable Global interrupt INTM
    ERTM;       // Enable Global realtime interrupt DBGM
    //
    // Wait for SCI to be idle and ready for transmission
    //
    watchdoginit();
    iowrite(30,0); // turn off fan
    while(LinaRegs.SCIFLR.bit.IDLE == 1);
    {

    }

    scia_msg("\r\nC2000-PM_Motor_Drive V0.1\n\0");
    scia_msg("\r\n Copyright 2021 John Figie\n\0");
    scia_msg("\r\n License GPL V2.0 or newer\n\0");
    scia_msg("\r\n C2000-PM_Motor_drive comes with ABSOLUTELY NO WARRANTY\n\0");
    axis2.fault = 0;
    axis2.fault_last = 0;
    axis2.state = 0;
    axis1.fault = 0;
    axis1.fault_last = 0;
    axis1.state = 0;

    for(;;)
    {
        scia_msg("\r\nClausingDrive> \0");
        ReceivedChar = 0;
        j=0;
        // wait for a new line
        while((ReceivedChar != (int) '\r') && (j < 40))
        {




            //
            // Wait for a character to by typed
            //
            while(LinaRegs.SCIFLR.bit.RXRDY == 0)
            {

                // run some logic in this loop while waiting for input.

                // check for a change in the fault status and report if changed.
                        fault_temp = axis1.fault;
                        if (fault_temp != axis1.fault_last){
                            itoa(output_buffer, fault_temp);  //
                            scia_msg("\r\naxis1.fault");
                            scia_msg(output_buffer);
                            scia_msg("\r\nClausingDrive> \0");
                            axis1.fault_last = fault_temp;
                            iowrite(31, 1);

                        }
                        fault_temp = axis2.fault;
                        if (fault_temp != axis2.fault_last){
                            itoa(output_buffer, fault_temp);  //
                            scia_msg("\r\naxis2.fault");
                            scia_msg(output_buffer);
                            scia_msg("\r\nClausingDrive> \0");
                            axis2.fault_last = fault_temp;
                            iowrite(31, 1);

                        }
                        // check bus voltage and update I/O
                        bus_up = bus_state(ADCch[2]);
                        iowrite(27, bus_up);
                        // check fault status and update I/O
                        if (axis1.fault != 0 || axis2.fault != 0){
                            iowrite(31, 1);
                        }
                        else{
                            iowrite(31, 0);
                        }

                // axis state machine logic.
                        //  state variable
                        //  00 = Bus_Up_N
                        //  01 = Bus_Up && Enable_N
                        //  11 = Running - Enable && Bus_Up && Fault_N

                        if (axis1.loop_mode == 1) {
                           if (axis1.state == 1 && axis1.fault == 0) {
                                if (0 == ioread(39)) {
                                    pwm_on(1);
                                    axis1.state = 3;
                                    iowrite(30,1); // fan on
                                }
                           }
                           if (axis1.state == 0) {
                               if (bus_up == 1){
                                   axis1.state = 1;
                               }
                           }
                           if (axis1.state == 3) {
                                if (1 == ioread(39)) {
                                    pwm_off(1);
                                    axis1.state = 1;
                                }
                                if (axis1.fault != 0 && axis2.fault != 0){
                                    pwm_off(1);
                                    axis1.state = 1;
                                }
                                if (bus_up == 0){
                                    pwm_off(1);
                                    axis1.state = 0;
                                }
                                if (axis1.state == 1) {
                                    if (bus_up == 0){
                                        axis1.state = 0;
                                    }
                                }
                           }
                        }
                        if (axis1.loop_mode == 1 ) {  // loop_mode is only axis1 variable - not independent
                            if (axis2.state == 1 && axis2.fault == 0) {
                                if (0 == ioread(44)) {
                                    pwm_on(2);
                                    axis2.state = 3;
                                    iowrite(30,1); // fan on
                                }

                           }
                            if (axis2.state == 0) {
                                if (bus_up == 1){
                                    axis2.state = 1;
                                }
                            }
                           if (axis2.state == 3) {
                                if (1 == ioread(44)) {
                                    pwm_off(2);
                                    axis2.state = 1;
                                }
                                if (axis2.fault != 0 && axis1.fault !=0){
                                    pwm_off(2);
                                    axis2.state = 1;
                                }
                                if (bus_up == 0){
                                    pwm_off(1);
                                    axis1.state = 0;
                                }
                           }
                           if (axis2.state == 1) {
                               if (bus_up == 0){
                                   axis2.state = 0;
                               }
                           }

                        }






            }

            ReceivedChar = LinaRegs.SCIRD;
            if (escape_flag > 0)  // an escape sequence was started
            {

                if (escape_flag == 4)
                {
                    if ((char) ReceivedChar == '~') escape_flag = -111; // F1 Key detected
                    if (escape_flag == 4) escape_flag = -1;   // escape the escape sequence
                }
                if (escape_flag == 3)
                {
                    if ((char) ReceivedChar == '1') escape_flag = 4;
                    if (escape_flag == 3) escape_flag = -1;   // escape the escape sequence

                }
                if (escape_flag == 2)
                {
                    if ((char) ReceivedChar == '1') escape_flag = 3;
                    if ((char) ReceivedChar == 'A') escape_flag = -100;  // up arrow detected
                    if (escape_flag == 2) escape_flag = -1;   // escape the escape sequence
                }
                if (escape_flag == 1)
                {
                    if ((char) ReceivedChar == '[') escape_flag = 2;
                }

            }
            if ((char) ReceivedChar == '\x1b')  // test for Escape. note: up down etc adds more characters after escape
            {
                // escape sequence is started look for specific cases
                escape_flag = 1;
            }
            if (escape_flag == -100)  // escape_flag == 2 means up arrow UP arrow with j=0 allows last line reuse
            {
                if (j==0) for (;j<jl;)
                {
                    if (ReceivedLine[j] == '\x00')  ReceivedLine[j] = '\x20'; // replace null with space
                    scia_xmit(ReceivedLine[j++]); // if this is the first char entered then get reuse line buffer
                }
                escape_flag = -1;
            }
            if (escape_flag == -111) // test for F1 key escape_flag 11==F1, 12==F2, etc
            {
                pwm_off((Uint16) 0);   // SW force all PWMs off
                pwm_off_message((Uint16) 0);
                ReceivedChar = (int) '\r';
                ReceivedLine[j++] = (char) ReceivedChar;
                escape_flag = -1;
            }
            if (escape_flag ==0)  // not an escape sequence
            {

                scia_xmit(ReceivedChar);
                if ((char) ReceivedChar != '\x7f') // x7f is the delete character Backspace on Keyboard
                        ReceivedLine[j++] = (char) ReceivedChar;
                else
                {
                    if (j>0) j--;
                }
            }
            if (escape_flag == -1) escape_flag = 0; // escape sequence is over - allow echoback.
        }
        jl = j-1;  // make J last one less to ignore CR at end Jl is used to reuse the buffer.
        if (ExecuteCommand(ReceivedLine) == 0)
            scia_msg("\r\ncommand not found");

    }
}

//
// scia_xmit -
//
void
scia_xmit(char Char)
{
    //
    // Wait for the module to be ready to transmit
    //
    while(LinaRegs.SCIFLR.bit.TXRDY == 0);

    //
    // Begin transmission
    //
    LinaRegs.SCITD = Char;
}

//
// scia_msg -
//
void
scia_msg(char *msg)
{
    int it;
    it = 0;

    while(msg[it] != '\0')
    {
        scia_xmit(msg[it]);
        it++;
    }
}
//
// SetupSCI -
//
void
SetupSCI(void)
{
    //
    // Allow write to protected registers
    //
    EALLOW;

    LinaRegs.SCIGCR0.bit.RESET = 0;     // Into reset
    LinaRegs.SCIGCR0.bit.RESET = 1;     // Out of reset

    LinaRegs.SCIGCR1.bit.SWnRST = 0;    // Into software reset

    //
    // SCI Configurations
    //
    LinaRegs.SCIGCR1.bit.COMMMODE = 0;      // Idle-Line Mode
    LinaRegs.SCIGCR1.bit.TIMINGMODE = 1;    // Asynchronous Timing
    LinaRegs.SCIGCR1.bit.PARITYENA = 0;     // No Parity Check
    LinaRegs.SCIGCR1.bit.PARITY = 0;        // Odd Parity
    LinaRegs.SCIGCR1.bit.STOP = 0;          // One Stop Bit
    LinaRegs.SCIGCR1.bit.CLK_MASTER = 1;    // Enable SCI Clock
    LinaRegs.SCIGCR1.bit.LINMODE = 0;       // SCI Mode
    LinaRegs.SCIGCR1.bit.SLEEP = 0;         // Ensure Out of Sleep
    LinaRegs.SCIGCR1.bit.MBUFMODE = 0;      // No Buffers Mode
    LinaRegs.SCIGCR1.bit.LOOPBACK = 0;      // External Loopback
    LinaRegs.SCIGCR1.bit.CONT = 1;          // Continue on Suspend
    LinaRegs.SCIGCR1.bit.RXENA = 1;         // Enable RX
    LinaRegs.SCIGCR1.bit.TXENA = 1;         // Enable TX

    //
    // Ensure IODFT is disabled
    //
    LinaRegs.IODFTCTRL.bit.IODFTENA = 0x0;

    //
    // Set transmission length
    //
    LinaRegs.SCIFORMAT.bit.CHAR = 7;     //Eight bits
    LinaRegs.SCIFORMAT.bit.LENGTH = 0;   //One byte

    //
    // Set baudrate
    //
    LinaRegs.BRSR.bit.SCI_LIN_PSL = 194;          //Baud = 9.6khz
    LinaRegs.BRSR.bit.M = 5;

    LinaRegs.SCIGCR1.bit.SWnRST = 1;  //bring out of software reset

    //
    // Disable write to protected registers
    //
    EDIS;
}

//
// error - Error checking
//
void
error(void)
{
    __asm("     ESTOP0");   // Test failed!! Stop!
    for (;;);
}


void set_pwm(Uint16 axis, Uint16 pwm_value)
{
    if (axis == 1)
    {
        EPwm1Regs.CMPA.half.CMPA = pwm_value;
        EPwm2Regs.CMPA.half.CMPA = EPWM_TIMER_PRD - pwm_value;
    }
    if (axis == 2)
    {
        EPwm4Regs.CMPA.half.CMPA = pwm_value;
        EPwm5Regs.CMPA.half.CMPA = EPWM_TIMER_PRD - pwm_value;
    }
    if (axis == 7)
    {
        EPwm7Regs.CMPA.half.CMPA = pwm_value;
    }
    if (axis == 8)
    {
        EPwm7Regs.CMPB = pwm_value;
    }

return;
}
void pulse_pwm(Uint16 axis, Uint16 pwm_value,Uint16 npulses)
// npulses,npulses1 are global! // assume pwm1 and pwm2 are not pulsed simultaneously
// this function works in conjunction with the PWM1 ISR to make the pulse
{
    static Uint16 saved_pwm1,saved_pwm2;
    if (axis == 1)
    {
        if (npulses!=0)
        {
            saved_pwm1 = EPwm1Regs.CMPA.half.CMPA;
            EPwm1Regs.CMPA.half.CMPA = pwm_value;
            EPwm2Regs.CMPA.half.CMPA = EPWM_TIMER_PRD - pwm_value;
            npulses1 = npulses;
        }
        else
        {
            EPwm1Regs.CMPA.half.CMPA = saved_pwm1;
            EPwm2Regs.CMPA.half.CMPA = EPWM_TIMER_PRD - saved_pwm1;
        }

    }
    if (axis == 2)
    {
        if (npulses!=0)
        {
            saved_pwm2 = EPwm4Regs.CMPA.half.CMPA;
            EPwm4Regs.CMPA.half.CMPA = pwm_value;
            EPwm5Regs.CMPA.half.CMPA = EPWM_TIMER_PRD - pwm_value;
            npulses2 = npulses;
        }
        else
        {
            EPwm4Regs.CMPA.half.CMPA = saved_pwm2;
            EPwm5Regs.CMPA.half.CMPA = EPWM_TIMER_PRD - saved_pwm2;
        }
    }
return;
}
// this function works in conjunction with the PWM1 ISR to make the pulse
// npulses,npulses1 are global! // assume cref1 and cref2 are not pulsed simultaneously
void pulse_cref(Uint16 axis, int16 cref_value,Uint16 npulses)
{
    if (axis == 1)
    {
        if (npulses!=0)
        {
            axis1.cref = cref_value;
            npulses1 = npulses;
        }
        else
        {
            axis1.cref = 0;            // set to zero after pulse
        }

    }
    if (axis == 2)
    {
        if (npulses!=0)
        {
            axis2.cref = cref_value;
            npulses2 = npulses;
        }
        else
        {
            axis2.cref = 0;            // set to zero after pulse
        }
    }
return;
}
void pwm_off(Uint16 axis)  // force the pwm outputs off by sw
{
    if (axis==1 | axis==0)
    {
        EPwm1Regs.AQCSFRC.bit.CSFA = AQ_CLEAR;   // Force all PWM outputs off
        EPwm1Regs.AQCSFRC.bit.CSFB = AQ_CLEAR;
        EPwm2Regs.AQCSFRC.bit.CSFA = AQ_CLEAR;
        EPwm2Regs.AQCSFRC.bit.CSFB = AQ_CLEAR;
        if (axis1.state == 3) {
            axis1.state = 1;
        }
    }
    if (axis==2 | axis==0)
    {
        EPwm4Regs.AQCSFRC.bit.CSFA = AQ_CLEAR;
        EPwm4Regs.AQCSFRC.bit.CSFB = AQ_CLEAR;
        EPwm5Regs.AQCSFRC.bit.CSFA = AQ_CLEAR;
        EPwm5Regs.AQCSFRC.bit.CSFB = AQ_CLEAR;
        if (axis2.state == 3) {
            axis2.state = 1;
        }
        axis2.state = 1;

    }

return;
}
void pwm_off_message(Uint16 axis)  // inform user that PWMs are off
{
    if (axis==1 | axis==0)
    {
        scia_msg("\r\nPWM 1 Disabled");          // this message takes time
    }
    if (axis==2 | axis==0)
    {
        scia_msg("\r\nPWM 2 Disabled");
    }

return;
}
void pwm_on(Uint16 axis)  // pwms are already set to run but outputs were forced off by sw
{
    if (axis==1)
    {
        axis1.iterm = 0;        // init values in case loop is closed
        axis1.pwm = 3750;
        set_pwm(1,3750);
        EPwm1Regs.AQCSFRC.bit.CSFA = AQ_NO_ACTION;   // disable output forcing
        EPwm1Regs.AQCSFRC.bit.CSFB = AQ_NO_ACTION;
        EPwm2Regs.AQCSFRC.bit.CSFA = AQ_NO_ACTION;
        EPwm2Regs.AQCSFRC.bit.CSFB = AQ_NO_ACTION;
        scia_msg("\r\nPWM 1 Enabled");

    }
    if (axis==2)
    {
        axis2.iterm = 0;        // init values in case loop is closed
        axis2.pwm = 3750;
        set_pwm(2,3750);
        EPwm4Regs.AQCSFRC.bit.CSFA = AQ_NO_ACTION;   // disable output forcing
        EPwm4Regs.AQCSFRC.bit.CSFB = AQ_NO_ACTION;
        EPwm5Regs.AQCSFRC.bit.CSFA = AQ_NO_ACTION;
        EPwm5Regs.AQCSFRC.bit.CSFB = AQ_NO_ACTION;
        scia_msg("\r\nPWM 2 Enabled");

    }

return;
}

void run_loop(void)
{
    pid(&axis1);
    pid(&axis2);
    set_pwm((Uint16)1, axis1.pwm);
    set_pwm((Uint16)2, axis2.pwm);
}
void pid(struct PID *axis)
{
int32 pterm,iterm,errorterm,cterm;

    errorterm = ( axis->cref - axis->fb_current); // current is already opp sign so add
    pterm = errorterm * (int32) axis->pgain * (int32) 64 ;
    iterm = axis->iterm + errorterm * (int32) axis->igain;
    if (iterm > 983040000) iterm = 983040000;    // these values are +/- 2^18 which are the limits for calculations
    if (iterm < -983040000) iterm = -983040000;
    axis->iterm = iterm;
    cterm = ((pterm+iterm) >> 16) + 3750;
    if (cterm > 7500) {
        axis->pwm = 7500;
    }
    else if (cterm < 0) {
        axis->pwm = 0;
    }
    else {
        axis->pwm = cterm;
    }

    // add overload detection

}
int16 ioread(Uint16 io_number)
{
    switch (io_number)
    {
    case 39:
        return(GpioDataRegs.GPBDAT.bit.GPIO39);
    case 44:
        return(GpioDataRegs.GPBDAT.bit.GPIO44);
    }
    scia_msg("\r\nbad port number");
    return(-1);
}
void iowrite(Uint16 io_number, Uint16 io_value)
{
    switch (io_number)
    {
    case 27:
        if (io_value == 1) GpioDataRegs.GPASET.bit.GPIO27 = 1;
        if (io_value == 0) GpioDataRegs.GPACLEAR.bit.GPIO27 = 1;
        break;
    case 30:
        if (io_value == 1) GpioDataRegs.GPASET.bit.GPIO30 = 1;
        if (io_value == 0) GpioDataRegs.GPACLEAR.bit.GPIO30 = 1;
        break;
    case 31:
        if (io_value == 1) GpioDataRegs.GPASET.bit.GPIO31 = 1;
        if (io_value == 0) GpioDataRegs.GPACLEAR.bit.GPIO31 = 1;
        break;
    case 34:
        if (io_value == 1) GpioDataRegs.GPBSET.bit.GPIO34 = 1;
        if (io_value == 0) GpioDataRegs.GPBCLEAR.bit.GPIO34 = 1;
        break;
    default:
        scia_msg("\r\nbad port number");
    }

}
Uint16 bus_state(int16 bus_measurement)
{
    static Uint16 bus_state;
    if ((Uint16)bus_measurement > bus_up_threshold){
        bus_state = 1;
    }
    if ((Uint16)bus_measurement < bus_down_threshold){
        bus_state = 0;
    }
    return bus_state;

}
//
// End of File

//
