//###########################################################################


//
// Included Files
//
#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include <stdio.h>
#include <string.h>
//
// Typedefs
//
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

// external functions

extern void GPIO_setPinMuxConfig(void);
extern int ExecuteCommand(char *);
extern void Gpio_setup(void);

//
// Functions Prototypes
//
void SetupSCI(void);
void error(void);
void scia_xmit(char Char);
void scia_msg(char *msg);
void set_pwm(Uint16 axis, Uint16 pwm_value);
void pulse_pwm(Uint16 axis, Uint16 pwm_value, Uint16 npulses);
void pwm_on(Uint16 axis);
void pwm_off(Uint16 axis);


// pwm functions
void InitEPwm1Example(void);
void InitEPwm2Example(void);
void InitEPwm4Example(void);
void InitEPwm5Example(void);

__interrupt void epwm1_isr(void);
__interrupt void epwm2_isr(void);
__interrupt void epwm4_isr(void);
__interrupt void adc_isr(void);


//
// Globals
//
Uint16 LoopCount;
Uint16 ErrorCount;
Uint16 CurrentU1;
Uint16 npulses1,npulses2,npulses1_last,npulses2_last;
Uint16 cc_enable;



// pwm globals
EPWM_INFO epwm1_info;
EPWM_INFO epwm2_info;
EPWM_INFO epwm4_info;
EPWM_INFO epwm5_info;

//
// Exeternal defines d by the linker
//
extern Uint16 RamfuncsLoadStart;
extern Uint16 RamfuncsLoadSize;
extern Uint16 RamfuncsRunStart;

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



//
// Main
//
void main(void)
{
    Uint16 ReceivedChar;
    char ReceivedLine[80];
    int j,jl;
    jl = 0;
    int escape_flag = 0;
    npulses1 = 0;  // init global vaiables
    npulses2 = 0;
    npulses1_last = 0;
    npulses2_last = 0;
    cc_enable = 1; // start with CC pulse enabled
    //
    // Step 1. Initialize System Control:
    // PLL, WatchDog, enable Peripheral Clocks
    // This example function is found in the DSP2803x_SysCtrl.c file.
    //
    InitSysCtrl();

    //
    // Step 2. Initialize GPIO:
    // This example function is found in the DSP2803x_Gpio.c file and
    // illustrates how to set the GPIO to it's default state.
    //
    // InitGpio(); Skipped for this example

    //
    // For this example, only init the pins for the SCI-A port.
    // This function is found in the DSP2803x_Sci.c file.
    //
    GPIO_setPinMuxConfig();
    //
    // Step 3. Clear all interrupts and initialize PIE vector table:
    // Disable CPU interrupts
    //
    DINT;

    //
    // Initialize PIE control registers to their default state.
    // The default state is all PIE interrupts disabled and flags
    // are cleared.
    // This function is found in the DSP2803x_PieCtrl.c file.
    //
    InitPieCtrl();

    //
    // Disable CPU interrupts and clear all CPU interrupt flags:
    //
    IER = 0x0000;
    IFR = 0x0000;

    //
    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    // This will populate the entire table, even if the interrupt
    // is not used in this example.  This is useful for debug purposes.
    // The shell ISR routines are found in DSP2803x_DefaultIsr.c.
    // This function is found in DSP2803x_PieVect.c.
    //
    InitPieVectTable();
    //
    // Interrupts that are used in this example are re-mapped to
    // ISR functions found within this file.
    //
    EALLOW;  // This is needed to write to EALLOW protected registers
    PieVectTable.EPWM1_INT = &epwm1_isr;
    PieVectTable.EPWM2_INT = &epwm2_isr;
    PieVectTable.EPWM4_INT = &epwm4_isr;
    PieVectTable.ADCINT1 = &adc_isr;
    EDIS;    // This is needed to disable write to EALLOW protected registers

    //
    // Step 4. Initialize all the Device Peripherals:
    // Copy time critical code and Flash setup code to RAM
    // This includes the following ISR functions:  InitFlash();
    // The  RamfuncsLoadStart, RamfuncsLoadEnd, and RamfuncsRunStart
    // symbols are created by the linker.
    //
    memcpy((Uint16 *)&RamfuncsRunStart,(Uint16 *)&RamfuncsLoadStart,
            (unsigned long)&RamfuncsLoadSize);

    //
    // Initialize and Enable BLIN SCI module
    //
    SetupSCI();
    //
    // Initialize the ePWM
    //
    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;
    EDIS;

    InitEPwm1Example();
    InitEPwm2Example();
    InitEPwm4Example();
    InitEPwm5Example();

    InitAdc();  // For this example, init the ADC
    //AdcOffsetSelfCal();


    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;
    EDIS;

    // Configure ADC
     // Note: Channel ADCINA4  will be double sampled to workaround the
     // ADC 1st sample issue for rev0 silicon errata
     //
     EALLOW;
     AdcRegs.ADCCTL1.bit.INTPULSEPOS = 1; //ADCINT1 trips after AdcResults latch
     AdcRegs.INTSEL1N2.bit.INT1E     = 1; // Enabled ADCINT1
     AdcRegs.INTSEL1N2.bit.INT1CONT  = 0; // Disable ADCINT1 Continuous mode
     AdcRegs.INTSEL1N2.bit.INT1SEL   = 0;  // setup EOC0 to trigger
                                           // ADCINT1 to fire

     //
     // set SOC0 channel select to ADCINA4
     // (dummy sample for rev0 errata workaround)
     // for this program do not worry about errata
     AdcRegs.ADCSOC0CTL.bit.CHSEL    = 0;  // Axis 1 U phase amps
     //
     AdcRegs.ADCSOC0CTL.bit.TRIGSEL  = 5;  // set SOC0 start trigger on EPWM1A,
                                           //due to round-robin SOC0 converts
                                           // first then SOC1, then SOC2
    //
     // set SOC0 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
     //
     AdcRegs.ADCSOC0CTL.bit.ACQPS    = 6;
     EDIS;

    //
    // Step 5. User specific code, enable interrupts:
    //

    //

    //
    // Call Flash Initialization to setup flash wait states
    // This function must reside in RAM
    //
    InitFlash();

    //
    // Step 5. User specific code:
    //
    // EALLOW;
    Gpio_setup();
    //LoopCount = 0;

    //
    // Enable CPU INT3 which is connected to EPWM1-3 INT:
    //
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1; // Enable INT 1.1 in the PIE

    IER |= M_INT3;
    IER |= M_INT1;  // ADC interrupt

    //
    // Enable EPWM INTn in the PIE: Group 3 interrupt 1-3
    //
    PieCtrlRegs.PIEIER3.bit.INTx1 = 1;
    //PieCtrlRegs.PIEIER3.bit.INTx2 = 1;
    //PieCtrlRegs.PIEIER3.bit.INTx3 = 1;

    //
    // Enable global Interrupts and higher priority real-time debug events:
    //
    EINT;       // Enable Global interrupt INTM
    ERTM;       // Enable Global realtime interrupt DBGM

    //
    // Step 6. IDLE loop. Just sit and loop forever (optional):
    //



    //
    // Wait for SCI to be idle and ready for transmission
    //
    while(LinaRegs.SCIFLR.bit.IDLE == 1);
    {

    }

    scia_msg("\r\nClausingDrive Monitor and control V0.0 \
            \n\0");

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
            while(LinaRegs.SCIFLR.bit.RXRDY == 0);
            {

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

//
// epwm1_isr - EPwm1 ISR
//
__interrupt void
epwm1_isr(void)
{
    //
    // Update the CMPA and CMPB values
    //
    //update_compare(&epwm1_info);

    //
    // Clear INT flag for this timer
    //
    EPwm1Regs.ETCLR.bit.INT = 1;
    //EPwm1Regs.ETCLR.bit.SOCA = 1; //Clear Start of conversion flag
       //
    // Acknowledge this interrupt to receive more interrupts from group 3
    //
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;

    if (npulses1_last == 1 & npulses1 == 0 ) pulse_pwm((Uint16) 1,(Uint16) 0, (Uint16) 0);
    npulses1_last = npulses1;
    if (npulses1 > 0) npulses1--;

}

//
// epwm2_isr - EPwm2 ISR
//
__interrupt void
epwm2_isr(void)
{
    //
    // Update the CMPA and CMPB values
    //
    //update_compare(&epwm2_info);

    //
    // Clear INT flag for this timer
    //
    EPwm2Regs.ETCLR.bit.INT = 1;

    //
    // Acknowledge this interrupt to receive more interrupts from group 3
    //
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}

//
// epwm3_isr - EPwm3 ISR
//
__interrupt void
epwm4_isr(void)
{
    //
    // Update the CMPA and CMPB values
    //
    //update_compare(&epwm3_info);

    //
    // Clear INT flag for this timer
    //
    EPwm3Regs.ETCLR.bit.INT = 1;

    //
    // Acknowledge this interrupt to receive more interrupts from group 3
    //
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}
//
// adc_isr -
//
__interrupt void
adc_isr(void)
{
    //
    // discard ADCRESULT0 as part of the workaround to the
    // 1st sample errata for rev0
    //
    if (cc_enable) GpioDataRegs.GPASET.bit.GPIO24 = 1;   // make CC pulse

    CurrentU1 =  AdcResult.ADCRESULT0;

 /*   Voltage2[ConversionCount] = AdcResult.ADCRESULT2;

    //
    // If 20 conversions have been logged, start over
    //
    if(ConversionCount == 9)
    {
        ConversionCount = 0;
    }
    else
    {
        ConversionCount++;
    }

    //
    // Clear ADCINT1 flag reinitialize for next SOC
 */   //
    AdcRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;
    GpioDataRegs.GPACLEAR.bit.GPIO24 = 1;   // finish CC pulse

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;   // Acknowledge interrupt to PIE

    return;
}

//
// InitEPwm1Example - EPwm1 example
//
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
    EPwm1Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm1Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;        // A is active High complementary
    EPwm1Regs.DBCTL.bit.IN_MODE = DBA_ALL;
    EPwm1Regs.DBRED = EPWM1_MIN_DB;
    EPwm1Regs.DBFED = EPWM1_MIN_DB;
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
    epwm1_info.EPwm_CMPA_Direction = EPWM_CMP_UP;
    epwm1_info.EPwm_CMPB_Direction = EPWM_CMP_DOWN;

    epwm1_info.EPwmTimerIntCount = 0;      //Zero the interrupt counter
    epwm1_info.EPwmRegHandle = &EPwm1Regs; //Set the pointer to the ePWM module
    epwm1_info.EPwmMaxCMPA = EPWM1_MAX_CMPA;  // Setup min/max CMPA/CMPB values
    epwm1_info.EPwmMinCMPA = EPWM1_MIN_CMPA;
    epwm1_info.EPwmMaxCMPB = EPWM1_MAX_CMPB;
    epwm1_info.EPwmMinCMPB = EPWM1_MIN_CMPB;
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
    EPwm2Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;     // Select INT on Zero event
    EPwm2Regs.ETSEL.bit.INTEN = 1;                // Enable INT
    EPwm2Regs.ETPS.bit.INTPRD = ET_3RD;           // Generate INT on 3rd event


    // Setup ADC SOC trigger
    EPwm2Regs.ETSEL.bit.SOCBSEL = ET_CTRU_CMPB;
    //
    // Information this example uses to keep track of the direction the
    // CMPA/CMPB values are moving, the min and max allowed values and
    // a pointer to the correct ePWM registers
    //

    //
    // Start by increasing CMPA & increasing CMPB
    //
    epwm2_info.EPwm_CMPA_Direction = EPWM_CMP_UP;
    epwm2_info.EPwm_CMPB_Direction = EPWM_CMP_UP;

    epwm2_info.EPwmTimerIntCount = 0;      //Zero the interrupt counter
    epwm2_info.EPwmRegHandle = &EPwm2Regs; //Set the pointer to the ePWM module
    epwm2_info.EPwmMaxCMPA = EPWM2_MAX_CMPA;  // Setup min/max CMPA/CMPB values
    epwm2_info.EPwmMinCMPA = EPWM2_MIN_CMPA;
    epwm2_info.EPwmMaxCMPB = EPWM2_MAX_CMPB;
    epwm2_info.EPwmMinCMPB = EPWM2_MIN_CMPB;
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

void set_pwm(Uint16 axis, Uint16 pwm_value)
{
    if (axis == 1)
    {
        EPwm1Regs.CMPA.half.CMPA = pwm_value;
        EPwm2Regs.CMPA.half.CMPA = pwm_value;
    }
    if (axis == 2)
    {
        EPwm4Regs.CMPA.half.CMPA = pwm_value;
        EPwm5Regs.CMPA.half.CMPA = pwm_value;
    }
return;
}
void pulse_pwm(Uint16 axis, Uint16 pwm_value,Uint16 npulses)
{
    static Uint16 saved_pwm1,saved_pwm2;
    if (axis == 1)
    {
        if (npulses!=0)
        {
            saved_pwm1 = EPwm1Regs.CMPA.half.CMPA;
            EPwm1Regs.CMPA.half.CMPA = pwm_value;
            EPwm2Regs.CMPA.half.CMPA = pwm_value;
            npulses1 = npulses;
        }
        else
        {
            EPwm1Regs.CMPA.half.CMPA = saved_pwm1;
            EPwm2Regs.CMPA.half.CMPA = saved_pwm1;
        }

    }
    if (axis == 2)
    {
        if (npulses!=0)
        {
            saved_pwm2 = EPwm4Regs.CMPA.half.CMPA;
            EPwm4Regs.CMPA.half.CMPA = pwm_value;
            EPwm5Regs.CMPA.half.CMPA = pwm_value;
            npulses2 = npulses;
        }
        else
        {
            EPwm4Regs.CMPA.half.CMPA = saved_pwm2;
            EPwm5Regs.CMPA.half.CMPA = saved_pwm2;
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
        cc_enable = 0;    // disable the CC signal to trigger scope
        scia_msg("\r\nPWM 1 Disabled");          // this message takes time
    }
    if (axis==2 | axis==0)
    {
        EPwm4Regs.AQCSFRC.bit.CSFA = AQ_CLEAR;
        EPwm4Regs.AQCSFRC.bit.CSFB = AQ_CLEAR;
        EPwm5Regs.AQCSFRC.bit.CSFA = AQ_CLEAR;
        EPwm5Regs.AQCSFRC.bit.CSFB = AQ_CLEAR;
        cc_enable = 0;    // disable the CC signal to trigger scope
        scia_msg("\r\nPWM 2 Disabled");

    }

return;
}
void pwm_on(Uint16 axis)  // pwms are already set to run but outputs were forced off by sw
{
    if (axis==1)
    {
        EPwm1Regs.AQCSFRC.bit.CSFA = AQ_NO_ACTION;   // disable output forcing
        EPwm1Regs.AQCSFRC.bit.CSFB = AQ_NO_ACTION;
        EPwm2Regs.AQCSFRC.bit.CSFA = AQ_NO_ACTION;
        EPwm2Regs.AQCSFRC.bit.CSFB = AQ_NO_ACTION;
        scia_msg("\r\nPWM 1 Enabled");

    }
    if (axis==2)
    {
        EPwm4Regs.AQCSFRC.bit.CSFA = AQ_NO_ACTION;   // disable output forcing
        EPwm4Regs.AQCSFRC.bit.CSFB = AQ_NO_ACTION;
        EPwm5Regs.AQCSFRC.bit.CSFA = AQ_NO_ACTION;
        EPwm5Regs.AQCSFRC.bit.CSFB = AQ_NO_ACTION;
        scia_msg("\r\nPWM 2 Enabled");

    }
    cc_enable = 1;
return;
}

//
// End of File
//
