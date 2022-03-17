//###########################################################################
//
//    Copyright (C) 2022 John Figie
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
/*
 * PMDC_drive.h
 *
 *  Created on: Jan 1, 2022
 *      Author: zephyr
 */

#ifndef PMDC_DRIVE_H_
#define PMDC_DRIVE_H_

#define I2C_SLAVE_ADDR        0x50
#define I2C_NUMBYTES          2
#define I2C_EEPROM_HIGH_ADDR  0x00
#define I2C_EEPROM_LOW_ADDR   0x30

#define EE_CV_P1 0x10   // 16 address for P gain1
#define EE_CV_I1 0x12   // 18 I gain 1
#define EE_CV_FF1 0x14  // 20 Feed Forward gain 1
#define EE_CV_CREF1 0x16 // 22 initial current reference
#define EE_CV_LM1 0x18  // 24 loop mode (open or closed)
#define EE_CV_IM1 0x1A  // 26 input mode LNCNC or monitor
#define EE_CV_CLIM1 0x1C // 28 current limit exceeded
#define EE_CV_FAULT 0x1E // 30 Fault code
#define EE_CV_CREFLIMIT1 0x22 // 34 max input allowed.  address 32 doesn't work?????

#define EE_CV_P2 0x90   // 144 address for PI gain2
#define EE_CV_I2 0x92   // 146
#define EE_CV_FF2 0x94  // 148
#define EE_CV_CREF2 0x96 // 150
#define EE_CV_LM2 0x98   // 152 loop mode (open or closed) -- not used
#define EE_CV_IM2 0x9A  //  154 input mode LNCNC or monitor -- not used
#define EE_CV_CLIM2 0x9C // 156 current limit exceeded
#define EE_CV_FAUL2 0x9E // 158 Fault code
#define EE_CV_CREFLIMIT3 0xA0 // 160 max input allowed.

#define OPEN_LOOP 0
#define CLOSED_LOOP 1
#define LCNC 1
#define OVERCURRENT 1
#define INPUT_SAT 2

#define PWM_Enable1 GpioDataRegs.GPBDAT.bit.GPIO39
#define PMW_Enable2 GpioDataRegs.GPBDAT.bit.GPIO44

struct PID {
    int32 iterm;
    Uint16 pgain;
    Uint16 igain;
    Uint16 ff;
    int16 cref;
    int16 fb_current;
    Uint16 pwm;
    Uint16 loop_mode;
    Uint16 input_mode;
    int16 current_limit;
    Uint16 fault;
    int16 cref_limit;
};


#endif /* PMDC_DRIVE_H_ */
