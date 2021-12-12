/*
 * Execute_Command.c
 *
 *  Created on: Jan 4, 2021
 *      Author: zephyr
 */
//
// Included Files
//
#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
extern void set_pwm(Uint16 axis, Uint16 pwm_value);
extern void pulse_pwm(Uint16 axis, Uint16 pwm_value, Uint16 npulses);
extern void scia_msg(char *);
extern void pwm_on(Uint16 axis);
extern void pwm_off(Uint16 axis);
extern Uint16 CurrentU1;
//function prototypes
char* itoa( char * , Uint16);

char output_buffer[80];

int ExecuteCommand(char *ReceivedLine)
{
int axis = 0;
int pwm_mag,npulses,pulse_pwm_mag;
char* token;
const char s[2] = " ";
//char output_buffer[80];
;

    if (ReceivedLine[0] ==  '\r')
        return (1);
    else

    {
        /* get the first token */

        token = strtok(ReceivedLine, s);


        if (strcmp(token, "s_pwm") == 0)

        {

                token = strtok(NULL, s);

                axis = atoi(token);
                token = strtok(NULL, s);
                pwm_mag = atoi(token);
//                sprintf(output_buffer, "%i %i", axis, pwm_mag);
                scia_msg("\r\nSetting PWM");
                set_pwm((Uint16) axis,(Uint16) pwm_mag);
                return (1);

        }

        if (strcmp(token, "r_adc") == 0)
        {
            token = strtok(NULL, s);

            axis = atoi(token);
            itoa(output_buffer, CurrentU1);
            scia_msg("\r\n");
            scia_msg(output_buffer);
            return (1);

        }


        if (strcmp(token, "p_pwm") == 0)

        {

                token = strtok(NULL, s);

                axis = atoi(token);
                token = strtok(NULL, s);
                pulse_pwm_mag = atoi(token);
                token = strtok(NULL, s);
                npulses = atoi(token);
                scia_msg("\r\nPulsing PWM");
                pulse_pwm((Uint16) axis,(Uint16) pulse_pwm_mag, (Uint16) npulses);
                return (1);
        }
        if (strcmp(token, "pwm_off") == 0)

        {

                token = strtok(NULL, s);

                axis = atoi(token);
                pwm_off((Uint16) axis);
                return (1);
        }
        if (strcmp(token, "pwm_on") == 0)

        {

                token = strtok(NULL, s);

                axis = atoi(token);
                pwm_on((Uint16) axis);
                return (1);
        }
        if (strcmp(token, "eewrite") == 0)
        {
                token = strtok(NULL, s);
                eeaddress = atoi(token);    // address is 0 - 1022 these are byte addresses only even numbers are valid
                token = strtok(NULL, s);
                eedata = atoi(token);       // data is 16 bits and must be on a even address.
                eewrite((Uint16)eeaddress, (Uint16)eedata);
                return (1);

        }


        return (0);
    }

}

char* itoa( char *output_buffer , Uint16 value)
{
    int i,it;
    Uint16 temp;
    const char* string1 = "ADC reading =";
    const char* itoa_s = "0123456789";
    for (i=0; (string1[i] != '\x00'); i++)
    {
        output_buffer[i] = string1[i];
    }
    output_buffer[i] = '\x20';
    it = i;
    for (i=i+5; i>it; i--)
    {
        temp = value % 10;
        output_buffer[i] = itoa_s[temp];
        value = (value - temp)/10;
    }
    output_buffer[i+6] = '\x00';
    return (output_buffer);
}

