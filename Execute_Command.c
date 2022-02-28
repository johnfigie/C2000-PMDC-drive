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
#include "PMDC_Drive.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
extern void set_pwm(Uint16 axis, Uint16 pwm_value);
extern void pulse_pwm(Uint16 axis, Uint16 pwm_value, Uint16 npulses);
extern void pulse_cref(Uint16 axis, int16 cref_value,Uint16 npulses);
extern void scia_msg(char *);
extern void pwm_on(Uint16 axis);
extern void pwm_off(Uint16 axis);
extern void pwm_off_message(Uint16 axis);
extern void eewrite(Uint16 eeaddress, Uint16 eedata);
extern Uint16 eeread(Uint16 eeaddress);
extern Uint16 ADCch[6];
extern struct PID axis1,axis2;
//function prototypes
char* itoa( char * , int32);
char* itoas( char * , int16);
char output_buffer[80];

int ExecuteCommand(char *ReceivedLine)
{
int axis = 0;
struct PID *axisp;
int pwm_mag,npulses,pulse_pwm_mag;
Uint16 eeaddress, eedata;
char* token;
const char s[2] = " ";
const char t[2] = "\015";


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
            itoas(output_buffer, ADCch[axis]);
            scia_msg("\r\nADC read");
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
                pwm_off_message((Uint16) axis);
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
                eeaddress = atoi(token);    // address is 0 - 1022 these are byte addresses for 16b it words, only even numbers are valid
                token = strtok(NULL, s);
                if (token == '\x00') return (0);
                eedata = atoi(token);       // data is 16 bits and must be on a even address.
                eewrite((Uint16)eeaddress, (Uint16)eedata);
                return (1);

        }
        if (strcmp(token, "eeread") == 0)
        {
            token = strtok(NULL, s);

            eeaddress = atoi(token);       // address is 0 - 1022 these are byte addresses for 16b it words, only even numbers are valid
            itoas(output_buffer, eeread((Uint16)eeaddress));
            scia_msg("\r\n");
            scia_msg(output_buffer);
            return (1);

        }
        if (strcmp(token, "p_cref") == 0)

        {
                token = strtok(NULL, s);
                axis = atoi(token);
                token = strtok(NULL, s);
                pulse_pwm_mag = atoi(token);
                token = strtok(NULL, s);
                if (token){
                    npulses = atoi(token);
                    scia_msg("\r\nPulsing CREF");
                    pulse_cref((Uint16) axis,(int16) pulse_pwm_mag, (Uint16) npulses);
                    return (1);
                }
                else return(0);

        }

        if (strcmp(token, "r_axis") == 0)

        {
            token = strtok(NULL, s);   // setup pointer to axis structure
            axis = atoi(token);
            switch (axis)
            {
                case 1:
                    axisp = &axis1;
                    break;
                case 2:
                    axisp = &axis2;
                    break;
                default:
                    return(0);
            }


            token = strtok(NULL, t);
            if (strcmp(token, "pgain") == 0){
                itoas(output_buffer, axisp->pgain);    // to do: make this actually print the
                scia_msg("\r\naxisx.pgain");          // real axis number in the message.
                scia_msg(output_buffer);
                return (1);
            }
            if (strcmp(token, "igain") == 0){
                itoas(output_buffer, axisp->igain);
                scia_msg("\r\naxisx.igain");
                scia_msg(output_buffer);
                return (1);
            }
            if (strcmp(token, "ff") == 0){
                itoas(output_buffer, axisp->ff);
                scia_msg("\r\naxisx.ff");
                scia_msg(output_buffer);
                return (1);
            }
            if (strcmp(token, "fb_current") == 0){
                itoas(output_buffer, axisp->fb_current);
                scia_msg("\r\naxisx.fb_current");
                scia_msg(output_buffer);
                return (1);
            }
            if (strcmp(token, "pwm") == 0){
                itoas(output_buffer, (int16)axisp->pwm);
                scia_msg("\r\naxisx.pwm");
                scia_msg(output_buffer);
                return (1);
            }
            if (strcmp(token, "loop_mode") == 0){
                itoas(output_buffer, (int16)axisp->loop_mode);
                scia_msg("\r\naxisx.loop_mode");
                scia_msg(output_buffer);
                return (1);
            }
            if (strcmp(token, "input_mode") == 0){
                itoas(output_buffer, (int16)axisp->input_mode);
                scia_msg("\r\naxisx.input_mode");
                scia_msg(output_buffer);
                return (1);
            }
            if (strcmp(token, "cref") == 0){
                itoas(output_buffer, axisp->cref);
                scia_msg("\r\naxisx.cref");
                scia_msg(output_buffer);
                return (1);
            }
            if (strcmp(token, "cref_limit") == 0){
                itoas(output_buffer, axisp->cref_limit);
                scia_msg("\r\naxisx.cref_limit");
                scia_msg(output_buffer);
                return (1);
            }
            if (strcmp(token, "current_limit") == 0){
                itoas(output_buffer, axisp->current_limit);
                scia_msg("\r\naxisx.current_limit");
                scia_msg(output_buffer);
                return (1);
            }
            if (strcmp(token, "fault") == 0){
                itoa(output_buffer, (int32)axisp->fault);  // make this more user friendly
                scia_msg("\r\naxisx.fault");
                scia_msg(output_buffer);
                return (1);
            }

            return (0);
        }

            if (strcmp(token, "w_axis") == 0)
            {
                token = strtok(NULL, s);
                axis = atoi(token);
                switch (axis)
                {
                    case 1:
                        axisp = &axis1;
                        break;
                    case 2:
                        axisp = &axis2;
                        break;
                    default:
                        return(0);
                }
                token = strtok(NULL, s);
                if (strcmp(token, "pgain") == 0){
                    token = strtok(NULL, s);
                    axisp->pgain = atoi(token);
                    return (1);
                }
                if (strcmp(token, "igain") == 0){
                    token = strtok(NULL, s);
                    axisp->igain = atoi(token);
                    return (1);
                }
                if (strcmp(token, "ff") == 0){
                    token = strtok(NULL, s);
                    axisp->ff = atoi(token);
                    return (1);
                }
                if (strcmp(token, "fb_current") == 0){
                    token = strtok(NULL, s);
                    axisp->fb_current = atoi(token);
                    return (1);
                }
                if (strcmp(token, "pwm") == 0){
                    token = strtok(NULL, s);
                    axisp->pwm = atoi(token);
                    return (1);
                }
                if (strcmp(token, "loop_mode") == 0){
                    token = strtok(NULL, s);
                    axisp->loop_mode = atoi(token);
                    return (1);
                }
                if (strcmp(token, "input_mode") == 0){
                    token = strtok(NULL, s);
                    axisp->input_mode = atoi(token);
                    return (1);
                }

                if (strcmp(token, "cref") == 0){
                    token = strtok(NULL, s);
                    axisp->cref = atoi(token);
                    return (1);
                }
                if (strcmp(token, "cref_limit") == 0){
                    token = strtok(NULL, s);
                    axisp->cref_limit = atoi(token);
                    return (1);
                }
                if (strcmp(token, "fault") == 0){
                    token = strtok(NULL, s);
                    axisp->fault = atoi(token);
                    return (1);
                }
                if (strcmp(token, "current_limit") == 0){
                    token = strtok(NULL, s);
                    axisp->current_limit = atoi(token);
                    return (1);
                }
                return (0);

        }



        return (0);
    }

}

char* itoa( char *output_buffer , int32 value)
{
    int i,k=0;
    Uint16 temp;
    char temp_buffer[10];
    const char* string1 = " =";
    const char* itoa_s = "0123456789";
    for (i=0; (string1[i] != '\x00'); i++)  // get output string ready
    {
        output_buffer[i] = string1[i];
    }
    output_buffer[i++] = '\x20';
    if (value < 0){
        output_buffer[i++] = '-';
        value = -value;
    }
    while(value>0){                // convert string
        temp = value % 10;
        temp_buffer[k++] = itoa_s[temp];
        value = (value - temp)/10;
    }
    if (k==0){                    // special case where value was 0
        output_buffer[i++] = '0';
    }
    while ( k > 0){               // reverse the converted string
        output_buffer[i++] = temp_buffer[--k];
    }
    output_buffer[i] = '\x00';
    return (output_buffer);
}

char* itoas( char *output_buffer , int16 value)
{
    int i,k=0;
    Uint16 temp;
    char temp_buffer[10];
    const char* string1 = " =";
    const char* itoa_s = "0123456789";
    for (i=0; (string1[i] != '\x00'); i++)  // get output string ready
    {
        output_buffer[i] = string1[i];
    }
    output_buffer[i++] = '\x20';
    if (value < 0){
        output_buffer[i++] = '-';
        value = -value;
    }
    while(value>0){                // convert string
        temp = value % 10;
        temp_buffer[k++] = itoa_s[temp];
        value = (value - temp)/10;
    }
    if (k==0){                    // special case where value was 0
        output_buffer[i++] = '0';
    }
    while ( k > 0){               // reverse the converted string
        output_buffer[i++] = temp_buffer[--k];
    }
    output_buffer[i] = '\x00';
    return (output_buffer);
}
