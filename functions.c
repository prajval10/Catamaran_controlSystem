/*
 * File:   functions.c
 * Author: Prajval
 *
 * Created on 18 December, 2017, 4:17 PM
 */


#include "xc.h"
#include "functions.h"
#include <stdio.h>
#include <string.h>
#include <p30F4011.h>


// Macros
#define DEFAULT_MIN  (-8000)
#define DEFAULT_MAX  (8000)
#define BUFFER_SIZE (60)

void enable_SPI() {
    SPI1CONbits.MSTEN = 1; //Master mode
    SPI1CONbits.MODE16 = 0; //8-bit mode
    SPI1CONbits.PPRE = 3; //1:1 primary pre-scaler
    SPI1CONbits.SPRE = 3; //5:1 secondary pre-scaler
    SPI1STATbits.SPIEN = 1; //Enable SPI
}

void enable_ADC() {
    ADPCFG = 0xFFFF; //Set all bits to digital
    ADPCFGbits.PCFG3 = 0; //setting temperature sensor to analog
    ADCON3bits.SAMC = 16; //Sample time 16 Tad
    ADCON3bits.ADCS = 63; // 32*Tcy
    ADCON2bits.CHPS = 0; //Selects Channel CH0  
    ADCON1bits.SSRC = 7; //conversion starts after time specified by SAMC
    ADCON1bits.ASAM = 1; //automatic start - automatic end
    ADCON2bits.CSCNA = 0; // Switch off Scan mode
    ADCHSbits.CH0SA = 3; //channel 0 positive input is AN3
    ADCHSbits.CH0NA = 0; //channel 0 negative input is VREF-
    ADCON1bits.ADON = 1; //Turn on ADC Module
}

//Function waits until the SPI Transmit Buffer is empty

void wait_buffer() {
    while (SPI1STATbits.SPITBF) {
    }
    return;
}

//Function to clear screen and point cursor to start position

void clear_screen(int kill) {
    char clr = ' ';

    if (kill == 1) {
        wait_buffer();
        SPI1BUF = 0x80; //Point cursor to start position of LCD
        tmr2_wait_ms(1);
    } else if (kill == 2) {
        wait_buffer();
        SPI1BUF = 0xC0; //Point cursor to start position of LCD
        tmr2_wait_ms(1);
    }
    int j;
    for (j = 0; j < 16; j++) {
        wait_buffer();
        SPI1BUF = clr;
    }
    wait_buffer();
    SPI1BUF = 0x80;
    tmr2_wait_ms(1);
    return;
}


// Function to write the number of seconds elapsed on the LCD screen

void write_temp_screen(double adc) {
    char str[100]; //Declare character string of dimension 100
    sprintf(str, "Temperature= %f", adc); //Typecast integer to char type 
    clear_screen(1); //clear the LCD screen
    int i = 0;
    wait_buffer();
    SPI1BUF = 0x80; //Point cursor to start position of LCD
    tmr2_wait_ms(1);
    while (str[i] != '\0') {
        wait_buffer();
        SPI1BUF = str[i]; //Write the temperature on the LCD
        i++;
    }
    return;
}

void tmr2_wait_ms(int ms) {
    T2CONbits.TON = 0;
    int tckps, pr;
    choose_prescaler(ms, &tckps, &pr);
    TMR2 = 0;
    T2CONbits.TCKPS = tckps;
    PR2 = pr;
    T2CONbits.TON = 1;
    while (IFS0bits .T2IF == 0) {
    }
    IFS0bits .T2IF = 0;
    T2CONbits.TON = 0;
}

//Function to choose the correct pre-scaler

void choose_prescaler(int ms, int* tckps, int* pr) {
    // Fcy = 2MHz ?> 2000 clock ticks in 1 ms
    long ticks = 1843L * ms; // notice 1843L to avoid int overflow (2000L for 8Mhz osc))
    if (ticks <= 65535) { // if ticks is > 65535 it cannot be put in PR1 (only 16 bits )
        *tckps = 0;
        *pr = ticks;
        return;
    }
    ticks = ticks / 8; // prescaler 1:8;
    if (ticks <= 65535) {
        *tckps = 1;
        *pr = ticks;
        return;
    }
    ticks = ticks / 8; // prescaler 1:64;
    if (ticks <= 65535) {
        *tckps = 2;
        *pr = ticks;
        return;
    }
    ticks = ticks / 4; // prescaler 1:256;
    *tckps = 3;
    *pr = ticks;
    return;
}

// Drive the motor given the input values

void motor_command(volatile velocity_command* fdata,volatile velocity_ref* rdata) {
    PWMCON1bits.PEN2H = 1; //PWM 2H pin is enabled for PWM output
    PWMCON1bits.PEN3H = 1; //PWM 3H pin is enabled for PWM output
    PTCONbits.PTMOD = 0; //PWM time base operates in a free running mode
    PTMRbits.PTDIR = 0; //PWM time base is counting up
    PTCONbits.PTCKPS = 0; // Pre-scalar is 1:1
    PTPER = 1842; //setting PTPER value: PTPER = Fcy/(Fpwm*pre-scal)-1: (((7372800)/4)/(1000*1))-1
    PTCONbits.PTOPS = 0; //1:1 post-scaling
    PTCONbits.PTSIDL = 0; //PWM runs in idle mode
    PTCONbits.PTEN = 1; //Switch on the PWM time base module

    //Saturating the max allowed rpm
    int max_rpm = rdata->max;
    int min_rpm = rdata->min;

    if (rdata->max > 8000)
        rdata->max = 8000;
    if (rdata->min < -8000)
        rdata->min = -8000;

    // Saturating the vehicle commands
    if (fdata->n1 < min_rpm)
        fdata->n1 = min_rpm;
    if (fdata->n1 > max_rpm)
        fdata->n1 = max_rpm;
    if (fdata->n2 < min_rpm)
        fdata->n2 = min_rpm;
    if (fdata->n2 > max_rpm)
        fdata->n2 = max_rpm;

    //left motor control

    PDC2 = 0.1842 * (fdata->n1) + 1842;

    //right motor control
    PDC3 = 0.1842 * (fdata->n2) + 1842;

    
    return;
}

void tmr1_setup_period(int ms) {
    T1CONbits.TON = 0; //Stop Timer 1
    TMR1 = 0; // reset the current value;
    T1CON = 0; //Clear all bits of Timer T1
    IFS0bits.T1IF = 0; //Clearing Timer1 Flag
    T1CONbits.TCS = 0; //Using the internal clock

    int tckps, pr;
    choose_prescaler(ms, &tckps, &pr); //choose the prescaler as required depending on the minimum timing period. 
    T1CONbits.TCKPS = tckps; //Set the prescaler
    PR1 = pr; //Set the value the timer must count to
    T1CONbits.TON = 1; //Start TIMER T1
    return;
}

void tmr3_setup_period(int ms) {
    T3CONbits.TON = 0; //Stop Timer 1
    TMR3 = 0; // reset the current value;
    T3CON = 0; //Clear all bits of Timer T1
    IFS0bits.T3IF = 0; //Clearing Timer1 Flag
    T3CONbits.TCS = 0; //Using the internal clock

    int tckps, pr;
    choose_prescaler(ms, &tckps, &pr); //choose the prescaler as required depending on the minimum timing period. 
    T3CONbits.TCKPS = tckps; //Set the prescaler
    PR3 = pr; //Set the value the timer must count to
    T3CONbits.TON = 1; //Start TIMER T1
    return;
}

void tmr1_wait_period() {
    while (IFS0bits.T1IF == 0) { //Poll on the flag until the timer expires
    }
    IFS0bits.T1IF = 0; //Reset the Timer Flag 
    return;
}

void safe_mode(volatile velocity_command* vel_safe) {
    //Stop motors
    velocity_ref ref_safe;
    vel_safe->n1 = 0;
    vel_safe->n2 = 0;
    ref_safe.max = DEFAULT_MAX;
    ref_safe.min = DEFAULT_MIN;
    motor_command(vel_safe, &ref_safe);
    return;
}

void timeout_mode(volatile velocity_command* vel_to) {
    //Stop motors
    velocity_ref ref_to;
    vel_to->n1 = 0;
    vel_to->n2 = 0;
    ref_to.max = DEFAULT_MAX;
    ref_to.min = DEFAULT_MIN;
    motor_command(vel_to, &ref_to);
    return;
}

void write_buffer(volatile CircularBuffer* cb, char value) {
    cb->buffer[cb->writeIndex] = value;
    cb->writeIndex++;
    if (cb->writeIndex == BUFFER_SIZE)
        cb->writeIndex = 0;
}

int read_buffer(volatile CircularBuffer* cb, char* value) {
    if (cb->readIndex == cb->writeIndex)
        return 0;
    *value = cb->buffer[cb->readIndex];
    cb->readIndex++;
    if (cb->readIndex == BUFFER_SIZE)
        cb->readIndex = 0;
    return 1;
}

int avl_in_buffer(volatile CircularBuffer* cb) {
    IEC1bits.U2RXIE = 0;//disable interrupt
    int wri = cb->writeIndex;//copy to local variables
    int rdi = cb->readIndex;
    IEC1bits.U2RXIE = 1;//re-enable interrupt
    if (wri >= rdi) {
        return wri - rdi;
    } else {
        return wri - rdi + BUFFER_SIZE;
    }
}

int extract_integer(const char* str) {
    int i = 0, number = 0, sign = 1;

    if (str[i] == '-') {
        sign = -1;
        i++;
    } else if (str[i] == '+') {
        sign = 1;
        i++;
    }
    while (str[i] != ',' && str[i] != '\0') {
        number *= 10; // multiply the current number by 10; 
        number += str[i] - '0'; // converting character to decimal number 
        i++;
    }
    return sign*number;
}

int next_value(const char* msg, int i) {
    while (msg[i] != ',' && msg[i] != '\0') {
        i++;
    }
    if (msg[i] == ',')
        i++;
    return i;
}

void parse_velc(const char* msg,volatile velocity_command* fdata) {
    int i = 0;
    fdata->n1 = extract_integer(msg);
    i = next_value(msg, i);
    fdata->n2 = extract_integer(msg + i);
}

void parse_velr(const char* msg,volatile velocity_ref* fdata) {
    int i = 0;
    fdata->min = extract_integer(msg);
    i = next_value(msg, i);
    fdata->max = extract_integer(msg + i);
}

void write_screen_command(volatile velocity_command* fdata) {
    char str[100]; //Declare character string of dimension 100
    sprintf(str, "n1=%d n2=%d", fdata->n1, fdata->n2); //Typecast integer to char type 
    clear_screen(2); //clear the LCD screen
    int i = 0;
    wait_buffer();
    SPI1BUF = 0xC0; //Point cursor to start position of LCD
    tmr2_wait_ms(1);
    while (str[i] != '\0') {
        wait_buffer();
        SPI1BUF = str[i]; //Write the temperature on the LCD
        i++;
    }
    return;
}

void write_screen_ref(volatile velocity_ref* fdata) {
    char str[100]; //Declare character string of dimension 100
    sprintf(str, "min=%d max=%d", fdata->min, fdata->max); //Typecast integer to char type 
    clear_screen(2); //clear the LCD screen
    int i = 0;
    wait_buffer();
    SPI1BUF = 0xC0; //Point cursor to start position of LCD
    tmr2_wait_ms(1);
    while (str[i] != '\0') {
        wait_buffer();
        SPI1BUF = str[i]; //Write the temperature on the LCD
        i++;
    }
    return;
}

void write_screen(int left, int right) {
    char str[100]; //Declare character string of dimension 100
    sprintf(str, "L=%d R=%d", left, right); //Typecast integer to char type 
    clear_screen(1); //clear the LCD screen
    int i = 0;
    wait_buffer();
    SPI1BUF = 0x80; //Point cursor to start position of LCD
    tmr2_wait_ms(1);
    while (str[i] != '\0') {
        wait_buffer();
        SPI1BUF = str[i]; //Write the temperature on the LCD
        i++;
    }
    return;
}

// Messages to PC: MCFBK - n1,n2 and state and MCTEM - Temperature

void msg2PC(double temp,volatile velocity_command* pc_vel, volatile int state) {
    char str1[100], str2[100];
    sprintf(str1, "$MCFBK,%d,%d,%d*", pc_vel->n1, pc_vel->n2, state);
    sprintf(str2, "$MCTEM,%f*", temp);
    int i = 0;
    int j = 0;
    while (str1[i] != '\0') {
        if (U2STAbits.UTXBF == 0) {
            U2TXREG = str1[i];
            i++;
        }
    }
    while (str2[j] != '\0') {
        if (U2STAbits.UTXBF == 0) {
            U2TXREG = str2[j];
            j++;
        }
    }
    return;
}