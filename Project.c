/*
 * File:   project.c
 * Author: Prajval Kumar MURALI and Luca Macchiusi
 * Student ID: Prajval : S4546126
 * Student ID: Luca : S3973554
 * Created on 13 December, 2017, 10:41 AM
 */


// DSPIC30F4011 Configuration Bit Settings

// 'C' source line config statements

// 'C' source line config statements

// FOSC
#pragma config FPR = XT                 // Primary Oscillator Mode (XT)
#pragma config FOS = PRI                // Oscillator Source (Primary Oscillator)
#pragma config FCKSMEN = CSW_FSCM_OFF   // Clock Switching and Monitor (Sw Disabled, Mon Disabled)

// FWDT
#pragma config FWPSB = WDTPSB_16        // WDT Prescaler B (1:16)
#pragma config FWPSA = WDTPSA_512       // WDT Prescaler A (1:512)
#pragma config WDT = WDT_OFF            // Watchdog Timer (Disabled)

// FBORPOR
#pragma config FPWRT = PWRT_64          // POR Timer Value (64ms)
#pragma config BODENV = BORV20          // Brown Out Voltage (Reserved)
#pragma config BOREN = PBOR_ON          // PBOR Enable (Enabled)
#pragma config LPOL = PWMxL_ACT_HI      // Low-side PWM Output Polarity (Active High)
#pragma config HPOL = PWMxH_ACT_HI      // High-side PWM Output Polarity (Active High)
#pragma config PWMPIN = RST_IOPIN       // PWM Output Pin Reset (Control with PORT/TRIS regs)
#pragma config MCLRE = MCLR_EN          // Master Clear Enable (Enabled)

// FGS
#pragma config GWRP = GWRP_OFF          // General Code Segment Write Protect (Disabled)
#pragma config GCP = CODE_PROT_OFF      // General Segment Code Protection (Disabled)

// FICD
#pragma config ICS = ICS_PGD            // Comm Channel Select (Use PGC/EMUC and PGD/EMUD)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdio.h>
#include <string.h>
#include <p30F4011.h>
#include "functions.h"

// Macros
#define DEFAULT_MIN  (-8000)
#define DEFAULT_MAX  (8000)
#define SAFE_MODE (2)
#define TIMEOUT_MODE (1)
#define NORMAL_MODE (0)

// Global Variables
double temperature;
//shared global variable protection
volatile CircularBuffer cb;
volatile int SafeModeFlag = 0; 
volatile int stateStatus = NORMAL_MODE;
volatile velocity_command pc_vel;
volatile velocity_ref pc_ref;

// Interrupt Service Routines

void __attribute__((__interrupt__, __auto_psv__)) _INT0Interrupt() {
    IFS0bits.INT0IF = 0; //Clear the Interrupt Flag
    SafeModeFlag = 1; // Set flag to ignore incoming reference values
    safe_mode(&pc_vel);
    stateStatus = SAFE_MODE;
};

void __attribute__((__interrupt__, __auto_psv__)) _INT1Interrupt() {
    IFS1bits.INT1IF = 0; //Clear the Interrupt Flag
    SafeModeFlag = 1;
    safe_mode(&pc_vel);
    stateStatus = SAFE_MODE;
};

void __attribute__((__interrupt__, __auto_psv__)) _U2RXInterrupt() {
    IFS1bits.U2RXIF = 0;
    char val = U2RXREG;
    write_buffer(&cb, val);
}

void __attribute__((__interrupt__, __auto_psv__)) _T3Interrupt() {
    IFS0bits.T3IF = 0;
    timeout_mode(&pc_vel);
    if (stateStatus != SAFE_MODE) {
        stateStatus = TIMEOUT_MODE;
    }
}

int main(void) {
    // parser initialization
    parser_state pstate;
    pstate.state = STATE_DOLLAR;
    pstate.index_type = 0;
    pstate.index_payload = 0;

    //UART 
    cb.writeIndex = 0;
    cb.readIndex = 0;

    pc_ref.max = DEFAULT_MAX;
    pc_ref.min = DEFAULT_MIN;

    enable_SPI();//used to print debug messages on LCD
    tmr2_wait_ms(100);
    enable_ADC();//For temperature sensor
    tmr2_wait_ms(100);

    clear_screen(1);
    clear_screen(2);

    TRISBbits.TRISB0 = 0; // output (led)
    TRISBbits.TRISB1 = 0; // output (led)

    //Interrupts
    IEC0bits.INT0IE = 1; //Enable External interrupt INT0  
    IEC1bits.INT1IE = 1; //Enable External interrupt INT1  
    IEC0bits.T3IE = 1; //Enable Interrupt for Timer 3

    //Enable UART2. UART1 pins are shared with SPI so we use UART2
    U2BRG = 11; //(7372800/4)/(16*9600)-1

    U2MODEbits.UARTEN = 1; //Enable UART
    IEC1bits.U2RXIE = 1; //Enable UART Receiver interrupt
    U2STAbits.URXISEL = 0; //an interrupt is generated each time a data word is
    //transferred from the Receive Shift register (UxRSR) to the receive buffer. There may be
    //one or more characters in the receive buffer.
    U2STAbits.UTXEN = 1; //Enable U2TX 

    int i;
    tmr1_setup_period(1000);
    tmr3_setup_period(5000);// For timeout mode

    while (1) {
        while (ADCON1bits.DONE == 0);
        int adc_val = ADCBUF0;
        temperature = ((adc_val / 1023.0 * 5.0) - 0.5) * 100;
        
        char byte;
        int avl = avl_in_buffer(&cb);

        if (avl > 0) {
            for (i = 0; i < avl; i++) {
                read_buffer(&cb, &byte);
                int ret = parse_byte(&pstate, byte);
                if (ret == NEW_MESSAGE) {
                    TMR3 = 0; //Reset timer 3 for timeout mode counter
                    LATBbits.LATB1 = 0; //Stop blinking 
                    if (SafeModeFlag == 0) {
                        stateStatus = NORMAL_MODE;
                        // check if the message received was HLREF
                        // if yes, parse the message and extract the values of the sensors
                        if (strcmp(pstate.msg_type, "HLREF") == 0) {
                            parse_velc(pstate.msg_payload, &pc_vel);
                            //write_screen_command(&pc_vel);
                            motor_command(&pc_vel, &pc_ref);

                        } else if (strcmp(pstate.msg_type, "HLSAT") == 0) {
                            parse_velr(pstate.msg_payload, &pc_ref);
                        }
                    } else if (SafeModeFlag == 1) {
                        if (strcmp(pstate.msg_type, "HLENA") == 0) {
                            SafeModeFlag = 0;
                            stateStatus = NORMAL_MODE;
                        }
                    }
                }
            }
        }
        LATBbits.LATB0 = !LATBbits.LATB0;
        if (stateStatus == TIMEOUT_MODE && stateStatus != SAFE_MODE) {
            LATBbits.LATB1 = !LATBbits.LATB1;
        }
        msg2PC(temperature, &pc_vel, stateStatus);
        tmr1_wait_period();
    }
    return 0;
}
