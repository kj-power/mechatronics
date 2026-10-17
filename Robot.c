/*
 * File:   Robot.c
 * Author: ryanyip
 *
 * Created on October 12, 2026, 4:18 PM
 */

/*
 * PINOUT
     2 (RA0) Motor 1 Direction
     3 (RA1) Motor 2 Direction
     4 (OC2) Motor 1 PWM
     5 (OC3) Motor 2 PWM
 */


#include "xc.h"
#pragma config FNOSC = FRC
#pragma config ICS = PGx3

enum { FORWARD, BACK, WAITING, TURNRIGHT, TURNLEFT, TURNAROUND } state;
int numStepsTaken = 0;

void SetPins();
void SetTimer();
void Motor1Forward();
void Motor2Forward();
void Motor1Back();
void Motor2Back();
void Motor1Off();
void Motor2Off();
void __attribute__((interrupt, no_auto_psv)) _OC2Interrupt(void);

int main(void) {
    SetPins();

    // Set up the interrupt on OC2
    _OC2IP = 4; // Select the interrupt priority
    _OC2IE = 1; // Enable the OC2 interrupt
    _OC2IF = 0; // Clear the interrupt flag

    T1CONbits.TON = 1;
    T1CONbits.TCS = 0;
    T1CONbits.TCKPS = 0b10;

    state = FORWARD;
    Motor1Forward();
    Motor2Forward();

    while (1) {
        switch (state) {
            case FORWARD: ;
                int numStepsUntilRight = 1200;
                int numStepsUntil180 = 2600;
                int numStepsUntilStop = 4200;
                if (numStepsTaken == numStepsUntilRight) {
                    Motor1Forward();
                    Motor2Back();
                    state = TURNRIGHT;
                    break;
                }
                
                if (numStepsTaken == numStepsUntil180) {
                    Motor1Forward();
                    Motor2Back();
                    state = TURNAROUND;
                    break;
                }

                if (numStepsTaken == numStepsUntilStop) {
                    Motor1Off();
                    Motor2Off();
                    TMR1 = 0;
                    break;
                }
                break;

            case BACK:
                break;

            case WAITING:
                if (TMR1 = 10000) {
                    OC2CON1bits.OCM = 0b110;
                    OC3CON1bits.OCM = 0b110;
                    Motor1Forward();
                    Motor2Forward();
                    numStepsTaken = 0;
                    state = FORWARD;
                }
                break;

            case TURNRIGHT: ;
                int numStepsUntilForward = 1400;
                if (numStepsTaken == numStepsUntilForward) {
                    Motor1Forward();
                    Motor2Forward();
                    state = FORWARD;
                }
                break;

            case TURNLEFT:
                break;

            case TURNAROUND:
                numStepsUntilForward = 3000;
                if (numStepsTaken == numStepsUntilForward) {
                    Motor1Forward();
                    Motor2Forward();
                    state = FORWARD;
                }
                break;
    
        }
    }

    
}

void SetPins() {
    // turn off analog
    _ANSA0 = 0;
    _ANSA1 = 0;

    // digital output
    _TRISA0 = 0;
    _TRISA1 = 0;

    // configure OC2 & OC3
    OC2CON1 = 0;
    OC2CON2 = 0;
    OC2CON1bits.OCTSEL = 0b111;
    OC2CON2bits.SYNCSEL = 0b11111;
    OC2CON2bits.OCTRIG = 0;
    OC2RS = 5000;
    OC2R = 2500;
    OC2CON1bits.OCM = 0b110; // change to 0 to turn off
    
    OC3CON1 = 0;
    OC3CON2 = 0;
    OC3CON1bits.OCTSEL = 0b111;
    OC3CON2bits.SYNCSEL = 0b11111;
    OC3CON2bits.OCTRIG = 0;
    OC3RS = 5000;
    OC3R = 2500;
    OC3CON1bits.OCM = 0b110; // change to 0 to stop PWM
}

void Motor1Forward() {
    _LATA0 = 1;
    OC2RS = 5000;
    OC2R = 2500;
}

void Motor1Back() {
    _LATA0 = 0;
    OC2RS = 5000;
    OC2R = 2500;
}

void Motor1Off() {
    OC2CON1bits.OCM = 0;
}

void Motor2Forward() {
    _LATA1 = 1;
    OC3RS = 5000;
    OC3R = 2500;
}

void Motor2Back() {
    _LATA1 = 0;
    OC3RS = 5000;
    OC3R = 2500;
}

void Motor2Off() {
    OC3CON1bits.OCM = 0;
}

void __attribute__((interrupt, no_auto_psv)) _OC2Interrupt(void)
{
    _OC2IF = 0; // Clear the interrupt flag
    numStepsTaken++;
}
