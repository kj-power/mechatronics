/*
 * File:   Robot.c
 * Author: ryanyip
 *
 * Created on October 12, 2026, 4:18 PM
 */

/*
 * PINOUT
     2 (RA0) Motor 1 Direction 1
     3 (RA1) Motor 1 Direction 2
     6 (RB2) Motor 2 Direction 1
     7 (RA2) Motor 2 Direction 2
     4 (OC2) Motor 1 PWM
     5 (OC3) Motor 2 PWM
 */


#include "xc.h"
#pragma config FNOSC = FRCDIV
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
    _RCDIV = 0b010;

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
            case FORWARD:
                numStepsUntilRight = 1200;
                numStepsUntil180 = 2800;
                numStepsUntilStop = 4800;
                if (numStepsTaken == numStepsUntilRight) {
                    Motor1Forward();
                    Motor2Backward();
                    state = TURNRIGHT;
                    break;
                }
                
                if (numStepsTaken == numStepsUntil180) {
                    Motor1Forward();
                    Motor2Backward();
                    state = TURNAROUND;
                    break;
                }

                if (numStepsTaken == numStepsUntilStop) {
                    Motor1Off();
                    Motor2Off();
                    TMR1 = 0;
                    state = WAITING;
                    break;
                }
                
                break;

            case BACKWARD:
                break;

            case WAITING:
                if (TMR1 >= 10000) {
                    Motor1Forward();
                    Motor2Forward();
                    state = FORWARD;
                    numStepsTaken = 0;
                }
                break;

            case TURNRIGHT:
                numStepsUntilForward = 1600;
                if (numStepsTaken == numStepsUntilForward) {
                    Motor1Forward();
                    Motor2Forward();
                    state = FORWARD;
                }
                break;

            case TURNLEFT:
                break;

            case TURNAROUND:
                numStepsUntilForward = 3600;
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
    _ANSB2 = 0;
    _ANSA2 = 0;

    // digital output
    _TRISA0 = 0;
    _TRISA1 = 0;
    _TRISB2 = 0;
    _TRISA2 = 0;

    // configure OC2 & OC3
    OC2CON1 = 0x1C06;
    OC2CON2 = 0x001F;
    OC2CON1 = 0x1C06;
    OC2CON2 = 0x001F;
}

void Motor1Forward() {
    _LATA0 = 1;
    _LATA1 = 0;
    OC2RS = 5000;
    OC2R = 2500;
}

void Motor1Back() {
    _LATA0 = 0;
    _LATA1 = 1;
    OC2RS = 5000;
    OC2R = 2500;
}

void Motor1Off() {
    _LATA0 = 0;
    _LATA1 = 0;
}

void Motor2Forward() {
    _LATB2 = 1;
    _LATA2 = 0;
    OC3RS = 5000;
    OC3R = 2500;
}

void Motor2Back() {
    _LATB2 = 0;
    _LATA2 = 1;
    OC3RS = 5000;
    OC3R = 2500;
}

void Motor1Off() {
    _LATB2 = 0;
    _LATA2 = 0;
}

void __attribute__((interrupt, no_auto_psv)) _OC2Interrupt(void)
{
    _OC2IF = 0; // Clear the interrupt flag
    numberOfStepsTaken++;
}
