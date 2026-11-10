/*
 * File:   Robot.c
 * Author: ryanyip
 *
 * Created on October 12, 2026, 4:18 PM
 */

/*
 * PINOUT
 *   2 (RA0 DO) Motor R Direction
 *   3 (RA1 DO) Motor L Direction
 *   4 (OC2) Motor R PWM
 *   5 (OC3) Motor L PWM
 *   6 (RB2 - AN4) left distance sensor
 *   7 (RA2 - AN13) center distance sensor
 *   8 (RA3 - AN14) right distance sensor
 *   9 N/A
 *   10 N/A
 *   11 (RB7 - DO) laser
 *   12 (RB8 - DO) left motor direction
 *   13 (RB9 - DO) right motor direction
 *   14 (OC1) Servo Motors PWM
 *   15 (AN12) Ball Color QRD
 *   16 (AN11) Right line following QRD
 *   17 (AN10) Left line following QRD
 *   18 (AN9) Center line following QRD
 *   19 Ground
 *   20 Power with 12V
 */
 

#include "xc.h"
#pragma config FNOSC = FRC
#pragma config ICS = PGx3

enum { LINEFOLLOW, CANYONNAV } mode;
enum { FORWARD, BACK, WAITING, TURNRIGHT, TURNLEFT, TURNAROUND, ADJUSTRIGHT, ADJUSTLEFT, FOLLOWLINE } state;

int numStepsTaken = 0;
int lineFollowingThreshold = 3000;
int wallThreshold = 1750;

void SetPins();
void Config_ADC();
void SetTimer();
void MotorRForward();
void MotorLForward();
void MotorRBack();
void MotorLBack();
void MotorROff();
void MotorLOff();
void __attribute__((interrupt, no_auto_psv)) _OC2Interrupt(void);

int main(void) {
    SetPins();
    Config_ADC();

    // Set up the interrupt on OC2
    _OC2IP = 4; // Select the interrupt priority
    _OC2IE = 1; // Enable the OC2 interrupt
    _OC2IF = 0; // Clear the interrupt flag

    T1CONbits.TON = 1;
    T1CONbits.TCS = 0;
    T1CONbits.TCKPS = 0b10;

    mode = CANYONNAV;
    state = FORWARD;
    MotorRForward();
    MotorLForward();
    
    int turned90 = 0;

    while (1) {
        switch (mode) {
            
            case LINEFOLLOW: 
                switch (state) {

                    case FOLLOWLINE:
                        if (ADC1BUF10 <= lineFollowingThreshold)
                        {
                            MotorLOff();
                            state = ADJUSTLEFT;
                            break;
                        }
                        else if (ADC1BUF11 <= lineFollowingThreshold)
                        {
                            MotorROff();
                            state = ADJUSTRIGHT;
                            break;
                        }
                        break;

                    case ADJUSTRIGHT:
                        if (ADC1BUF9 <= lineFollowingThreshold)
                        {
                            MotorRForward();
                            state = FOLLOWLINE;
                            break;
                        }
                        else if (ADC1BUF10 <= lineFollowingThreshold)
                        {
                            MotorRForward();
                            MotorLOff();
                            state = ADJUSTLEFT;
                            break;
                        }
                        break;

                    case ADJUSTLEFT:
                        if (ADC1BUF9 <= lineFollowingThreshold)
                        {
                            MotorLForward();
                            state = FOLLOWLINE;
                            break;
                        }
                        else if (ADC1BUF11 <= lineFollowingThreshold)
                        {
                            MotorLForward();
                            MotorROff();
                            state = ADJUSTRIGHT;
                            break;
                        }
                        break;
                    break;
                    
                case CANYONNAV:
                    case FORWARD: ;
                        if (ADC1BUF13 >= wallThreshold) {
                            MotorROff();
                            MotorLOff();
                            state = WAITING;
                            break;
                        }
                        break;
                        
                    case WAITING:
                        if (ADC1BUF4 > ADC1BUF14) {
                            MotorRBack();
                            MotorLForward();
                            numStepsTaken = 0;
                            state = TURNRIGHT;
                            break;
                        }
                        else if (ADC1BUF14 > ADC1BUF4) {
                            MotorRForward();
                            MotorLBack();
                            numStepsTaken = 0;
                            state = TURNLEFT;
                            break;
                        }
                        break;
                        
                    case TURNRIGHT: ;
                        int numStepsUntilForward = 300;
                        if (numStepsTaken == numStepsUntilForward) {
                            MotorRForward();
                            MotorLForward();
                            state = FORWARD;
                            numStepsTaken = 0;
                            break;
                        }
                        break;
                        
                    case TURNLEFT: ;
                        if (numStepsTaken == numStepsUntilForward) {
                            MotorRForward();
                            MotorLForward();
                            state = FORWARD;
                            numStepsTaken = 0;
                            break;
                        }
                        break;
                    break;
            }
        }
        
    }

    
}

void Config_ADC(void) {
    _ADON = 0;    // Disable A/D module during configuration
    
    // AD1CON1
    _MODE12 = 1;  // 12-bit resolution
    _FORM = 0;    // unsigned integer output
    _SSRC = 7;    // auto convert
    _ASAM = 1;    // auto sample

    // AD1CON2
    _PVCFG = 0;   // use VDD as positive reference
    _NVCFG = 0;   // use VSS as negative reference
    _BUFREGEN = 1;// store results in buffer corresponding to channel number
    _CSCNA = 1;   // scanning mode
    _SMPI = 0b00101;    // begin new sampling sequence after every *insert num of A2D pins* samples -- num pins - 1 = binary
    _ALTS = 0;    // sample MUXA only

    // AD1CON3
    _ADRC = 0;    // use system clock
    _SAMC = 1;    // sample every A/D period
    _ADCS = 0x3F; // TAD = 64*TCY

    // AD1CSS -- Choose which channel/pin to scan
    _CSS9 = 1; // pin 18 (center line following)
    _CSS10 = 1; // pin 17 (left line following)
    _CSS11 = 1; // pin 16 (right line following)
    _CSS4 = 1; // pin 6 (left sharp sensor)
    _CSS13 = 1; // pin 7 (center sharp sensor)
    _CSS14 = 1; // pin 8 (right sharp sensor)
    
    _ADON = 1;    // enable module

}

void SetPins() {
    // turn off analog

    // digital output
    _TRISB8 = 0;
    _TRISB9 = 0;

    // configure OC2 & OC3
    OC2CON1 = 0;
    OC2CON2 = 0;
    OC2CON1bits.OCTSEL = 0b111;
    OC2CON2bits.SYNCSEL = 0b11111;
    OC2CON2bits.OCTRIG = 0;
    OC2RS = 6000;
    OC2R = 3000;
    OC2CON1bits.OCM = 0b110; // change to 0 to turn off
    
    OC3CON1 = 0;
    OC3CON2 = 0;
    OC3CON1bits.OCTSEL = 0b111;
    OC3CON2bits.SYNCSEL = 0b11111;
    OC3CON2bits.OCTRIG = 0;
    OC3RS = 6000;
    OC3R = 3000;
    OC3CON1bits.OCM = 0b110; // change to 0 to stop PWM
    
    // line following pins
    _ANSB13 = 1;
    _TRISB13 = 1;
    _ANSB14 = 1;
    _TRISB14 = 1;
    _ANSB15 = 1;
    _TRISB15 = 1;
    
    //  sharp sensor pins
    _ANSB2 = 1;
    _TRISB2 = 1;
    _ANSA2 = 1;
    _TRISA2 = 1;
    _ANSA3 = 1;
    _TRISA3 = 1;
}

void MotorRForward() {
    _LATB9 = 1;
    OC2RS = 6900;
    OC2R = 3450;
    OC2CON1bits.OCM = 0b110;
}

void MotorRBack() {
    _LATB9 = 0;
    OC2RS = 6900;
    OC2R = 3450;
    OC2CON1bits.OCM = 0b110;
}

void MotorROff() {
    OC2CON1bits.OCM = 0;
}

void MotorLForward() {
    _LATB8 = 1;
    OC3RS = 6900;
    OC3R = 3450;
    OC3CON1bits.OCM = 0b110;
}

void MotorLBack() {
    _LATB8 = 0;
    OC3RS = 6900;
    OC3R = 3450;
    OC3CON1bits.OCM = 0b110;
}

void MotorLOff() {
    OC3CON1bits.OCM = 0;
}

void __attribute__((interrupt, no_auto_psv)) _OC2Interrupt(void)
{
    _OC2IF = 0; // Clear the interrupt flag
    numStepsTaken++;
}
