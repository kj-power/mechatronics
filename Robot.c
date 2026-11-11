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

// this is from git

#include "xc.h"
#pragma config FNOSC = FRC
#pragma config ICS = PGx3
#pragma config OSCIOFNC = OFF

#define LEFT_QRD ADC1BUF10 
#define RIGHT_QRD ADC1BUF11
#define CENTER_QRD ADC1BUF9
#define LEFT_SHARP ADC1BUF4 
#define RIGHT_SHARP ADC1BUF14
#define CENTER_SHARP ADC1BUF13

enum { LINEFOLLOW, CANYONNAV } mode;
enum { TURNAROUND, ADJUSTRIGHT, ADJUSTLEFT, ONLINE } linestate;
enum { FORWARD, BACK, WAITING, TURNRIGHT, TURNLEFT } canyonstate;

int numStepsTaken = 0;
const int numStepsFor90 = 540;
int canyon_exit = 0;

const int lineFollowingThreshold = 3300;
const int transitionThreshold = 3600;

const int wallThreshold = 2200;
const int sideWallThreshold = 500;
const int infinityThreshold = 400;

void SetPins();
void Config_ADC();
void SetTimer();
void ConfigureInterrupt();
void MotorRForward();
void MotorLForward();
void MotorRBack();
void MotorLBack();
void MotorROff();
void MotorLOff();
void RampRMotor(int newSpeed);

void __attribute__((interrupt, no_auto_psv)) _OC2Interrupt(void);

int main(void) {
    SetPins();
    Config_ADC();
    SetTimer();
    ConfigureInterrupt();
    
    TMR1 = 0;
    
    while (TMR1 < 3000) {
        MotorROff();
        MotorLOff();
    }
    
    // set initial state
    mode = LINEFOLLOW;
    linestate = ONLINE;
    MotorRForward();
    MotorLForward();

    while (1) {
        
    switch (mode) {
            
            case LINEFOLLOW:
                if (CENTER_QRD >= transitionThreshold && LEFT_QRD >= transitionThreshold && RIGHT_QRD >= transitionThreshold) {
                        mode = CANYONNAV;
                        canyonstate = FORWARD;
                        MotorRForward();
                        MotorLForward();
                        break;
                    }
                
                switch (linestate) {
                    case ONLINE:
                        if (LEFT_QRD <= lineFollowingThreshold)
                        {
                            MotorLBack();
                            linestate = ADJUSTLEFT;
                            break;
                        }
                        else if (RIGHT_QRD <= lineFollowingThreshold)
                        {
                            MotorRBack();
                            linestate = ADJUSTRIGHT;
                            break;
                        }
                        break;

                    case ADJUSTRIGHT:
                        if (CENTER_QRD <= lineFollowingThreshold)
                        {
                            MotorRForward();
                            linestate = ONLINE;
                            break;
                        }
                        else if (LEFT_QRD <= lineFollowingThreshold)
                        {
                            MotorRForward();
                            MotorLBack();
                            linestate = ADJUSTLEFT;
                            break;
                        }
                        break;

                    case ADJUSTLEFT:
                        if (CENTER_QRD <= lineFollowingThreshold)
                        {
                            MotorLForward();
                            linestate = ONLINE;
                            break;
                        }
                        else if (RIGHT_QRD <= lineFollowingThreshold)
                        {
                            MotorLForward();
                            MotorRBack();
                            linestate = ADJUSTRIGHT;
                            break;
                        }
                        break;
                }
                break;
                
                    
            case CANYONNAV:
                if (CENTER_QRD <= transitionThreshold || LEFT_QRD <= transitionThreshold || RIGHT_QRD <= transitionThreshold) {
                        MotorROff();
                        MotorLOff();
                        if (LEFT_SHARP >= sideWallThreshold && RIGHT_SHARP < infinityThreshold) {
                            canyonstate = TURNLEFT;
                            MotorRForward();
                            MotorLBack();
                            canyon_exit = 1;
                            break;
                        }
                        else if (LEFT_SHARP >= sideWallThreshold && RIGHT_SHARP < infinityThreshold) {
                            canyonstate = TURNRIGHT;
                            MotorLForward();
                            MotorRBack();
                            canyon_exit = 1;
                            break;
                        }
                        break;
                    }
                
                switch (canyonstate) {
                    case FORWARD:
                        if (CENTER_SHARP >= wallThreshold) {
                            MotorROff();
                            MotorLOff();
                            canyonstate = WAITING;
                            break;
                        }
                        break;
                        
                    case WAITING:
                        if (LEFT_SHARP >= sideWallThreshold && RIGHT_SHARP < infinityThreshold) {
                            MotorRBack();
                            MotorLForward();
                            numStepsTaken = 0;
                            canyonstate = TURNRIGHT;
                            break;
                        }
                        else if (RIGHT_SHARP >= sideWallThreshold && LEFT_SHARP < infinityThreshold) {
                            MotorRForward();
                            MotorLBack();
                            numStepsTaken = 0;
                            canyonstate = TURNLEFT;
                            break;
                        }
                        break;
                        
                    case TURNRIGHT:
                        if (numStepsTaken == numStepsFor90) {
                            MotorRForward();
                            MotorLForward();
                            canyonstate = FORWARD;
                            numStepsTaken = 0;
                            if (canyon_exit == 1) {
                                mode = LINEFOLLOW;
                                linestate = ONLINE;
                                MotorRForward();
                                MotorLForward();
                            }
                            break;
                        }
                        break;
                        
                    case TURNLEFT:
                        if (numStepsTaken == numStepsFor90) {
                            MotorRForward();
                            MotorLForward();
                            canyonstate = FORWARD;
                            numStepsTaken = 0;
                            if (canyon_exit == 1) {
                                mode = LINEFOLLOW;
                                linestate = ONLINE;
                                MotorRForward();
                                MotorLForward();
                            }
                            break;
                        }
                        break;
                }
                break;
                
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

    // digital output for motor direction
    _TRISB8 = 0;
    _TRISB9 = 0;

    // configure OC2 & OC3 for motor PWM
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

void ConfigureInterrupt() {
    // Set up the interrupt on OC2
    _OC2IP = 4; // Select the interrupt priority
    _OC2IE = 1; // Enable the OC2 interrupt
    _OC2IF = 0; // Clear the interrupt flag
}

void SetTimer() {
    T1CONbits.TON = 1;
    T1CONbits.TCS = 0;
    T1CONbits.TCKPS = 0b10;
}

void MotorRForward() {
    _LATB9 = 1;
    OC2RS = 5000;
    OC2R = 2500;
    OC2CON1bits.OCM = 0b110;
}

void MotorRBack() {
    _LATB9 = 0;
    OC2RS = 5000;
    OC2R = 2500;
    OC2CON1bits.OCM = 0b110;
}

void MotorROff() {
    OC2CON1bits.OCM = 0;
}

void MotorLForward() {
    _LATB8 = 1;
    OC3RS = 5000;
    OC3R = 2500;
    OC3CON1bits.OCM = 0b110;
}

void MotorLBack() {
    _LATB8 = 0;
    OC3RS = 5000;
    OC3R = 2500;
    OC3CON1bits.OCM = 0b110;
}

void MotorLOff() {
    OC3CON1bits.OCM = 0;
}

void RampRMotor(int newSpeed) {
    int currentSpeed = OC2RS;
    int iterationVal = (newSpeed - currentSpeed) / 10;
    
    while (currentSpeed != newSpeed) {
        if (currentSpeed < newSpeed) {
            currentSpeed = currentSpeed + iterationVal;
            OC2RS = currentSpeed;
        }
        else if  (currentSpeed > newSpeed) {
            currentSpeed = currentSpeed - iterationVal;
            OC2RS = currentSpeed;
        }
        OC2R = (int)OC2RS/2;
    } 
}

void __attribute__((interrupt, no_auto_psv)) _OC2Interrupt(void)
{
    _OC2IF = 0; // Clear the interrupt flag
    numStepsTaken++;
}
