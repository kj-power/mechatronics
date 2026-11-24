/*
 * File:   Robot.c
 * Author: ryanyip
 *
 * Created on October 12, 2026, 4:18 PM
 */

/*
 * PINOUT
// *   2 (RA0 - AN0) Ball retrieval photodiode
 *   3 (RA1 - AN1) Data transmission photodiode
 *   4 (OC2) Motor R PWM
 *   5 (OC3) Motor L PWM
 *   6 (RB2 - AN4) left distance sensor
 *   7 (RA2 - AN13) center distance sensor
 *   8 (RA3 - AN14) right distance sensor
 *   9 (RB4 - DO) left ball box detector
 *   10 (RA4 - DO) right ball box detector
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
#pragma config OSCIOFNC = OFF
#pragma config SOSCSRC = DIG

#define LEFT_QRD ADC1BUF10 
#define RIGHT_QRD ADC1BUF11
#define CENTER_QRD ADC1BUF9
#define LEFT_SHARP ADC1BUF4 
#define RIGHT_SHARP ADC1BUF14
#define CENTER_SHARP ADC1BUF13
#define BALL_GRAB_SENSOR ADC1BUF0
#define BALL_COLOR_QRD ADC1BUF12
#define SATELLITE_DETECTOR ADC1BUF1

enum { EXITLANDER, ENTERLANDER, LINEFOLLOW, CANYONNAV, BALLGRAB, BALLDROP, DATATRANSMIT } mode;
enum { TURNAROUND, ADJUSTRIGHT, ADJUSTLEFT, ONLINE } linestate;
enum { FORWARD, BACK, WAITING, TURNRIGHT, TURNLEFT, EXIT } canyonstate;
enum { PREPARE, TURNTOBALL, COLLECT, TOBALL, TOLINE, TURNTOLINE } grabstate;
enum { GETSET, RIGHTTURN, LEFTTURN, TOBOX, REVERSE, DROP, TURNLEFTEND, TURNRIGHTEND } dropstate;
enum { STRAIGHT, PREP, TURN, RIGHT, LEFT } exitstate;
enum { LINEUP, TURNIN, ENTER } enterstate;
enum { FINDSIGNAL, EMIT } datastate;

int numStepsTaken = 0;
const int numStepsFor90 = 540;
const int numStepsBeforeTurn = 300;
int canyon_exit = 0;
int ballColor = 0; // 0 for black, 1 for white

const int lineFollowingThreshold = 2500;

const int sideThreshold = 3600;
const int transitionThreshold = 3300;

const int wallThreshold = 2200;
const int sideWallThreshold = 400;
const int infinityThreshold = 300;

const int ballColorThreshold = 2500;
const int ballGrabThreshold = 1750;
const int satelliteThreshold = 1000;

int numStepsToBall = 800;
int numStepsToDrop = 100;
int ballDropDone = 0;
int ballGrabDone = 0;

int ready = 0;

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
void MotorRSlow();
void MotorLSlow();
void MotorRFast();
void MotorLFast();
void MotorRFaster();
void MotorLFaster();
void RampRMotor(int newSpeed);
void RampLMotor(int newSpeed);
void ServoUp();
void ServoDown();

void __attribute__((interrupt, no_auto_psv)) _OC2Interrupt(void);

int main(void) {
    SetPins();
    Config_ADC();
    SetTimer();
    ConfigureInterrupt();
    
    TMR1 = 0;
    
    while (TMR1 < 10000) {
        MotorROff();
        MotorLOff();
    }
    
    // set initial state
    mode = EXITLANDER;
    exitstate = STRAIGHT;
    MotorRForward();
    MotorLForward();
    MotorRFast();
    MotorLFast();

    while (1) {
        
    switch (mode) {
            
            case EXITLANDER:
                switch (exitstate) {
                      
                    case PREP:
                        if (numStepsTaken == 400) {
                            MotorLBack();
                            numStepsTaken = 0;
                            exitstate = TURN;
                            break;
                        }
                        break;
                        
                    case TURN:
                        if (numStepsTaken >= numStepsFor90) {
                            MotorRForward();
                            MotorLForward();
                            mode = LINEFOLLOW;
                            linestate = ONLINE;
                            break;
                        }
                        break;
                        
                    case STRAIGHT:
                        if (CENTER_QRD <= transitionThreshold && LEFT_QRD <= transitionThreshold && RIGHT_QRD <= transitionThreshold) {
                            numStepsTaken = 0;
                            exitstate = PREP;
                            MotorRFast();
                            MotorLFast();
                            break;
                        }
                        else if (LEFT_QRD <= sideThreshold)
                        {
                            MotorRFaster();
                            MotorLSlow();
                            exitstate = LEFT;
                            break;
                        }
                        else if (RIGHT_QRD <= sideThreshold)
                        {
                            MotorRSlow();
                            MotorLFaster();
                            exitstate = RIGHT;
                            break;
                        }
                        break;

                    case RIGHT:
                        if (CENTER_QRD <= transitionThreshold && LEFT_QRD <= transitionThreshold && RIGHT_QRD <= transitionThreshold) {
                            numStepsTaken = 0;
                            exitstate = PREP;
                            MotorRFast();
                            MotorLFast();
                            break;
                        }
                        if (CENTER_QRD <= lineFollowingThreshold)
                        {
                            MotorRFast();
                            MotorLFast();
                            exitstate = STRAIGHT;
                            break;
                        }
                        else if (LEFT_QRD <= sideThreshold)
                        {
                            MotorRFaster();
                            MotorLSlow();
                            exitstate = LEFT;
                            break;
                        }
                        break;

                    case LEFT:
                        if (CENTER_QRD <= transitionThreshold && LEFT_QRD <= transitionThreshold && RIGHT_QRD <= transitionThreshold) {
                            numStepsTaken = 0;
                            exitstate = PREP;
                            MotorRFast();
                            MotorLFast();
                            break;
                        }
                        if (CENTER_QRD <= lineFollowingThreshold)
                        {
                            MotorLFast();
                            MotorRFast();
                            exitstate = STRAIGHT;
                            break;
                        }
                        else if (RIGHT_QRD <= sideThreshold)
                        {
                            MotorLFaster();
                            MotorRSlow();
                            exitstate = RIGHT;
                            break;
                        }
                        break;
                }
                break;
                
            case ENTERLANDER:
                switch (enterstate) {
                    case LINEUP:
                        if (numStepsTaken == 450) {
                            MotorLBack();
                            numStepsTaken = 0;
                            enterstate = TURNIN;
                            break;
                        }
                        break;
                        
                    case TURNIN:
                        if (numStepsTaken == numStepsFor90) {
                            MotorLForward();
                            enterstate = ENTER;
                            break;
                        }
                        break;
                        
                    case ENTER:
                        if (CENTER_SHARP >= wallThreshold) {
                            MotorROff();
                            MotorLOff();
                            mode = DATATRANSMIT;
                            datastate = FINDSIGNAL;
                            break;
                        }
                        break;  
                }
                break;
                
            case DATATRANSMIT:
                switch (datastate) {
                    case FINDSIGNAL:
                        if (SATELLITE_DETECTOR >= satelliteThreshold) {
                            datastate = EMIT;
                            break;
                        }
//                        else if (OC1R < 5000) {
//                            OC1R++;
//                            for(int i = 0; i<1000; i++);
//                            break;
//                        }
                        else if (OC1R > 250) {
                            OC1R--;
                            for(int i = 0; i<1000; i++);
                            break;
                        }
                        break;

                    case EMIT:
                        _LATB7 = 1;
                        break;
                }
                break;
               
            
        
            case BALLGRAB:
                switch (grabstate) {
                    case PREPARE:
                        if (numStepsTaken >= 100) {
                            MotorRBack();
                            MotorLForward();
                            grabstate = TURNTOBALL;
                            numStepsTaken = 0;
                            break;
                        }
                        break;
                    case TURNTOBALL:
                        if (numStepsTaken >= 520) {
                            MotorRForward();
                            MotorLForward();
                            grabstate = TOBALL;
                            numStepsTaken = 0;
                            break;
                        }
                        break;
                    case COLLECT:
                        if (TMR1 >= 30000) {
                            MotorRBack();
                            MotorLBack();
                            grabstate = TOLINE;
                            numStepsTaken = 0;
                            ballGrabDone = 1;
                            break;
                        }
                        break;
                    case TOBALL:
                        if (numStepsTaken >= numStepsToBall) {
                            MotorROff();
                            MotorLOff();
                            grabstate = COLLECT;
                            numStepsTaken = 0;
                            TMR1 = 0;
                            break;
                        }
                        break;
                    case TOLINE: 
                        if (numStepsTaken >= numStepsToBall) {
                            MotorRForward();
                            MotorLBack();
                            grabstate = TURNTOLINE;
                            numStepsTaken = 0;
                            break;
                        }
                        break;
                    case TURNTOLINE:
                        if (numStepsTaken == numStepsFor90) {
                            MotorRForward();
                            MotorLForward();
                            mode = LINEFOLLOW;
                            linestate = ONLINE;
                            break;
                        }
                        break;
                }
                break;
                
            case BALLDROP:
                    switch (dropstate) {
                        case GETSET:
                            if (BALL_COLOR_QRD <= ballColorThreshold) {
                                ballColor = 1;
                            }
                            if (numStepsTaken == 500) {
                                if (ballColor == 0) {
                                    dropstate = RIGHTTURN;
                                    MotorRBack();
                                    MotorLForward();
                                    numStepsTaken = 0;
                                    break;
                                } 
                                else if (ballColor == 1) {
                                    dropstate = LEFTTURN;
                                    MotorLBack();
                                    MotorRForward();
                                    numStepsTaken = 0;
                                    break;
                                }
                            }
                            break;
                            
                        case RIGHTTURN:
                            if (numStepsTaken >= numStepsFor90) {
                                MotorRForward();
                                MotorLForward();
                                dropstate = TOBOX;
                                numStepsTaken = 0;
                                ServoUp();
                                break;
                            }
                            break;
                            
                        case LEFTTURN:
                            if (numStepsTaken >= numStepsFor90) {
                                MotorRForward();
                                MotorLForward();
                                dropstate = TOBOX;  
                                numStepsTaken = 0;
                                ServoUp();
                                break;
                            }
                            break;
                        case DROP:
                            if (TMR1 == 30000) {
                                MotorRBack();
                                MotorLBack();
                                dropstate = REVERSE;
                                numStepsTaken = 0;
                                ballDropDone = 1;
                            }
                            break;
                        case TOBOX:
                            
                            if (numStepsTaken >= numStepsToDrop) {
                                dropstate = REVERSE;
                                numStepsTaken = 0;
                                MotorRBack();
                                MotorLBack();
                                break;
                            }
                            break;
                        case REVERSE: 
                            if (numStepsTaken == numStepsToDrop) {
                                if (ballColor == 0) {
                                    MotorRForward();
                                    MotorLBack();
                                    dropstate = TURNLEFTEND;
                                    break;
                                }
                                else if (ballColor == 1) {
                                    MotorLForward();
                                    MotorRBack();
                                    dropstate = TURNRIGHTEND;
                                    break;
                                }
                                numStepsTaken = 0;
                                break;
                            }
                            break;
                            
                        case TURNRIGHTEND:
                            if (numStepsTaken == 720) {
                                MotorRForward();
                                MotorLForward();
//                                MotorRFast();
//                                MotorLFast();
                                mode = LINEFOLLOW;
                                linestate = ONLINE;
                                break;
                            }
                            break;
                            
                        case TURNLEFTEND:
                            if (numStepsTaken == 720) {
                                MotorRForward();
                                MotorLForward();
//                                MotorRFast();
//                                MotorLFast();
                                mode = LINEFOLLOW;
                                linestate = ONLINE;
                                break;
                            }
                            break;
                    }
                    break;
            
                
            
            case LINEFOLLOW:
                if (CENTER_QRD <= 2100 && LEFT_QRD <= 3200 && canyon_exit == 1 && ballDropDone == 1) {
                    mode = ENTERLANDER;
                    MotorRFast();
                    MotorLFast();
                    enterstate = LINEUP;
                    numStepsTaken = 0;
                    break;
                }
                
                if (canyon_exit == 0 && CENTER_QRD >= 3500 && LEFT_QRD >= 3800 && RIGHT_QRD >= 3800) {
                        mode = CANYONNAV;
                        canyonstate = FORWARD;
                        MotorRFaster();
                        MotorLFaster();
                        TMR1 = 0;
                        break;
                }
                
                if (BALL_GRAB_SENSOR >= ballGrabThreshold && ballGrabDone == 0) {
                    mode = BALLGRAB;
                    grabstate = PREPARE;
                    MotorRFast();
                    MotorLFast();
                    numStepsTaken = 0;
                    break;
                }

                if (_RB4 == 0 && _RA4 == 0 && ballDropDone == 0 && ballGrabDone == 1) {
                    mode = BALLDROP;
                    dropstate = GETSET;
                    MotorLSlow();
                    MotorRSlow();
                    numStepsTaken = 0;
                    break;
                }
                
                switch (linestate) {
                    case ONLINE:
                        if (LEFT_QRD <= sideThreshold)
                        {
                            MotorRFaster();
                            MotorLSlow();
                            linestate = ADJUSTLEFT;
                            break;
                        }
                        else if (RIGHT_QRD <= sideThreshold)
                        {
                            MotorRSlow();
                            MotorLFaster();
                            linestate = ADJUSTRIGHT;
                            break;
                        }
                        break;

                    case ADJUSTRIGHT:
                        if (CENTER_QRD <= lineFollowingThreshold)
                        {
                            MotorRFast();
                            MotorLFast();
                            linestate = ONLINE;
                            break;
                        }
                        else if (LEFT_QRD <= sideThreshold)
                        {
                            MotorRFaster();
                            MotorLSlow();
                            linestate = ADJUSTLEFT;
                            break;
                        }
                        break;

                    case ADJUSTLEFT:
                        if (CENTER_QRD <= lineFollowingThreshold)
                        {
                            MotorLFast();
                            MotorRFast();
                            linestate = ONLINE;
                            break;
                        }
                        else if (RIGHT_QRD <= sideThreshold)
                        {
                            MotorLFaster();
                            MotorRSlow();
                            linestate = ADJUSTRIGHT;
                            break;
                        }
                        break;      
                }
                break;
                
                    
            case CANYONNAV:
                if (canyon_exit == 0) {
                    if (CENTER_QRD <= transitionThreshold && LEFT_QRD <= transitionThreshold && RIGHT_QRD <= transitionThreshold) {
                        canyonstate = EXIT;
                        numStepsTaken = 0;
                        MotorRFast();
                        MotorLFast();
                        break; 
                    }
                }
                
                switch (canyonstate) {
                    case EXIT:
                        if (numStepsTaken == numStepsBeforeTurn) {
                            MotorROff();
                            MotorLOff();
                            if (RIGHT_SHARP >= sideWallThreshold) {
                                MotorRForward();
                                MotorLBack();
                                canyon_exit = 1;
                                canyonstate = TURNLEFT;
                                numStepsTaken = 0;
                                break;
                            }
                            else if (LEFT_SHARP >= sideWallThreshold) {
                                MotorLForward();
                                MotorRBack();
                                canyon_exit = 1;
                                canyonstate = TURNRIGHT;
                                numStepsTaken = 0;
                                break;
                            }
                        }
                        break;
                        
                    case FORWARD:
                        if (TMR1 > 15000) {
                            ready = 1;
                        }
                        if (CENTER_SHARP >= wallThreshold && ready == 1) {
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
                        if (canyon_exit == 1) {
                            if (numStepsTaken >= numStepsFor90) {
                                mode = LINEFOLLOW;
                                linestate = ONLINE;
                                MotorRForward();
                                MotorLForward();
                                MotorRFast();
                                MotorLFast();
                                break;
                            }      
                          break;
                        }
                        else if (numStepsTaken >= numStepsFor90) {
                            MotorRForward();
                            MotorLForward();
                            canyonstate = FORWARD;
                            numStepsTaken = 0;
                            TMR1 = 0;
                            ready = 0;
                            break;
                        }
                        break;
                        
                    case TURNLEFT:
                        if (canyon_exit == 1) {
                            if (numStepsTaken >= numStepsFor90) {
                                mode = LINEFOLLOW;
                                linestate = ONLINE;
                                MotorRForward();
                                MotorLForward();
                                MotorRFast();
                                MotorLFast();
                                break;
                            }      
                          break;
                        }
                        else if (numStepsTaken >= numStepsFor90) {
                            MotorRForward();
                            MotorLForward();
                            canyonstate = FORWARD;
                            numStepsTaken = 0;
                            TMR1 = 0;
                            ready = 0;
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
    _SMPI = 8;    // begin new sampling sequence after every *insert num of A2D pins* samples -- num pins - 1
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
    _CSS0 = 1; // pin 2 (ball grab sensor)
    _CSS12 = 1; // ball color QRD
    _CSS1 = 1; // data transmit photodiode
    
    _ADON = 1;    // enable module

}

void SetPins() {

    // digital output for motor direction
    _TRISB8 = 0;
    _TRISB9 = 0;
    
    // configure OC1 for servo
    OC1CON1 = 0;
    OC1CON2 = 0;
    OC1CON1bits.OCTSEL = 0b111;
    OC1CON2bits.SYNCSEL = 0b11111;
    OC1CON2bits.OCTRIG = 0;
    OC1RS = 20000;
    OC1R = 5000;
    OC1CON1bits.OCM = 0b110; // Start PWM on pin 14. Set to zero to disable.

    // configure OC2 & OC3 for motor PWM
    OC2CON1 = 0;
    OC2CON2 = 0;
    OC2CON1bits.OCTSEL = 0b111;
    OC2CON2bits.SYNCSEL = 0b11111;
    OC2CON2bits.OCTRIG = 0;
    OC2RS = 2500;
    OC2R = 5000;
    OC2CON1bits.OCM = 0b110; // change to 0 to turn off
    
    OC3CON1 = 0;
    OC3CON2 = 0;
    OC3CON1bits.OCTSEL = 0b111;
    OC3CON2bits.SYNCSEL = 0b11111;
    OC3CON2bits.OCTRIG = 0;
    OC3RS = 5000;
    OC3R = 2500;
    OC3CON1bits.OCM = 0b110; // change to 0 to stop PWM
    
    // line following pins
    _ANSB13 = 1;
    _TRISB13 = 1;
    _ANSB14 = 1;
    _TRISB14 = 1;
    _ANSB15 = 1;
    _TRISB15 = 1;
    
    // sharp sensor pins
    _ANSB2 = 1;
    _TRISB2 = 1;
    _ANSA2 = 1;
    _TRISA2 = 1;
    _ANSA3 = 1;
    _TRISA3 = 1;
    
    // configure ball delivery pins
    _ANSB4 = 0;
    _TRISA4 = 1;
    _TRISB4 = 1;
    
    // configure ball grab sensor
    _ANSA0 = 1;
    _TRISA0 = 1;
    
    // configure ball color sensor
    _ANSB12 = 1;
    _TRISB12 = 1;
    
    // configure data transmit photodiode
    _ANSA1 = 1;
    _TRISA1 = 1;
    
    // configure laser
    _TRISB7 = 0;
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
    OC2CON1bits.OCM = 0b110;
}

void MotorRBack() {
    _LATB9 = 0;
    OC2CON1bits.OCM = 0b110;
}

void MotorROff() {
    OC2CON1bits.OCM = 0;
}

void MotorLForward() {
    _LATB8 = 1;
    OC3CON1bits.OCM = 0b110;
}

void MotorLBack() {
    _LATB8 = 0;
    OC3CON1bits.OCM = 0b110;
}

void MotorLOff() {
    OC3CON1bits.OCM = 0;
}

void MotorRSlow() {
   OC2RS = 10000;
   OC2R = 5000;
}

void MotorLSlow() {
    OC3RS = 10000;
    OC3R = 5000;
}

void MotorRFast() {
    OC2RS = 5000;
    OC2R = 2500;
}

void MotorLFast() {
    OC3RS = 5000;
    OC3R = 2500;
}

void MotorRFaster() {
    OC2RS = 3500;
    OC2R = 1750;
}

void MotorLFaster() {
    OC3RS = 3500;
    OC3R = 1750;
}

void ServoUp() {
    while (OC1R > 250) {
        OC1R--;
//        for(int i = 0; i<100; i++);
    }
}

void ServoDown() {
    while (OC1R < 5000) {
        OC1R++;
//        for(int i = 0; i<100; i++);
    }
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

void RampLMotor(int newSpeed) {
    int currentSpeed = OC3RS;
    int iterationVal = (newSpeed - currentSpeed) / 10;
    
    while (currentSpeed != newSpeed) {
        if (currentSpeed < newSpeed) {
            currentSpeed = currentSpeed + iterationVal;
            OC3RS = currentSpeed;
        }
        else if  (currentSpeed > newSpeed) {
            currentSpeed = currentSpeed - iterationVal;
            OC3RS = currentSpeed;
        }
        OC3R = (int)OC3RS/2;
    } 
}

void __attribute__((interrupt, no_auto_psv)) _OC2Interrupt(void)
{
    _OC2IF = 0; // Clear the interrupt flag
    numStepsTaken++;
}
