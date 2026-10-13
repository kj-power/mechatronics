/*
 * File:   Robot.c
 * Author: ryanyip
 *
 * Created on October 12, 2026, 4:18 PM
 */


#include "xc.h"
#pragma config FNOSC = FRCDIV

int TurnRigh(int a, int b){
    _LATA0 = 1;
    _LATA1 = 1;
    OC2RS = a;
    OC2R = OC2RS/2;
    OC3RS = a;
    OC3R = OC3RS/2;
    T1CONbits.TON = 1;
    T1CONbits.TCS = 0;
    T1CONbits.TCKPS = 0b10;
    TMR1 = 0;
    PR1 = b;
    if(TMR1 >= PR1){
        break;
    }
    return 0;
}

int TurnLeft(int a, int b){
    _LATA0 = 0;
    _LATA1 = 0;
    OC2RS = a;
    OC2R = OC2RS/2;
    OC3RS = a;
    OC3R = OC3RS/2;
    T1CONbits.TON = 1;
    T1CONbits.TCS = 0;
    T1CONbits.TCKPS = 0b10;
    TMR1 = 0;
    PR1 = b;
    if(TMR1 >= PR1){
        break;
    }
    return 0;
}

int GoStraight(int a){
    _LATA0 = 1;
    _LATA1 = 0;
    OC2RS = a; // half step, 2rev/s
    OC2R = OC2RS/2;
    OC3RS = a;
    OC3R = OC3RS/2;
    return 0;
}

int GoBack(int a){
    _LATA0 = 0;
    _LATA1 = 1;
    OC2RS = a; // half step, 2rev/s
    OC2R = OC2RS/2;
    OC3RS = a;
    OC3R = OC3RS/2;
    return 0;
}


int main(void) {
    _RCDIV = 0;
    ANSA = 0;
    ANSB = 0;
    TRISA = 0;
    TRISB = 0;
    
    OC2CON1 = 0x1C06;
    OC2CON2 = 0x001F;
    OC2CON1bits.OCM = 0b110;
    
    OC3CON1 = 0x1C06;
    OC3CON2 = 0x001F;
    OC3CON1bits.OCM = 0b110;
    
    GoStraight(4999);
    TurnRigh(4999, 280); //need to find the value
    GoStraight(4999);
    TurnRigh(4999, 280); //need to find the value
    TurnRigh(4999, 280); //need to find the value
    GoStraight(4999);
    
}
