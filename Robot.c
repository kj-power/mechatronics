
#include "xc.h"
#pragma config FNOSC = FRCDIV

int step = 0;
int N = 0;
int count = 0; //count of how many times does it go forward

void __attribute__((interrupt, no_auto_psv)) _OC1Interrupt(void){
    _OC2IF = 0;
    step++;
}

int TurnRigh(int a){
    _LATA0 = 1;
    _LATA1 = 1;
    OC2RS = a;
    OC2R = OC2RS/2;
    OC3RS = a;
    OC3R = OC3RS/2;
}

int TurnLeft(int a){
    _LATA0 = 0;
    _LATA1 = 0;
    OC2RS = a;
    OC2R = OC2RS/2;
    OC3RS = a;
    OC3R = OC3RS/2;
}

int GoStraight(int a){
    _LATA0 = 1;
    _LATA1 = 0;
    OC2RS = a; // half step, 2rev/s
    OC2R = OC2RS/2;
    OC3RS = a;
    OC3R = OC3RS/2;
}

int GoBack(int a){
    _LATA0 = 0;
    _LATA1 = 1;
    OC2RS = a; // half step, 2rev/s
    OC2R = OC2RS/2;
    OC3RS = a;
    OC3R = OC3RS/2;
}


int main(void) {
    enum { Stop, Forward, T180, RightTurn, LeftTurn} state;
    
    _RCDIV = 0;
    ANSA = 0;
    ANSB = 0;
    TRISA = 0;
    TRISB = 0;

    _TON = 1;
    _TCKPS = 0b10;
    _TCS = 0;
    TMR1 = 0;
    _TON = 0;
    
    OC2CON1 = 0x1C06;
    OC2CON2 = 0x001F;
    OC2CON1bits.OCM = 0b110;
    _OC2IF = 0;
    _OC2IP = 4;
    _OC2IE = 0;
    
    OC3CON1 = 0x1C06;
    OC3CON2 = 0x001F;
    OC3CON1bits.OCM = 0b110;


    state = Stop;

    while (1){
        _OC2IE = 0;
        step = 0
        _T1ON = 1;
        TMR1 = 0; 
        if (state == Stop){
            if (TMR1 >= 62500){ //Start going forward after 1s
                _OC2IE = 0;
                step = 0
                _T1ON = 1;
                TMR1 = 0; 
                GoStraight(4999);
                state = Forward;
            }
        }
        else if (state == Forward){
            if (count == 2){
                if(TMR1 >= 62500){
                    count ++;
                    N = 0;
                    _OC2IE = 0;
                    step = 0
                    _T1ON = 0;
                    TMR1 = 0;
                    state = Stop;                    
                }
            }            
            else if (count == 1){
                if(TMR1 >= 62500){
                    count ++;
                    N = 280 * 2;
                    _OC2IE = 1;
                    step = 0
                    _T1ON = 0;
                    TMR1 = 0;
                    TurnRigh(4999);
                    state = T180;                    
                }
            }
            else if (TMR1 >= 62500){ // Start turning right after 1s
                count ++;
                N = 280;
                _OC2IE = 1;
                step = 0
                _T1ON = 0;
                TMR1 = 0;
                TurnRigh(4999);
                state = RightTurn;
            }
        }
        else if (state == T180){
            if (step >= N){
                _OC2IE = 0;
                step = 0
                _T1ON = 1;
                TMR1 = 0; 
                TurnRigh(4999);
                state = Forward;
            }
        }
        else if (state == RightTurn){
            if (step >= N){ // start going forward after 90 degree (hopefully)
                step = 0;
                _OC2IE = 0;
                _T1ON = 1;
                TMR1 = 0;
                GoStraight(4999);
                state = Forward;
            }
        }
        else if (state == LeftTurn){
        }
    }
}
