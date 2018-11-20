/******************************************************************************
 * 
 * Rock Paper Scissors Game Project: Red vs Green
>>>>>>> ae4a110a1415847768f6d8d463b8541fea92f3bd
 * 
 * Microprocessors II EECE.4800
 * 
 * Rock-Paper-Scissors Game
 *
 * Fall 2018
 * 
 * Jacob Sword, Jack DeGregorio
 * 
 * 11/14/2018

 * Description:
 * Binary rock paper scissors game.
 * Start button starts countdown.
 * Each player makes choice from binary 1-3 via switches within countdown.
 * At end of countdown, switch states judged to see who wins RPS.
 * Player's LEDs are lit to indicate to players/viewers what each player chose.
 * RGB LED is lit to indicate winner of game.
 * LCD screen is updated with running count of wins for each side.
 */

#include "xc.h"

#pragma config BWRP = WRPROTECT_OFF
#pragma config BSS = NO_FLASH

//FGS
#pragma config GWRP = OFF

#pragma config GSS = OFF

//FOSCEL
#pragma config FNOSC = FRC
#pragma config IESO = OFF

//FOSC
#pragma config POSCMD = NONE
#pragma config OSCIOFNC = ON
#pragma config IOL1WAY = OFF
#pragma config FCKSM = CSDCMD

//FWDT
#pragma config WDTPOST = PS512
#pragma config WDTPRE = PR128
#pragma config WINDIS = OFF
#pragma config FWDTEN = OFF

//FPOR
#pragma config FPWRT = PWR16
#pragma config ALTI2C = OFF
//FICD
#pragma config ICS = PGD1
#pragma config JTAGEN = OFF            

#define P1_SWITCH_0 PORTBbits.RB15
#define P1_SWITCH_1 PORTBbits.RB14
#define P2_SWITCH_0 PORTBbits.RB13
#define P2_SWITCH_1 PORTBbits.RB12

void __attribute__((__interrupt__, no_auto_psv)) _T2Interrupt(void)
{
    _LATA2 = P1_SWITCH_0;
    _LATA3 = P1_SWITCH_1;
    _LATB4 = P2_SWITCH_0;
    _LATA4 = P2_SWITCH_1;
    IFS0bits.T2IF = 0; //Clear Timer interrupt flag
}
void config_io(void);
void config_timer(void);

int main()
{
    config_io();
    config_timer();
    while(1) { }  
}

void config_io()
{
    //Outputs
    _ODCA4 = 0; 					// Disable open drain
    _TRISA4 = 0;					// Config RB14 as output
    _LATA4 = 0;					// RB14 initially low   

    _ODCA3 = 0;
    _TRISA3 = 0;
    _LATA3 = 0;

    _ODCA2 = 0;
    _TRISA2 = 0;
    _LATA2 = 0;

    _ODCB4 = 0;
    _TRISB4 = 0;
    _LATB4 = 0;

    _ODCB2 = 0;
    _TRISB2 = 0;
    _LATB2 = 0;

    _ODCB3 = 0;
    _TRISB3 = 0;
    _LATB3 = 0;

    //Inputs
    _ODCB15 = 0; 					// Disable open drain
    _TRISB15 = 1;					// Config RB13 as input
    _CN11PUE = 1;                  // Enables Internal Weak-pull-up
    _PCFG9 = 1;                   // Disables analog function

    _ODCB14 = 0;
    _TRISB14 = 1;
    _CN12PUE = 1;
    _PCFG10 = 1;

    _ODCB13 = 0;
    _TRISB13 = 1;
    _CN13PUE = 1;
    _PCFG11 = 1;

    _ODCB12 = 0;
    _TRISB12 = 1;
    _CN14PUE = 1;
    _PCFG12 = 1;
}

void config_timer()
{
    T2CONbits.TON = 0; // Disable Timer
    T2CONbits.TCS = 0; // Select internal instruction cycle clock
    T2CONbits.TGATE = 0; // Disable Gated Timer mode
    T2CONbits.TCKPS = 0b11; // Select 1:256 Prescaler
    TMR2 = 0x00; // Clear timer register
    PR2 = 0x1C52; // Load the period value
    IPC1bits.T2IP = 0x01; // Set Timer 2 Interrupt Priority Level
    IFS0bits.T2IF = 0; // Clear Timer 2 Interrupt Flag
    IEC0bits.T2IE = 1; // Enable Timer2 interrupt 
    T2CONbits.TON = 1;
}

