/******************************************************************************
 * 
 * Rock Paper Scissors Game Project: Red vs Green
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
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

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

/**FROM LAB 4**/
#define E  _LATA0             		//  Define the LCD Control Pins
#define RS _LATB5

// LCD Functions
void Configure_LCD_pins(void);
void LCDWrite(int LCDData, int RSValue);
void Init_LCD(void);
void Clear_LCD(void);
void LCD_Display(char Display[16]);

const int Twentyms = 3000;
const int Fivems = 1000;
const int TwoHundredus = 40;
/**END FROM LAB 4*/
const int ROCK = 0;
const int PAPER = 1;
const int SCISSORS = 2;

void config_io(void);
void config_timer(void);
int calculate_winner();
void show_wins();


int countdown = 0;
int end_round = 0;
int green_wins = 0;
int red_wins = 0;

char Display[16];

void __attribute__((__interrupt__, no_auto_psv)) _T2Interrupt(void)
{
    Clear_LCD();
    _LATB2 = 0;
    _LATB3 = 0;
    sprintf(Display,"Game starts: %d", 10 - countdown);
    LCD_Display(Display);
    show_wins();
    if (countdown == 10) {
        _LATA2 = P1_SWITCH_1;
        _LATA3 = P1_SWITCH_0;
        _LATB4 = P2_SWITCH_1;
        _LATA4 = P2_SWITCH_0;
        countdown = 0;
        T2CONbits.TON = 0; // Start Timer
        end_round = 1;
    } else {
        end_round = 0;
        countdown++;
    }
    
    IFS0bits.T2IF = 0; //Clear Timer interrupt flag
}

void __attribute__((__interrupt__,, no_auto_psv)) _CNInterrupt(void)
{
    T2CONbits.TON = 1; // Start Timer
    IFS1bits.CNIF = 0;
}


int main()
{
    config_io();
    CNEN1bits.CN3IE = 1;   // Enable CN3 pin for interrupt detection
    IEC1bits.CNIE = 1;      // Enable CN interrupts
    IFS1bits.CNIF = 0;      // Clear CN interrupts
    config_timer();
    Configure_LCD_pins();
    Init_LCD();
    Clear_LCD();
    LCD_Display("Push to start");
    while(1) {
        if (end_round) {
           int winner = calculate_winner();
           char* winning_color;
            if (winner == 0) {
                winning_color = "Tie! No Winner:(";
                _LATB2 = 1;
                _LATB3 = 1;
            } else if (winner == 1) {
                winning_color = "Green Wins!";
                green_wins++;
                _LATB2 = 0;
                _LATB3 = 1;
           } else {
                winning_color = "Red Wins!";
                red_wins++;
                _LATB2 = 1;
                _LATB3 = 0;
           }
            Clear_LCD();
  
            LCD_Display(winning_color); // Winner: Green/Red
            show_wins();
            end_round = 0;
        }
    }  
}

/**
 * Calculate winner of rock paper scissors by conventional logic
 * @return 0 if tie, 1 if green wins, 2 if red wins
 */
int calculate_winner() {
    int player1 = (PORTBbits.RB13 * 2) + PORTBbits.RB12;
    int player2 = (PORTBbits.RB15 * 2) + PORTBbits.RB14;
    if (player1 > SCISSORS)
        player1 = ROCK;
    if (player2 > SCISSORS)
        player2 = ROCK;  
    if (player1 == player2)
        return 0;
    switch(player1) {
        case 0:
            switch(player2) {
                case 1:
                    return 2;
                case 2:
                    return 1;
            }
        case 1:
            switch(player2) {
                case 0:
                    return 1;
                case 2:
                    return 2;
            }
        case 2:
            switch(player2) {
                case 0:
                    return 2;
                case 1:
                    return 1;
            }
        default:
            return 0;
    }
}

void show_wins()
{
    LCDWrite(0b11000000, 0);    //  Move Cursor to the Second Line
    // LCD DISPLAY Win Count
    sprintf(Display,"Green:%d Red:%d", green_wins, red_wins);
    LCD_Display(Display);
}

void config_io()
{
    //Outputs
    _ODCA4 = 0; 					// Disable open drain
    _TRISA4 = 0;					// Config RB14 as output
    _LATA4 = 0;					    // RB14 initially low   

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
    _TRISB15 = 1;					// Config RB15 as input
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
    
    _ODCA1 = 0;
    _TRISA1 = 1;
    _CN3PUE = 1;
    _PCFG1 = 1;
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
    T2CONbits.TON = 0;
}

/***************LCD FUNCTIONS**************/

void Configure_LCD_pins(void)
{
    _PCFG0 = 1;   //disable analog function on pin 2

    _TRISA0 = 0;   //RA0 is an output
    _TRISB5 = 0;   //PORTB 5-7, 10-11 are outputs
    _TRISB6 = 0;
    _TRISB7 = 0;
    _TRISB10 = 0;
    _TRISB11 = 0;

    _ODCA0 = 0;   //disable open drain on all LCD pins
    _ODCB5 = 0;
    _ODCB6 = 0;
    _ODCB7 = 0;
    _ODCB10 = 0;
    _ODCB11 = 0;

    _LATA0 = 0;  //initialize all LCD pins to 0
    _LATB5 = 0;
    _LATB6 = 0;
    _LATB7 = 0;
    _LATB10 = 0;
    _LATB11 = 0;
}  //end Configure_LCD_pins()

/**FUNCTIONS FROM LAB 4*/
void LCDWrite(int LCDData, int RSValue)
{
    int n, k;

//  Get High 4 Bits for Output
    _LATB6 = (LCDData >> 4)  & 0b0001;
    _LATB7 = (LCDData >> 5)  & 0b0001;
    _LATB10 = (LCDData >> 6) & 0b0001;
    _LATB11 = (LCDData >> 7) & 0b0001;

    RS = RSValue;
    E = 1;
    E = 0;              		//  Toggle the High 4 Bits Out

//  Get Low 4 Bits for Output
    _LATB6 = LCDData  & 0b0001;
    _LATB7 = (LCDData >> 1) & 0b0001;
    _LATB10 = (LCDData >> 2) & 0b0001;
    _LATB11 = (LCDData >> 3) & 0b0001;
    RS = RSValue;
    E = 1;
    E = 0;              		//  Toggle the Low 4 Bits Out

    if ((0 == (LCDData & 0xFC)) && (0 == RSValue))
        n = Fivems;
    else
        n = TwoHundredus;
        
    for (k = 0; k < n; k++);    		//  Delay for Character

}  //  End LCDWrite

void Init_LCD(void)
{
    int i, j;
//  Initialize LCD according to the Web Page
    j = 2*Twentyms;
    for (i = 0; i < j; i++);            //  Wait for LCD to Power Up

// Initialize LCD 8 bit mode
    _LATB6 = 1;
    _LATB7 = 1;
    _LATB10 = 0;
    _LATB11 = 0;

    E = 1;
    E = 0;              		//  Send Command
    j = 3*Fivems;
    for (i = 0; i < j; i++);

    E = 1;
    E = 0;              		//  Repeat Command
    j = 8*TwoHundredus;
    for (i = 0; i < j; i++);

    E = 1;
    E = 0;              		//  Repeat Command Third Time
    j = 8*TwoHundredus;
    for (i = 0; i < j; i++);

//  Initialize LCD 4 Bit Mode
    _LATB6 = 0;
    _LATB7 = 1;
    _LATB10 = 0;
    _LATB11 = 0;

    E = 1;
    E = 0;
    j = 4*TwoHundredus;
    for (i = 0; i < j; i++);

    LCDWrite(0b00101000, 0);    		//  LCD is 4 Bit I/F, 2 Line

    LCDWrite(0b00001000, 0);                    //  display off

    LCDWrite(0b00000001, 0);    		//  Clear LCD

    LCDWrite(0b00000110, 0);    		//  Move Cursor After Each Character

    LCDWrite(0b00001100, 0);    		//  Turn On LCD
}  						//  End Init_LCD

void Clear_LCD(void)
{
	LCDWrite(0b00000001, 0);    		//  Clear LCD

}  						//End Clear_LCD

void LCD_Display(char Display[16])
{
    int ind;
	for (ind = 0; Display[ind] != 0; ind++)
        LCDWrite(Display[ind], 1);
}  						// End LCD_Display
/** END FUNCTIONS FROM LAB 4*/