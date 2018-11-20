/******************************************************************************
 * 
 * Program Title: Project_rps.c
 * 
 * Microprocessors II EECE.4800
 * 
 * Rock-Paper-Scissors Game
 *
 * Fall 2018
 * 
 * Jacob Sword, Jack DeGregorio
 * 
 * 11/14/2018*/

// This is a test comment to see if I can edit the git repo

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

#define SWITCH_UP 1
#define SWITCH_DOWN 0;
#define SWITCH PORTBbits.RB13
void __attribute__((__interrupt__, no_auto_psv)) _T2Interrupt(void)
{
_LATA2 = PORTBbits.RB15;
//_LATA2 = 1;
IFS0bits.T2IF = 0; //Clear Timer interrupt flag
}

void delay_routine(void)
{

	unsigned short i, j;			// 16 bits

		for (i = 0; i < 100; i++)
            for (j = 0; j < 100; j++);
	return;

}


/********************************************************
* Function: main
*
* Description: button pressed toggles LED of and on
*
* Notes:
*
* RB13 input from button
 *RB14 output to LED
 * 
* Returns:  This routine contains an infinite loop
*
********************************************************/
int main()
{
  _ODCA4 = 0; 					// Disable open drain
 _TRISA4 = 0;					// Config RB14 as output
 _LATA4 = 0;					// RB14 initially low   
 
 _ODCA3 = 0; 					// Disable open drain
 _TRISA3 = 0;					// Config RB14 as output
 _LATA3 = 0;					// RB14 initially low   
 
  _ODCA2 = 0; 					// Disable open drain
 _TRISA2 = 0;					// Config RB14 as output
 _LATA2 = 0;					// RB14 initially low 
 
  _ODCB4 = 0; 					// Disable open drain
 _TRISB4 = 0;					// Config RB14 as output
 _LATB4 = 0;					// RB14 initially low 
 
  _ODCB2 = 0; 					// Disable open drain
 _TRISB2 = 0;					// Config RB14 as output
 _LATB2 = 0;					// RB14 initially low 
 
 _ODCB3 = 0; 					// Disable open drain
 _TRISB3 = 0;					// Config RB14 as output
 _LATB3 = 0;					// RB14 initially low 
 
 _ODCB15 = 0; 					// Disable open drain
 _TRISB15 = 1;					// Config RB13 as input
 _CN11PUE = 1;                  // Enables Internal Weak-pull-up
 _PCFG9 = 1;                   // Disables analog function
 
 _ODCB14 = 0; 					// Disable open drain
 _TRISB14 = 1;					// Config RB13 as input
 _CN12PUE = 1;                  // Enables Internal Weak-pull-up
 _PCFG10 = 1;                   // Disables analog function
 
 _ODCB13 = 0; 					// Disable open drain
 _TRISB13 = 1;					// Config RB13 as input
 _CN13PUE = 1;                  // Enables Internal Weak-pull-up
 _PCFG11 = 1;                   // Disables analog function
 
  _ODCB12 = 0; 					// Disable open drain
 _TRISB12 = 1;					// Config RB13 as input
 _CN14PUE = 1;                  // Enables Internal Weak-pull-up
 _PCFG12 = 1;                   // Disables analog function


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
 while(1) {
         
 }
}
/******** END OF main ROUTINE ***************************/

