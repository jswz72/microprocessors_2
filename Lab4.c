#include "xc.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


/********************************************************
 * Lab 4
 * Jacob Sword and Jack Degregorio
 * 
 * 

********************************************************/

// PIC24HJ32GP202 Configuration Bit Settings

// FBS
#pragma config BWRP = WRPROTECT_OFF     // Boot Segment Write Protect (Boot Segment may be written)
#pragma config BSS = NO_FLASH           // Boot Segment Program Flash Code Protection (No Boot program Flash segment)

// FGS
#pragma config GWRP = OFF               // General Code Segment Write Protect (User program memory is not write-protected)
#pragma config GSS = OFF                // General Segment Code Protection (User program memory is not code-protected)

// FOSCSEL
#pragma config FNOSC = FRC              // Oscillator Mode (Internal Fast RC (FRC))
#pragma config IESO = OFF               // Internal External Switch Over Mode (Start-up device with user-selected oscillator source)

// FOSC
#pragma config POSCMD = NONE            // Primary Oscillator Source (Primary Oscillator Disabled)
#pragma config OSCIOFNC = ON            // OSC2 Pin Function (OSC2 pin has digital I/O function)
#pragma config IOL1WAY = OFF            // Peripheral Pin Select Configuration (Allow Multiple Re-configurations)
#pragma config FCKSM = CSDCMD           // Clock Switching and Monitor (Both Clock Switching and Fail-Safe Clock Monitor are disabled)

// FWDT
#pragma config WDTPOST = PS512          // Watchdog Timer Postscaler (1:512)
#pragma config WDTPRE = PR128           // WDT Prescaler (1:128)
#pragma config WINDIS = OFF             // Watchdog Timer Window (Watchdog Timer in Non-Window mode)
#pragma config FWDTEN = OFF             // Watchdog Timer Enable (Watchdog timer enabled/disabled by user software)

// FPOR
#pragma config FPWRT = PWR16            // POR Timer Value (16ms)
#pragma config ALTI2C = OFF             // Alternate I2C  pins (I2C mapped to SDA1/SCL1 pins)

// FICD
#pragma config ICS = PGD1               // Comm Channel Select (Communicate on PGC1/EMUC1 and PGD1/EMUD1)
#pragma config JTAGEN = OFF             // JTAG Port Enable (JTAG is Disabled)

#define E  _LATA0             		//  Define the LCD Control Pins
#define RS _LATB5

const int Twentyms = 3000;
const int Fivems = 1000;
const int TwoHundredus = 40;

// LCD Functions
void Configure_LCD_pins(void);
void LCDWrite(int LCDData, int RSValue);
void Init_LCD(void);
void Clear_LCD(void);
void LCD_Display(char Display[16]);

//I2C Functions
void init_I2C(void);
void nack(void);
void ack(void);
void restart();
void stop();
void start();
void wait_for_idle();
void write_to_rtc(int);
void read_from_rtc(int*);

void delay_routine(void)
{

	unsigned short i, j;			// 16 bits
	
		for (i = 0; i < 800; i++)  
                for (j = 0; j < 800; j++);
	return;

}

int main()
{

    char Display[32];
    int sec_val; 
    int sec_val1 = 0; 
    int sec_val2 = 0;
    int min_val;
    int min_val1 = 0;
    int min_val2 = 0;
    int hr_val;
    int hr_val1 = 0;
    int hr_val2 = 0;
    int day_val;
    int date_val, date_val1, date_val2, month_val, month_val1, month_val2, year_val, year_val1, year_val2;
    
    int time_config[7] = {
        0x00,   // second
        0x20,   // minute
        0x15,   // hour
        0x05,   // day of week
        0x09,   // date of month
        0x11,   // month
        0x18    // year
    };
    
    Configure_LCD_pins();

    Init_LCD();					//initialize the LCD Display
    Clear_LCD();				//clear the LCD screen
    
    init_I2C();
    start();
    wait_for_idle();
    rtc_set_time(time_config, 7);
    
    
    while (1) {
        stop();
        start();
        write_to_rtc(0xD0); //x68 is addr, followed by binary 0 for write
        write_to_rtc(0x00); //reset address pointer
        stop();
        start();
        write_to_rtc(0xD1); //addr + 1 for read now
        Clear_LCD();
        
        I2C1CONbits.RCEN = 1;   //enable receiver mode
        while (I2C1CONbits.RCEN);   //wait till over
        read_from_rtc(&sec_val);
        ack();
        
        I2C1CONbits.RCEN = 1;   //enable receiver mode
        while (I2C1CONbits.RCEN);   //wait till over
        read_from_rtc(&min_val);
        ack();
        
        I2C1CONbits.RCEN = 1;   //enable receiver mode
        while (I2C1CONbits.RCEN);   //wait till over
        read_from_rtc(&hr_val);
        ack();
        
        I2C1CONbits.RCEN = 1;   //enable receiver mode
        while (I2C1CONbits.RCEN);   //wait till over
        read_from_rtc(&day_val);
        ack();
        
        I2C1CONbits.RCEN = 1;   //enable receiver mode
        while (I2C1CONbits.RCEN);   //wait till over
        read_from_rtc(&date_val);
        ack();
        
        I2C1CONbits.RCEN = 1;   //enable receiver mode
        while (I2C1CONbits.RCEN);   //wait till over
        read_from_rtc(&month_val);
        ack();
        
        I2C1CONbits.RCEN = 1;   //enable receiver mode
        while (I2C1CONbits.RCEN);   //wait till over
        read_from_rtc(&year_val); 
        nack();
        
        hr_val1 = hr_val & 0x0F;
        hr_val2 = hr_val >> 4;
        min_val1 = min_val & 0x0F;
        min_val2 = min_val >> 4;
        sec_val1 = sec_val & 0x0F;
        sec_val2 = sec_val >> 4;
        date_val1 = date_val & 0x0F;
        date_val2 = date_val >> 4;
        month_val1 = month_val & 0x0F;
        month_val2 = month_val >> 4;
        year_val1 = year_val & 0x0F;
        year_val2 = year_val >> 4;
        
        sprintf(Display,"Time: %d%d:%d%d:%d%d", hr_val2, hr_val1, min_val2, min_val1, sec_val2, sec_val1);
        LCD_Display(Display);
        LCDWrite(0b11000000, 0);    		//  Move Cursor to the Second Line
        sprintf(Display,"Date: %d %d%d-%d%d-%d%d", day_val, date_val2, date_val1, month_val2, month_val1, year_val2, year_val1);
        LCD_Display(Display);
    }
    return 0;
 
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

//    if ((0 == (LCDData & 0xFC)) && (0 == RSValue))
//        n = Fivems;
//    else
//        n = TwoHundredus;
        
    for (k = 0; k < Fivems; k++);    		//  Delay for Character

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

/*******************I2C FUNCTIONS***********************/

void wait_for_idle(void)
{
    // I2C1CONbits = 00000 corresponds to bus idle/wait
    while(I2C1CON & 0x1F); // Acknowledge sequences/conditions not in progress
    while(I2C1STATbits.TRSTAT); // Ensure Master transmit is not in progress
}

void init_I2C(void)
{
    I2C1BRG = 34;           //baud rate generator
    I2C1CONbits.I2CEN = 1;  //enable I2C module --> I2C pins controlled by ports
    I2C1CONbits.DISSLW = 1; //slew rate control
}

void start(void)
{
    wait_for_idle();
    I2C1CONbits.SEN = 1;     //start condition
    while (I2C1CONbits.SEN); //wait till end of start sequence
}

void stop(void)
{
    wait_for_idle();
    I2C1CONbits.PEN = 1;    // stop condition
    while (I2C1CONbits.PEN); //wait till stop sequence over
}

void restart(void)
{
    wait_for_idle();
    I2C1CONbits.RSEN = 1;   //initiate repeated start on SDAx/SCLx
    while (I2C1CONbits.RSEN);    //wait till repeated start sequence over
}

void ack(void)
{
    wait_for_idle();
    I2C1CONbits.ACKDT = 0;  //send ack during acknowledge
    I2C1CONbits.ACKEN = 1;  //init acknowledge --> trans ACKDT
    while (I2C1CONbits.ACKEN);  //wait till ack seq over
}

void nack(void)
{
    wait_for_idle();
    I2C1CONbits.ACKDT = 1;  //send nack during acknowledge
    I2C1CONbits.ACKEN = 1;  //init acknowledge --> trans ACKDT
    while (I2C1CONbits.ACKEN);   //wait till nack seq over
}

void write_to_rtc(int val)
{
    I2C1TRN = val; //55 seconds
    while (I2C1STATbits.TRSTAT);    //wait till trans over
}

void read_from_rtc(int *buffer)
{
    while(!I2C1STATbits.RBF);   //wait until receive buffer full
    *buffer = I2C1RCV;  //take from full receive register
}

void rtc_set_time(int time_config[], int size)
{
    int i;
    write_to_rtc(0xD0);  //x68 is addr, followed by binary 0 for write
    write_to_rtc(0); //send beginning addr, will auto-increment in following writes
    for (i = 0; i < size; i++) {
        write_to_rtc(time_config[i]);
    }
}
