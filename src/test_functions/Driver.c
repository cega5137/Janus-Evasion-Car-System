/****** ASEN 4/5519 Lab 6 ******************************************************
 * Author: Cesar Galan
 * Date  : 10/27/2016
 *
 * Description
 * On power up exceute the following sequence:
 *      D2 ON for 0.5s +/- 10ms then off
 *      D3 ON for 0.5s +/- 10ms then off
 *      D4 ON for 0.5s +/- 10ms then off
 * The following then occurs forever:
 *      D6 blinks: 100ms +/- 10ms ON, then 900ms +/- 10ms OFF
 *      LCD Displays the following lines:
 *          'T=xx.x F'
 *          'PT=x.xxV'
 *      Where the 'x' is replaced by a digit in the measurement.
 *          Temperature data must be calculated / displayed with one digit to
 *          the right of the decimal as shown.  The sensor itself can have
 *          errors up to +/- 5 degrees Fahrenheit.
 *          Potentiometer data must be calculated / displayed with two digits
 *          to the right of the decimal as shown.
 *          These measurements must be refreshed at LEAST at a frequency of 5Hz.
 *      USART Commands are read / executed properly. '\n' is a Line Feed char (0x0A)
 *          ASEN 4519:
 *              'TEMP\n'     - Transmits temperature data in format: 'XX.XF'
 *              'POT\n'      - Transmits potentiometer data in format: X.XXV'
 *          ASEN 5519: Same as ASEN 4519, plus two additional commands
 *              'CONT_ON\n'  - Begins continuous transmission of data over USART
 *              'CONT_OFF\n' - Ends continuous transmission of data over USART
 *
 *              Continuous transmission should output in the following format:
 *                  'T=XX.XF; PT = X.XXV\n'
 *******************************************************************************
 *
 * Program hierarchy 
 *
 * Mainline
 *   Initial
 *
 * HiPriISR (included just to show structure)
 *
 * LoPriISR
 *   TMR0handler
 ******************************************************************************/

 
#include <p18cxxx.h>
#include <delays.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <timers.h>
#include "LCDroutines.h"
#pragma config FOSC=HS1, PWRTEN=ON, BOREN=ON, BORV=2, PLLCFG=OFF
#pragma config WDTEN=OFF, CCP2MX=PORTC, XINST=OFF

/******************************************************************************
 * Global variables
 ******************************************************************************/
const rom far char LCDRow1[] = {0x80,'T','=','x','x','.','x','F',' ',0x00};
const rom far char LCDRow2[] = {0xC0,'P','T','=','x','.','x','x','V',0x00}; // Change line
unsigned int Alive_count = 0, USART_Check = 0;
unsigned char bitmask = 0x01 ;
char Temp[1]; //Temp_str = "TEMP", Pot_str = "Pot",Temp_str[5];
char Temp_str[5], Pot_str[] = "POT", TEMP_str[] = "TEMP";
far char T_print[] = {0x80,'T','=','x','x','.','x','F',' ',0x00};
far char PT_print[] = {0xC0,'P','T','=','x','.','x','x','V',0x00};
unsigned int ALIVE_test[] = {976,8788}; // 4880
unsigned char y;
unsigned int PT_X, CHECK;
float PT_V, T;
int T_Temp;
int SW_check, word_Rx = 0;
int USART_INDEX, Rx_check,USART_INDEXold, bit_send;
//char USART_Buffer[4] = "    ";
int Done = 1;
unsigned int t0, tend, t_spent;
float t;
// For the Driver
unsigned float TMR1X, CCPR1X;
int check_signal = 0;
float dt_on, dt_off;
const float Power = .5;
int DTIME;

/******************************************************************************
 * Function prototypes
 ******************************************************************************/
void Initial(void);         // Function to initialize hardware and interrupts
void HiPriISR(void);
void LoPriISR(void);
void TMR0handler(void);     // Interrupt handler for TMR0, typo in main
void USARThandler_Tx(void);
void Blinky(void);
void Port_init(void);
void Interr0_init(void);
void Timer0_init(void);
void Interr1_init(void);
void Word_Rx(void);
//void Ultrasonic_init(void);
void Timer1_init(void);
void Driver(void);
void Driver_int(void);
void TMR1handler(void);

#pragma code highVector=0x08
void atHighVector(void)
{
 _asm GOTO HiPriISR _endasm
}
#pragma code

#pragma code lowVector=0x18
void atLowVector(void)
{
 _asm GOTO LoPriISR _endasm
}
#pragma code

/******************************************************************************
 * main()
 ******************************************************************************/
void main() {
    //int test = 0;
    Initial();                 // Initialize everything
    
    while(1) {
        /*T1CONbits.TMR1ON = 0;
        WriteTimer1(0);
        T1CONbits.TMR1ON = 1;
        t0 = ReadTimer1();
        //while(PORTCbits.RC4 == 0){}
        while(test < 10){test++;}
        tend = ReadTimer1();
        t_spent = tend - t0;
        t = (float) t_spent*0.0000004;
         */ 
    }
}
/******************************************************************************
 * Initial()
 *
 * This subroutine performs all initializations of variables and registers.
 * It enables TMR0 and sets CCP1 for compare, and enables LoPri interrupts for
 * both.
 ******************************************************************************/
void Initial() {
    /******************Port Set up******************/   
    Port_init();
    
    /******** Ultrasonic Sensor set up ************/
    //Ultrasonic_init();
    
    /***************** Set up Driver  ****************/
    Driver_int();
    
    /******* LED blinky blinky ******************/
    Blinky();
    
    /***** Initialize the LCD and print to it ********/
    //InitLCD();

    /************** Initializing TMR0 **************/
    Timer0_init();

    /************ Initialize TMR1 *****************/
    Timer1_init();
    
    /******* Interrupt initialization routine ********/
    Interr0_init();
    
    
    
    /****** Set up DAC ***********/
    //SPI_init();
    
    // Dummy variables //
    y = 0;
    CHECK = 0;
    SW_check = 0;
    USART_INDEX = 0;
    USART_INDEXold = 0;
    Rx_check = 0;
    bit_send = 1;
    TMR1X = 0;
    CCPR1X = 0;
}

/***********************
***** Port Set up *****
************************/
void Port_init(void){
    // Configure the IO ports
    TRISB   = 0b00001111;
    LATB    = 0b00000000;
    TRISC   = 0b10010011;
    LATC    = 0b00000000;
    
    // Set Potentiometer and thermometer
    TRISA   = 0b00000011;
    LATA    = 0b00000000;
    
    // Configure the LCD pins for output. Defined in LCDRoutines.h
    LCD_RS_TRIS   = 0;              // 
    LCD_E_TRIS    = 0;
    LCD_DATA_TRIS = 0b00001111;     // Note the LCD is only on the upper nibble
                                    // The lower nibble is all inputs
}
void Driver_int(void){
    // Add initialization from lab 5 
    CCP1CON = 0b00001000;
    TRISEbits.TRISE0 = 0; // Output trigger pin Change this pin 
    IPR1bits.TMR1IP = 0; // Assign low priority to TMR1
    IPR3bits.CCP1IP = 0; // Assign low priority to CCP1
    PIE1bits.TMR1IE = 1; // Enable CCP1 interrupts 
    PIE3bits.CCP1IE = 1; // Enable TMR1 interrupts
    TRISCbits.TRISC2 = 0;
    LATCbits.LATC2 = 0;
    CCPR1L = 62500;
    CCPR1X = 0;
    LATEbits.LATE0 = 1; // Turn on sleep mode
}

/* Initialize sequence*/
void Blinky(void){
    Delay10KTCYx(125);
    LATB |= (bitmask << 5); //to set D2
    Delay10KTCYx(125);
    LATB &= ~(bitmask << 5); //to clear D2
    LATB |= (bitmask << 6); //to set D3
    Delay10KTCYx(125);
    LATB &= ~(bitmask << 6); //to clear D3
    LATB |= (bitmask << 7); //to set D4
    Delay10KTCYx(125);
    LATB &= ~(bitmask << 7); //to clear D4   

}

/***********************
***** Timer0 set up *****
************************/
void Timer0_init(void){
    T0CON = 0b01001000;     // 8-bit, Fosc / 4, no pre/post scale timer
    TMR0L = 0;              // Clearing TMR0 registers
    TMR0H = 0;
}

/***********************
***** Interrupts0 set up**
************************/
void Interr0_init(void){
    // Configuring Interrupts
    RCONbits.IPEN = 1;              // Enable priority levels
    INTCON2bits.TMR0IP = 0;         // Assign low priority to TMR0 interrupt

    INTCONbits.TMR0IE = 1;          // Enable TMR0 interrupts
    INTCONbits.GIEL = 1;            // Enable low-priority interrupts to CPU
    INTCONbits.GIEH = 1;            // Enable all interrupts

    T0CONbits.TMR0ON = 1;           // Turning on TMR0
}


/***********************
***** Interrupts0 set up**
************************/
void Interr1_init(void){
    // Configuring Interrupts
    
    // Set PIR3 / 
    PIR1bits.RC1IF = 0; // Receive
    //PIR1bits.TX1IF = 0; // Transmit
    
    // Set PIE3  On or off the transmit
    PIE1bits.RC1IE = 1; // Turn on Transmit
    //PIE1bits.TX1IE = 1; // Turn on Receive
    
    // Set IPR3, Priority level
    IPR1bits.RC1IP = 1;
    //IPR1bits.TX1IP = 0;
}


/***********************
***** Timer1 set up *****
***********************
 * Timer use for measuring the 
 * waiting time for the sensor
 */
void Timer1_init(void){
    T1CON = 0b00000010;     // 8-bit, Fosc / 4, no pre/post scale timer
    TMR1L = 0;              // Clearing TMR0 registers
    TMR1H = 0;
    T1CONbits.TMR1ON = 1;  // Turn on Timer
}

void TMR1handler(void){
    TMR1X++;
    PIR1bits.TMR1IF = 0;
}

/******************************************************************************
 * HiPriISR interrupt service routine
 *
 * Included to show form, does nothing
 ******************************************************************************/
#pragma interrupt HiPriISR
void HiPriISR() {
    while(1) { 
        /*if (PIR3bits.CCP1IF) {
            Driver;
            continue;
        }*/
        break;
    }
}	// Supports retfie FAST automatically

/******************************************************************************
 * LoPriISR interrupt service routine
 *
 * Calls the individual interrupt routines. It sits in a loop calling the required
 * handler functions until until TMR0IF and CCP1IF are clear.
 ******************************************************************************/
#pragma interruptlow LoPriISR nosave=TBLPTR, TBLPTRU, TABLAT, PCLATH, PCLATU, PROD, section("MATH_DATA")//, section(".tmpdata")
void LoPriISR() {
    while(1) {
        if (PIR3bits.CCP1IF) {
            Driver();
            continue;
        }
        
        if( INTCONbits.TMR0IF ) {
            TMR0handler();
            continue;
        }
        
        if( PIR1bits.TMR1IF){
            TMR1handler();
            continue;
        }
        
        break;
    }
}

/*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 * Change the temporary data section for all code following this #pragma
 * statement. Normally the compiler/linker uses .tmpdata section for all non
 * ISR code and a separate tempdata section for each ISR (Hi and Lo). However,
 * when your primary ISR calls other functions the called functions share the
 * .tmpdata section with the main code. This can cause issues, just like not
 * saving WREG, STATUS and BSR. There are two options:
 *
 *   1. have the ISR save the .tmpdata section. This may have large effects on
 *      latency
 *   2. Force the functions called by the ISR to have their own temp data
 *      section.
 *
 * We are using option 2. However, this means that no main code functions can
 * appear after the following pragma, unless you change the temp data section
 * back.
 *
 * The temp data section is used for complex math operations such as 
 * a = b*c + d*e so you may not need a temp data section depending on what
 * you are doing.
 *
 * Keep in mind you may have the same issue with the MATH_DATA section.
 * MATH_DATA is used for arguments, return values and temporary locations
 * for math library functions.
 *!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/
#pragma tmpdata handler_temp

/******************************************************************************
 * TMR0handler interrupt service routine.
 *
 * Handles Alive LED Blinking via counter
 ******************************************************************************/
void TMR0handler() { // < 4880
    if( Alive_count < ALIVE_test[y] ) { Alive_count++; } 
    else {
        LATBbits.LATB4 = ~LATBbits.LATB4; // 4
        y ^= (bitmask << 0); //to toggle the Index bit
        Alive_count = 0;
    }

    INTCONbits.TMR0IF = 0;      //Clear flag and return to polling routine
}


//*********************** This function goes to main***************/
void Driver(void){
    if (PIR1bits.TMR1IF == 1){    // Check to see if TMR1 Flag is set
        if (!(CCPR1H & (1 << (7)))){ // 
            TMR1X++;              // Increment Timer 1 upper
            PIR1bits.TMR1IF = 0;  // Clear flag
        }
    }    
    
    if (CCPR1X == TMR1X){ // Check if extensions are equal 
        if (check_signal == 0){ // Check if the pwm is low 
            dt_on = 125000*Power; // Time on *Power
            dt_off = 125000 - dt_on; // Time off
            CCPR1 += dt_on;
            if (STATUSbits.C == 1){
                CCPR1X += 1;    // Adds 1 to the extra count if there is a carrier
            }
            LATBbits.LATB5 = 1;
            CCP1CONbits.CCP1M0 = 1;
            check_signal = 1;
            
        }
        else if (check_signal == 1){
            CCPR1 += dt_off;
            if (STATUSbits.C == 1){
                CCPR1X += 1;    // Adds 1 to the extra count if there is a carrier
            }
            LATBbits.LATB5 = 0; //to turn off D2
            CCP1CONbits.CCP1M0 = 0;
            check_signal = 0;
        }
    }
    
    PIR3bits.CCP1IF = 0;  // Clear Flag
}


/************Bit Manipulation *********************
LATB ^= (bitmask << 4); //to toggle the Index bit
LATB &= ~(bitmask << 4); //to clear the Index bit
LATB |= (bitmask << 4); //to set the Index bit
 ****************************************************/