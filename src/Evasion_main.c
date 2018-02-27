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
 *  find the distance from the obstacle and will read from an ultrasonic 
 * sensor, and the output to the 
 * outside world will be a PWM signal to a 
 * DC motor driver which will control the 
 * DC motor.
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

/*
 ;;;;; PW Times ;;
dtPW1	    EQU 2500
dtPW1_2	    EQU	3000
dtPW1_4	    EQU	3500
dtPW1_6	    EQU	4000	    
dtPW1_8	    EQU	4500
dtPW2	    EQU	5000
dtPW18	    EQU	45000	    
dtPW18_2    EQU	45500	    
dtPW18_4    EQU	46000 
dtPW18_6    EQU	46500	    
dtPW18_8    EQU 47000    	    
dtPW19	    EQU 47500	
 
 */

 
#include <p18cxxx.h>
#include <delays.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <timers.h>
#include <math.h>
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
far char Init_print[] = {0x80,'S','T','A','R','T',' ','U','P',0x00};
far char d_print[] = {0x80,'D','=','x','x','x',' ','c','m',0x00};
far char P_print[] = {0xC0,'P','=',' ',' ',' ','%',' ',' ',0x00};
unsigned int ALIVE_test[] = {976,8788}; // 4880
unsigned char y;
unsigned int PT_X, CHECK;
float PT_V, T;
int T_Temp;
int SW_check, word_Rx = 0;
int USART_INDEX, Rx_check,USART_INDEXold, bit_send;
int Done = 1;
unsigned int t0, tend, t_spent; // Time 
float t;
float d_old, d, Error, Integral, P,I,D;
unsigned float Motor, Motor_scale;
unsigned int TMR1X, CCPR1X;
// Driver
int d_num, Power_num;
unsigned int d_dec;
float  Power, oldPower;
long dt_on, dt_off;
int check_signal;
// Control Constants
const float kP = 3;
const float kD = 0;
const float kI = 0;
const float dt = .5;
const float Floc = 10;
const float Int_tol = 200;
const float tol = 20;
const float max_P = .90;
//float Motor_scale;
int Phase, Enable, sleep = 0;

/******************************************************************************
 * Function prototypes
 ******************************************************************************/
void Initial(void);         // Function to initialize hardware and interrupts
void HiPriISR(void);
void LoPriISR(void);
void TMR0handler(void);     // Interrupt handler for TMR0, typo in main
void Blinky(void);
void Port_init(void);
void Interr0_init(void);
void Timer0_init(void);
void Interr1_init(void);
void Ultrasonic_init(void);
void Timer1_init(void);
void TMR1handler(void);
void Control_loop(void);
void distance_measurement(void);
void LCD_print(void);
void Driver_int(void);
void Driver(void);

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
    Initial();                 // Initialize everything
    while(1) {
        distance_measurement(); // Find distance
        Control_loop();       // Control 
        Delay10KTCYx(20);       // Wait for dt = .5
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
    
    /***** Initialize the LCD and print to it ********/
    InitLCD();
    //DisplayC(Init_print);
    
    /******** Ultrasonic Sensor set up ************/
    Ultrasonic_init();
    
    /***************** Driver set up ****************/
    Driver_int();
    
    /******* LED blinky blinky ******************/
    Blinky();   

    /************** Initializing TMR0 **************/
    //Timer0_init();

    /******* Interrupt initialization routine ********/
    Interr0_init();
    
    /************ Initialize TMR1 *****************/
    Timer1_init();
    
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
    check_signal = 0;
    Integral = 0;
    oldPower = 0;
    Power = max_P;
    Motor_scale = max_P/400;
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

void Ultrasonic_init(void){
    TRISDbits.TRISD4 = 1; // Input echo pin
    TRISEbits.TRISE1 = 0; // Output trigger pin
    //ANCON1bits.ANSEL8 = 0;
}

/***********************
***** Set up Driver *****
************************/
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
    TMR1X++;                // Add one to the extension 
    PIR1bits.TMR1IF = 0;    // Clear the flag
}

/***********************
***** Timer3 set up *****
***********************
 * Timer use for measuring the 
 * waiting time for the sensor
 */
void Timer3_init(void){
    T3CON = 0b00001010;     // 8-bit, Fosc / 4, no pre/post scale timer
    TMR3L = 0;              // Clearing TMR0 registers
    TMR3H = 0;
}


/******************************************************************************
 * HiPriISR interrupt service routine
 *
 * Included to show form, does nothing
 ******************************************************************************/
#pragma interrupt HiPriISR
void HiPriISR() {
    
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
        if (CCPR1X < TMR1X){CCPR1X = TMR1X + 1;}
        
        if (PIR3bits.CCP1IF) {
            Driver();
            continue;
        }
        
        //INTCONbits.TMR0IF = 0;
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


void distance_measurement(void){
    T3CONbits.TMR3ON = 0;       // Turn off the timer
    WriteTimer3(0);             // Reset the timer
    T3CONbits.TMR3ON = 1;       // Turn on timer
    LATEbits.LATE1 = 0;         // make sure signal is low
    Delay1KTCYx(1);             // 2 ms delay
    LATEbits.LATE1 = 1;         // set signal high
    Delay1KTCYx(7);             // 10 ms delay
    LATEbits.LATE1 = 0;         // Set signal low
    t0 = ReadTimer3();          // Read Timer
    while(PORTDbits.AD4 == 0){}
    while(PORTDbits.AD4 == 1){} // Wait for the signal to go up and down
    tend = ReadTimer3();        // Read Timer
    t_spent = tend - t0;        // Get the counts
    t = (float) t_spent*4e-7;   // get the actual time
    d = t*16864;                // Get the distance 
    d_num = (int) d;
    d_dec = (unsigned int) ((d - d_num)*10);
    sprintf(Temp,"%i",d_num);   // save the value to a string
    if (d < 10){
        d_print[4] = '0';
        d_print[5] = '0';
        d_print[5] = Temp[0];

    }
    else if (d >= 10 && d < 100){
        d_print[3] = '0';
        d_print[4] = Temp[0];
        d_print[5] = Temp[1];
    //    d_print[5] = Temp[2];
    }
    else if (d >= 100){
        d_print[3] = Temp[0];
        d_print[4] = Temp[1];
        d_print[5] = Temp[2];
    }

    DisplayC(d_print);      // Print the value to the LCD
}

void Control_loop(void){
    float Error_abs;
    Error = d - Floc;       // Calculate the error
    
    
    // Calculate the absolute value
    if (Error < 0) {Error_abs = -1*Error;}
    (Error < 0)? (Error_abs = -1*Error) : (Error_abs = Error);  
    
    if (Error_abs > tol){
        if(Error < Int_tol){
            Integral = Integral + Error;  // Add the error if the error if it is  in tolerance
        }
        else{
            Integral = 0;  // Reset the error
        }
        // Find the const
        P = Error*kP;                   // Proportional gain
        I = Integral*kI*dt;             // Integral gain 
        D = (d_old - d)*kD/dt;          // Derivative gain
        Motor = P ;//+ I + D;
        Motor = Motor*Motor_scale; // If needs to be scaled
        
        /*if (Motor < 0){
            sleep = 0;
        }
        else {
            Phase = 0;
            //enable; // add pwm 
        }
        */
        LATEbits.LATE0 = 1; // Turn on sleep mode 
    }
    else if (Error > 400){
        Motor = max_P; // If wall is out of distance go full throttle
        LATEbits.LATE0 = 1; // Turn on sleep mode 
    }
    if (Error_abs < tol){
        //Motor = 0;
        LATEbits.LATE0 = 0; // Turn off sleep mode 
        //P_print[8] = '0';
    }
    // Transform error to Power
    if (Motor > max_P){
        Motor = max_P;
    }
    
    Power = Motor;          // Save the power to the PWM variable
    Power_num = (int) (Power*100);
    sprintf(Temp,"%i",Power_num);
    P_print[3] = Temp[0];
    P_print[4] = Temp[1];
    //P_print[5] = Temp[2];
    
    DisplayC(P_print);      // Print the Power
    d_old = d;              // Save the distance into old distance
    
}
float abs(float x){
    if (x < 0){x = -1*x; }
    return x;
}

void Driver(void){
    
    if (PIR1bits.TMR1IF == 1){    // Check to see if TMR1 Flag is set
        if (!(CCPR1H & (1 << (7)))){ // 
            TMR1X++;              // Increment Timer 1 upper
            PIR1bits.TMR1IF = 0;  // Clear flag
        }
    }    
    
    if (CCPR1X == TMR1X){ // Check if extensions are equal 
        if (check_signal == 0){ // Check if the pwm is low 
            if (Power > 1 || Power < 0){Power = oldPower;} // Make sure it doesn't over power it 
            dt_on = (long)(125000*Power); // Time on *Power
            dt_off = 125000 - dt_on; // Time off
            CCPR1 += (int)dt_off;
            if (STATUSbits.C == 1){
                CCPR1X += 1 ;    // Adds 1 to the extra count if there is a carrier
            }
            CCPR1X += (dt_off>>16);
            LATBbits.LATB5 = 1;
            CCP1CONbits.CCP1M0 = 1;
            check_signal = 1;
        }
        else if (check_signal == 1){
            CCPR1 += (int) dt_on;
            if (STATUSbits.C == 1){
                CCPR1X +=  1;    // Adds 1 to the extra count if there is a carrier
            }
            CCPR1X += (dt_on>>16);
            LATBbits.LATB5 = 0; //to turn off D2
            CCP1CONbits.CCP1M0 = 0;
            check_signal = 0;
        }
    }
    oldPower = Power;
    PIR3bits.CCP1IF = 0;  // Clear Flag
}

/************Bit Manipulation *********************
LATB ^= (bitmask << 4); //to toggle the Index bit
LATB &= ~(bitmask << 4); //to clear the Index bit
LATB |= (bitmask << 4); //to set the Index bit
 ****************************************************/