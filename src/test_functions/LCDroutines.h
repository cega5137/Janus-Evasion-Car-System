/******************************************************************************
 *
 * Author: Gabriel LoDolce
 * Author of last change: Matthew Paley
 * Date of last change: 7/25/2014
 * Revision: 1.0
 *
 *******************************************************************************
 *
 * FileName:        LCDroutines.h
 * Dependencies:    Delays.h and p18cxxx.h
 * Processor:       PIC18F
 * Compiler:        C18
 *
 *******************************************************************************
 * File Description: This library contains a set of functions for the Optrex 8x2
 *  character LCD display with the Hitachi HD44780U driver. It is configured
 *  to work with the setup of the QwikFlash development board running a 10MHz
 *  clock. It can be adapted for other pin setups by redefining the pin assignment
 *  #defines in the header below. It can be configured for other oscillator
 *  frequencies by modifying the delay constants in the implementation file.
 * 
 ******************************************************************************/

#include <p18cxxx.h>

#ifndef _LCD_ROUTINES_H_
#define _LCD_ROUTINES_H_

/*------------------------------------------------------------------------------
 * Definitions for this LCD interface
 -----------------------------------------------------------------------------*/

/* RS Pin Assignments */
#define LCD_RS_TRIS     TRISHbits.TRISH1   //These structs are defined in p18f87k22.h
#define LCD_RS_LAT      LATHbits.LATH1

/* E Pin Assignments */
#define LCD_E_TRIS	TRISHbits.TRISH2
#define LCD_E_LAT	LATHbits.LATH2

/* Data Pin Assignments. Note we only need the upper nibble
** but it is hard to break apart by nibbles. We will assume
** that writing to the whole byte is not an issue. THIS IS UP
** TO THE USER TO VERIFY
*/
#define LCD_DATA_TRIS TRISJ //Will change all of PORTJ but we only use RJ4:7 upper bits
#define LCD_DATA_LAT LATJ

/*------------------------------------------------------------------------------
 * Public Library Functions
 -----------------------------------------------------------------------------*/

/******************************************************************************
 *     Function Name:	InitLCD
 *     Parameters:      None
 *     Description:		This function initializes the Optrex 8x2 character LCD.
 *						This function generates a 0.1s and 0.01s delay using the
 *						c18 Delay library
 *
 ******************************************************************************/
void InitLCD( void );

/******************************************************************************
 *     Function Name:	DisplayC
 *     Parameters:      Pointer to a character array in program memory
 *     Description:		This function sends a character array in program memory
 *						to the LCD display. Note the first character of the
 *						string is the positioning commmand. The string must
 *						also be terminated by the null character
 *
 *						This function generates a 40us delay using the c18
 *						Delay library
 *
 ******************************************************************************/
void DisplayC( far char * LCDStr );

/******************************************************************************
 *     Function Name:	DisplayV
 *     Parameters:      Pointer to a character array in data memory
 *     Description:		This function sends a character array in data memory
 *						to the LCD display. Note the first character of the
 *						string is the positioning commmand. The string must
 *						also be terminated by the null character
 *
 *						This function generates a 40us delay using the c18
 *						Delay library
 *
 ******************************************************************************/
void DisplayV( char * LCDStr );

#endif


