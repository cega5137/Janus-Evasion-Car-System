/******************************************************************************
 *
 * Author: Gabriel LoDolce
 * Author of last change: Trudy Schwartz
 * Date of last change: 8/9/2016
 * Revision: 1.0
 *
 *******************************************************************************
 * 
 * FileName:        LCDroutines.c
 * Dependencies:    Delays.h and p18cxxx.h
 * Processor:       PIC18F
 * Compiler:        C18
 *
 *******************************************************************************
 *
 * File Description: Implementation file for the LCD routines library
 *
 ******************************************************************************/

#include <Delays.h>
#include <string.h>
#include "LCDroutines.h"

/*------------------------------------------------------------------------------
 * Static variables for internal use (not visible outside of this file)
 -----------------------------------------------------------------------------*/

/* The following instruction delays assume 10MHz clock. These can be changed for
** other oscillator frequencies but note the valid range is only 0-255
*/
static const unsigned char InitDelayCount_ = 25; // Instruction cycles/10000 for 0.1 second delay using Delay10KTCY
                                                 //     Also used for Inst Cycles/1000 for 10 ms delay in Delay1KTCY
static const unsigned char CharDelayCount_ = 10; // Instruction cycles/10 for 40usec delay using Delay10TCY
                                                 // This last delay should be 10 not 30 from given file!!
static const rom far char LCDInitStr_[] = "\x33\x32\x28\x01\x0C\x06\x00"; // LCD initialization

/*------------------------------------------------------------------------------
 * Public functions intended for the user
 -----------------------------------------------------------------------------*/

/********************************************************************
 *     Function Name:    
 *     Parameters:       
 *     Description:      
 *
 ********************************************************************/
void InitLCD(void) {
	unsigned char count = 0;    //Used uchar to save memory space & time

	// Delay 0.1 second for the LCD controller to come out of reset
	Delay10KTCYx( InitDelayCount_ );

	// Drive RS low for command
	LCD_RS_LAT = 0;

	// Send Each Byte one nibble at a time until we see the null character
	while( LCDInitStr_[count] != 0x00 ) {
		LCD_E_LAT = 1;                          // Drive E high
		LCD_DATA_LAT = LCDInitStr_[count];      // Send the upper nibble, last four bits should be ignored
		LCD_E_LAT = 0;                          // Drive E low so LCD will process input
		Delay1KTCYx( InitDelayCount_);          // Delay 25*1000 is 10ms
   
		LCD_E_LAT = 1;                          // Drive E high
		LCD_DATA_LAT = LCDInitStr_[count] << 4;	// Send the lower nibble, bit shift left 4 times
		LCD_E_LAT = 0;                          // Drive E low so LCD will process input
		Delay1KTCYx( InitDelayCount_);          // Delay 25*1000 is 10ms

		count++;                                // Increment the counter
	}

	LCD_RS_LAT = 1;                                 // Drive RS high to exit command mode
}

/******************************************************************************
 *     Function Name:	DisplayC
 *     Parameters:      Pointer to a character array in program memory
 *     Description:		This function sends a character array in program memory
 *						to the LCD display. Note the first character of the
 *						string is the positioning commmand. The string must
 *						also be terminated by the null character
 *
 *						
 *
 ******************************************************************************/
void DisplayC( far char * LCDStr ) {
    char temp[10];   // Temporary buffer to store input in, 10 bits(loc,8data,null)
    //strcpypgm2ram(temp,LCDStr); // Creating proper data input for DisplayV ?????
                                // Moves LCDStr from prog mem to temp in ram, why is this necessary?
    strcpy(temp,LCDStr);
    DisplayV(temp);             // Calling DisplayV function
}

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
void DisplayV( char * LCDStr ) {
    unsigned char count = 0;        //Set type and init count to 0
    LCD_RS_LAT = 0;                 // Telling the LDC we are about to transmit
    while(LCDStr[count] != 0x00) {
        // Transmitting the upper nibble
        LCD_E_LAT = 1;
        LCD_DATA_LAT = LCDStr[count];
        LCD_E_LAT = 0;

        // Transmitting the lower nibble
        LCD_E_LAT = 1;
        LCD_DATA_LAT = (LCDStr[count] << 4);
        LCD_E_LAT = 0;

        Delay10TCYx( CharDelayCount_ ); // Delay 40usec to allow LCD to display char

        count++;
        LCD_RS_LAT = 1;
    }
}
