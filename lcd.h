/* 
 * File:   lcd.h
 * Author: Eike
 *
 * Created on April 14, 2021, 11:31 PM
 */

#ifndef LCD_H
#define	LCD_H

#ifdef	__cplusplus
extern "C" {
#endif
    
//#include <stdbool.h>
//#include <stdint.h>

// Commands
#define LCD_Address                   0x3E //i2c address 7bit 0b0111110
#define CNTRBIT                       0x00 //followed by command bytes
#define CNTRBIT_CO                    0x80 //followed by 1 command byte
#define CNTRBIT_RS                    0x40 //last control byte, followed by DDRAM data byte(s)
#define CLEAR_DISPLAY                 0x01 //Clear display
#define RETURN_HOME                   0x02 //Cursor home to 00H
#define ENTRY_MODE_SET                0x04 //Sets cursor move direction and specifies display shift.
#define DISPLAY_ON_OFF                0x08 //display on, cursor on, cursor position on
#define FUNCTION_SET                  0x20 //DL: interface data is 8/4 bits, N: number of line is 2/1 DH: double height font, IS: instruction table select
#define SET_DDRAM_ADDRESS             0x80 //Set DDRAM address in address counter
#define CURSOR_OR_DISPLAY_SHIFT       0x10 //Set cursor moving and display shift control bit, and the direction without changing DDRAM data.
#define SET_CGRAM_ADDRESS             0x40 //Set CGRAM address in address counter
#define INTERNAL_OSC_FREQ             0x10 //BS=1:1/4 bias, BS=0:1/5 bias, F2~0: adjust internal OSC frequency for FR frequency.
#define POWER_ICON_BOST_CONTR         0x50 //Ion: ICON display on/off, Bon: set booster circuit on/off, C5,C4: Contrast set
#define FOLLOWER_CONTROL              0x60 //Fon: set follower circuit on/off, Rab2~0: select follower amplified ratio.
#define CONTRAST_SET                  0x70 //C0-C3: Contrast set 
#define LINE_1_ADR                    0x00
#define LINE_2_ADR                    0x40

// Various flags and masks
#define ENTRY_MODE_SET_S              0x01 //S: Shift of entire display, see data sheet
#define ENTRY_MODE_SET_ID             0x02 //I/D : Increment / decrement of DDRAM address (cursor or blink), see  data sheet
#define DISPLAY_ON_OFF_B              0x01 //cursor position on
#define DISPLAY_ON_OFF_C              0x02 //cursor on
#define DISPLAY_ON_OFF_D              0x04 //display on
#define FUNCTION_SET_IS               0x01 //IS: instruction table select
#define FUNCTION_SET_DH               0x04 //DH: double height font
#define FUNCTION_SET_N                0x08 //N: number of line is 2/1
#define FUNCTION_SET_DL               0x10 //DL: interface data is 8/4 bits
#define CURSOR_OR_DISPLAY_SHIFT_RL    0x04 //
#define CURSOR_OR_DISPLAY_SHIFT_SC    0x08 //
#define INTERNAL_OSC_FREQ_F0          0x01 //F2~0: adjust internal OSC frequency for FR frequency.
#define INTERNAL_OSC_FREQ_F1          0x02 //F2~0: adjust internal OSC frequency for FR frequency.
#define INTERNAL_OSC_FREQ_F2          0x04 //F2~0: adjust internal OSC frequency for FR frequency.
#define INTERNAL_OSC_FREQ_BS          0x08 //BS=1:1/4 bias (BS=0:1/5 bias)
#define POWER_ICON_BOST_CONTR_Bon     0x04 //Ion: ICON display on/off
#define POWER_ICON_BOST_CONTR_Ion     0x08 //Bon: set booster circuit on/off
#define FOLLOWER_CONTROL_Rab0         0x01 //Rab2~0: select follower amplified ratio
#define FOLLOWER_CONTROL_Rab1         0x02 //Rab2~0: select follower amplified ratio
#define FOLLOWER_CONTROL_Rab2         0x04 //Rab2~0: select follower amplified ratio
#define FOLLOWER_CONTROL_Fon          0x08 //Fon: set follower circuit on/off

#define CONTRAST_MAX                  0x3F //limit range max value (0x00 - 0x3F)
#define CONTRAST_MIN                  0x00 //limit range min value (0x00 - 0x3F)
#define WRITE_DELAY_MS                  30 //see data sheet
#define HOME_CLEAR_DELAY_MS			     2 //see data sheet
#define WRITE_DELAY_US                  30 //30us for 380KHz, 100us for 100KHz

/**
 * LCD module intialisation.
 */
void LCD_Init2(void);
void LCD_Init(void);
/**
 * send single 2 byte command & instruction to Instruction Register
 */
void LCD_WriteIR(uint8_t Cmd);
/**
 * send single 2 byte command & data to Data Register
 * e.g. display a character
 */
void LCD_WriteDR(char *Data);

/**
 * send a series of command & data bytes to Data Register
 * e.g. display several characters, starting at Position
 */
void LCD_WriteString(char *Str, uint8_t Position);

void LCD_Clear(void);
#ifdef	__cplusplus
}
#endif

#endif	/* LCD_H */



