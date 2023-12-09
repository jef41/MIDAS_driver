#include "mcc_generated_files/mcc.h"
#include "mcc_generated_files/examples/i2c1_master_example.h"
#include "lcd.h"
#include <string.h> 

void LCD_Init(void){
    __delay_ms(45);
    //Function set 8bit, 2 line display                                         //0x38
    LCD_WriteIR(FUNCTION_SET | FUNCTION_SET_DL | FUNCTION_SET_N);
    //Function set, as above, select Instruction Table 1 (set up contrast etc.) //0x39                                                                     
    LCD_WriteIR(FUNCTION_SET | FUNCTION_SET_DL | FUNCTION_SET_N | FUNCTION_SET_IS);
    //Int OSC Frequency (FREQUENCY | FREQUENCY_BS F2 F1 F0)1/4bias              0x14
    LCD_WriteIR(INTERNAL_OSC_FREQ | INTERNAL_OSC_FREQ_F2);
    //Contrast set (CONTRAST C3 C2 C1 C0)    0b01110100                         0x70
    LCD_WriteIR(CONTRAST_SET);
    // Power/ICON/Contrast Control CONTRAST_CTL Ion Bon C5 C4)                  0x56
    LCD_WriteIR(POWER_ICON_BOST_CONTR | POWER_ICON_BOST_CONTR_Ion | POWER_ICON_BOST_CONTR_Bon | 0x2);
    //Follower control (FOLLOWER Fon Rab2 Rab1 Rab0) Fon Rab2   0b01101101      0x6c
    LCD_WriteIR(FOLLOWER_CONTROL | FOLLOWER_CONTROL_Fon | FOLLOWER_CONTROL_Rab2);
    __delay_ms(200);
    
    //Disable extension Function set, 8bit, 2 line display                      //0x38
    LCD_WriteIR(FUNCTION_SET | FUNCTION_SET_DL | FUNCTION_SET_N);
    //display on, cursor on
    LCD_WriteIR(DISPLAY_ON_OFF | DISPLAY_ON_OFF_D ); //| DISPLAY_ON_OFF_C);
    //clear
    LCD_WriteIR(CLEAR_DISPLAY);
    __delay_ms(HOME_CLEAR_DELAY_MS);
    //entry mode set
    //LCD_WriteIR(ENTRY_MODE_SET | ENTRY_MODE_SET_ID);    
    // Init end
    // at this point we are still in instruction Table1, so character shift won't work yet
    // display should be writable, send home or move to RAM address first
    //however seems to require and additional 
    //LCD_WriteIR(FUNCTION_SET);
    //right LSB nibble does not matter, but 0x20 sets to 1 lines display
}

void LCD_WriteIR(uint8_t Cmd)
{
  //Control bit & Command Register Select instruction: Control=1 RS=0 RW=0
  
  uint8_t data[2];
  data[0] = CNTRBIT_CO;    //Command byte, & write to instruction register
  data[1] = Cmd;           //command
  I2C1_WriteNBytes(LCD_Address, data, 2);
  /*
  //alternate 1 byte into instruction register
  I2C1_Write1ByteRegister(LCD_Address, CNTRBIT_CO, Cmd);
   */
  __delay_us(WRITE_DELAY_US); 
}

void LCD_WriteDR(char *Data)
{
  uint8_t data[2];
  //Control bit & Command Register Select data: Control=1 RS=1 RW=0
  data[0] = CNTRBIT_CO | CNTRBIT_RS; //Command byte, & write to data register
  data[1] = Data[0];
  // send address then data
  I2C1_WriteNBytes(LCD_Address, data, 2);
  /*
  //alternate 1 byte, written to Data Register
  I2C1_Write1ByteRegister(LCD_Address, CNTRBIT_CO | CNTRBIT_RS, Data[0]);
  */
   
  __delay_us(WRITE_DELAY_US);
}

void LCD_WriteString(char *Str, uint8_t Position)
{
    uint8_t i;
    uint8_t longData[0x20];                          // sensible number for 1 line - improve/error check this
    //Control bit & Command Register Select instruction: Control=1 RS=0 RW=0
    longData[0] = CNTRBIT_CO;                        //Command byte IR follows
    longData[1] = SET_DDRAM_ADDRESS | Position;      //location
    //!Control bit & Command Register Select data: Control=0 RS=1 RW=0
    longData[2] = CNTRBIT_RS;                        //This is the last control byte, only Data byte(s) follow 
    for( i=0; Str[i]!='\0'; i++ ) 
    {
       longData[i+3]=(Str[i]);
    }
    I2C1_WriteNBytes(LCD_Address, longData, i+3);
    __delay_us(WRITE_DELAY_US);
    //why does it need this line? sets IR -> Function & set to 1 line display
    //without this nothing is displayed
    //with this the command sets to 1 line display
    //LCD_WriteIR(0b00100011); //Function set 
}

void LCD_Init2(void){
    __delay_ms(45);
    uint8_t data[7];
    data[0] = CNTRBIT;
    //Function set 8bit, 2 line display                                         //0x38
    data[1] = FUNCTION_SET | FUNCTION_SET_DL | FUNCTION_SET_N;
    //Function set, as above, select Instruction Table 1 (set up contrast etc.) //0x39                                                                     
    data[2] = FUNCTION_SET | FUNCTION_SET_DL | FUNCTION_SET_N | FUNCTION_SET_IS;
    //Int OSC Frequency (FREQUENCY | FREQUENCY_BS F2 F1 F0)1/4bias              0x14
    data[3] = INTERNAL_OSC_FREQ | INTERNAL_OSC_FREQ_F2;
    //Contrast set (CONTRAST C3 C2 C1 C0)    0b01110100                         0x70
    data[4] = CONTRAST_SET;
    // Power/ICON/Contrast Control CONTRAST_CTL Ion Bon C5 C4)                  0x56
    data[5] = POWER_ICON_BOST_CONTR | POWER_ICON_BOST_CONTR_Ion | POWER_ICON_BOST_CONTR_Bon | 0x2;
    //Follower control (FOLLOWER Fon Rab2 Rab1 Rab0) Fon Rab2   0b01101101      0x6c
    data[6] = FOLLOWER_CONTROL | FOLLOWER_CONTROL_Fon | FOLLOWER_CONTROL_Rab2;
    I2C1_WriteNBytes(LCD_Address, data, 7);
    __delay_ms(200);
    
    //Disable extension Function set, 8bit, 2 line display                      //0x38
    data[1] = FUNCTION_SET | FUNCTION_SET_DL | FUNCTION_SET_N;
    //display on, cursor on
    data[2] = DISPLAY_ON_OFF | DISPLAY_ON_OFF_D ; //| DISPLAY_ON_OFF_C);
    //clear
    data[3] = CLEAR_DISPLAY;
    I2C1_WriteNBytes(LCD_Address, data, 4);
    __delay_ms(HOME_CLEAR_DELAY_MS);
    //entry mode set
    //LCD_WriteIR(ENTRY_MODE_SET | ENTRY_MODE_SET_ID);    
    // Init end

}

void LCD_Clear(void){
    LCD_WriteIR(CLEAR_DISPLAY);
    __delay_ms(HOME_CLEAR_DELAY_MS);
}
